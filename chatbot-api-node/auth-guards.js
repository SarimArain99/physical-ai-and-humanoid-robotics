// Express guards that sit in front of the better-auth handler
// (mounted in api/index.js). Kept in their own module so they can be
// unit-tested without booting the whole app or a database.

// Finding #6 (email enumeration): better-auth answers a duplicate
// sign-up with 422 USER_ALREADY_EXISTS_USE_ANOTHER_EMAIL, which lets
// anyone test whether an email has an account. There is no server hook
// to change that, so this middleware buffers only the sign-up/email
// response and rewrites that one error into the same neutral message a
// fresh sign-up gets. Every other auth response passes through
// untouched. The buffer is small (a JSON error body) and only applies
// to this one route.
//
// The writeHead interception matters: better-auth's Node adapter sends
// the status line via res.writeHead BEFORE the body reaches res.end,
// so changing res.statusCode inside res.end would be too late — the
// 422 line would already be on the wire, still leaking existence.
// writeHead is therefore held (status + headers captured, not flushed)
// until res.end, when the final status and body are sent together.
export function duplicateSignupGuard(req, res, next) {
  if (req.method !== "POST" || req.path !== "/sign-up/email") return next();
  const chunks = [];
  const origWrite = res.write.bind(res);
  const origEnd = res.end.bind(res);
  const origWriteHead = res.writeHead.bind(res);
  let pendingStatus = null;
  let pendingHeaders = null;
  res.writeHead = (statusCode, statusMessage, headers) => {
    pendingStatus = statusCode;
    if (statusMessage && typeof statusMessage === "object") pendingHeaders = statusMessage;
    else if (headers) pendingHeaders = headers;
    return res;
  };
  res.write = (chunk, ...args) => {
    if (chunk) chunks.push(Buffer.isBuffer(chunk) ? chunk : Buffer.from(chunk));
    return true;
  };
  res.end = (chunk, ...args) => {
    if (chunk) chunks.push(Buffer.isBuffer(chunk) ? chunk : Buffer.from(chunk));
    const body = Buffer.concat(chunks).toString("utf8");
    let out = body;
    let status = pendingStatus ?? res.statusCode;
    try {
      const data = JSON.parse(body);
      if (status === 422 && data && data.code === "USER_ALREADY_EXISTS_USE_ANOTHER_EMAIL") {
        status = 200;
        out = JSON.stringify({
          message: "If this email is not registered, an account has been created. Please check your inbox to continue.",
        });
      }
    } catch {
      // Not JSON — pass the response through unchanged.
    }
    res.write = origWrite;
    res.end = origEnd;
    res.writeHead = origWriteHead;
    // The rewritten body is a different length; drop a stale
    // Content-Length so Node recalculates it.
    res.removeHeader("Content-Length");
    // Send the (possibly rewritten) status now that the body is final.
    if (pendingHeaders) origWriteHead(status, pendingHeaders);
    else origWriteHead(status);
    return origEnd(out);
  };
  next();
}

// Finding #5 (password reset): better-auth runs sendResetPassword in
// the background, so a failure there can never change the HTTP
// response — the user would be told to "check your inbox" even when no
// mail can ever arrive. When no mail transport is configured at all,
// fail honestly HERE, before better-auth creates a useless token: a 503
// for every reset request (existing and non-existing emails alike, so
// this leaks nothing about which emails have accounts).
export function resetMailGuard(req, res, next) {
  if (
    req.method === "POST" &&
    req.path === "/request-password-reset" &&
    !process.env.SMTP_HOST
  ) {
    console.error("Password reset requested but SMTP_HOST is not configured — no reset email can be sent.");
    return res.status(503).json({
      error: "Password reset email is not available right now. Please contact support.",
    });
  }
  next();
}
