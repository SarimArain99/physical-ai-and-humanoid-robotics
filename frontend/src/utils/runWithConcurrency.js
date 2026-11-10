// Workstream C — Level rewrite speed: bounded-concurrency pool.
//
// The old runChunked sent every chunk one after another and, on a
// delimiter mismatch, every paragraph one after another — up to ~35
// sequential OpenAI calls for one long chapter (~7 minutes live).
// This pool runs at most `limit` items at once, preserves input order
// in the returned array, and reports each finished item so the UI can
// show honest counts ("12 of 35") instead of a made-up percentage.
//
// Contract:
//   runWithConcurrency(items, limit, fn, { signal, onSettled, onItemError })
//   - fn(item, index, signal) may throw; a failed item becomes null and
//     does not stop the others (onItemError is told, if given).
//   - Aborting the signal stops new items from starting; in-flight
//     items settle (their fetch aborts via the same signal) and the
//     remaining slots stay null. The result is { results, aborted }.
//   - onSettled(index, value) fires for every item, including nulls,
//     so progress accounting stays exact under parallelism.
export async function runWithConcurrency(items, limit, fn, options = {}) {
  const { signal, onSettled, onItemError } = options;
  const results = new Array(items.length).fill(null);
  let next = 0;
  let aborted = Boolean(signal?.aborted);

  const workerCount = Math.max(1, Math.min(limit || 1, items.length));
  const workers = Array.from({ length: workerCount }, async () => {
    for (;;) {
      if (signal?.aborted) {
        aborted = true;
        return;
      }
      const index = next;
      next += 1;
      if (index >= items.length) return;
      try {
        const value = await fn(items[index], index, signal);
        results[index] = value;
        if (onSettled) onSettled(index, value);
      } catch (error) {
        if (error?.name === "AbortError" || signal?.aborted) {
          aborted = true;
        } else if (onItemError) {
          onItemError(error, index);
        }
        results[index] = null;
        if (onSettled) onSettled(index, null);
      }
    }
  });

  await Promise.all(workers);
  if (signal?.aborted) aborted = true;
  return { results, aborted };
}
