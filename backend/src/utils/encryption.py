"""
Field-level encryption utilities for user data protection (FR-014).

This module provides AES-256-GCM encryption for sensitive user data stored in the database.
Key management relies on environment variables for security.
"""
import os
import base64
import hashlib
from typing import Optional
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC

# Encryption key derivation from environment variable
_ENCRYPTION_KEY: Optional[bytes] = None


def _get_encryption_key() -> bytes:
    """
    Get or derive the encryption key from environment variable.
    Uses PBKDF2 to derive a 256-bit key from the secret.
    """
    global _ENCRYPTION_KEY

    if _ENCRYPTION_KEY is not None:
        return _ENCRYPTION_KEY

    secret = os.environ.get("DATA_ENCRYPTION_KEY")
    if not secret:
        # For backwards compatibility, allow operation without encryption
        # but log a warning. In production, this should be required.
        import logging
        logging.warning(
            "DATA_ENCRYPTION_KEY not set. Encryption functions will return plaintext. "
            "Set this in production for FR-014 compliance."
        )
        return b""

    # Derive a 256-bit key using PBKDF2
    salt = b"physical-ai-textbook-salt-v1"  # Static salt - key rotation handled via secret change
    kdf = PBKDF2HMAC(
        algorithm=hashes.SHA256(),
        length=32,
        salt=salt,
        iterations=100000,
    )
    _ENCRYPTION_KEY = kdf.derive(secret.encode())
    return _ENCRYPTION_KEY


def encrypt_field(plaintext: str) -> str:
    """
    Encrypt a string field using AES-256-GCM.

    Args:
        plaintext: The string to encrypt

    Returns:
        Base64-encoded encrypted string with nonce prepended,
        or the original plaintext if encryption is not configured.
    """
    if not plaintext:
        return plaintext

    key = _get_encryption_key()
    if not key:
        # Encryption not configured - return plaintext
        return plaintext

    # Generate a random 96-bit nonce
    nonce = os.urandom(12)

    # Encrypt using AES-256-GCM
    aesgcm = AESGCM(key)
    ciphertext = aesgcm.encrypt(nonce, plaintext.encode('utf-8'), None)

    # Combine nonce + ciphertext and encode as base64
    encrypted_data = nonce + ciphertext
    return "ENC:" + base64.b64encode(encrypted_data).decode('utf-8')


def decrypt_field(encrypted_text: str) -> str:
    """
    Decrypt a string field encrypted with encrypt_field.

    Args:
        encrypted_text: The encrypted string (base64 with ENC: prefix)

    Returns:
        The decrypted plaintext string,
        or the original text if not encrypted or decryption fails.
    """
    if not encrypted_text:
        return encrypted_text

    # Check if this is actually encrypted
    if not encrypted_text.startswith("ENC:"):
        # Not encrypted - return as-is (backwards compatibility)
        return encrypted_text

    key = _get_encryption_key()
    if not key:
        # Encryption not configured - cannot decrypt
        raise ValueError("Cannot decrypt: DATA_ENCRYPTION_KEY not set")

    try:
        # Decode from base64
        encrypted_data = base64.b64decode(encrypted_text[4:])  # Skip "ENC:" prefix

        # Extract nonce (first 12 bytes) and ciphertext
        nonce = encrypted_data[:12]
        ciphertext = encrypted_data[12:]

        # Decrypt using AES-256-GCM
        aesgcm = AESGCM(key)
        plaintext = aesgcm.decrypt(nonce, ciphertext, None)

        return plaintext.decode('utf-8')
    except Exception as e:
        # If decryption fails, it might be plaintext from before encryption was enabled
        import logging
        logging.error(f"Decryption failed: {e}")
        raise ValueError(f"Failed to decrypt field: {e}")


def hash_email_for_lookup(email: str) -> str:
    """
    Create a deterministic hash of an email for database lookups.
    This allows searching encrypted emails without decrypting all records.

    Args:
        email: The email address to hash

    Returns:
        A hex-encoded SHA-256 hash of the lowercase email
    """
    if not email:
        return ""
    return hashlib.sha256(email.lower().encode('utf-8')).hexdigest()


def is_encrypted(value: str) -> bool:
    """Check if a value is encrypted (has ENC: prefix)."""
    return value and value.startswith("ENC:")


# Convenience functions for specific field types
def encrypt_email(email: str) -> str:
    """Encrypt an email address."""
    return encrypt_field(email)


def decrypt_email(encrypted_email: str) -> str:
    """Decrypt an email address."""
    return decrypt_field(encrypted_email)


def encrypt_name(name: str) -> str:
    """Encrypt a user's name."""
    return encrypt_field(name)


def decrypt_name(encrypted_name: str) -> str:
    """Decrypt a user's name."""
    return decrypt_field(encrypted_name)


def encrypt_profile_data(data: str) -> str:
    """Encrypt profile data (JSON string)."""
    return encrypt_field(data)


def decrypt_profile_data(encrypted_data: str) -> str:
    """Decrypt profile data (JSON string)."""
    return decrypt_field(encrypted_data)
