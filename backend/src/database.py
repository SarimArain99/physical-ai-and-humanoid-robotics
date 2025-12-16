from sqlalchemy import create_engine, text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import sys
import os

# --- PATH FIX: Force Python to look in the parent 'backend' folder ---
# This ensures we find the real 'config.py', not a random system file
current_dir = os.path.dirname(os.path.abspath(__file__)) # backend/src
parent_dir = os.path.dirname(current_dir)                # backend
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
# ---------------------------------------------------------------------

from config import settings
from src.utils.encryption import (
    encrypt_email, decrypt_email, encrypt_name, decrypt_name,
    hash_email_for_lookup, is_encrypted
)

# 1. Get the URL
raw_db_url = settings.neon_api_key

# 2. Fix for Neon/Render connection strings (postgres:// -> postgresql://)
if raw_db_url and raw_db_url.startswith("postgres://"):
    DATABASE_URL = raw_db_url.replace("postgres://", "postgresql://", 1)
else:
    DATABASE_URL = raw_db_url

engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=5,
    max_overflow=10,
    pool_pre_ping=True,
    pool_recycle=300,
)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def create_tables():
    """Create all necessary tables in the database"""

    with engine.connect() as conn:
        trans = conn.begin()
        try:
            # Create users table with email_hash for encrypted email lookups (FR-014)
            conn.execute(text("""
                CREATE TABLE IF NOT EXISTS users (
                    id VARCHAR(36) PRIMARY KEY,
                    email VARCHAR(512) NOT NULL,
                    email_hash VARCHAR(64) UNIQUE NOT NULL,
                    name VARCHAR(512),
                    password_hash VARCHAR(255) NOT NULL,
                    proficiency VARCHAR(50) DEFAULT 'pro',
                    provider VARCHAR(50) DEFAULT 'credentials',
                    provider_id VARCHAR(255),
                    email_verified BOOLEAN DEFAULT FALSE,
                    is_active BOOLEAN DEFAULT TRUE,
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
                );
            """))

            # Add email_hash column if it doesn't exist (migration for existing tables)
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.columns
                        WHERE table_name = 'users' AND column_name = 'email_hash'
                    ) THEN
                        ALTER TABLE users ADD COLUMN email_hash VARCHAR(64);
                        -- Drop old unique constraint on email if exists
                        ALTER TABLE users DROP CONSTRAINT IF EXISTS users_email_key;
                        -- Add unique constraint on email_hash
                        ALTER TABLE users ADD CONSTRAINT users_email_hash_key UNIQUE (email_hash);
                    END IF;
                END $$;
            """))

            # Create user_profiles table (profile data is encrypted via application layer)
            conn.execute(text("""
                CREATE TABLE IF NOT EXISTS user_profiles (
                    id VARCHAR(36) PRIMARY KEY,
                    user_id VARCHAR(36) NOT NULL,
                    technical_background TEXT,
                    hardware_access TEXT,
                    learning_goals TEXT,
                    preferences TEXT,
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
                );
            """))

            # --- PHASE 10: Chat History Tables (T149) ---

            # Create chat_sessions table for conversation sessions
            conn.execute(text("""
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    id VARCHAR(36) PRIMARY KEY,
                    user_id VARCHAR(36) NOT NULL,
                    title VARCHAR(255) NOT NULL DEFAULT 'New Chat',
                    is_active BOOLEAN DEFAULT TRUE,
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
                );
            """))

            # Create index on user_id for faster session lookups
            conn.execute(text("""
                CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id
                ON chat_sessions(user_id);
            """))

            # Create index on updated_at for ordering by recent
            conn.execute(text("""
                CREATE INDEX IF NOT EXISTS idx_chat_sessions_updated_at
                ON chat_sessions(updated_at DESC);
            """))

            # Create chat_messages table for individual messages
            conn.execute(text("""
                CREATE TABLE IF NOT EXISTS chat_messages (
                    id VARCHAR(36) PRIMARY KEY,
                    session_id VARCHAR(36) NOT NULL,
                    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
                    content TEXT NOT NULL,
                    selected_text TEXT,
                    sources TEXT,
                    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (session_id) REFERENCES chat_sessions(id) ON DELETE CASCADE
                );
            """))

            # Create index on session_id for faster message retrieval
            conn.execute(text("""
                CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id
                ON chat_messages(session_id);
            """))

            # Create index on created_at for ordering messages
            conn.execute(text("""
                CREATE INDEX IF NOT EXISTS idx_chat_messages_created_at
                ON chat_messages(created_at);
            """))

            trans.commit()
            print("Database tables verified/created successfully")
        except Exception as e:
            trans.rollback()
            print(f"Error creating tables: {e}")
            raise


# --- ENCRYPTION HELPER FUNCTIONS FOR USER DATA (FR-014) ---

def encrypt_user_data(email: str, name: str) -> tuple:
    """
    Encrypt user email and name for storage.
    Returns (encrypted_email, email_hash, encrypted_name)
    """
    encrypted_email = encrypt_email(email)
    email_hash = hash_email_for_lookup(email)
    encrypted_name = encrypt_name(name) if name else None
    return encrypted_email, email_hash, encrypted_name


def decrypt_user_data(encrypted_email: str, encrypted_name: str) -> tuple:
    """
    Decrypt user email and name for display.
    Returns (email, name) - handles both encrypted and plaintext data.
    """
    email = decrypt_email(encrypted_email) if is_encrypted(encrypted_email) else encrypted_email
    name = decrypt_name(encrypted_name) if encrypted_name and is_encrypted(encrypted_name) else encrypted_name
    return email, name


def find_user_by_email(db_session, email: str):
    """
    Find a user by email using the email_hash for efficient lookup.
    This allows searching without decrypting all records.
    """
    email_hash = hash_email_for_lookup(email)
    result = db_session.execute(
        text("SELECT * FROM users WHERE email_hash = :email_hash"),
        {"email_hash": email_hash}
    )
    return result.mappings().fetchone()