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
            # Create users table
            conn.execute(text("""
                CREATE TABLE IF NOT EXISTS users (
                    id VARCHAR(36) PRIMARY KEY,
                    email VARCHAR(255) UNIQUE NOT NULL,
                    name VARCHAR(255),
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

            # Create user_profiles table
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

            trans.commit()
            print("Database tables verified/created successfully")
        except Exception as e:
            trans.rollback()
            print(f"Error creating tables: {e}")
            raise