"""
Better Auth compatible endpoints for FastAPI backend
"""
from fastapi import APIRouter, HTTPException, Depends, Request
from pydantic import BaseModel
from typing import Optional
import jwt
import bcrypt
from datetime import datetime, timedelta, timezone
from sqlalchemy.orm import Session
from sqlalchemy import text
import uuid
import os
import sys

# --- PATH FIX: Force Python to see the 'backend' folder ---
current_dir = os.path.dirname(os.path.abspath(__file__)) # backend/src
parent_dir = os.path.dirname(current_dir)                # backend
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
# ----------------------------------------------------------

# Load .env file BEFORE checking environment variables
from dotenv import load_dotenv
env_path = os.path.join(parent_dir, ".env")
load_dotenv(env_path)

from src.database import get_db, encrypt_user_data, decrypt_user_data, find_user_by_email
from src.utils.encryption import encrypt_profile_data, decrypt_profile_data, hash_email_for_lookup

router = APIRouter()

# Secret for JWT tokens - MUST be set in environment, no fallback for security
JWT_SECRET = os.environ.get("BETTER_AUTH_SECRET")
if not JWT_SECRET:
    raise RuntimeError(
        "CRITICAL: BETTER_AUTH_SECRET environment variable is not set. "
        "This is required for secure JWT token generation. "
        "Set it in your .env file or environment before starting the server."
    )
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# --- Request Models ---
class UserRegisterRequest(BaseModel):
    email: str
    name: str
    password: str
    proficiency: str = "pro"

class UserLoginRequest(BaseModel):
    email: str
    password: str

class ProfileUpdateRequest(BaseModel):
    technical_background: Optional[str] = ""
    hardware_access: Optional[str] = ""
    learning_goals: Optional[str] = ""

class UserResponse(BaseModel):
    id: str
    email: str
    name: str
    emailVerified: Optional[bool] = False
    createdAt: datetime
    updatedAt: datetime

# --- Helper Functions ---
def create_access_token(data: dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, JWT_SECRET, algorithm=ALGORITHM)
    return encoded_jwt

def verify_password(plain_password: str, hashed_password: str):
    return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))

def hash_password(password: str):
    salt = bcrypt.gensalt()
    return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')

# --- Routes ---

@router.post("/sign-up")
async def register_user(request: UserRegisterRequest, db: Session = Depends(get_db)):
    """
    Better Auth compatible registration endpoint with encrypted user data (FR-014)
    """
    try:
        # Check if user already exists using email hash for efficient lookup
        existing_user = find_user_by_email(db, request.email)

        if existing_user:
            raise HTTPException(status_code=400, detail="User already exists")

        # Hash the password
        hashed_password = hash_password(request.password)

        # Encrypt user email and name (FR-014)
        encrypted_email, email_hash, encrypted_name = encrypt_user_data(request.email, request.name)

        # Create new user
        user_id = str(uuid.uuid4())
        now = datetime.now(timezone.utc)

        db.execute(text("""
            INSERT INTO users (id, email, email_hash, name, password_hash, proficiency, created_at, updated_at, email_verified)
            VALUES (:id, :email, :email_hash, :name, :password_hash, :proficiency, :created_at, :updated_at, :email_verified)
        """), {
            "id": user_id,
            "email": encrypted_email,
            "email_hash": email_hash,
            "name": encrypted_name,
            "password_hash": hashed_password,
            "proficiency": request.proficiency,
            "created_at": now,
            "updated_at": now,
            "email_verified": False
        })

        db.commit()

        # Create access token (uses email hash for subject to avoid storing plaintext)
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data={"sub": email_hash, "user_id": user_id},
            expires_delta=access_token_expires
        )

        # Return decrypted data to client
        return {
            "user": {
                "id": user_id,
                "email": request.email,  # Return original email to client
                "name": request.name,
                "proficiency": request.proficiency,
                "emailVerified": False,
                "createdAt": now.isoformat(),
                "updatedAt": now.isoformat()
            },
            "session": {
                "accessToken": access_token,
                "tokenType": "Bearer",
                "expiresAt": (now + access_token_expires).isoformat()
            }
        }
    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Registration failed: {str(e)}")


@router.post("/sign-in/email")
async def login_user(request: UserLoginRequest, db: Session = Depends(get_db)):
    """
    Better Auth compatible email/password login endpoint with encrypted data support (FR-014)
    """
    try:
        # Find user by email using hash lookup (works with encrypted data)
        user_row = find_user_by_email(db, request.email)

        # Verify password
        if not user_row or not verify_password(request.password, user_row['password_hash']):
            raise HTTPException(status_code=401, detail="Invalid credentials")

        # Update last login time
        db.execute(text("""
            UPDATE users SET updated_at = :updated_at WHERE id = :id
        """), {
            "updated_at": datetime.now(timezone.utc),
            "id": user_row['id']
        })
        db.commit()

        # Decrypt user data for response (FR-014)
        decrypted_email, decrypted_name = decrypt_user_data(
            user_row['email'],
            user_row['name']
        )

        # Get email_hash for token subject
        email_hash = user_row.get('email_hash') or hash_email_for_lookup(request.email)

        # Create access token
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data={"sub": email_hash, "user_id": user_row['id']},
            expires_delta=access_token_expires
        )

        return {
            "user": {
                "id": user_row['id'],
                "email": decrypted_email,
                "name": decrypted_name,
                "proficiency": user_row.get('proficiency', 'pro'),
                "emailVerified": user_row.get('email_verified', False),
                "createdAt": user_row['created_at'].isoformat() if user_row['created_at'] else None,
                "updatedAt": user_row['updated_at'].isoformat() if user_row['updated_at'] else None
            },
            "session": {
                "accessToken": access_token,
                "tokenType": "Bearer",
                "expiresAt": (datetime.now(timezone.utc) + access_token_expires).isoformat()
            }
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Login failed: {str(e)}")


@router.post("/sign-out")
async def logout_user(request: Request):
    """
    Better Auth compatible logout endpoint
    """
    return {"success": True}


@router.get("/verify")
async def verify_token(request: Request, db: Session = Depends(get_db)):
    """
    Verify JWT token validity and return user info
    Used by frontend to check if token is still valid
    """
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="No authorization token provided")

    token = auth_header[7:]

    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[ALGORITHM])
        user_id = payload.get("user_id")
        email_or_hash = payload.get("sub")

        if not user_id and not email_or_hash:
            raise HTTPException(status_code=401, detail="Invalid token payload")

        # Try to find user by user_id first
        user_row = None
        if user_id:
            result = db.execute(text("SELECT * FROM users WHERE id = :id"), {"id": user_id})
            user_row = result.mappings().fetchone()

        # Fallback: try email_hash lookup
        if not user_row and email_or_hash:
            result = db.execute(
                text("SELECT * FROM users WHERE email_hash = :email_hash"),
                {"email_hash": email_or_hash}
            )
            user_row = result.mappings().fetchone()

        if not user_row:
            raise HTTPException(status_code=401, detail="User not found")

        # Decrypt user data for response
        decrypted_email, decrypted_name = decrypt_user_data(
            user_row['email'],
            user_row['name']
        )

        return {
            "valid": True,
            "user": {
                "id": user_row['id'],
                "email": decrypted_email,
                "name": decrypted_name,
                "proficiency": user_row.get('proficiency', 'pro')
            }
        }
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Token verification failed: {str(e)}")


@router.get("/get-session")
async def get_session(request: Request, db: Session = Depends(get_db)):
    """
    Better Auth compatible session endpoint to get current user with decrypted data (FR-014)
    """
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        return {"user": None, "session": None}

    token = auth_header[7:]

    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[ALGORITHM])
        email_or_hash = payload.get("sub")
        user_id = payload.get("user_id")

        if not email_or_hash and not user_id:
            return {"user": None, "session": None}

        # Try to find user by user_id first (more reliable with encryption)
        user_row = None
        if user_id:
            result = db.execute(text("SELECT * FROM users WHERE id = :id"), {"id": user_id})
            user_row = result.mappings().fetchone()

        # Fallback: try email_hash lookup
        if not user_row and email_or_hash:
            result = db.execute(
                text("SELECT * FROM users WHERE email_hash = :email_hash"),
                {"email_hash": email_or_hash}
            )
            user_row = result.mappings().fetchone()

        if not user_row:
            return {"user": None, "session": None}

        # Decrypt user data for response (FR-014)
        decrypted_email, decrypted_name = decrypt_user_data(
            user_row['email'],
            user_row['name']
        )

        return {
            "user": {
                "id": user_row['id'],
                "email": decrypted_email,
                "name": decrypted_name,
                "proficiency": user_row.get('proficiency', 'pro'),
                "emailVerified": user_row.get('email_verified', False),
                "createdAt": user_row['created_at'].isoformat() if user_row['created_at'] else None,
                "updatedAt": user_row['updated_at'].isoformat() if user_row['updated_at'] else None
            },
            "session": {
                "accessToken": token,
                "tokenType": "Bearer",
                "expiresAt": payload.get("exp")
            }
        }
    except Exception:
        return {"user": None, "session": None}


# --- Profile Update Endpoint with Encryption (FR-014) ---
@router.post("/profile/update")
async def update_user_profile(
    profile_data: ProfileUpdateRequest,
    request: Request,
    db: Session = Depends(get_db)
):
    """
    Update user profile information with encrypted data storage (FR-014)
    """
    # 1. Get the user from the token
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Not authenticated")

    token = auth_header[7:]

    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[ALGORITHM])
        user_id = payload.get("user_id")

        if not user_id:
            raise HTTPException(status_code=401, detail="Invalid token")

        # 2. Encrypt profile data (FR-014)
        encrypted_tb = encrypt_profile_data(profile_data.technical_background) if profile_data.technical_background else ""
        encrypted_ha = encrypt_profile_data(profile_data.hardware_access) if profile_data.hardware_access else ""
        encrypted_lg = encrypt_profile_data(profile_data.learning_goals) if profile_data.learning_goals else ""

        # 3. Check if profile exists
        result = db.execute(text("SELECT * FROM user_profiles WHERE user_id = :user_id"), {"user_id": user_id})
        existing_profile = result.mappings().fetchone()

        now = datetime.now(timezone.utc)

        if existing_profile:
            # Update existing with encrypted data
            db.execute(text("""
                UPDATE user_profiles
                SET technical_background = :tb,
                    hardware_access = :ha,
                    learning_goals = :lg,
                    updated_at = :now
                WHERE user_id = :user_id
            """), {
                "tb": encrypted_tb,
                "ha": encrypted_ha,
                "lg": encrypted_lg,
                "now": now,
                "user_id": user_id
            })
        else:
            # Create new with encrypted data
            new_id = str(uuid.uuid4())
            db.execute(text("""
                INSERT INTO user_profiles (id, user_id, technical_background, hardware_access, learning_goals, created_at, updated_at)
                VALUES (:id, :user_id, :tb, :ha, :lg, :now, :now)
            """), {
                "id": new_id,
                "user_id": user_id,
                "tb": encrypted_tb,
                "ha": encrypted_ha,
                "lg": encrypted_lg,
                "now": now
            })

        db.commit()
        return {"success": True}

    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to update profile: {str(e)}")


@router.get("/profile")
async def get_user_profile(request: Request, db: Session = Depends(get_db)):
    """
    Get user profile with decrypted data (FR-014)
    """
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Not authenticated")

    token = auth_header[7:]

    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[ALGORITHM])
        user_id = payload.get("user_id")

        if not user_id:
            raise HTTPException(status_code=401, detail="Invalid token")

        result = db.execute(text("SELECT * FROM user_profiles WHERE user_id = :user_id"), {"user_id": user_id})
        profile_row = result.mappings().fetchone()

        if not profile_row:
            return {"profile": None}

        # Decrypt profile data (FR-014)
        return {
            "profile": {
                "technical_background": decrypt_profile_data(profile_row['technical_background']) if profile_row['technical_background'] else "",
                "hardware_access": decrypt_profile_data(profile_row['hardware_access']) if profile_row['hardware_access'] else "",
                "learning_goals": decrypt_profile_data(profile_row['learning_goals']) if profile_row['learning_goals'] else "",
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get profile: {str(e)}")