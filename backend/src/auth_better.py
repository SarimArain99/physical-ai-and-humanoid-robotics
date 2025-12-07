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

from src.database import get_db

router = APIRouter()

# Secret for JWT tokens
JWT_SECRET = os.getenv("BETTER_AUTH_SECRET", "fallback-secret-for-development")
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
    Better Auth compatible registration endpoint
    """
    try:
        # Check if user already exists
        result = db.execute(text("SELECT * FROM users WHERE email = :email"), {"email": request.email})
        existing_user = result.mappings().fetchone()

        if existing_user:
            raise HTTPException(status_code=400, detail="User already exists")

        # Hash the password
        hashed_password = hash_password(request.password)

        # Create new user
        user_id = str(uuid.uuid4())
        now = datetime.now(timezone.utc)

        db.execute(text("""
            INSERT INTO users (id, email, name, password_hash, created_at, updated_at, email_verified)
            VALUES (:id, :email, :name, :password_hash, :created_at, :updated_at, :email_verified)
        """), {
            "id": user_id,
            "email": request.email,
            "name": request.name,
            "password_hash": hashed_password,
            "proficiency": request.proficiency,
            "created_at": now,
            "updated_at": now,
            "email_verified": False
        })

        db.commit()

        # Create access token
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data={"sub": request.email, "user_id": user_id},
            expires_delta=access_token_expires
        )

        return {
            "user": {
                "id": user_id,
                "email": request.email,
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
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Registration failed: {str(e)}")


@router.post("/sign-in/email")
async def login_user(request: UserLoginRequest, db: Session = Depends(get_db)):
    """
    Better Auth compatible email/password login endpoint
    """
    try:
        # Find user by email
        result = db.execute(text("SELECT * FROM users WHERE email = :email"), {"email": request.email})
        user_row = result.mappings().fetchone()

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

        # Create access token
        access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        access_token = create_access_token(
            data={"sub": user_row['email'], "user_id": user_row['id']},
            expires_delta=access_token_expires
        )

        return {
            "user": {
                "id": user_row['id'],
                "email": user_row['email'],
                "name": user_row['name'],
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


@router.get("/get-session")
async def get_session(request: Request, db: Session = Depends(get_db)):
    """
    Better Auth compatible session endpoint to get current user
    """
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        return {"user": None, "session": None}

    token = auth_header[7:]

    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[ALGORITHM])
        email = payload.get("sub")

        if not email:
            return {"user": None, "session": None}

        result = db.execute(text("SELECT * FROM users WHERE email = :email"), {"email": email})
        user_row = result.mappings().fetchone()

        if not user_row:
            return {"user": None, "session": None}

        return {
            "user": {
                "id": user_row['id'],
                "email": user_row['email'],
                "name": user_row['name'],
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


# --- NEW: Profile Update Endpoint ---
@router.post("/profile/update")
async def update_user_profile(
    profile_data: ProfileUpdateRequest, 
    request: Request, 
    db: Session = Depends(get_db)
):
    """
    Update user profile information
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

        # 2. Check if profile exists
        result = db.execute(text("SELECT * FROM user_profiles WHERE user_id = :user_id"), {"user_id": user_id})
        existing_profile = result.mappings().fetchone()
        
        now = datetime.now(timezone.utc)

        if existing_profile:
            # Update existing
            db.execute(text("""
                UPDATE user_profiles 
                SET technical_background = :tb, 
                    hardware_access = :ha, 
                    learning_goals = :lg,
                    updated_at = :now
                WHERE user_id = :user_id
            """), {
                "tb": profile_data.technical_background,
                "ha": profile_data.hardware_access,
                "lg": profile_data.learning_goals,
                "now": now,
                "user_id": user_id
            })
        else:
            # Create new
            new_id = str(uuid.uuid4())
            db.execute(text("""
                INSERT INTO user_profiles (id, user_id, technical_background, hardware_access, learning_goals, created_at, updated_at)
                VALUES (:id, :user_id, :tb, :ha, :lg, :now, :now)
            """), {
                "id": new_id,
                "user_id": user_id,
                "tb": profile_data.technical_background,
                "ha": profile_data.hardware_access,
                "lg": profile_data.learning_goals,
                "now": now
            })
            
        db.commit()
        return {"success": True}
        
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to update profile: {str(e)}")