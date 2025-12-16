import os
import sys

# --- PATH FIX: Ensure backend is in python path ---
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)
# --------------------------------------------------

import asyncio
from contextlib import asynccontextmanager
from typing import Dict, Any, AsyncGenerator, Optional
from fastapi import FastAPI, HTTPException, BackgroundTasks, Header
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from openai import OpenAI
from qdrant_client import QdrantClient
from config import settings
from ingest import DocumentIngestor
from src.database import create_tables
from src.auth_better import router as auth_router
import logging
import json
import uuid
import time
from functools import wraps
import uvicorn

# --- STRUCTURED LOGGING SETUP (FR-015) ---
class JSONFormatter(logging.Formatter):
    """JSON formatter for structured logging with correlation IDs."""

    def format(self, record):
        log_obj = {
            "timestamp": self.formatTime(record, self.datefmt),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "correlation_id": getattr(record, 'correlation_id', None),
        }
        if record.exc_info:
            log_obj["exception"] = self.formatException(record.exc_info)
        return json.dumps(log_obj)


# Set up structured logging
logging.basicConfig(level=logging.INFO)
handler = logging.StreamHandler()
handler.setFormatter(JSONFormatter())
logging.root.handlers = [handler]
logger = logging.getLogger(__name__)


# --- RETRY DECORATOR (FR-016) ---
def with_retry(max_retries: int = 3, delay: float = 1.0, backoff: float = 2.0):
    """
    Retry decorator with exponential backoff for external API calls.

    Args:
        max_retries: Maximum number of retry attempts
        delay: Initial delay between retries in seconds
        backoff: Multiplier for delay after each retry
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            last_exception = None
            current_delay = delay

            for attempt in range(max_retries + 1):
                try:
                    return await func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt < max_retries:
                        logger.warning(
                            f"Retry {attempt + 1}/{max_retries} for {func.__name__}: {e}",
                            extra={"correlation_id": kwargs.get("correlation_id")}
                        )
                        await asyncio.sleep(current_delay)
                        current_delay *= backoff
                    else:
                        logger.error(
                            f"All {max_retries} retries failed for {func.__name__}: {e}",
                            extra={"correlation_id": kwargs.get("correlation_id")}
                        )

            raise last_exception
        return wrapper
    return decorator


# Generate correlation ID for request tracing
def generate_correlation_id() -> str:
    """Generate a unique correlation ID for request tracing."""
    return str(uuid.uuid4())[:8]

# --- DATA MODELS ---
class QueryRequest(BaseModel):
    query: str
    selected_text: str = ""

class ChatResponse(BaseModel):
    response: str

class TranslateRequest(BaseModel):
    text: str

class ContentRequest(BaseModel):
    text: str
    target_level: str

# --- LIFESPAN MANAGER (Startup/Shutdown) ---
@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Initialize resources on startup"""
    # 1. Initialize OpenAI
    app.state.openai_client = OpenAI(api_key=settings.openai_api_key)

    # 2. Initialize Qdrant
    app.state.qdrant_client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
    )

    # 3. Verify Qdrant Connection
    try:
        app.state.qdrant_client.get_collection(settings.qdrant_collection_name)
        logger.info(f"✅ Connected to Qdrant collection: {settings.qdrant_collection_name}")
    except Exception as e:
        logger.error(f"⚠️ Could not connect to Qdrant collection: {e}")
        # We pass here so the app doesn't crash if Qdrant is sleeping
        pass

    # 4. Create Database Tables
    try:
        create_tables()
        logger.info("✅ Database tables created successfully")
    except Exception as e:
        logger.error(f"❌ Error creating database tables: {e}")
        raise

    yield

# --- APP CREATION ---
app = FastAPI(lifespan=lifespan)

# --- CORS Configuration (Secure) ---
# Allowed origins - configurable via environment variable
ALLOWED_ORIGINS = os.environ.get(
    "CORS_ALLOWED_ORIGINS",
    "http://localhost:3000,http://localhost:3001,https://physical-ai-textbook.vercel.app"
).split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,  # Restricted to specific frontend domains
    allow_credentials=False,  # Using Bearer tokens, not cookies
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],  # Explicit methods
    allow_headers=["Authorization", "Content-Type", "Accept"],  # Explicit headers
)

# --- ROUTERS ---
app.include_router(auth_router, prefix="/api/auth", tags=["authentication"])


# --- 🟢 ROOT ENDPOINT (For Health Check) ---
@app.get("/")
async def root():
    return {
        "status": "online", 
        "service": "Physical AI Backend", 
        "cors": "open_to_world"
    }

@app.get("/health")
async def health_check():
    return {"status": "healthy"}


# --- ENDPOINTS ---

@app.post("/ingest")
async def ingest_documents(background_tasks: BackgroundTasks):
    """Ingest all documents from the docs folder into Qdrant"""
    try:
        ingestor = DocumentIngestor()
        background_tasks.add_task(ingestor.ingest_documents)
        return {"message": "Document ingestion started in the background"}
    except Exception as e:
        logger.error(f"Error starting document ingestion: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: QueryRequest):
    """Chat endpoint that retrieves context from Qdrant and generates response"""
    correlation_id = generate_correlation_id()
    start_time = time.time()

    logger.info(
        f"Chat request received - query length: {len(request.query)}, has_selection: {bool(request.selected_text)}",
        extra={"correlation_id": correlation_id}
    )

    try:
        qdrant_client = app.state.qdrant_client
        openai_client = app.state.openai_client

        search_query = request.selected_text if request.selected_text else request.query

        # Generate embedding with retry logic (FR-016)
        max_retries = 3
        retry_delay = 1.0
        embedding_response = None

        for attempt in range(max_retries):
            try:
                embedding_response = openai_client.embeddings.create(
                    input=search_query,
                    model="text-embedding-3-small"
                )
                break
            except Exception as e:
                if attempt < max_retries - 1:
                    logger.warning(
                        f"Embedding retry {attempt + 1}/{max_retries}: {e}",
                        extra={"correlation_id": correlation_id}
                    )
                    await asyncio.sleep(retry_delay * (2 ** attempt))
                else:
                    raise

        query_embedding = embedding_response.data[0].embedding

        # Search Qdrant with retry logic (FR-016)
        search_results = None
        for attempt in range(max_retries):
            try:
                search_results = qdrant_client.search(
                    collection_name=settings.qdrant_collection_name,
                    query_vector=query_embedding,
                    limit=5,
                    with_payload=True
                )
                break
            except Exception as e:
                if attempt < max_retries - 1:
                    logger.warning(
                        f"Qdrant search retry {attempt + 1}/{max_retries}: {e}",
                        extra={"correlation_id": correlation_id}
                    )
                    await asyncio.sleep(retry_delay * (2 ** attempt))
                else:
                    raise

        # Context processing
        context_parts = []
        for result in search_results:
            content = result.payload.get("content", "")
            context_meta = result.payload.get("context", "")
            full_text = f"{context_meta}\n\n{content}" if context_meta else content
            context_parts.append(full_text)

        context = "\n\n".join(context_parts)

        logger.info(
            f"Retrieved {len(search_results)} context chunks",
            extra={"correlation_id": correlation_id}
        )

        system_prompt = (
            "You are an expert on the Physical AI & Humanoid Robotics textbook. "
            "Answer ONLY using the provided context. If the user provided 'selected_text', "
            "prioritize that specific snippet."
        )

        user_message = (
            f"Context:\n{context}\n\n"
            f"Question: {request.query}\n\n"
            f"Selected Text (if any): {request.selected_text if request.selected_text else 'None'}"
        )

        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            max_tokens=500,
            temperature=0.7
        )

        answer = response.choices[0].message.content
        return ChatResponse(response=answer)

    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/translate")
async def translate_text(request: TranslateRequest):
    """Translates technical text into Urdu using AI"""
    try:
        openai_client = app.state.openai_client
        
        system_prompt = (
            "You are a professional translator for a Robotics & AI textbook. "
            "Translate the following text into Urdu. "
            "Rules:\n"
            "1. Keep the tone academic and professional.\n"
            "2. Do NOT translate technical terms like 'ROS', 'Python', 'Algorithm', 'Sensor'. Keep them in English script.\n"
            "3. Return ONLY the translated text."
        )

        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.text}
            ],
            max_tokens=2000,
            temperature=0.3
        )

        translated_text = response.choices[0].message.content
        return {"translation": translated_text}

    except Exception as e:
        logger.error(f"Translation error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/adjust-content")
async def adjust_content_level(request: ContentRequest):
    """Rewrites text to match the user's proficiency level"""
    try:
        openai_client = app.state.openai_client
        
        if request.target_level == "beginner":
            instruction = "Rewrite this technical content for a high school student. Use simple analogies, avoid dense jargon."
        elif request.target_level == "intermediate":
            instruction = "Rewrite this content for an undergraduate engineering student. Simplify complex theory but keep technical terms."
        else:
            return {"content": request.text}

        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": f"You are a Physics & AI tutor. {instruction}"},
                {"role": "user", "content": request.text}
            ],
            max_tokens=2000
        )

        return {"content": response.choices[0].message.content}

    except Exception as e:
        logger.error(f"Adjustment error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# --- RAILWAY & LOCAL EXECUTION ---
if __name__ == "__main__":
    # Get port from environment variable (Railway sets this automatically)
    # Default to 8080 if not set
    port = int(os.environ.get("PORT", 8080))
    
    # Run the server
    uvicorn.run("main:app", host="0.0.0.0", port=port, reload=True)