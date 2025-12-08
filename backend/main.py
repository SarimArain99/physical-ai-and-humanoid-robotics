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
import uvicorn

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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

# --- 🟢 NUCLEAR CORS FIX (MUST BE HERE) ---
# This Regex allows HTTP and HTTPS from ANY website (Vercel, localhost, etc.)
app.add_middleware(
    CORSMiddleware,
    allow_origin_regex="https?://.*", 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
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
    try:
        qdrant_client = app.state.qdrant_client
        openai_client = app.state.openai_client

        search_query = request.selected_text if request.selected_text else request.query

        # Generate embedding
        embedding_response = openai_client.embeddings.create(
            input=search_query,
            model="text-embedding-3-small"
        )
        query_embedding = embedding_response.data[0].embedding

        # Search Qdrant
        search_results = qdrant_client.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=query_embedding,
            limit=5,
            with_payload=True
        )

        # Context processing
        context_parts = []
        for result in search_results:
            content = result.payload.get("content", "")
            context_meta = result.payload.get("context", "")
            full_text = f"{context_meta}\n\n{content}" if context_meta else content
            context_parts.append(full_text)

        context = "\n\n".join(context_parts)

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