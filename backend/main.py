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
from fastapi import FastAPI, HTTPException, BackgroundTasks, Header, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from openai import OpenAI
from qdrant_client import QdrantClient
from config import settings
from ingest import DocumentIngestor
from src.database import create_tables, get_db
from src.auth_better import router as auth_router, JWT_SECRET, ALGORITHM
from src.services.chat_service import ChatService
from src.models.chat import MessageRole
import logging
import json
import jwt
import uuid
import time
from functools import wraps
from enum import Enum
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
import uvicorn

# --- OPENTELEMETRY TRACING (FR-015) ---
# Optional: Import OpenTelemetry if available
OTEL_ENABLED = False
try:
    from opentelemetry import trace
    from opentelemetry.sdk.trace import TracerProvider
    from opentelemetry.sdk.trace.export import BatchSpanProcessor, ConsoleSpanExporter
    from opentelemetry.instrumentation.fastapi import FastAPIInstrumentor
    from opentelemetry.sdk.resources import Resource
    from opentelemetry.semconv.resource import ResourceAttributes

    # Check if OTEL endpoint is configured
    if os.environ.get("OTEL_EXPORTER_OTLP_ENDPOINT"):
        from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter

        resource = Resource(attributes={
            ResourceAttributes.SERVICE_NAME: "physical-ai-backend",
            ResourceAttributes.SERVICE_VERSION: "1.0.0",
        })
        provider = TracerProvider(resource=resource)
        processor = BatchSpanProcessor(OTLPSpanExporter())
        provider.add_span_processor(processor)
        trace.set_tracer_provider(provider)
        OTEL_ENABLED = True
    elif os.environ.get("OTEL_CONSOLE_EXPORT"):
        # Console export for development
        resource = Resource(attributes={
            ResourceAttributes.SERVICE_NAME: "physical-ai-backend",
        })
        provider = TracerProvider(resource=resource)
        provider.add_span_processor(BatchSpanProcessor(ConsoleSpanExporter()))
        trace.set_tracer_provider(provider)
        OTEL_ENABLED = True
except ImportError:
    pass  # OpenTelemetry not installed - continue without tracing

tracer = trace.get_tracer(__name__) if OTEL_ENABLED else None

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


# --- CIRCUIT BREAKER PATTERN (FR-016) ---
class CircuitState(Enum):
    """Circuit breaker states."""
    CLOSED = "closed"      # Normal operation
    OPEN = "open"          # Failing, reject requests
    HALF_OPEN = "half_open"  # Testing if service recovered


@dataclass
class CircuitBreaker:
    """
    Circuit breaker for external API calls (FR-016).
    Prevents cascading failures when external services are down.
    """
    name: str
    failure_threshold: int = 5      # Number of failures before opening
    recovery_timeout: int = 30      # Seconds before trying again
    success_threshold: int = 2      # Successes needed to close circuit

    state: CircuitState = field(default=CircuitState.CLOSED)
    failure_count: int = field(default=0)
    success_count: int = field(default=0)
    last_failure_time: Optional[datetime] = field(default=None)

    def can_execute(self) -> bool:
        """Check if the circuit allows execution."""
        if self.state == CircuitState.CLOSED:
            return True

        if self.state == CircuitState.OPEN:
            # Check if recovery timeout has passed
            if self.last_failure_time and datetime.now() - self.last_failure_time > timedelta(seconds=self.recovery_timeout):
                self.state = CircuitState.HALF_OPEN
                self.success_count = 0
                logger.info(f"Circuit breaker '{self.name}' transitioning to HALF_OPEN")
                return True
            return False

        # HALF_OPEN - allow limited requests
        return True

    def record_success(self):
        """Record a successful call."""
        if self.state == CircuitState.HALF_OPEN:
            self.success_count += 1
            if self.success_count >= self.success_threshold:
                self.state = CircuitState.CLOSED
                self.failure_count = 0
                logger.info(f"Circuit breaker '{self.name}' closed - service recovered")
        elif self.state == CircuitState.CLOSED:
            self.failure_count = 0  # Reset on success

    def record_failure(self):
        """Record a failed call."""
        self.failure_count += 1
        self.last_failure_time = datetime.now()

        if self.state == CircuitState.HALF_OPEN:
            # Any failure in half-open goes back to open
            self.state = CircuitState.OPEN
            logger.warning(f"Circuit breaker '{self.name}' reopened after failure in half-open state")
        elif self.state == CircuitState.CLOSED and self.failure_count >= self.failure_threshold:
            self.state = CircuitState.OPEN
            logger.warning(f"Circuit breaker '{self.name}' opened after {self.failure_count} failures")


# Initialize circuit breakers for external services
openai_circuit = CircuitBreaker(name="openai", failure_threshold=5, recovery_timeout=30)
qdrant_circuit = CircuitBreaker(name="qdrant", failure_threshold=3, recovery_timeout=60)

# --- DATA MODELS ---
class QueryRequest(BaseModel):
    query: str
    selected_text: str = ""
    session_id: Optional[str] = None  # Optional session ID for stateful chat (Phase 10)

class ChatResponse(BaseModel):
    response: str
    session_id: Optional[str] = None  # Return session ID if created/used

class TranslateRequest(BaseModel):
    text: str

class ContentRequest(BaseModel):
    text: str
    target_level: str

# --- PHASE 10: Chat History Data Models ---
class CreateSessionRequest(BaseModel):
    title: Optional[str] = None

class SessionResponse(BaseModel):
    id: str
    user_id: str
    title: str
    created_at: str
    updated_at: str
    is_active: bool
    message_count: int

class MessageResponse(BaseModel):
    id: str
    session_id: str
    role: str
    content: str
    selected_text: Optional[str]
    created_at: str
    sources: Optional[list]

class SessionWithMessagesResponse(BaseModel):
    session: SessionResponse
    messages: list[MessageResponse]

class UpdateTitleRequest(BaseModel):
    title: str

# --- AUTH DEPENDENCY ---
async def get_current_user(authorization: Optional[str] = Header(None)):
    """
    Extract and verify JWT token from Authorization header.
    Returns user_id if valid, raises 401 if invalid/missing.
    """
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing or invalid authorization header")

    token = authorization.replace("Bearer ", "")

    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[ALGORITHM])
        user_id = payload.get("sub")
        if not user_id:
            raise HTTPException(status_code=401, detail="Invalid token payload")
        return user_id
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token has expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")

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

# --- CORS Configuration (FIXED) ---
# We define specific origins. Note NO trailing slashes for Vercel.
env_origins = os.environ.get("CORS_ALLOWED_ORIGINS", "").split(",")
default_origins = [
    "http://localhost:3000",
    "http://localhost:3001",
    "https://physical-ai-and-humanoid-robotics-omega.vercel.app",
    "https://physical-ai-and-humanoid-robotics-production.up.railway.app"
]
# Combine and filter empty strings
origins = [o.strip() for o in env_origins if o.strip()] + default_origins

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,    # Allows the listed origins
    allow_credentials=True,   # <--- CRITICAL: Must be True for Auth to work
    allow_methods=["*"],      # Allows all methods (GET, POST, etc.)
    allow_headers=["*"],      # Allows all headers (Authorization, etc.)
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
async def chat_endpoint(
    request: QueryRequest,
    authorization: Optional[str] = Header(None),
    db: Session = Depends(get_db)
):
    """
    T164: Chat endpoint with optional session support.
    - Anonymous users: Stateless chat (no history)
    - Authenticated users: Store messages in session
    """
    correlation_id = generate_correlation_id()
    start_time = time.time()

    # Optional: Extract user_id from token if provided
    user_id = None
    if authorization and authorization.startswith("Bearer "):
        try:
            token = authorization.replace("Bearer ", "")
            payload = jwt.decode(token, JWT_SECRET, algorithms=[ALGORITHM])
            user_id = payload.get("sub")
        except (jwt.ExpiredSignatureError, jwt.InvalidTokenError):
            pass  # Continue as anonymous user if token invalid

    logger.info(
        f"Chat request received - query length: {len(request.query)}, has_selection: {bool(request.selected_text)}, authenticated: {bool(user_id)}",
        extra={"correlation_id": correlation_id}
    )

    # Check circuit breakers before proceeding (FR-016)
    if not openai_circuit.can_execute():
        logger.warning(
            f"OpenAI circuit breaker OPEN - rejecting request",
            extra={"correlation_id": correlation_id}
        )
        raise HTTPException(
            status_code=503,
            detail="The AI service is temporarily unavailable. Please try again in a few moments."
        )

    if not qdrant_circuit.can_execute():
        logger.warning(
            f"Qdrant circuit breaker OPEN - rejecting request",
            extra={"correlation_id": correlation_id}
        )
        raise HTTPException(
            status_code=503,
            detail="The search service is temporarily unavailable. Please try again in a few moments."
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
                openai_circuit.record_success()
                break
            except Exception as e:
                openai_circuit.record_failure()
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
                qdrant_circuit.record_success()
                break
            except Exception as e:
                qdrant_circuit.record_failure()
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

        # T165-T167: Include conversation history for context-aware responses
        conversation_history = []
        if user_id and request.session_id:
            try:
                chat_service = ChatService(db)
                session_with_messages = chat_service.get_session_messages(request.session_id, user_id)
                # Get last N messages for context (limit to 10 to avoid token overflow)
                recent_messages = session_with_messages.messages[-10:] if len(session_with_messages.messages) > 10 else session_with_messages.messages
                for msg in recent_messages:
                    conversation_history.append({
                        "role": msg.role.value if isinstance(msg.role, MessageRole) else msg.role,
                        "content": msg.content
                    })
                logger.info(
                    f"Loaded {len(conversation_history)} previous messages for context",
                    extra={"correlation_id": correlation_id}
                )
            except Exception as e:
                logger.warning(
                    f"Could not load conversation history: {e}",
                    extra={"correlation_id": correlation_id}
                )

        system_prompt = (
            "You are an expert on the Physical AI & Humanoid Robotics textbook. "
            "Answer ONLY using the provided context. If the user provided 'selected_text', "
            "prioritize that specific snippet. Use the conversation history to provide coherent, contextual responses."
        )

        user_message = (
            f"Context from textbook:\n{context}\n\n"
            f"Question: {request.query}\n\n"
            f"Selected Text (if any): {request.selected_text if request.selected_text else 'None'}"
        )

        # Chat completion with circuit breaker and conversation history
        try:
            # Build messages array with conversation history
            messages = [{"role": "system", "content": system_prompt}]

            # Add conversation history if available (T165-T167)
            if conversation_history:
                messages.extend(conversation_history)

            # Add current user message
            messages.append({"role": "user", "content": user_message})

            response = openai_client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                max_tokens=500,
                temperature=0.7
            )
            openai_circuit.record_success()
        except Exception as e:
            openai_circuit.record_failure()
            raise

        answer = response.choices[0].message.content

        # T164: Save messages to session if user is authenticated
        session_id = request.session_id
        if user_id:
            try:
                chat_service = ChatService(db)

                # Get or create session
                if not session_id:
                    # Create new session or get active one
                    active_session = chat_service.get_active_session(user_id)
                    if active_session:
                        session_id = active_session.id
                    else:
                        # Auto-generate title from first question
                        title = request.query[:50] + "..." if len(request.query) > 50 else request.query
                        new_session = chat_service.create_session(user_id, title)
                        session_id = new_session.id

                # Save user message
                chat_service.add_message(
                    session_id=session_id,
                    role=MessageRole.USER,
                    content=request.query,
                    selected_text=request.selected_text if request.selected_text else None
                )

                # Save assistant response
                chat_service.add_message(
                    session_id=session_id,
                    role=MessageRole.ASSISTANT,
                    content=answer
                )

                logger.info(
                    f"Messages saved to session {session_id}",
                    extra={"correlation_id": correlation_id}
                )

            except Exception as e:
                # Don't fail the entire request if session save fails
                logger.error(
                    f"Failed to save messages to session: {e}",
                    extra={"correlation_id": correlation_id}
                )

        # Log successful completion with timing
        elapsed_time = time.time() - start_time
        logger.info(
            f"Chat request completed in {elapsed_time:.2f}s - response length: {len(answer)}",
            extra={"correlation_id": correlation_id}
        )

        return ChatResponse(response=answer, session_id=session_id)

    except HTTPException:
        raise
    except Exception as e:
        elapsed_time = time.time() - start_time
        logger.error(
            f"Error in chat endpoint after {elapsed_time:.2f}s: {e}",
            extra={"correlation_id": correlation_id}
        )
        # Graceful degradation (FR-016) - provide user-friendly error
        raise HTTPException(
            status_code=500,
            detail="I'm having trouble answering right now. Please try again in a moment."
        )


@app.post("/translate")
async def translate_text(request: TranslateRequest):
    """Translates technical text into Urdu using AI with circuit breaker (FR-016)"""
    # Check circuit breaker
    if not openai_circuit.can_execute():
        raise HTTPException(
            status_code=503,
            detail="Translation service temporarily unavailable. Please try again shortly."
        )

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
        openai_circuit.record_success()

        translated_text = response.choices[0].message.content
        return {"translation": translated_text}

    except HTTPException:
        raise
    except Exception as e:
        openai_circuit.record_failure()
        logger.error(f"Translation error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/adjust-content")
async def adjust_content_level(request: ContentRequest):
    """Rewrites text to match the user's proficiency level with circuit breaker (FR-016)"""
    # Check circuit breaker
    if not openai_circuit.can_execute():
        raise HTTPException(
            status_code=503,
            detail="Content adjustment service temporarily unavailable. Please try again shortly."
        )

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
        openai_circuit.record_success()

        return {"content": response.choices[0].message.content}

    except HTTPException:
        raise
    except Exception as e:
        openai_circuit.record_failure()
        logger.error(f"Adjustment error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# --- BATCH ENDPOINTS FOR FRONTEND OPTIMIZATION ---
class BatchTranslateRequest(BaseModel):
    texts: list[str]


class BatchContentRequest(BaseModel):
    texts: list[str]
    target_level: str


@app.post("/translate/batch")
async def batch_translate_text(request: BatchTranslateRequest):
    """Batch translate multiple texts in a single request (FR-016 optimization)"""
    if not openai_circuit.can_execute():
        raise HTTPException(
            status_code=503,
            detail="Translation service temporarily unavailable."
        )

    try:
        openai_client = app.state.openai_client
        translations = []

        # Process texts in batches to avoid token limits
        for text in request.texts[:20]:  # Limit to 20 texts per batch
            if not text or len(text.strip()) < 2:
                translations.append(text)
                continue

            system_prompt = (
                "You are a professional translator for a Robotics & AI textbook. "
                "Translate the following text into Urdu. "
                "Keep technical terms like 'ROS', 'Python', 'Algorithm' in English."
            )

            response = openai_client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": text}
                ],
                max_tokens=2000,
                temperature=0.3
            )
            translations.append(response.choices[0].message.content)

        openai_circuit.record_success()
        return {"translations": translations}

    except HTTPException:
        raise
    except Exception as e:
        openai_circuit.record_failure()
        logger.error(f"Batch translation error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/adjust-content/batch")
async def batch_adjust_content(request: BatchContentRequest):
    """Batch adjust content level for multiple texts (FR-016 optimization)"""
    if not openai_circuit.can_execute():
        raise HTTPException(
            status_code=503,
            detail="Content adjustment service temporarily unavailable."
        )

    try:
        openai_client = app.state.openai_client
        adjusted = []

        if request.target_level == "beginner":
            instruction = "Rewrite for a high school student. Use simple analogies."
        elif request.target_level == "intermediate":
            instruction = "Rewrite for an undergraduate. Simplify complex theory."
        else:
            return {"contents": request.texts}

        for text in request.texts[:20]:  # Limit to 20 texts per batch
            if not text or len(text.strip()) < 30:
                adjusted.append(text)
                continue

            response = openai_client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {"role": "system", "content": f"You are a Physics & AI tutor. {instruction}"},
                    {"role": "user", "content": text}
                ],
                max_tokens=2000
            )
            adjusted.append(response.choices[0].message.content)

        openai_circuit.record_success()
        return {"contents": adjusted}

    except HTTPException:
        raise
    except Exception as e:
        openai_circuit.record_failure()
        logger.error(f"Batch adjustment error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# --- PHASE 10: CHAT HISTORY API ENDPOINTS (T158-T164) ---

@app.get("/api/chat/sessions")
async def list_sessions(
    user_id: str = Depends(get_current_user),
    db: Session = Depends(get_db),
    limit: int = 20,
    offset: int = 0
):
    """
    T158: List user's chat sessions (requires auth).
    Returns paginated list of sessions ordered by most recent.
    """
    try:
        chat_service = ChatService(db)
        sessions = chat_service.get_user_sessions(user_id, limit, offset)
        return {"sessions": [s.to_dict() for s in sessions]}
    except Exception as e:
        logger.error(f"Error listing sessions: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/chat/sessions")
async def create_session(
    request: CreateSessionRequest,
    user_id: str = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    T159: Create new chat session (requires auth).
    """
    try:
        chat_service = ChatService(db)
        session = chat_service.create_session(user_id, request.title)
        return session.to_dict()
    except Exception as e:
        logger.error(f"Error creating session: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/chat/sessions/{session_id}")
async def get_session(
    session_id: str,
    user_id: str = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    T160: Get session with messages (requires auth, ownership check).
    """
    try:
        chat_service = ChatService(db)
        session_with_messages = chat_service.get_session_messages(session_id, user_id)
        return session_with_messages.to_dict()
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except Exception as e:
        logger.error(f"Error getting session: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.delete("/api/chat/sessions/{session_id}")
async def delete_session(
    session_id: str,
    user_id: str = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    T161: Delete a session (requires auth, ownership check).
    """
    try:
        chat_service = ChatService(db)
        deleted = chat_service.delete_session(session_id, user_id)
        if not deleted:
            raise HTTPException(status_code=404, detail="Session not found")
        return {"message": "Session deleted successfully"}
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting session: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/chat/sessions/{session_id}/export")
async def export_session(
    session_id: str,
    user_id: str = Depends(get_current_user),
    db: Session = Depends(get_db),
    format: str = "text"
):
    """
    T162: Export session as text or JSON (requires auth, ownership check).
    """
    try:
        chat_service = ChatService(db)
        exported = chat_service.export_session(session_id, user_id, format)

        if format == "json":
            return {"format": "json", "content": json.loads(exported)}
        else:
            return {"format": "text", "content": exported}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except Exception as e:
        logger.error(f"Error exporting session: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.delete("/api/chat/history")
async def clear_history(
    user_id: str = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    T163: Clear all user's chat history (requires auth).
    """
    try:
        chat_service = ChatService(db)
        count = chat_service.clear_all_history(user_id)
        return {"message": f"Cleared {count} sessions successfully"}
    except Exception as e:
        logger.error(f"Error clearing history: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.patch("/api/chat/sessions/{session_id}/title")
async def update_session_title(
    session_id: str,
    request: UpdateTitleRequest,
    user_id: str = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Update session title (bonus utility endpoint).
    """
    try:
        chat_service = ChatService(db)
        updated = chat_service.update_session_title(session_id, user_id, request.title)
        if not updated:
            raise HTTPException(status_code=404, detail="Session not found")
        return {"message": "Title updated successfully"}
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating title: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health/circuits")
async def circuit_status():
    """Get circuit breaker status for monitoring (FR-015)"""
    return {
        "openai": {
            "state": openai_circuit.state.value,
            "failure_count": openai_circuit.failure_count,
        },
        "qdrant": {
            "state": qdrant_circuit.state.value,
            "failure_count": qdrant_circuit.failure_count,
        }
    }


# --- RAILWAY & LOCAL EXECUTION ---
if __name__ == "__main__":
    # Get port from environment variable (Railway sets this automatically)
    # Default to 8080 if not set
    port = int(os.environ.get("PORT", 8080))
    
    # Run the server
    uvicorn.run("main:app", host="0.0.0.0", port=port, reload=True)