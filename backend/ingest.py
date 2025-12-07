import os
import re
import uuid
from typing import List, Dict, Any
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
from config import settings


class DocumentIngestor:
    def __init__(self):
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name

    def extract_headers(self, text: str) -> Dict[str, str]:
        """Extract headers from markdown text to maintain context"""
        headers = {"h1": "", "h2": "", "h3": "", "h4": "", "h5": "", "h6": ""}

        lines = text.split('\n')
        for line in lines:
            # Check for ATX-style headers (# Header)
            if line.startswith('#'):
                level = 0
                for char in line:
                    if char == '#':
                        level += 1
                    else:
                        break
                header_type = f"h{level}"
                if header_type in headers:
                    header_text = line[level:].strip().lstrip()
                    headers[header_type] = header_text

        # Create a context string from headers
        context_parts = []
        for i in range(1, 7):
            header_type = f"h{i}"
            if headers[header_type]:
                context_parts.append(headers[header_type])

        return {
            "headers": headers,
            "context": " > ".join(context_parts)
        }

    def split_markdown(self, text: str, max_chars: int = 1500) -> List[Dict[str, Any]]:
        """Split markdown text into chunks while preserving context"""
        # Extract headers for context
        header_info = self.extract_headers(text)

        # Split text into paragraphs
        paragraphs = text.split('\n\n')

        chunks = []
        current_chunk = ""
        current_length = 0

        for paragraph in paragraphs:
            # If paragraph is too large, split it into sentences
            if len(paragraph) > max_chars:
                sentences = re.split(r'[.!?]+\s+', paragraph)
                temp_chunk = ""

                for sentence in sentences:
                    sentence = sentence.strip()
                    if not sentence:
                        continue

                    if current_length + len(sentence) > max_chars and current_chunk:
                        # Save current chunk and start new one
                        chunks.append({
                            "content": current_chunk.strip(),
                            "context": header_info["context"],
                            "headers": header_info["headers"]
                        })
                        current_chunk = sentence
                        current_length = len(sentence)
                    elif len(temp_chunk) + len(sentence) > max_chars:
                        # Save temp chunk and add to current
                        if temp_chunk:
                            chunks.append({
                                "content": temp_chunk.strip(),
                                "context": header_info["context"],
                                "headers": header_info["headers"]
                            })
                            temp_chunk = sentence
                        else:
                            temp_chunk = sentence
                    else:
                        temp_chunk += " " + sentence if temp_chunk else sentence

                # Add remaining temp chunk to current chunk
                if temp_chunk:
                    if current_length + len(temp_chunk) > max_chars and current_chunk:
                        chunks.append({
                            "content": current_chunk.strip(),
                            "context": header_info["context"],
                            "headers": header_info["headers"]
                        })
                        current_chunk = temp_chunk
                        current_length = len(temp_chunk)
                    else:
                        current_chunk += " " + temp_chunk
                        current_length += len(temp_chunk)
            else:
                # Check if adding this paragraph exceeds character limit
                if current_length + len(paragraph) > max_chars and current_chunk:
                    # Save current chunk and start new one
                    chunks.append({
                        "content": current_chunk.strip(),
                        "context": header_info["context"],
                        "headers": header_info["headers"]
                    })
                    current_chunk = paragraph
                    current_length = len(paragraph)
                else:
                    current_chunk += "\n\n" + paragraph if current_chunk else paragraph
                    current_length += len(paragraph)

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                "content": current_chunk.strip(),
                "context": header_info["context"],
                "headers": header_info["headers"]
            })

        return chunks

    def get_embedding(self, text: str) -> List[float]:
        """Get embedding for text using OpenAI API"""
        response = self.client.embeddings.create(
            input=text,
            model="text-embedding-3-small"
        )
        return response.data[0].embedding

    def create_collection(self):
        """Create Qdrant collection if it doesn't exist"""
        try:
            self.qdrant_client.get_collection(self.collection_name)
            print(f"Collection {self.collection_name} already exists")
        except:
            print(f"Creating collection {self.collection_name}")
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
            )

    def ingest_documents(self):
        """Main method to ingest all markdown documents"""
        docs_path = Path(settings.docs_path)

        # Create collection
        self.create_collection()

        # Find all markdown files
        md_files = list(docs_path.rglob("*.md"))
        print(f"Found {len(md_files)} markdown files")

        points = []

        for md_file in md_files:
            print(f"Processing {md_file}")

            # Read the markdown file
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract file path information
            relative_path = md_file.relative_to(docs_path.parent)

            # Split the content into chunks
            chunks = self.split_markdown(content)

            for i, chunk in enumerate(chunks):
                # Combine content and context for embedding
                text_for_embedding = f"{chunk['context']}\n\n{chunk['content']}"

                # Get embedding
                embedding = self.get_embedding(text_for_embedding)

                # Create a unique ID for this chunk
                chunk_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{relative_path}-{i}"))

                # Create the point
                point = models.PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload={
                        "content": chunk["content"],
                        "context": chunk["context"],
                        "headers": chunk["headers"],
                        "file_path": str(relative_path),
                        "chunk_index": i
                    }
                )

                points.append(point)

        # Upload all points to Qdrant
        print(f"Uploading {len(points)} chunks to Qdrant...")
        self.qdrant_client.upload_points(
            collection_name=self.collection_name,
            points=points
        )

        print(f"Successfully ingested {len(points)} chunks from {len(md_files)} files")


if __name__ == "__main__":
    ingestor = DocumentIngestor()
    ingestor.ingest_documents()