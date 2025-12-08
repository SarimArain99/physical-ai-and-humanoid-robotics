# 1. Use an official Python image
FROM python:3.11-slim

# 2. Set the working directory inside the container
WORKDIR /app

# 3. Copy requirements and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 4. Copy the rest of the app code
COPY . .

# 🟢 THE FIX: Remove the brackets [] to force Shell Mode
# This allows $PORT to be read correctly
CMD uvicorn main:app --host 0.0.0.0 --port $PORT