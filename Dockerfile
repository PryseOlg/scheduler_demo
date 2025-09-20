# Use Python 3.9 slim image as base
FROM python:3.9-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application code
COPY src/ ./src/
COPY web/ ./web/
COPY scenarios/ ./scenarios/
COPY README.md .

# Create a non-root user for security
RUN useradd -m -u 1000 scheduler && \
    chown -R scheduler:scheduler /app
USER scheduler

# Expose port for web interface
EXPOSE 5000

# Set environment variables
ENV PYTHONPATH=/app/src
ENV FLASK_APP=web/server.py
ENV FLASK_ENV=production

# Default command - run the web server
CMD ["python", "-m", "flask", "run", "--host=0.0.0.0", "--port=5000"]
