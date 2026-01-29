FROM python:3.9-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app.py .

# Create uploads directory
RUN mkdir -p uploads

# Expose the configured port
EXPOSE 5123

# Run the application
CMD ["python", "app.py"]
