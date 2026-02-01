from flask import Flask, request
import os
import time
from werkzeug.utils import secure_filename
from markupsafe import escape

app = Flask(__name__)

# Configuration
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# Mandatory Authentication
API_KEY = os.environ.get('API_KEY')
if not API_KEY:
    raise ValueError("Using this server requires an API_KEY environment variable to be set for security.")

@app.route('/')
def index():
    images = reversed(sorted(os.listdir(UPLOAD_FOLDER)))
    html = "<h1>Captured Images</h1>"
    html += "<div style='display: flex; flex-wrap: wrap; gap: 10px;'>"
    for img in images:
        if img.endswith('.jpg'):
            # Security: Escape filename to prevent XSS
            safe_img = escape(img)
            html += f"<div style='border: 1px solid #ccc; padding: 5px;'><a href='/uploads/{safe_img}'><img src='/uploads/{safe_img}' width='200'><br>{safe_img}</a></div>"
    html += "</div>"
    return html

@app.route('/uploads/<path:filename>')
def uploaded_file(filename):
    from flask import send_from_directory
    return send_from_directory(UPLOAD_FOLDER, filename)

@app.route('/api/upload/<device_id>', methods=['POST'])
def upload(device_id):
    # Sanitize device_id to prevent directory traversal
    device_id = secure_filename(device_id)
    
    # Create device specific folder
    device_folder = os.path.join(UPLOAD_FOLDER, device_id)
    os.makedirs(device_folder, exist_ok=True)
    
    # 1. Mandatory Authentication Check
    # Check X-Api-Key header
    client_key = request.headers.get('X-Api-Key')
    # Check Authorization header (Bearer token)
    auth_header = request.headers.get('Authorization')
    
    authorized = (client_key and client_key == API_KEY) or \
                 (auth_header and auth_header == f"Bearer {API_KEY}")
        
    if not authorized:
        return "Unauthorized", 401

    try:
        # Extract headers provided by ESPHome Data Collector
        # Security: sanitize headers used in file paths
        raw_value = request.headers.get('X-Meter-Value', 'unknown')
        value = secure_filename(raw_value)
        
        conf = request.headers.get('X-Meter-Confidence', '0.0')
        
        # Create a descriptive filename
        # Format: collect_{timestamp}_{value}_{confidence}.jpg
        timestamp = int(time.time())
        try:
            conf_val = float(conf)
            filename = f"collect_{timestamp}_{value}_{conf_val:.2f}.jpg"
        except ValueError:
            # Fallback for non-float confidence, ensuring safety
            safe_conf = secure_filename(conf)
            filename = f"collect_{timestamp}_{value}_{safe_conf}.jpg"
            
        filepath = os.path.join(device_folder, filename)
        
        # Save the raw binary data from the request body
        with open(filepath, "wb") as f:
            f.write(request.data)
            
        print(f"Saved {filepath} (Value: {value}, Conf: {conf})")
        return "OK", 200
        
    except Exception as e:
        # Security: Log error internally, do not expose details to client
        print(f"Error saving upload: {e}")
        return "Internal Server Error", 500

if __name__ == '__main__':
    # Listen on all interfaces, port 5123
    app.run(host='0.0.0.0', port=5123)
