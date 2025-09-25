import cv2
import time
from pymavlink import mavutil
import piexif
from PIL import Image
from ultralytics import YOLO
import os
from datetime import datetime

# -----------------------------
# CONFIG
# -----------------------------
RTSP_URL = "rtsp://192.168.144.25:8554/main.264"   # Drone video stream
MAVLINK_CONN = "udp:127.0.0.1:14550"              # Telemetry from Mission Planner/QGC
SAVE_INTERVAL = 5  # seconds between captures
LOG_FILE = "detections_log.txt"  # Text file to store detection data

# YOLO class names for reference
CLASS_NAMES = {0: "person", 1: "person"}
TARGET_CLASSES = [0, 1]  # Classes to detect (person and bicycle)

# -----------------------------
# CONNECT TO MAVLINK
# -----------------------------
mav = mavutil.mavlink_connection(MAVLINK_CONN)

def get_gps():
    """Fetch latest GPS fix from MAVLink"""
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0  # Convert from mm to meters
        return lat, lon, alt
    return None, None, None

def deg_to_dms_rational(deg):
    d = int(deg)
    m = int((deg - d) * 60)
    s = round((deg - d - m / 60) * 3600 * 100)
    return ((d, 1), (m, 1), (s, 100))

def embed_gps(image_path, lat, lon, out_path):
    """Embed GPS coordinates into image EXIF data"""
    exif_dict = {"GPS": {
        piexif.GPSIFD.GPSLatitude: deg_to_dms_rational(abs(lat)),
        piexif.GPSIFD.GPSLatitudeRef: 'N' if lat >= 0 else 'S',
        piexif.GPSIFD.GPSLongitude: deg_to_dms_rational(abs(lon)),
        piexif.GPSIFD.GPSLongitudeRef: 'E' if lon >= 0 else 'W',
    }}
    exif_bytes = piexif.dump(exif_dict)
    im = Image.open(image_path)
    im.save(out_path, exif=exif_bytes)

def log_detection(timestamp, class_id, class_name, confidence, lat, lon, alt, filename):
    """Log detection data to text file"""
    with open(LOG_FILE, 'a', encoding='utf-8') as f:
        f.write(f"{timestamp},{class_id},{class_name},{confidence:.3f},{lat:.7f},{lon:.7f},{alt:.2f},{filename}\n")

def initialize_log_file():
    """Create log file with headers if it doesn't exist"""
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, 'w', encoding='utf-8') as f:
            f.write("Timestamp,ClassID,ClassName,Confidence,Latitude,Longitude,Altitude,Filename\n")
        print(f"📄 Created log file: {LOG_FILE}")

# -----------------------------
# YOLO CONFIG (Ultralytics YOLOv8n)
# -----------------------------
model = YOLO("yolov8n.pt")  # Pre-trained model

def detect_targets(frame):
    """Detect target classes (person and bicycle) in the frame using YOLOv8n"""
    results = model(frame)
    
    detections = []
    if results[0].boxes is not None:
        class_ids = results[0].boxes.cls.tolist()
        confidences = results[0].boxes.conf.tolist()
        boxes = results[0].boxes.xyxy.tolist()  # Get bounding boxes in xyxy format
        
        for i, class_id in enumerate(class_ids):
            if int(class_id) in TARGET_CLASSES:
                detections.append({
                    'class_id': int(class_id),
                    'confidence': confidences[i],
                    'box': boxes[i]
                })
    
    return detections

def draw_detections(frame, detections):
    """Draw bounding boxes and labels on the frame"""
    for detection in detections:
        class_id = detection['class_id']
        confidence = detection['confidence']
        box = detection['box']
        
        # Get bounding box coordinates
        x1, y1, x2, y2 = map(int, box)
        
        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw label
        label = f"{CLASS_NAMES[class_id]}: {confidence:.2f}"
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return frame

# -----------------------------
# VIDEO STREAM HANDLER
# -----------------------------
def open_stream():
    return cv2.VideoCapture(RTSP_URL)

# Initialize log file
initialize_log_file()

cap = open_stream()
last_save = time.time()

print("🚁 Drone detection system started...")
print(f"📊 Detecting: {', '.join([CLASS_NAMES[i] for i in TARGET_CLASSES])}")
print(f"📄 Logging to: {LOG_FILE}")
print("Press 'q' to quit")

# -----------------------------
# MAIN LOOP
# -----------------------------
while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("⚠️ Stream lost... reconnecting")
        cap.release()
        time.sleep(2)
        cap = open_stream()
        continue

    # Run YOLO detection
    try:
        detections = detect_targets(frame)
    except Exception as e:
        print(f"YOLO error: {e}")
        continue

    # Draw detections on frame for visualization
    if detections:
        frame = draw_detections(frame, detections)

    # Process detections
    if detections:
        lat, lon, alt = get_gps()
        if lat and lon:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            for detection in detections:
                class_id = detection['class_id']
                confidence = detection['confidence']
                class_name = CLASS_NAMES[class_id]
                
                # Create filename with timestamp and detection info
                filename = f"{class_name}_detected_{int(time.time())}_{class_id}.jpg"
                
                # Save the frame
                cv2.imwrite(filename, frame)
                
                # Embed GPS data
                geo_filename = f"geo_{filename}"
                embed_gps(filename, lat, lon, geo_filename)
                
                # Log detection to text file
                log_detection(timestamp, class_id, class_name, confidence, lat, lon, alt, geo_filename)
                
                print(f"✅ {class_name.title()} detected! Confidence: {confidence:.3f}")
                print(f"📍 GPS: {lat:.7f}, {lon:.7f}, Alt: {alt:.2f}m")
                print(f"💾 Saved: {geo_filename}")
                print(f"📝 Logged to: {LOG_FILE}")
        else:
            print("⚠️ No GPS data available yet")

    # Display frame
    cv2.imshow("Drone Feed - Detection System", frame)

    # Periodic save (optional - you can remove this if not needed)
    if time.time() - last_save > SAVE_INTERVAL:
        last_save = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("🛑 Detection system stopped")
