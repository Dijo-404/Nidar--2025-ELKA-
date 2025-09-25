    import cv2
    import time
    from pymavlink import mavutil
    import piexif
    from PIL import Image
    from ultralytics import YOLO

    # -----------------------------
    # CONFIG
    # -----------------------------
    RTSP_URL = "rtsp://192.168.144.25:8554/main.264"   # Drone video stream
    MAVLINK_CONN = "udp:127.0.0.1:14550"              # Telemetry from Mission Planner/QGC
    SAVE_INTERVAL = 5  # seconds between captures

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
            return lat, lon
        return None, None

    def deg_to_dms_rational(deg):
        d = int(deg)
        m = int((deg - d) * 60)
        s = round((deg - d - m / 60) * 3600 * 100)
        return ((d, 1), (m, 1), (s, 100))

    def embed_gps(image_path, lat, lon, out_path):
        exif_dict = {"GPS": {
            piexif.GPSIFD.GPSLatitude: deg_to_dms_rational(abs(lat)),
            piexif.GPSIFD.GPSLatitudeRef: 'N' if lat >= 0 else 'S',
            piexif.GPSIFD.GPSLongitude: deg_to_dms_rational(abs(lon)),
            piexif.GPSIFD.GPSLongitudeRef: 'E' if lon >= 0 else 'W',
        }}
        exif_bytes = piexif.dump(exif_dict)
        im = Image.open(image_path)
        im.save(out_path, exif=exif_bytes)

    # -----------------------------
    # YOLO CONFIG (Ultralytics YOLOv8n)
    # -----------------------------
    model = YOLO("yolov8n.pt")  # Pre-trained model (replace with your custom model if needed)

    def detect_human(frame):
        """Detect humans in the frame using YOLOv8n"""
        results = model(frame)

        class_ids = results[0].boxes.cls.tolist() if results[0].boxes else []
        confidences = results[0].boxes.conf.tolist() if results[0].boxes else []
        boxes = results[0].boxes.xywh.tolist() if results[0].boxes else []

        return class_ids, confidences, boxes

    # -----------------------------
    # VIDEO STREAM HANDLER
    # -----------------------------
    def open_stream():
        return cv2.VideoCapture(RTSP_URL)  # keep original RTSP

    cap = open_stream()
    last_save = time.time()

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

        cv2.imshow("Drone Feed", frame)

        # Run YOLO human detection
        try:
            class_ids, confidences, boxes = detect_human(frame)
        except Exception as e:
            print("YOLO error:", e)
            continue

        if 0 in class_ids:  # Class ID 0 is for 'person'
            lat, lon = get_gps()
            if lat and lon:
                filename = f"human_detected_{int(time.time())}.jpg"
                cv2.imwrite(filename, frame)

                # Embed GPS
                embed_gps(filename, lat, lon, f"geo_{filename}")
                print(f"✅ Human detected! Saved {filename} with GPS {lat}, {lon}")
            else:
                print("⚠️ No GPS data available yet")

        # Periodic save (optional)
        if time.time() - last_save > SAVE_INTERVAL:
            last_save = time.time()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
