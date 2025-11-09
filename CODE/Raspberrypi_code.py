import cv2
import numpy as np
from threading import Thread
import time
import socket  # <-- Added for communication

# =========================================================
# CAMERA CONFIGURATION
# =========================================================
username = "100548367"
password = "e6mrEvJA"
ip = "192.168.132.102"
stream_url = f"rtsp://{username}:{password}@{ip}/live"

# =========================================================
# ESP32 MASTER CONFIGURATION
# =========================================================
ESP32_IP = "192.168.132.150"   # <-- Change to your Master ESP32 IP
ESP32_PORT = 5000              # <-- Match port in ESP32 code

def send_to_esp(command: str):
    """Send a single command to ESP32 over TCP."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((ESP32_IP, ESP32_PORT))
        s.sendall(command.encode())
        s.close()
        print(f" Sent to ESP32: {command}")
    except Exception as e:
        print(f"ESP32 connection error: {e}")

# =========================================================
# CAMERA THREAD CLASS
# =========================================================
class CameraStream:
    def _init_(self, url):
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise ValueError("Cannot open RTSP stream.")
        self.grabbed, self.frame = self.cap.read()
        self.stopped = False
        Thread(target=self.update, daemon=True).start()

    def update(self):
        while not self.stopped:
            grabbed, frame = self.cap.read()
            if grabbed:
                self.grabbed, self.frame = grabbed, frame
            else:
                time.sleep(0.02)

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.cap.release()

# =========================================================
# GREEN TIME CALCULATION
# =========================================================
def calculate_green_time(total_density, base_time=10, factor=0.5):
    green_time = base_time + factor * total_density
    return max(base_time, min(green_time, 120))

# =========================================================
# START CAMERA THREAD
# =========================================================
try:
    cam = CameraStream(stream_url)
    print("Camera stream started.")
except Exception as e:
    print("Error:", e)
    exit()

# =========================================================
# CAPTURE REFERENCE FRAME (Press SPACE)
# =========================================================
print("Press SPACE to capture a frame for ROI selection.")
while True:
    frame = cam.read()
    if frame is None:
        continue
    cv2.imshow("Press SPACE", frame)
    if cv2.waitKey(1) & 0xFF == 32:  # SPACE
        ref_frame = frame.copy()
        break
cv2.destroyAllWindows()

# =========================================================
# DEFINE ROI AREA (Click and Enter)
# =========================================================
roi_points = []
def click_roi(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        roi_points.append((x, y))
cv2.namedWindow("Select ROI")
cv2.setMouseCallback("Select ROI", click_roi)

while True:
    temp = ref_frame.copy()
    for p in roi_points:
        cv2.circle(temp, p, 5, (0, 0, 255), -1)
    if len(roi_points) >= 3:
        cv2.polylines(temp, [np.array(roi_points, np.int32)], True, (0, 255, 0), 2)
    cv2.imshow("Select ROI", temp)
    if cv2.waitKey(20) & 0xFF == 13:  # ENTER
        break
cv2.destroyAllWindows()

roi_mask = np.zeros(ref_frame.shape[:2], dtype=np.uint8)
cv2.fillPoly(roi_mask, [np.array(roi_points, np.int32)], 255)
background_gray = cv2.cvtColor(ref_frame, cv2.COLOR_BGR2GRAY)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

# =========================================================
# LIVE LOOP WITH COMMAND CONTROL
# =========================================================
print("🚦 Starting density detection and light control.")
GREEN_FIXED = 10   # base green
YELLOW_FIXED = 3
RED_FIXED = 5
cycle_state = "RED"

while True:
    frame = cam.read()
    if frame is None:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray, background_gray)
    _, mask = cv2.threshold(diff, 40, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    covered = cv2.countNonZero(cv2.bitwise_and(mask, roi_mask))
    total = cv2.countNonZero(roi_mask)
    density = (covered / total * 100) if total > 0 else 0
    green_time = calculate_green_time(density, base_time=GREEN_FIXED)

    # Display Info
    disp = frame.copy()
    cv2.putText(disp, f"Density: {density:.1f}%", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
    cv2.putText(disp, f"Green: {green_time:.1f}s", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    cv2.imshow("Traffic Density", disp)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # =====================================================
    # TRAFFIC LIGHT CYCLE LOGIC
    # =====================================================
    if cycle_state == "RED":
        send_to_esp("GREEN_ON")
        cycle_state = "GREEN"
        print("GREEN ON for", green_time, "s")
        time.sleep(green_time)

        send_to_esp("YELLOW_ON")
        cycle_state = "YELLOW"
        print("YELLOW ON for", YELLOW_FIXED, "s")
        time.sleep(YELLOW_FIXED)

        send_to_esp("RED_ON")
        cycle_state = "RED"
        print("RED ON for", RED_FIXED, "s")
        time.sleep(RED_FIXED)

cam.stop()
cv2.destroyAllWindows()