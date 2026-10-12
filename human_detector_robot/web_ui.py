from flask import Flask, render_template, Response, jsonify, request
import threading
import cv2
import time

app = Flask(__name__)

# Naya Multi-Robot State Dictionary
system_state = {
    "robots": {
        "1": {
            "yaw": 0.0, "v": 0.0, "omega": 0.0, 
            "dist_front_cm": "--", "dist_right_cm": "--", 
            "pwm_l": 0, "pwm_r": 0, "status": "BOOTING"
        },
        "2": {
            "yaw": 0.0, "v": 0.0, "omega": 0.0, 
            "dist_front_cm": "--", "dist_right_cm": "--", 
            "pwm_l": 0, "pwm_r": 0, "status": "OFFLINE"
        }
    }
}

latest_frame_1 = None
latest_frame_2 = None

# Manual command hata di gayi hai frontend se, par backend variable rakha hai taake 
# main_controller.py crash na ho agar wo isay call kare.
manual_command = {"v": 0.0, "omega": 0.0, "active": False}

@app.route('/')
def index():
    # Make sure tumhari naye HTML file ka naam 'index.html' ya 'dashboard.html' hai 
    # templates folder ke andar. Yahan wahi naam likhna.
    return render_template('index.html') 

@app.route('/api/state')
def get_state():
    # Frontend ko dono robots ka data yahan se milega
    return jsonify(system_state)

def generate_frames(robot_id):
    global latest_frame_1, latest_frame_2
    while True:
        frame_to_show = latest_frame_1 if robot_id == '1' else latest_frame_2
        
        if frame_to_show is not None:
            ret, buffer = cv2.imencode('.jpg', frame_to_show)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # Agar frame nahi hai toh thoda wait kare taake CPU hang na ho
            time.sleep(0.1)
        time.sleep(0.05)

@app.route('/video_feed/<robot_id>')
def video_feed(robot_id):
    # HTML dashboard /video_feed/1 call karega toh Robot 1 ka camera chalega
    return Response(generate_frames(robot_id), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_server():
    # Run Flask in a background daemon thread
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True).start()

def update_state(new_state, frame):
    global system_state, latest_frame_1
    
    # main_controller.py se jo bhi data aayega, hum usay Robot 1 ke andar map kar denge
    r1 = system_state["robots"]["1"]
    
    if 'yaw' in new_state: r1['yaw'] = new_state['yaw']
    if 'v' in new_state: r1['v'] = new_state['v']
    if 'dist_front' in new_state: r1['dist_front_cm'] = new_state['dist_front']
    if 'dist_right' in new_state: r1['dist_right_cm'] = new_state['dist_right']
    if 'pwm_l' in new_state: r1['pwm_l'] = new_state['pwm_l']
    if 'pwm_r' in new_state: r1['pwm_r'] = new_state['pwm_r']
    
    latest_frame_1 = frame

def get_manual_override():
    return manual_command