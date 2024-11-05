from collections import defaultdict

import socket
import paho.mqtt.client as mqtt

import asyncio
import websockets

import cv2
import numpy as np
from ultralytics import YOLO

import time
import keyboard

## MQTT configuration
mqtt_broker_ip = "your_broker_ip"

mqtt_username = "your_username"
mqtt_password = "your_password"

request_topic = "requestIP"
response_topic = "getIP"
mqtt_client_name = "laptop_mqtt_client"

## Params
CLASS_TRACKED = 0  # 0 is the class ID for human
CONF_THRESHOLD = 0.65  # Confidence threshold
MIN_SPEED = 0
MAX_SPEED = 3
KEY_COOLDOWN = 0.5  # Cooldown period in seconds

## State
movement_mode = 0 # 0 is RC, 1 is CV
movespeed = 0
turn_command = "S"
last_key_press_time = {key: 0 for key in ['e', 'w', 's', 'h', 'a', 'd']}  # Initialize last key press times

# Define the actions for each key
actions = {
    'e': lambda: toggle_movement_mode(),
    'w': lambda: increase_speed(),
    's': lambda: decrease_speed(),
    'h': lambda: reset_speed(),
    'a': lambda: move_left() if movement_mode == 0 else None,
    'd': lambda: move_right() if movement_mode == 0 else None
}

def toggle_movement_mode():
    global movement_mode
    movement_mode = 1 - movement_mode  # Toggle movement mode

def increase_speed():
    global movespeed
    movespeed = min(movespeed + 1, MAX_SPEED)

def decrease_speed():
    global movespeed
    movespeed = max(movespeed - 1, MIN_SPEED)

def reset_speed():
    global movespeed
    movespeed = MIN_SPEED

def move_left():
    global turn_command
    if turn_command == "S":
        turn_command = "L"
    elif turn_command == "R":
        turn_command = "S"
    else:
        turn_command = "L"

def move_right():
    global turn_command
    if turn_command == "S":
        turn_command = "R"
    elif turn_command == "L":
        turn_command = "S"
    else: 
        turn_command = "R"

model = YOLO("yolov8x-pose.pt") # n,s,m,l,x
model.to('cuda')

async def handle_client(websocket, path):
    global movement_mode, movespeed, turn_command
    client_ip = websocket.remote_address[0]
    print(f"Client connected: {client_ip}")

    track_history = defaultdict(lambda: [])

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                print("\nReceived image data from ESP32")

                try:
                    # Decode the JPEG image
                    img_array = cv2.imdecode(np.frombuffer(message, np.uint8), cv2.IMREAD_COLOR)

                    # Inference
                    results = model.track(img_array, persist=True, conf=CONF_THRESHOLD, classes=CLASS_TRACKED)

                    # Detection Bounding Box Annotation
                    annotated_frame = results[0].plot() # Visualize the results on the frame (Everything)

                    # Selected Tracked Object Track Line
                    track_ids = results[0].boxes.id
                    if track_ids is not None:
                        track_ids = track_ids.int().cpu()
                        kypts = results[0].keypoints.data.cpu()
                        eye_dist = np.array([])

                        # Plot Track Lines
                        for kypt, track_id in zip(kypts, track_ids):
                            track_id = track_id.item()
                            nose = kypt[0]
                            left_eye = kypt[1]
                            right_eye = kypt[2]
                            if left_eye[2] > CONF_THRESHOLD and right_eye[2] > CONF_THRESHOLD:
                                eye_dist = np.append(eye_dist, np.linalg.norm(left_eye[:2] - right_eye[:2]))
                            else:
                                eye_dist = np.append(eye_dist, -1)

                            nose_x, nose_y, nose_conf = nose[0], nose[1], nose[2]
                            if nose_conf > CONF_THRESHOLD:
                                track = track_history[track_id]
                                track.append((float(nose_x), float(nose_y)))  # x, y center point

                                if len(track) > 30:  # retain 30 tracks for 30 frames
                                    track.pop(0)

                                print(f"track_{track_id}, x: {nose_x}, y: {nose_y}, conf: {nose_conf}")

                                points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                                cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
                            else:
                                print(f"track_{track_id}, Low Nose Confidence")
                    else:
                        print("No tracked objects")

                    cv2.imshow("YOLOv8 Inference", annotated_frame)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                    # Largest Tracked Object
                    if movement_mode == 1: # CV MODE
                        if len(eye_dist) == 0:
                            turn_command = "S"
                        else:
                            max_eye_dist_idx = eye_dist.argmax(0)
                            x_largest, y_largest, conf_largest = kypts[max_eye_dist_idx][0]
                            print(f"x_largest: {x_largest}, y_largest: {y_largest}")
                            if x_largest < 0.40 * img_array.shape[1]:
                                turn_command = "L"
                            elif x_largest > 0.60 * img_array.shape[1]:
                                turn_command = "R"
                            else:
                                turn_command = "S"

                    # Send Movement command
                    print(f"movement_mode: {movement_mode}, movespeed: {movespeed}, turn_command: {turn_command}")
                    movement_command = str(movespeed) + turn_command
                    await websocket.send(movement_command)
                    print(f"Sent command to ESP32: {movement_command}")

                except Exception as e:
                    print(f"Error decoding image: {e}")
            
            else:
                # Other message types
                print(f"Received message from ESP32: {message}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection with {client_ip} closed: {e}")

async def check_keyboard():
    while True:
        current_time = time.time()
        for key, action in actions.items():
            if keyboard.is_pressed(key) and (current_time - last_key_press_time[key] > KEY_COOLDOWN):
                action()
                last_key_press_time[key] = current_time
        await asyncio.sleep(0.1)  # Delay between checks

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # Connect to Google's public DNS
        ip = s.getsockname()[0]  # Get the local IP address used for the connection
    except Exception as e:
        ip = "Unable to get IP"
    finally:
        s.close()
    return ip

# Callback for when a message is received
def on_message(client, userdata, msg):
    if msg.topic == request_topic:
        laptop_ip = get_local_ip()
        client.publish(response_topic, laptop_ip)
        print(f"Responded with IP: {laptop_ip}")

# Setup WebSocket server
laptop_ip = get_local_ip()
print(f"Laptop IP: {laptop_ip}")
start_server = websockets.serve(handle_client, "0.0.0.0", 8765)

# Setup MQTT client
client = mqtt.Client(mqtt_client_name)
client.username_pw_set(mqtt_username, mqtt_password)
client.on_message = on_message

client.connect(mqtt_broker_ip)
client.subscribe(request_topic)

# Start listening for messages
client.loop_start()

# Run the WebSocket server forever
loop = asyncio.get_event_loop()
loop.run_until_complete(start_server)
loop.create_task(check_keyboard())
loop.run_forever()