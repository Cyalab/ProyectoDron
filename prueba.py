#La funcion de este codigo es mandar <STATUS_TEXT> a Mission Planner por puertos UDP 
#Esto en parte es una simulación en base a lo que observa la cámara y lo analiza establecemos una acción que mande  Mission Planner
#en concreto para que en este caso, si se detecta una persona el código le avisa a Mission Planner que hay una persona detectada <STATUS_TEXT>


import torch
import time
import cv2
import warnings
from pymavlink import mavutil

# Quitar warnings específicos
warnings.filterwarnings("ignore", message=".*torch.cuda.amp.autocast.*")

# Conexión MAVLink por UDP (Mission Planner debe estar escuchando en 14550)
connection_string = 'udpin:127.0.0.1:14551'
print(f"Conectando a MAVLink en {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

# Esperar latido de vida
master.wait_heartbeat()
print(f"✅ Conectado a sistema {master.target_system} componente {master.target_component}")

# Función para enviar mensaje a Mission Planner
def send_statustext(text, severity=6):
    master.mav.statustext_send(severity, text.encode())

# Cargar modelo YOLOv5n (rápido y ligero, sin GPU)
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

# Abrir cámara
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("❌ No se pudo abrir la cámara")
    exit()

start_time = time.time()
last_print_time = 0
alert_sent = False
returning_to_base_sent = False
processed_frames = 0

print("🎥 INICIANDO SIMULACIÓN")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_resized = cv2.resize(frame, (640, 480))
    results = model(frame_resized)
    labels = results.pandas().xyxy[0]['name'].tolist()
    annotated_frame = results.render()[0]
    processed_frames += 1

    current_time = time.time()

    if current_time - last_print_time >= 5:
        fps = processed_frames / (current_time - start_time)
        print(f"📈 FPS aproximados: {fps:.2f}")

        if 'person' in labels:
            if not alert_sent:
                aviso = "[SIMULACIÓN] Persona detectada, enviando aviso a ArduPilot"
                print(aviso)
                send_statustext(aviso)
                alert_sent = True
                returning_to_base_sent = False
            else:
                print("[SIMULACIÓN] Persona sigue detectada, sin enviar nuevo aviso")
        else:
            print("[SIMULACIÓN] No se detectaron personas")

        if alert_sent and not returning_to_base_sent and current_time - last_print_time >= 6:
            regreso = "[SIMULACIÓN] Regresando a base..."
            print(regreso)
            send_statustext(regreso)
            returning_to_base_sent = True

        last_print_time = current_time
        processed_frames = 0
        start_time = current_time

    cv2.imshow('Detección YOLOv5', annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
