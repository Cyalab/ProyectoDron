import torch
import time
import cv2
import warnings

# Quitar warnings 
warnings.filterwarnings("ignore", message=".*torch.cuda.amp.autocast.*")

# Cargar modelo YOLOv5x
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

start_time = time.time()
last_print_time = 0  # Para controlar impresión cada 5 segundos
alert_sent = False  # Para saber si ya detectó persona alguna vez

print("INICIANDO CAMARA (Q PARA SALIR)")

processed_frames = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Redimensionar para acelerar la inferencia 
    frame_resized = cv2.resize(frame, (640, 480))

    # Detectar objetos
    results = model(frame_resized)
    labels = results.pandas().xyxy[0]['name'].tolist()
    annotated_frame = results.render()[0]
    processed_frames += 1

    current_time = time.time()

    # Cada 5 segundos mostrar FPS y logs
    if current_time - last_print_time >= 5:
        fps = processed_frames / (current_time - start_time)
        print(f"FPS aproximados: {fps:.2f}")

        if 'person' in labels:
            print("[SIMULACIÓN] Persona detectada, enviando aviso a ArduPilot...")
            alert_sent = True
        else:
            print("[SIMULACIÓN] No se detectaron personas, sin acción requerida.")

        if alert_sent:
            print("[SIMULACIÓN] Realizando análisis profundo...")

        last_print_time = current_time
        processed_frames = 0
        start_time = current_time

    # Mostrar resultados en ventana
    cv2.imshow('PRUEBALOCALYOLO5x', annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
