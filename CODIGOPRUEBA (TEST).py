# INSTRUCCIONES IMPORTANTES 

# 1 CREAR ENTORNO VIRTUAL:
#    Windows (CMD):     py -m venv yoloenv
#    Linux/Mac:         python3 -m venv yoloenv

# 2 ACTIVAR ENTORNO:
#    Windows (CMD):        yoloenv\Scripts\activate.bat
#    Windows (PowerShell): Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
#                          yoloenv\Scripts\Activate.ps1
#    Ubuntu/Mac:           source yoloenv/bin/activate

# 3 INSTALAR LIBRERÍAS:
#    pip install -r requirements.txt
#    O
#    py -m pip install -r requirements.txt
#    (Debe incluir: dronekit, torch, opencv-python, pymavlink)

# 4 TENER PYTHON 3.10 INSTALADO (y agregado al PATH)

# 5 AJUSTAR EL PUERTO Y BAUDRATE CORRECTOS PARA TU CONEXIÓN SERIAL (ej. /dev/ttyUSB0 en Linux, COMx en Windows)

# 6 INSTALAR DRIVERS DE 3DR SIK (solo si el sistema no reconoce las antenas)

# 7 SI LA CÁMARA NO FUNCIONA:
#    Modificar línea 118: cap = cv2.VideoCapture(0)
#    Prueba con otro valor: cap = cv2.VideoCapture(1) o usa v4l2-ctl para identificar la cámara

# 8 ESTE SCRIPT NO ABRE VENTANA DE VIDEO, ES DETECCIÓN AUTOMÁTICA

# 9 PARA ARMAR EL DRON:
#    - Encender el dron y el control remoto, esperar que la placa Pixhawk deje de emitir sonidos y parpadee en azul
#    - Presionar el safety switch (botón rojo) durante 3 a 5 segundos
#    - Mover los sticks del control hacia adentro-abajo hasta oír el pitido largo y giren las hélices

# 10 INSTALAR REQUIREMENTS ANTES DE EJECUTAR:
#     pip install -r requirements.txt
#    (Entorno virtual)   py -m pip install -r requirements.txt
# 11 EL DRON DEBE ESTAR ARMADO ANTES DE INICIAR LA MISIÓN

import warnings 
warnings.filterwarnings("ignore")

from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
import time
import torch
import cv2
import math

# CONFIGURACIONES INICIALES
ALTURA_SEGURA = 50  # metros
TIEMPO_RECONOCIMIENTO = 180  # segundos (3 minutos)
MIN_PERSONAS = 5
WAYPOINT_TIMEOUT = 60  # segundos máximo de espera antes de cancelar misión
RADIO_ORBITA = 30  # metros
VELOCIDAD_ORBITA = 2  # m/s

# CONEXIÓN CON EL DRON Y TELEMETRÍA
vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=True)

# UTILIDAD PARA ENVIAR MENSAJES A LA ESTACIÓN BASE (vía MAVLink STATUSTEXT)
def enviar_a_base(mensaje):
    print(mensaje)
    try:
        vehicle.message_factory.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            mensaje.encode('utf-8')[:50]  # límite de 50 bytes por mensaje
        )
    except Exception as e:
        print(f"[Error al enviar mensaje a base]: {e}")


def armar_y_despegar(altura_objetivo):
    enviar_a_base("Armando motores...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        enviar_a_base("Esperando armado...")
        time.sleep(1)

    enviar_a_base(f"Despegando a {altura_objetivo} metros...")
    vehicle.simple_takeoff(altura_objetivo)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        enviar_a_base(f"Altura actual: {alt:.2f} m")
        if alt >= altura_objetivo * 0.95:
            enviar_a_base("Altura alcanzada")
            break
        time.sleep(1)


def esperar_waypoint(timeout):
    enviar_a_base("Esperando que alcance waypoint de misión...")
    tiempo_inicial = time.time()
    while time.time() - tiempo_inicial < timeout:
        if vehicle.commands.next > 0:
            enviar_a_base("Waypoint alcanzado.")
            return True
        time.sleep(2)
    enviar_a_base("Timeout esperando waypoint.")
    return False


def contar_personas(modelo, tiempo_espera=10):
    cap = cv2.VideoCapture(0)
    total_detectadas = 0
    tiempo_inicio = time.time()

    while time.time() - tiempo_inicio < tiempo_espera:
        ret, frame = cap.read()
        if not ret:
            continue

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = modelo(img)
        detecciones = results.pandas().xyxy[0]  # pandas DataFrame con coordenadas [x_min, y_min, x_max, y_max]
        personas = detecciones[detecciones['name'] == 'person']

        enviar_a_base(f"Personas detectadas: {len(personas)}")
        total_detectadas = max(total_detectadas, len(personas))

    cap.release()
    return total_detectadas


def orbitar_lentamente(duracion, radio=30, velocidad=2):
    enviar_a_base(f"Iniciando órbita de {radio} m a velocidad {velocidad} m/s durante {duracion} segundos")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        0,
        0, 0, 0, 0,
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        vehicle.location.global_relative_frame.alt)
    vehicle.send_mavlink(msg)

    msg_orbit = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_ORBIT,
        0,
        radio, 0, 0, 0,
        0, 0, 0)
    vehicle.send_mavlink(msg_orbit)
    time.sleep(duracion)


def escanear_area(modelo, duracion):
    cap = cv2.VideoCapture(0)
    tiempo_inicio = time.time()
    enviar_a_base("Escaneando el área...")

    while time.time() - tiempo_inicio < duracion:
        ret, frame = cap.read()
        if not ret:
            continue

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = modelo(img)
        detecciones = results.pandas().xyxy[0]
        personas = detecciones[detecciones['name'] == 'person']
        enviar_a_base(f"[Escaneo] Personas detectadas: {len(personas)}")
        time.sleep(1)

    cap.release()


def esperar_y_detectar(modelo):
    enviar_a_base("Analizando durante 2 minutos en el waypoint...")
    return contar_personas(modelo, tiempo_espera=120)

# CARGAR MODELO
enviar_a_base("Cargando modelo YOLOv5...")
modelo = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

# INICIO DEL VUELO
armar_y_despegar(ALTURA_SEGURA)

enviar_a_base("Cambiando a modo AUTO para ir al waypoint...")
vehicle.mode = VehicleMode("AUTO")
llego = esperar_waypoint(WAYPOINT_TIMEOUT)

if llego:
    enviar_a_base("En waypoint. Analizando por 2 minutos...")
    num_personas = esperar_y_detectar(modelo)

    if num_personas >= MIN_PERSONAS:
        enviar_a_base(f"¡Detectadas {num_personas} personas! Iniciando escaneo en órbita...")
        vehicle.mode = VehicleMode("GUIDED")
        orbitar_lentamente(duracion=TIEMPO_RECONOCIMIENTO, radio=RADIO_ORBITA, velocidad=VELOCIDAD_ORBITA)
        escanear_area(modelo, TIEMPO_RECONOCIMIENTO)
    else:
        enviar_a_base(f"Menos de {MIN_PERSONAS} personas detectadas. Regresando a base...")
        vehicle.mode = VehicleMode("RTL")
        time.sleep(5)
        vehicle.close()
        exit()
        
enviar_a_base("Regresando a base...")
vehicle.mode = VehicleMode("RTL")

# FINALIZAR
time.sleep(5)
vehicle.close()

# NOTAS EXPLICATIVAS:
# - cv2 es la interfaz en Python de la librería OpenCV
# - results.pandas().xyxy[0] devuelve los resultados de detección en formato DataFrame para filtrado fácil  estructura de datos bidimensional, 
# similar a una tabla o una hoja de cálculo, que se utiliza para almacenar y manipular datos en Python
# - xyxy representa los bordes del bounding box: [x_min, y_min, x_max, y_max] se utiliza para representar un cuadro delimitador con cuatro valores en píxeles
