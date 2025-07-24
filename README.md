# <a name="x931f60067009076ceabaa6c1c35dc014ef9a2b9"></a>Proyecto de Dron Autónomo para Comunicaciones en Desastres Naturales
Este proyecto consiste en el desarrollo de un sistema aéreo no tripulado (UAV) completamente autónomo, basado en una controladora **Pixhawk 2.4.8**, una computadora **Jetson Nano con cámara Raspberry Pi HQ**, y comunicación por radio con módulos **3DR Sik** para realizar misiones de búsqueda, reconocimiento y transmisión de datos en entornos afectados por desastres naturales.

-----
## <a name="arquitectura-general"></a>Arquitectura General
- **Controladora de Vuelo:** Pixhawk 2.4.8
- **GPS:** Módulo M8N
- **Batería:** 4200 mAh 3S
- **Computadora de procesamiento:** Jetson Nano (con microSD)
- **Cámara:** Raspberry Pi HQ Camera (conectada a CSI o adaptador USB)
- **Comunicación base-dron:** Módulos 3DR Sik 915 MHz con antenas Yagi
- **Software de control:** Mission Planner + Python (con DroneKit y PyMAVLink)
- **Reconocimiento:** YOLOv5 (detección de más de 5 personas)
-----
## <a name="x89affeae57edcb5578c24aee12005dba1a96fb6"></a>Instalación JetPack en Jetson Nano (SD Card)
1. Descargar imagen desde [Jetson Downloads](https://developer.nvidia.com/embedded/jetpack)
1. Usar balenaEtcher o Raspberry Pi Imager para flashear la imagen en una tarjeta microSD (32GB+ recomendada, formato exFAT o ext4).
1. Insertar la SD, conectar a HDMI, teclado, mouse y fuente de 5V 4A.
1. Encender la Jetson y seguir los pasos de configuración inicial.
### <a name="problemas-comunes-y-soluciones"></a>Problemas comunes y soluciones
- Errores** o dependencias:
  - Asegúrate de tener python3.9 (necesario para DroneKit):

    sudo apt install python3.9 python3.9-venv python3.9-dev\
    sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.9 1
  - Crear entorno virtual compatible:

    python3.9 -m venv yoloenv\
    source yoloenv/bin/activate
  - Si pip da error, instalar manualmente:

    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py\
    python get-pip.py
  - Añadir al PATH si hay errores al correr python o pip:

    export PATH=$PATH:~/.local/bin
-----
## <a name="requisitos"></a>Requisitos
### <a name="requisitos-ubuntu-jetson-nano"></a>Requisitos Ubuntu / Jetson Nano
requirements\_ubuntu.txt

opencv-python\
pymavlink\
dronekit\
matplotlib\
torch\
torchvision\
ultralytics
### <a name="requisitos-windows"></a>Requisitos Windows
requirements\_windows.txt

opencv-python\
pymavlink\
dronekit\
matplotlib\
torch\
ultralytics

-----
## <a name="creación-y-activación-de-entorno-virtual"></a>Creación y Activación de Entorno Virtual
### <a name="windows"></a>Windows
py -m venv yoloenv\
yoloenv\Scripts\activate.bat
### <a name="powershell"></a>PowerShell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser\
yoloenv\Scripts\Activate.ps1
### <a name="ubuntulinux"></a>Ubuntu/Linux
python3.9 -m venv yoloenv\
source yoloenv/bin/activate

-----
## <a name="instalación-de-mission-planner"></a>Instalación de Mission Planner
### <a name="windows-1"></a>Windows:
- Descargar desde <https://ardupilot.org/planner/docs/mission-planner-installation.html>
- Ejecutar el instalador.
### <a name="ubuntu-con-wsl-o-wine"></a>Ubuntu (con WSL o Wine):
- Opción recomendada: usar Windows directamente.
- Alternativa: usar MAVProxy + SITL en Ubuntu para simulaciones.
-----
## <a name="comunicación-vía-3dr-sik-915-mhz"></a>Comunicación vía 3DR Sik 915 MHz
- Funcionan como un puente serie entre la base y el dron.
- **Velocidad típica:** 57600 o 115200 baud.
- **Antenas recomendadas:** Yagi en base, omnidireccional en dron.
- **Conexión:** vía USB a Jetson o PC base (en Linux aparece como /dev/ttyUSB0, en Windows como COMx).

Para verificar:

ls /dev/ttyUSB\*\
dmesg **|** grep tty

-----
## <a name="script-de-detección-yolov5-sin-gui"></a>Script de Detección YOLOv5 (sin GUI)
- Usa cv2.VideoCapture para leer la cámara.
- Si no detecta correctamente la cámara:
  - Probar con otro índice:

    cap = cv2.VideoCapture(1)
  - Usar v4l2-ctl --list-devices para ver las cámaras conectadas.
-----
## <a name="encendido-y-armado-del-dron"></a>Encendido y Armado del Dron
1. Conectar la batería del dron y encender el control.
1. Esperar a que Pixhawk parpadee azul y deje de emitir sonidos.
1. Presionar el botón rojo (safety switch) de 3 a 5 segundos.
1. Mover los sticks del control hacia adentro-abajo (como un “ángulo”) hasta escuchar un pitido largo y ver girar hélices.
-----
## <a name="diagrama-ascii-simplificado"></a>Diagrama  Simplificado
`                `[ Estación Base PC ]\
`                       `|\
`                  `(3DR Sik USB)\
`                       `|\
`        `=============================\
`                       `|\
`                  `(3DR Sik UART)\
`                       `|\
`                  `[ Pixhawk 2.4.8 ]\
`                       `|\
`          `+------------+-------------+\
`          `|                          |\
`   `[ GPS M8N ]             [ Jetson Nano ]\
`                                  `|\
`                         `[ Pi HQ Camera ]

-----
## <a name="llamado-de-funciones-copter-agregar-aquí"></a>Llamado de Funciones Copter 
# Librerías, Firmware y Conexión

|**Elemento**|**Detalle**|
| :- | :- |
|Librerías|pymavlink, dronekit|
|Firmware|ArduCopter (v4.0 o superior)|
|Protocolo|MAVLink|
|Conexiones Soportadas|SITL, USB, Telemetría, WiFi, UDP|

# 1\. Conexión y Comunicación

|**Función**|**Descripción**|
| :- | :- |
|mavutil.mavlink\_connection('url')|Conecta al dron|
|master.wait\_heartbeat()|Espera el primer mensaje de latido|
|master.recv\_match(type='MSG\_TYPE', blocking=True)|Recibe un mensaje específico|
|master.mav.\*\*\*\_send()|Envía comandos MAVLink personalizados|
|master.close()|Cierra la conexión|


# 2\. Modos de Vuelo

|**Modo**|**Descripción**|
| :- | :- |
|STABILIZE|Control manual con estabilización|
|ALT\_HOLD|Mantiene altitud, control manual|
|LOITER|Mantiene posición con GPS|
|AUTO|Ejecuta misión automáticamente|
|GUIDED|Control remoto desde script|
|RTL|Retorno automático a punto de origen|
|LAND|Aterriza|
|POS\_HOLD|Mantiene posición y altitud|
|BRAKE|Frena y queda en el aire|
|SMART\_RTL|RTL evitando obstáculos|
|FLOWHOLD|Usa sensor óptico de flujo|
|AUTOTUNE|Ajuste automático de PID|

# Cambiar modo:
master.set\_mode(master.mode\_mapping()['GUIDED'])


# 3\. Armar y Desarmar

|**Función**|**Descripción**|
| :- | :- |
|master.arducopter\_arm()|Arma motores|
|master.motors\_armed\_wait()|Espera que se arme|
|master.arducopter\_disarm()|Desarma motores|
|master.motors\_disarmed\_wait()|Espera desarme|

# 4\. Despegue y Aterrizaje

|**Función**|**Descripción**|
| :- | :- |
|despegar(master, alt)|Despega a una altitud|
|aterrizar(master)|Cambia a modo LAND|
|rtl(master)|Cambia a modo RTL|
# Ejemplo:
master.mav.command\_long\_send(

`    `master.target\_system,

`    `master.target\_component,

`    `mavutil.mavlink.MAV\_CMD\_NAV\_TAKEOFF,

`    `0, 0, 0, 0, 0, 0, 0, 10

)

# 5\. Movimiento y Velocidad

|**Función**|**Descripción**|
| :- | :- |
|goto\_position\_target\_global\_int()|Mueve a coordenadas GPS|
|goto\_position\_target\_local\_ned()|Mueve en sistema NED local|
|send\_velocity\_body()|Velocidad relativa al dron|
|send\_velocity\_ned()|Velocidad absoluta en NED|

# Ejemplo:
def send\_velocity\_body(master, vx, vy, vz):

`    `master.mav.set\_position\_target\_local\_ned\_send(

`        `0, master.target\_system, master.target\_component,

`        `mavutil.mavlink.MAV\_FRAME\_BODY\_NED,

`        `0b0000111111000111,

`        `0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0

`    `)


# 6\. Misiones (Waypoints)

|**Función**|**Descripción**|
| :- | :- |
|mission\_count()|Cuenta waypoints|
|upload\_mission(waypoints)|Sube misión|
|download\_mission()|Descarga misión actual|
|clear\_mission()|Borra todos los puntos|
|start\_mission()|Inicia misión (modo AUTO)|
|set\_current\_mission\_item(idx)|Selecciona waypoint activo|



# 7\. Telemetría

|**Mensaje MAVLink**|**Información**|
| :- | :- |
|GLOBAL\_POSITION\_INT|GPS, altitud, velocidad|
|ATTITUDE|Orientación (roll, pitch, yaw)|
|VFR\_HUD|Velocidad, altitud, rumbo|
|SYS\_STATUS|Estado del sistema y batería|
|HEARTBEAT|Modo de vuelo actual|
|BATTERY\_STATUS|Voltaje, corriente|
# Ejemplo:
msg = master.recv\_match(type='GLOBAL\_POSITION\_INT', blocking=True)

print(f"Altitud: {msg.relative\_alt / 1000} m")

# 8\. Parámetros del Copter (Dron)

|**Función**|**Descripción**|
| :- | :- |
|param\_fetch\_all()|Solicita todos los parámetros|
|param\_set\_send('NOMBRE', valor)|Establece un parámetro|
|recv\_param\_value()|Recibe valor de parámetro|

**Ejemplo:**

master.param\_set\_send('RTL\_ALT', 20, mavutil.mavlink.MAV\_PARAM\_TYPE\_REAL32)

Parámetros útiles:

- ARMING\_CHECK = 0
- RTL\_ALT = 20
- MIS\_TAKEOFF\_ALT = 10

# 9\. Comandos MAVLink

|**Comando**|**Descripción**|
| :- | :- |
|MAV\_CMD\_NAV\_TAKEOFF|Despegue|
|MAV\_CMD\_NAV\_LAND|Aterrizaje|
|MAV\_CMD\_NAV\_RETURN\_TO\_LAUNCH|Retorno al inicio (RTL)|
|MAV\_CMD\_DO\_CHANGE\_SPEED|Cambiar velocidad|
|MAV\_CMD\_COMPONENT\_ARM\_DISARM|Armar / desarmar|


# 10\. Failsafe y Seguridad

|**Función**|**Descripción**|
| :- | :- |
|enable\_fence()|Habilita geofence|
|set\_fence\_alt\_max()|Altura máxima|
|set\_batt\_failsafe()|Acción ante batería baja|
|set\_rtl\_return\_alt()|Altura para regresar|


# 11\. Logs

|**Función**|**Descripción**|
| :- | :- |
|download\_logs()|Descargar logs de vuelo|
|erase\_logs()|Eliminar logs almacenados|
|set\_log\_bitmask()|Elegir qué se registra en logs|



# 12\. Avanzado

|**Función**|**Descripción**|
| :- | :- |
|set\_home\_location()|Establece coordenadas de inicio|
|get\_home\_location()|Obtiene posición de inicio|
|set\_message\_interval(msg, us)|Ajusta frecuencia de mensaje|
|send\_heartbeat()|Envía latido manual|

# 15\. Script de Prueba (en algunos casos se requieren librerías adicionales)
from pymavlink import mavutil

import time

master = mavutil.mavlink\_connection('udp:127.0.0.1:14550')

master.wait\_heartbeat()

print("Dron conectado")

master.set\_mode(master.mode\_mapping()['GUIDED'])

master.arducopter\_arm()

master.motors\_armed\_wait()

master.mav.command\_long\_send(

`    `master.target\_system, master.target\_component,

`    `mavutil.mavlink.MAV\_CMD\_NAV\_TAKEOFF,

`    `0, 0, 0, 0, 0, 0, 0, 10

)

time.sleep(10)

master.mav.set\_position\_target\_global\_int\_send(

`    `0, master.target\_system, master.target\_component,

`    `mavutil.mavlink.MAV\_FRAME\_GLOBAL\_RELATIVE\_ALT\_INT,

`    `int(1<<5),

`    `int(40.7128 \* 1e7), int(-74.0060 \* 1e7), 10,

`    `0, 0, 0, 0, 0, 0, 0, 0

)

time.sleep(15)

master.set\_mode(master.mode\_mapping()['RTL'])

time.sleep(20)

master.arducopter\_disarm()

print("Dron desarmado")

-----
## <a name="notas-finales"></a>Notas Finales
- Antes de iniciar misión o código:
  - Verifica que esté armado.
  - Ejecuta script sin dependencias faltantes desde entorno virtual activado.
  - Verifica conexión a Pixhawk (/dev/ttyUSB0, COMx).
- Drivers para 3DR Sik (Windows):
  - Si no es detectado en Mission Planner, instalar desde: <https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers>
-----
## <a name="troubleshooting-adicional"></a>Troubleshooting Adicional
- Si Jetson no inicia: revisar voltaje y formato de SD (Debe ser FAT 32, no exFAT, al igual que verificar imagen del ISO de jetpack, (de preferencia la imagen oficial del developer kit)).
- Si dronekit no se instala: confirmar que usas Python 3.9.
- Si falla conexión al Pixhawk:

  sudo usermod -aG dialout $USER\
  sudo chmod 666 /dev/ttyUSB0
- Si la cámara no detecta:

  sudo apt install v4l-utils\
  v4l2-ctl --list-devices
-----
-----
### <a name="formatear-la-sd-en-ubuntu-fat32"></a>Formatear la SD en Ubuntu (FAT32)
Para formatear la tarjeta microSD en formato FAT32 desde Ubuntu:

1. Identifica la SD:

lsblk

2. Desmonta si está montada:

sudo umount /dev/sdX1

3. Formatea a FAT32:

sudo mkfs.vfat -F 32 /dev/sdX

Reemplaza sdX con el nombre real del dispositivo. Asegúrate de no usar una partición (sdX1), sino el disco completo (sdX).

-----
### <a name="x83a33ca7d514b530d1761266b1d3265fe53b689"></a>Uso de Mission Planner en Ubuntu (experimental)
Mission Planner fue diseñado para Windows, pero se puede ejecutar en Ubuntu usando **Mono** (requiere instalación).

1. Instala Mono:

sudo apt update\
sudo apt install mono-complete

2. Descarga el ZIP de Mission Planner desde: <https://firmware.ardupilot.org/Tools/MissionPlanner/>
2. Extrae el ZIP:

unzip MissionPlanner.zip -d MissionPlanner

4. Entra al directorio y ejecuta:

cd MissionPlanner\
mono MissionPlanner.exe

En Linux se esperan errores, comportamientos inestables o funciones que no están completamente soportadas. Es recomendado usar Windows para un funcionamiento completo.

-----
### <a name="simulador-sitl-software-in-the-loop"></a>Simulador SITL (Software In The Loop)
SITL permite simular el comportamiento de un dron sin hardware físico. Es útil para pruebas con DroneKit, planificación de misiones o prueba de scripts en tierra.
#### <a name="instalación-en-ubuntu"></a>*Instalación en Ubuntu:*
sudo apt update\
sudo apt install git python3-pip python3-dev screen\
git clone https://github.com/ArduPilot/ardupilot.git\
cd ardupilot\
git submodule update --init --recursive\
Tools/environment\_install/install-prereqs-ubuntu.sh -y\
. ~/.profile

Para iniciar simulación:

cd ardupilot/ArduCopter\
sim\_vehicle.py -v ArduCopter -f gazebo-iris --console --map
#### <a name="instalación-en-windows"></a>*Instalación en Windows:*
1. Descargar y ejecutar MAVProxy Windows Installer
1. Instalar Git Bash y Python 3.10+ si aún no se tiene
1. Clonar ArduPilot y usar Cygwin o WSL para ejecutar el entorno Linux dentro de Windows
1. Alternativamente, usar Mission Planner -> *Simulador SITL Local*
#### <a name="requisitos-adicionales-para-sitl"></a>*Requisitos adicionales para SITL:*
- MAVProxy 

pip install MAVProxy

- Python >= 3.9
- DroneKit, pymavlink, matplotlib (como en requirements.txt)

Puedes conectar un script en Python a 127.0.0.1:14550 (Mission Planner) durante una simulación SITL para probar comportamiento autónomo sin dron físico. (Cabe resaltar que se esperan troubleshooting en Ubuntu al ejecutar este simulador de SITL)

-----
