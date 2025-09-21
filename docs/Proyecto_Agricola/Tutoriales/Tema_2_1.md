# Tutorial: Configuración de Raspberry Pi 5 (16 GB) con ROS 2 Jazzy y Gazebo Harmonic

Este tutorial describe paso a paso cómo preparar una **Raspberry Pi 5 (16 GB)** para trabajar con **ROS 2 Jazzy** y **Gazebo Harmonic**, incluyendo integración con `ros_gz` y controladores.

>  **Nota de compatibilidad**:  
> - ROS 2 **Jazzy** está soportado oficialmente en **Ubuntu 24.04 (Noble)** (arm64).  
> - Gazebo **Harmonic** es la versión recomendada en Ubuntu 24.04.  
> - **Gazebo Fortress no tiene soporte oficial en Ubuntu 24.04**, sólo compilando desde fuente.  

---

## Requisitos de hardware

- Raspberry Pi 5 (16 GB, recomendado para simulación pesada).  
- Almacenamiento: SSD USB 3.0 de 120 GB o más (mejor que microSD).  
- Fuente oficial 5V/5A para RPi5.  
- Sistema de refrigeración activa (ventilador + disipador).  
- Red: Ethernet preferido, Wi-Fi 5/6 opcional.  
- Periféricos (si no es headless): monitor HDMI, teclado, ratón.  
- Opcionales: sensores (LiDAR RPLidar A1/A2, cámara OAK-D, IMU).  

---

## Instalación de Ubuntu 24.04 (arm64)

1. Descarga **Raspberry Pi Imager** en tu PC.  
2. Selecciona: *Other general-purpose OS → Ubuntu Server 24.04 LTS (64-bit)*.  
3. Configura: hostname, usuario, SSH, Wi-Fi (si lo deseas headless).  
4. Flashea y arranca la Pi.  
5. En primer arranque, actualiza e instala utilidades:  

```bash
sudo apt update && sudo apt -y full-upgrade
sudo apt install -y curl gnupg lsb-release git build-essential cmake python3-pip
```

---

## Instalación de ROS 2 Jazzy

```bash
# Activar repos universe
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# Añadir repos ROS 2
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb   "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Instalar ROS 2 Jazzy
sudo apt update
sudo apt -y upgrade
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# Autocargar setup en cada sesión
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Prueba básica de ROS 2
En dos terminales distintas:
```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

Si ves mensajes de “Publishing…” y “I heard…”, ROS 2 está funcionando.

---

## Instalación de Gazebo Harmonic

Guía oficial: [Gazebo Harmonic – Install on Ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu/)

```bash
sudo apt update
sudo apt install gz-harmonic
```

> **Warning**: En algunos casos, el binario `gz` puede no quedar en el PATH hasta reiniciar sesión o volver a ejecutar `source /opt/ros/jazzy/setup.bash`.  

Prueba:
```bash
gz sim shapes.sdf
```

Debería abrirse una ventana de Gazebo con un mundo de prueba.

---

## Integración Gazebo ↔ ROS 2

Instalar paquetes puente y de control:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-joy
```

> **Warning**: El wildcard `ros-jazzy-joy*` no siempre funciona. Usa `ros-jazzy-joy` (sin asterisco).  

---

## Uso básico

### Lanzar Gazebo con ROS 2
```bash
ros2 launch ros_gz_sim gz_sim.launch.py
```

> ⚠️ **Warning**: El archivo de lanzamiento exacto (`gz_sim.launch.py`) puede variar según la versión instalada. Si falla, revisa la carpeta:  
> `/opt/ros/jazzy/share/ros_gz_sim/launch/`  

### Spawnear un robot desde URDF
Con un URDF publicado en el tópico `/robot_description`:

```bash
ros2 run ros_gz_sim create   -name my_robot   -topic robot_description   -entity bot1
```

> ⚠️ **Warning**: Los parámetros `-name`, `-topic` y `-entity` deben coincidir con los que tu versión de `ros_gz_sim create` soporte. Usa `--help` para confirmar.

---

## 🎮 Control con Joystick y ROS 2 Control

Ejemplo para integrar controladores:
```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-gz-ros2-control ros-jazzy-joy
```

- `ros2_control`: framework para manejar hardware/actuadores.  
- `gz_ros2_control`: plugin que conecta Gazebo ↔ ros2_control.  
- `joy`: soporte para joysticks/gamepads.  



