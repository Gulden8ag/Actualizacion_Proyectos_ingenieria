# Fase 1: Simulación y Validación (ROS 2 + Gazebo)

---

# Entregable — Fase 1: Simulación en WSL2 (ROS 2 **Jazzy** + Gazebo **Harmonic**)

**Contexto:** La simulación se realizará **en Windows 10/11** usando **WSL2 (Ubuntu 22.04/24.04)** con **WSLg** para interfaces gráficas. No se requiere Raspberry Pi en esta fase. Se asume que el hardware del alumno tiene soporte para **OpenGL 3.3** vía WSLg (o se usa modo headless con visualización remota).

---

## 🔹 1. Preparación de Windows + WSL2
**Meta:** Contar con un ambiente WSL2 funcional (Ubuntu) con soporte gráfico (WSLg) para ejecutar RViz/Gazebo y herramientas ROS 2.

**Checklist:**

- [ ] Habilitar **WSL2** y **Plataforma de Máquina Virtual** en Windows.
- [ ] Instalar **Ubuntu** desde Microsoft Store (22.04 o 24.04).
- [ ] Actualizar sistema: `sudo apt update && sudo apt upgrade -y`.
- [ ] Verificar **WSLg** (GUI): ejecutar `xclock` o cualquier app GUI de ejemplo.
- [ ] Limitar recursos si hace falta: crear/ajustar `C:\\Users\\<usuario>\\.wslconfig` (memoria, procesadores).

**Criterio de aceptación:** Aplicación GUI simple abre desde WSL (WSLg) y el entorno Ubuntu está actualizado.

---

## 🔹 2. Instalación base (ROS 2 Jazzy + Gazebo Harmonic)
**Meta:** Disponer de ROS 2 **Jazzy** y Gazebo **Harmonic** instalados en WSL2, con utilidades esenciales de desarrollo.

**Checklist:**

- [ ] Instalar **ROS 2 Jazzy Jalisco** (desktop) con `ros-dev-tools`.
- [ ] Instalar **Gazebo Harmonic** con sus plugins básicos.
- [ ] Instalar herramientas: `git`, `python3-pip`, `colcon`, `rosdep`.
- [ ] Inicializar `rosdep` y resolver dependencias del sistema.
- [ ] Verificar **OpenGL 3.3**: `glxinfo | grep "OpenGL version"` o ejecutar `gz gui` de prueba.
- [ ] (Fallback) Preparar **modo headless**: `gz sim -s` + uso de **Foxglove Studio** o RViz en “bridge”.

**Criterio de aceptación:** `rviz2` y `gz gui` abren en WSLg sin errores críticos **o** se confirma ejecución headless estable por ≥60 s.

---

## 🔹 3. Repositorio y workspace
**Meta:** Estructurar el proyecto en un workspace reproducible y modular.

**Checklist:**

- [ ] Crear workspace `~/ros2_ws/src/`.
- [ ] Inicializar repositorio (rama `main` y `dev`).
- [ ] Crear paquetes:  
  - `tractor_description/` (URDF/Xacro, meshes)  
  - `tractor_control/` (control longitudinal/lateral + PID/Pure Pursuit)  
  - `tractor_sensors/` (configs RTK sintético, LiDAR, cámara)  
  - `tractor_bringup/` (launch, params YAML, worlds)  
  - `tractor_experiments/` (DOE, lanzadores por lote)  
  - `tractor_analysis/` (métricas, plots)  
- [ ] Compilar con `colcon build` y **source** correcto (`. install/setup.bash`).

**Criterio de aceptación:** `colcon build` sin errores y `ros2 launch tractor_bringup demo_world.launch.py` levanta un stack mínimo.

---

## 🔹 4. Modelado (URDF/Xacro + ros2_control)
**Meta:** Contar con un modelo URDF compatible con `ros2_control` y TF coherente.

**Checklist:**

- [ ] URDF base (chasis, ruedas, volante, pedales).
- [ ] TF: `map → odom → base_link → base_footprint`, `camera_link`, `lidar_link`.
- [ ] Validación con `check_urdf` y visualización en RViz.
- [ ] Configuración inicial de `ros2_control` (interfaces simuladas de dirección y tracción).

**Criterio de aceptación:** `check_urdf` sin errores y TF visible en RViz con jerarquía correcta.

---

## 🔹 5. Control (seguimiento + PID)
**Meta:** Implementar control longitudinal (velocidad) y lateral (dirección) para recorrer trayectorias.

**Checklist:**

- [ ] Control longitudinal **PID** (1–2 m/s, anti‑windup).
- [ ] Control lateral **Pure Pursuit** o **Stanley** con `nav_msgs/Path`.
- [ ] Launch integrado: `sim_world`, `sensor_profile`, `controller` y `seed` como args.
- [ ] Prueba en recta: seguimiento estable.

**Criterio de aceptación:** Recta de 10 m con **error lateral < 10 cm** en RTK de bajo ruido.

---

## 🔹 6. Sensores virtuales (RTK, LiDAR, cámara)
**Meta:** Simular percepción y posicionamiento con perfiles configurables.

**Checklist:**

- [ ] RTK sintético con ruido {0.5, 2, 5 cm}; publicar `/rtk_fix` y pose con covarianzas.
- [ ] LiDAR 2D (FOV, range, resoluciones {0.5°, 0.25°}).
- [ ] Cámara RGB (perfiles: 640×480@60°, 1280×720@90°) con `camera_info`.
- [ ] Archivos YAML por perfil y argumentos en launch.

**Criterio de aceptación:** Publicación estable de `/rtk_fix`, `/scan` y `/image_raw` durante ≥60 s sin caídas (>90% msgs).

---

## 🔹 7. Escenarios y trayectorias
**Meta:** Proveer mundos y rutas objetivo para pruebas (recta, S, headland).

**Checklist:**

- [ ] `recta.world`, `s_curve.world`, `headland.world` en Gazebo.
- [ ] Waypoints (CSV) o `nav_msgs/Path` generados por script.
- [ ] Spawn y alineación inicial correctos.

**Criterio de aceptación:** El vehículo completa las tres trayectorias planificadas sin colisiones ni desbordes del mapa.

---

## 🔹 8. DOE y automatización
**Meta:** Ejecutar barridos sistemáticos con semillas y registro estandarizado.

**Checklist:**

- [ ] `plan.csv` con factores: RTK_noise {0.5, 2, 5 cm}, LiDAR_res {0.5°, 0.25°}, Cam {640×480@60°, 1280×720@90°}, Control {PID}, Bloque {recta, S, headland}, **réplicas n=3–5**.
- [ ] Script batch: levanta sim, fija `seed`, corre T máx o “hasta terminar path”.
- [ ] Grabar rosbag2 (tópicos mínimos) con convención `results/run_###/` y `meta.json`.
- [ ] Health‑check previo (CPU, RAM, disco).

**Criterio de aceptación:** Ejecución completa de ≥1 bloque del DOE y generación de carpetas `results/run_###/` válidas.

---

## 🔹 9. Métricas y análisis
**Meta:** Producir métricas comparables y gráficas base para informe.

**Checklist:**

- [ ] Extraer `/odom`, `/rtk_fix`, `/scan` (+ comandos de control si aplica).
- [ ] Calcular **RMSE** y **P95** de guiado, **min/ha**, **FNR/FPR** (o proxy), **FFT** de error lateral y **sobreimpulso**.
- [ ] Guardar `metrics.csv` por corrida y `summary.csv` consolidado.
- [ ] Graficar boxplots y comparativas (Matplotlib).

**Criterio de aceptación:** `summary.csv` consolidado + al menos 3 figuras: (i) boxplot RMSE, (ii) barras FNR/FPR, (iii) curva error vs. distancia.

---

## 🔹 10. Documentación y entregable
**Meta:** Entregar informe reproducible con resultados y guías de ejecución.

**Checklist:**

- [ ] Informe: descripción, setup WSL2, configuración, DOE, resultados, gráficas.
- [ ] README con pasos “clonar‑construir‑ejecutar” en WSL2 (<30 min).
- [ ] Versionado en `/docs` del repo.

**Criterio de aceptación:** Informe con 3–5 gráficas, `summary.csv`, y README reproducible.


### Notas operativas (performance y compatibilidad)
- Si **WSLg** no soporta la GPU/driver, usar **headless** (`gz sim -s`) y visualizar con **Foxglove Studio** desde Windows.
- Cerrar otras distros WSL: `wsl --shutdown` antes de corridas largas.
- Ajustar `.wslconfig` para evitar swapping (memoria suficiente y `swap=0` si el equipo lo permite).
- En portátiles sin dGPU, priorizar resoluciones de cámara bajas y LiDAR 2D.

