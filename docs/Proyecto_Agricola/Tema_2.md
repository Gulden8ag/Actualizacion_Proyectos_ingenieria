# Fase 1: Simulación y Validación (ROS 2 + Gazebo en RPi)

---

## 🔹 1. Infraestructura y entorno
**Meta:** Contar con un entorno funcional en RPi5 para simulación en ROS2 + Gazebo, accesible desde Windows vía VS Code + SSH.

**Checklist:**

- [ ] Configurar Raspberry Pi con **ROS 2 Jazzy** y **Gazebo Fortress**.  
- [ ] Crear cuentas de usuario y habilitar acceso remoto por **SSH**.  
- [ ] Probar conexión desde Windows con **VS Code + Remote-SSH**.  
- [ ] Instalar dependencias: `colcon`, `git`, `python3-pip`, `rosdep`.  
- [ ] Crear workspace `~/ros2_ws/` con carpeta `src/`.  
- [ ] Probar build y ejecución con el ejemplo `talker/listener`.  

**Criterio de aceptación:** Compilar y ejecutar `talker/listener` desde Windows en <30 min tras la conexión.

---

## 🔹 2. Repositorio y organización
**Meta:** Definir repositorio con estructura modular clara y lista para colaboración.

**Checklist:**

- [ ] Inicializar repositorio Git con ramas (`main`, `dev`).  
- [ ] Crear estructura de paquetes:  
  - `tractor_description/`  
  - `tractor_control/`  
  - `tractor_sensors/`  
  - `tractor_bringup/`  
  - `tractor_experiments/`  
  - `tractor_analysis/`  
- [ ] Subir README con guía de instalación y primeros pasos.  

**Criterio de aceptación:** `colcon build` sin errores y `ros2 launch tractor_bringup demo_world.launch.py` lanza un stack mínimo.

---

## 🔹 3. Modelado (URDF / Xacro)
**Meta:** Disponer de un modelo digital del tractor (URDF/Xacro) con cinemática simplificada y compatible con ros2_control.

**Checklist:**

- [ ] Crear modelo básico del tractor (chasis, ruedas, volante, pedales).  
- [ ] Definir frames TF (`map`, `odom`, `base_link`, `steer_joint`, `wheel_joints`).  
- [ ] Validar URDF con `check_urdf`.  
- [ ] Visualizar en **RViz** y comprobar jerarquía TF.  
- [ ] Documentar tabla de juntas, masas y ejes.  

**Criterio de aceptación:** `check_urdf` sin errores y visualización TF coherente en RViz.

---

## 🔹 4. Controladores
**Meta:** Implementar control básico de dirección y velocidad con PID, listo para pruebas de trayectorias rectas y curvas.

**Checklist:**

- [ ] Integrar `ros2_control` para volante y pedales.  
- [ ] Implementar **control longitudinal (PID velocidad)**.  
- [ ] Implementar **control lateral (PID ángulo dirección o Pure Pursuit)**.  
- [ ] Crear launch para probar comandos `/cmd_vel`.  
- [ ] Validar movimiento recto en Gazebo.  

**Criterio de aceptación:** Recorrido recto de 10 m con error lateral < 10 cm.

---

## 🔹 5. Sensores virtuales
**Meta:** Simular percepción mediante RTK sintético, LiDAR y cámara RGB en distintos perfiles de resolución.

**Checklist:**

- [ ] Configurar **RTK sintético** con ruido ajustable (0.5–5 cm).  
- [ ] Agregar **LiDAR 2D** (resoluciones 0.5° y 0.25°).  
- [ ] Configurar **cámara RGB** (640×480@60°, 1280×720@90°).  
- [ ] Verificar publicación de tópicos (`/scan`, `/image_raw`, `/rtk_fix`).  
- [ ] Documentar perfiles en YAML.  

**Criterio de aceptación:** Publicación estable de `/scan`, `/image_raw` y `/rtk_fix` durante ≥60 s.

---

## 🔹 6. Escenarios de simulación
**Meta:** Generar entornos de prueba (recta, curva en S, headland) con trayectorias definidas.

**Checklist:**

- [ ] Crear mundos `recta.world`, `s_curve.world`, `headland.world`.  
- [ ] Generar trayectorias (CSV o `nav_msgs/Path`).  
- [ ] Validar spawn y recorrido inicial en cada escenario.  

**Criterio de aceptación:** El tractor recorre correctamente las trayectorias predefinidas en los tres escenarios.

---

## 🔹 7. Diseño experimental (DOE)
**Meta:** Ejecutar corridas sistemáticas bajo distintas configuraciones de sensores y ruido RTK.

**Checklist:**

- [ ] Definir matriz experimental (RTK, LiDAR, cámara, control).  
- [ ] Implementar script de corridas automáticas con seeds.  
- [ ] Aleatorizar orden de ejecución.  
- [ ] Registrar rosbags con nombre estandarizado (`run_###`).  

**Criterio de aceptación:** Generación de `plan.csv` y ejecución de ≥1 bloque completo de corridas.

---

## 🔹 8. Métricas y análisis
**Meta:** Obtener métricas de guiado, percepción, estabilidad y tiempo, procesadas en CSV.

**Checklist:**

- [ ] Extraer datos de `/odom`, `/rtk_fix`, `/scan`.  
- [ ] Calcular **RMSE de guiado**.  
- [ ] Calcular **tiempo de operación (min/ha)**.  
- [ ] Medir **eficiencia de percepción** (FNR/FPR).  
- [ ] Evaluar **estabilidad del control** (FFT error lateral).  
- [ ] Consolidar en `metrics.csv` y `summary.csv`.  
- [ ] Generar boxplots y comparativos.  

**Criterio de aceptación:** Disponibilidad de `metrics.csv` consolidado con gráficos básicos exportados.

---

## 🔹 9. Documentación y entrega
**Meta:** Consolidar resultados en un informe reproducible y versionado en el repositorio.

**Checklist:**

- [ ] Redactar informe con descripción, configuración, DOE, resultados y gráficas.  
- [ ] Subir documentación al repositorio (`/docs`).  
- [ ] Verificar reproducibilidad (clonado y ejecución en <30 min).  

**Criterio de aceptación:** Informe final con 3–5 gráficas, `summary.csv`, y README con pasos reproducibles.
