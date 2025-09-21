# **Actividad 9**: Tabla ideal de recolección de datos

## Parte 1. Pregunta de investigación

La pregunta de investigación que guiará mi trabajo es:

> ¿Cómo influyen el nivel de ruido en el RTK y la estrategia de control (PID, LQR, Difuso) sobre la precisión de guiado y la estabilidad del tractor en simulación, considerando el tipo de trayectoria como bloque de análisis?

Esta pregunta es específica, medible y se ajusta a los datos que puedo obtener dentro de la simulación en ROS2 y Gazebo.

---

## Parte 2. Tabla ideal de recolección de datos

La siguiente tabla reúne las variables que considero más importantes para mi investigación. Incluyo variables de identificación (simulación, seed, hash), independientes (RTK, control, LiDAR, cámara, trayectoria) y dependientes (error, estabilidad, percepción, tiempos).

| Variable                | Sim_01                     | Sim_02                     |
|-------------------------|----------------------------|----------------------------|
| Trayectoria (bloque)    | Recta                      | Curva en “S”               |
| Ruido RTK (cm)          | 0.5                        | 2.0                        |
| Estrategia de control   | PID                        | LQR                        |
| Resolución LiDAR        | 0.5°                       | 0.25°                      |
| Cámara (res/FOV)        | 640×480 @ 60°              | 1280×720 @ 90°             |
| Error medio (cm)        | 2.5                        | …                          |
| RMSE (cm)               | 3.1                        | …                          |
| P95 (cm)                | 4.8                        | …                          |
| Sobreimpulso (%)        | 8                          | …                          |
| Frec. oscilación (Hz)   | 0.15                       | …                          |
| Tiempo (min/ha)         | 12.1                       | …                          |
| FPR (%)                 | 4                          | …                          |
| FNR (%)                 | 3                          | …                          |
| Seed corrida            | 12345                      | …                          |
| Hash commit             | abc123                     | …                          |

---

## Parte 3. Análisis crítico y reflexión

### 1. Variables que decidí excluir
- **Costo del retrofit**: en esta fase de simulación no tiene relevancia, sería más útil en la etapa de prototipo físico.  
- **Condiciones ambientales reales (lluvia, polvo, vibración)**: Gazebo no modela estas condiciones, así que quedan fuera del alcance inmediato.  
- **Fatiga del operador humano**: como el tractor se controla de manera virtual, esta variable no aplica.  

### 2. Criterios para conservar variables
Me quedé con las variables que realmente responden a la pregunta de investigación. Además, busqué que fueran fáciles de medir automáticamente dentro de ROS2 (rosbag, tópicos) y que tuvieran un impacto directo en la validación de mi hipótesis.

### 3. Cambios que haría si repitiera la actividad
Si tuviera oportunidad de repetir la actividad, incluiría terrenos irregulares en la simulación para mayor realismo, más niveles de ruido en el RTK para analizar no linealidades y también incorporaría una métrica de consumo energético simulado como parte de la evaluación.

### 4. Mecanismos de verificación y control de calidad
Para asegurar la trazabilidad de mis datos, mantendría semillas fijas y hash de commit en cada corrida, usaría scripts de lanzamiento con parámetros estandarizados y validaría el flujo con corridas piloto antes de lanzar el DOE completo.

### 5. Aplicación pedagógica
Con mis estudiantes aplicaría este mismo enfoque en proyectos más pequeños, por ejemplo con robots móviles. Me interesa que ellos definan variables, construyan sus tablas y justifiquen qué conservan o qué excluyen. Un ejemplo sencillo sería preguntar: *“¿Cómo afecta la velocidad del robot a la precisión de detección de obstáculos?”*.  
Así puedo evaluar su aprendizaje a partir de tablas reproducibles y reflexiones metodológicas bien fundamentadas.
