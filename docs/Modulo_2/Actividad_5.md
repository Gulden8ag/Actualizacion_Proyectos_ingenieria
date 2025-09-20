# **Actividad 7:** Variables y diseño experimental de tu proyecto

## Objetivos

**Objetivo del proyecto**

Desarrollar una simulación en ROS2 y Gazebo sobre un sistema de cómputo embebido (RPi5) que modele la adaptación de un tractor convencional mediante un sistema mecatrónico con control de pedales y volante, geolocalización RTK sintética y sensores virtuales (LiDAR y cámara), con el fin de evaluar la precisión de guiado, eficiencia de percepción y tiempos de operación en un entorno controlado.

**Objetivos específicos**

- Diseñar un modelo URDF de tractor convencional que incorpore actuadores virtuales para volante y pedales.
- Integrar un sistema de geolocalización RTK sintético en la simulación para obtener trayectorias centimétricas.
- Configurar sensores virtuales de percepción (LiDAR y cámaras) para la detección de obstáculos en Gazebo.
- Evaluar métricas de desempeño como precisión de guiado, eficiencia de percepción y tiempos de operación agrícola en escenarios simulados.

## **Variables del proyecto**

Variables independientes


- Nivel de precisión o el ruido simulado del sistema RTK (Real-Time Kinematics).
- Resolución de sistema LiDAR (Light Detection and Ranging).
- Resolución y campo de visión de la cámara virtual.
- Estrategia de control (PID, estado completo, difuso) mecatrónico aplicada al modelo URDF (Unified Robot Description Format).
- Tipos de trayectorias en el entorno simulado (líneas rectas, curvas, trayectos predefinidos).

Variables dependientes


- Precisión de guiado: error promedio en centímetros respecto a la trayectoria planificada.
- Tiempo de operación simulado: duración (min/ha) para completar una tarea agrícola en Gazebo.
- Eficiencia de percepción: tasa de falsos positivos y falsos negativos.
- Estabilidad del control: frecuencia y amplitud de oscilación o desviación del tractor en la simulación.

Variables de control


- Plataforma de simulación: ROS2 Humble Hawksbill + Gazebo sobre un RPi5.
- Modelo de tractor URDF (dimensiones y parámetros de masa, inercia y fricción fijos).
- Escenarios virtuales estandarizados (tipo de terreno plano, densidad de obstáculos).
- Condiciones de simulación (misma frecuencia de actualización, parámetros de RTK).


## Diseño de experimento

Con el fin de evaluar el desempeño del sistema propuesto dentro de los alcances definidos para este curso (simulación en ROS2 y Gazebo sobre un RPi5), diseñé un experimento que me permite analizar cómo influyen distintos factores en variables clave como la precisión de guiado, la eficiencia de percepción y la estabilidad del control.

### 1) Enfoque y factores

El estudio se plantea como una comparación de configuraciones mediante un diseño factorial fraccionado. Para reducir la variabilidad asociada a los diferentes recorridos, se utilizarán bloques definidos por el tipo de trayectoria. Además, se realizarán réplicas con distintas semillas de aleatoriedad para garantizar robustez en los resultados.

Los factores principales (variables independientes) y sus niveles son:

* **Ruido en el sistema RTK (Real-Time Kinematics):** {0.5, 2.0, 5.0 cm}.
* **Resolución del sensor LiDAR (Light Detection and Ranging):** {0.5°, 0.25°}.
* **Cámara virtual (resolución y campo de visión):** {640×480 px @ 60°, 1280×720 px @ 90°}.
* **Estrategia de control:** {PID (Proportional-Integral-Derivative), LQR (Linear Quadratic Regulator o control de estado completo), y Control Difuso}.
* **Trayectoria (bloque):** {recta larga, curvas en “S”, y vueltas en cabecera o *headland*}.

### 2) Respuestas y métricas

Las variables dependientes a observar se definieron con las siguientes métricas:

* **Precisión de guiado:** error medio, RMSE (*Root Mean Square Error*) y percentil 95 (P95) respecto a la trayectoria planificada.
* **Tiempo de operación simulado:** duración en minutos por hectárea (min/ha), normalizado por longitud de trayectoria.
* **Eficiencia de percepción:** tasa de falsos positivos (FPR, *False Positive Rate*) y falsos negativos (FNR, *False Negative Rate*) al detectar obstáculos.
* **Estabilidad del control:** sobreimpulso en porcentaje (%), amplitud y frecuencia dominante de las oscilaciones, calculadas mediante análisis FFT (*Fast Fourier Transform* o Transformada Rápida de Fourier).

### 3) Variables de control

Para que los resultados sean comparables, mantendré constantes las siguientes condiciones:

* Plataforma de simulación: ROS2 y Gazebo sobre un RPi5 (misma versión en todas las corridas).
* Modelo URDF (*Unified Robot Description Format*) del tractor: dimensiones, masa, inercia y fricción fijas.
* Escenarios virtuales: terreno plano con obstáculos de densidad y geometría estándar.
* Condiciones de simulación: frecuencia de actualización y parámetros base de RTK invariables.

### 4) Plan de diseño (DOE)

El diseño experimental (DOE, *Design of Experiments*) siguió esta estrategia:

* **Bloques:** cada tipo de trayectoria (recta, S, headland) se analiza por separado.
* **Factorial fraccionado:** dentro de cada bloque se probarán las combinaciones de LiDAR, Cámara, Ruido RTK y Estrategia de control, utilizando un arreglo reducido para optimizar el número de corridas.
* **Replicación:** se realizarán 5 corridas por combinación, variando las semillas aleatorias que controlan el ruido y la generación de obstáculos.
* **Aleatorización:** el orden de ejecución de las configuraciones se permutará dentro de cada bloque para evitar sesgos asociados al tiempo o condiciones del sistema.

### 5) Procedimiento de cada corrida

Cada prueba seguirá cuatro etapas:

1. **Inicialización:** carga del mundo virtual y del modelo URDF del tractor; definición de la semilla y parámetros de la corrida (RTK, LiDAR, cámara, control).
2. **Ejecución:** el tractor recorre la trayectoria del bloque hasta completarla o hasta alcanzar un tiempo máximo definido.
3. **Registro:** se guardan los datos de navegación y percepción mediante rosbag2 (tópicos /tf, /odom, sensores y comandos de control), junto con logs de semilla y parámetros.
4. **Postproceso:** se calculan las métricas (error, RMSE, P95, min/ha, FPR/FNR, estabilidad) y se exportan los resultados en CSV para análisis posterior.

### 6) Análisis estadístico

El análisis se realizará en las siguientes etapas:

* **ANOVA (Análisis de Varianza):** para efectos fijos dentro de cada bloque, identificando factores principales e interacciones clave (ruido RTK, LiDAR, cámara, control).
* **Modelo mixto:** considerando la trayectoria como efecto aleatorio, con el fin de estimar efectos generales.
* **Comparaciones post-hoc:** pruebas de Tukey HSD (*Honestly Significant Difference*) para comparar estrategias de control (PID, LQR y Difuso).
* **Tamaño de efecto:** mediante η² parcial y Cohen’s d, para priorizar qué factores influyen más.
* **Verificación de supuestos:** revisión de normalidad de residuos y homocedasticidad para validar los modelos.

### 7) Criterios de aceptación

Finalmente, se establecieron umbrales de desempeño para evaluar el éxito del sistema:

* **Precisión de guiado:** RMSE ≤ 3 cm con RTK de 0.5–2.0 cm; RMSE ≤ 7 cm con RTK de 5.0 cm.
* **Percepción:** FNR ≤ 5 % y FPR ≤ 10 % en detección de obstáculos.
* **Tiempo de operación:** no superar +10 % respecto al escenario base (PID + RTK con ruido mínimo).
* **Estabilidad del control:** evitar oscilaciones sostenidas mayores a 0.3 Hz y sobreimpulso superior al 15 % en cambios de rumbo.

### Nota sobre trazabilidad y validación previa

Para asegurar reproducibilidad, cada corrida registró:

* **Semilla aleatoria** y **hash de commit** en los resultados exportados.
* **Versión fija** de ROS2 y Gazebo.
* **Script único de lanzamiento** con parámetros configurables.

Además, antes de correr el diseño completo, se realizará un pilotaje reducido (trayectoria recta, 2×2×2×2 combinaciones, n=3 réplicas) para verificar la correcta captura de métricas y la viabilidad en tiempos de simulación.

!!!warning "Este diseño es preliminar y puede ajustarse según avances en la implementación, tiempos de desarrollo o descubrimientos durante el pilotaje. El objetivo es contar con un plan estructurado que guíe las pruebas y permita evaluar el desempeño del sistema propuesto dentro de los alcances definidos."
