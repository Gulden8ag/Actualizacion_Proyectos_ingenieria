# Planeación General del Proyecto


## Fases del proyecto

### Fase 1 – Simulación y validación (MVP en ROS2 + Gazebo)

- Instalar entorno en RPi5 con ROS2 Humble y Gazebo.
- Crear modelo URDF/Xacro del tractor con pedales y volante.
- Integrar RTK sintético, LiDAR y cámara virtual.
- Implementar controladores básicos (PID) y adaptativos.
- Correr DOE piloto y completo (trayectorias recta, S, headland).
- Métricas: error guiado, tiempo, percepción, estabilidad.

### Fase 2 – Prototipo a baja escala

- Construcción de un mini-tractor a escala 1:10 o 1:8 con actuadores para pedales y dirección.
- Integración de GNSS RTK real (por ejemplo, módulo u-blox F9P).
- Uso de LiDAR de bajo costo (RPLidar A1/A2) y cámara RGB.
- Implementación de control en ROS2 sobre RPi5 u otra SBC.
- Validación experimental en pista controlada.

### Fase 3 – Retrofit en tractor convencional

- Selección de un tractor de uso común en Puebla.
- Diseño de kits de actuadores para volante y pedales (modulares, reversibles).
- Integración del stack ROS2 con sensores reales (RTK, LiDAR, cámara).
- Pruebas en campo con trayectorias sencillas (recta, headland).
- Comparación frente a operación manual.

### Fase 4 – Optimización y escalamiento

- Optimizar estrategias de control (LQR, difuso, híbridos).
- Validar robustez en condiciones reales: polvo, vibración, oclusiones.
- Análisis económico: costo del retrofit vs. tractor inteligente nuevo.
- Plan de adopción con pequeños productores → impacto social.

### Fase 5 – Transferencia y servicio social

- Documentación y manual de implementación open source.
- Talleres de capacitación a agricultores y estudiantes.
- Escalamiento a diferentes cultivos y terrenos.
- Conexión con programas de apoyo gubernamental o cooperativas rurales.

## Roadmap General

```mermaid
gantt
    title Roadmap Proyecto Tractores Inteligentes
    dateFormat  YYYY-MM
    axisFormat  %b %Y

    section Fase 1 - Simulación (MVP)
    Revisión bibliográfica y DOE       :done, des1, 2025-09, 2025-10
    Entorno ROS2 + Gazebo en RPi5      :active, des2, 2025-09, 2025-10
    URDF + control PID                 :des3, 2025-10, 2025-11
    Sensores virtuales (RTK, LiDAR, cámara) :des4, 2025-10, 2025-11
    Ejecución DOE + análisis            :des5, 2025-11, 2025-12
    Informe y validación MVP            :milestone, des6, 2025-12, 1d

    section Fase 2 - Prototipo a baja escala
    Diseño mini-tractor (1:10 – 1:8)    :des7, 2026-01, 2026-02
    Integración sensores reales (RTK, LiDAR, cámara) :des8, 2026-02, 2026-03
    Implementación control en ROS2      :des9, 2026-03, 2026-04
    Pruebas controladas en pista        :des10, 2026-04, 2026-05

    section Fase 3 - Retrofit tractor real
    Selección tractor y actuadores      :des11, 2026-06, 2026-07
    Integración stack ROS2 completo     :des12, 2026-07, 2026-08
    Pruebas en campo (recta, headland)  :des13, 2026-09, 2026-10

    section Fase 4 - Optimización y escalamiento
    Estrategias avanzadas de control    :des14, 2026-11, 2027-01
    Validación robusta en campo         :des15, 2027-02, 2027-04
    Análisis económico y social         :des16, 2027-04, 2027-05

    section Fase 5 - Transferencia y servicio social
    Documentación open-source           :des17, 2027-06, 2027-07
    Talleres y capacitación             :des18, 2027-07, 2027-08
    Escalamiento con productores        :des19, 2027-09, 2027-12
```
