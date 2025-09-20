# **Actividad 8:** Estrategias de simulación y recolección de datos

**¿Es pertinente realizar una simulación? ¿Por qué?**

Sí, es totalmente pertinente porque la simulación es el eje central del proyecto. A través de ella puedo evaluar el desempeño de diferentes configuraciones sin necesidad de implementar físicamente todos los escenarios. Esto me permite validar hipótesis, optimizar parámetros y obtener resultados reproducibles de manera más eficiente.

**En caso de ser afirmativa la respuesta: ¿cuál sería el objetivo de esa simulación?**

El objetivo es aplicar el diseño de experimentos (DOE) para comparar configuraciones bajo distintas combinaciones de factores, medir métricas de desempeño y analizar la influencia de cada variable en los resultados. La simulación servirá como un “laboratorio virtual” donde se pueden ejecutar pruebas controladas y obtener datos consistentes para sustentar conclusiones.

**¿Qué herramienta te puede ayudar a completar la investigación?**

Puedo utilizar ROS2 con Gazebo para las simulaciones, ya que me permite modelar sistemas robóticos y replicar escenarios realistas. Además, usaré scripts en Python y notebooks para postprocesar los datos, junto con gestores de referencias (Zotero/Mendeley) para sostener el análisis con literatura científica.

**Desarrolla una nueva estrategia para recolectar datos.**

La estrategia se basará en el DOE ya planteado:

- Definir factores y niveles (por ejemplo: ruido RTK, parámetros de control, condiciones de trayectoria).
- Correr simulaciones por bloques siguiendo el diseño factorial fraccionado, con réplicas para cada combinación.
- Documentar seeds y parámetros en plantillas YAML versionadas en Git, asegurando trazabilidad.
- Recolectar métricas automáticamente (tiempo, precisión, estabilidad, consumo de recursos) mediante scripts de postproceso.
- Consolidar y analizar los datos en tablas y gráficas, aplicando estadística básica para identificar efectos significativos.
