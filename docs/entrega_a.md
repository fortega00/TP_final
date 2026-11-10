# Informe de la entrega A

## Método elegido y alcance

Se implementó un filtro Rao–Blackwellizado de partículas, siguiendo la arquitectura FastSLAM, con una grilla de ocupación independiente por partícula. La pose es planar, `x,y,yaw`; cada mapa almacena log-odds y una máscara de celdas observadas. Las entradas son incrementos de odometría y mediciones del LIDAR. Este diseño permite representar incertidumbre de pose y utilizar directamente un mapa de ocupación en la planificación de la parte B.

El fundamento es la separación entre hipótesis de trayectoria y mapa condicionado a cada trayectoria. El [FastSLAM original](https://robots.stanford.edu/papers/montemerlo.fastslam-tr.html) propone esa factorización con landmarks; aquí se aplica la representación de grillas usada en [RBPF para mapeo](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/grisetti07tro.pdf). El modelo inverso, la forma logarítmica de la evidencia y los modelos probabilísticos de movimiento y medición se inspiran en *[Probabilistic Robotics](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)*, de Thrun, Burgard y Fox.

La implementación de los algoritmos es propia. NumPy realiza operaciones numéricas y ROS proporciona mensajes, TF y herramientas; no se utiliza una biblioteca que resuelva SLAM. No se extraen landmarks ni se ejecuta un EKF por celda. Tampoco se implementa la propuesta informada por scan matching de FastSLAM 2.0: la propuesta actual viene de odometría.

El paquete se organiza en módulos y una máquina de estados. El núcleo no depende de ROS y se prueba con un mundo sintético. El adaptador ROS permite usar las mismas operaciones en simulación, reproducción de un bag y ejecución con sensores reales. Las instrucciones están en [A.md](../A.md) y el diseño en [architecture.md](architecture.md).

## Ajustes respecto de un filtro básico

- **Grillas condicionadas por partícula:** el remuestreo copia mapas independientes, preservando hipótesis distintas.
- **Pesos en logaritmos:** la suma de log-likelihoods y la normalización estable evitan subdesbordamientos al combinar varios haces.
- **Ponderación sobre el mapa anterior:** el scan no se inserta antes de evaluar su concordancia.
- **Likelihood robusta:** combina proximidad a obstáculos con una componente uniforme; usa un subconjunto de haces y una temperatura configurable para moderar la confianza.
- **Remuestreo selectivo por ESS:** evita eliminar diversidad en cada lectura; el remuestreo es sistemático.
- **Modelo inverso acotado:** separa unknown de ocupado/libre, resta el logit del prior, satura evidencia y distingue retornos reales de ausencia de retorno.
- **Control de evidencia por lectura:** se deduplican celdas para no acumular artificialmente muchos votos correlacionados del mismo scan; prevalece el impacto sobre el libre dentro de esa lectura.
- **Movimiento SE(2) holonómico:** conserva traslación lateral, retroceso y giro, relevantes para el RosMaster.
- **Tiempo y geometría explícitos:** se usa la extrínseca del sensor y se relaciona la odometría con el timestamp del scan. Las lecturas fuera de orden no actualizan el mapa.
- **Publicación coherente:** el mapa y la pose corresponden a una partícula; no se promedian grillas desalineadas.

Estas decisiones facilitan convergencia numérica y consistencia geométrica. No garantizan convergencia global en cualquier entorno: siguen siendo necesarias hipótesis representativas y mediciones informativas.

## Dificultades resueltas y pendientes de evaluación

Las principales dificultades de implementación fueron separar el modelo directo de medición del modelo inverso, mantener el orden correcto de ponderación y mapeo, conservar independencia después del remuestreo y manejar casos límite del sensor. También fue necesario fijar convenciones de grilla, origen, marcos y tiempos para que un mapa guardado coincida con el publicado.

La integración debe respetar el árbol del RosMaster y evitar que un map server o una transformación fija `map → odom` de trabajos anteriores compita con SLAM. El hecho de tener un mensaje `/odom` no permite suponer que su `child_frame_id` sea la base elegida; las diferencias de marco se verifican y transforman explícitamente.

La evaluación sintética usa ruido y referencia conocida. Las mediciones de calidad del aula, velocidad real admisible, calibración de los sensores y exploración completa deben medirse en sus propias sesiones. El toy no representa superficies especulares, vibración ni deslizamiento real.

## Evaluación reproducible

Actualización del 10/11/2026: se completaron sesiones independientes de Gazebo café y del bag físico del Parcial, con mapas, métricas y capturas reales de RViz. Café obtuvo RMSE de posición de 2,063 cm y cobertura observada aproximada de 99,863 %. El bag ajustado dejó separación inicio–fin de 0,842 m frente a 17,855 m de odometría, pero conserva discontinuidades que impiden certificarlo como mapa definitivo de navegación. El procedimiento, los parámetros y los límites están en [experimentos_a.md](experimentos_a.md). Los resultados sintéticos siguientes se conservan como evidencia de desarrollo.

El mundo toy representa una oficina de 9 × 7 m, con pared divisoria y puerta de 2 m. La ruta es cerrada y recorre ambos lados de la partición, con poses inicial y final coincidentes. El generador aporta ground truth de geometría y pose; el filtro sólo recibe LIDAR y odometría perturbados. Los scans son instantáneos: el paso de 0,28 m y el período de 0,25 s son un muestreo del benchmark y no un ensayo físico de barrido a 1,12 m/s. Los datos, parámetros y métricas se guardan juntos para evitar comparar mapas sin saber qué cambió.

```bash
python3 -m tp_slam.cli toy --output artifacts/toy --seed 7 --check
```

Las métricas se definen así:

| Métrica | Qué permite evaluar |
| --- | --- |
| Precision de obstáculos con tolerancia de una celda | Proporción de celdas predichas ocupadas que coinciden con una pared próxima; penaliza obstáculos fantasma. |
| Recall con la misma tolerancia | Proporción de paredes de referencia representadas; penaliza omisiones. |
| IoU exacta de ocupados | Intersección/unión de celdas ocupadas sin tolerancia; penaliza desplazamiento y engrosamiento de paredes. |
| RMSE de posición y orientación | Error de la pose estimada respecto de la referencia de la ruta. |
| Error de cierre final | Diferencia de pose al finalizar la ruta cerrada. |
| Cobertura conocida/libre | Fracción del entorno que recibió evidencia o quedó libre. |
| Conectividad conservadora | Alcanzabilidad de puntos visitados después de inflar obstáculos con un disco de radio del robot más margen; unknown bloquea el paso. |
| Tiempo y memoria de mapas | Costo de esa corrida y estructura de datos; no es una certificación de tiempo real. |

La tolerancia de una celda depende de la resolución; al comparar configuraciones deben examinarse también medidas en metros y la imagen. Un resultado de precisión alto con recall bajo puede ocultar paredes omitidas. La conectividad de ciertos puntos visitados no demuestra seguridad para cualquier coordenada ni implementa navegación de B.

Los resultados completos y los parámetros exactos de cada corrida están en los artefactos JSON generados. Los experimentos con el robot físico deben conservar además el bag y medidas del aula, por ejemplo ancho de puertas, distancia entre paredes y posiciones de esquinas. La inspección visual debe buscar paredes dobles, geometría torcida, huecos falsos y ruido disperso.

### Resultados medidos del toy

La corrida final de semilla 7 procesó 118 de 119 scans; una lectura se omitió por el umbral de movimiento. El mapa cubrió el 100 % de las celdas interiores con evidencia y dejó el 98,73 % como libre con confianza suficiente. Se conservó conectividad hacia todos los puntos evaluados tras inflar por radio de 22 cm más margen de 5 cm. El error final de cierre fue 1,82 cm y 0,0053 rad. Las grillas de las 24 partículas ocuparon 12.288.000 bytes, sin contar copias temporales, trayectoria ni otros objetos. La corrida sin ROS tardó aproximadamente 4,5 s en este equipo. Fuente reproducible: [metrics.json](../artifacts/toy/metrics.json).

Se ejecutaron además cuatro perfiles manteniendo mundo, ruta y semilla. Los ruidos de adquisición son independientes de los coeficientes de movimiento asumidos por el filtro:

| Perfil | Celda | Precision / recall, tolerancia 1 celda | RMSE posición | P95 distancia de celdas ocupadas a pared | Tiempo | Aceptación toy |
| --- | --- | --- | --- | --- | --- | --- |
| Base | 5 cm | 0,9032 / 1,0000 | 2,46 cm | 10 cm | 4,73 s | Aprueba |
| Grilla más gruesa | 10 cm | 0,9613 / 0,9921 | 6,42 cm | 10 cm | 2,35 s | Aprueba |
| LIDAR con σ=6 cm | 5 cm | 0,6087 / 0,9961 | 2,12 cm | 15 cm | 4,43 s | Falla precision |
| Odometría más ruidosa y sesgada | 5 cm | 0,6705 / 0,9712 | 5,37 cm | 15 cm | 4,52 s | Falla precision |

Datos y parámetros: [comparison.md](../artifacts/tuning/comparison.md), [comparison.json](../artifacts/tuning/comparison.json). Todas conservaron conectividad en los puntos evaluados; eso no compensa la degradación geométrica de las dos condiciones ruidosas. La tolerancia de una celda es de 10 cm en la grilla gruesa y 5 cm en las otras: su precision mayor no prueba una mejora de detalle. El RMSE usa la pose elegida online en cada timestamp, sin alinear la estimación a posteriori al ground truth.

La odometría base obtuvo RMSE de 2,01 cm, menor que los 2,46 cm de SLAM. En este escenario de odometría ya buena, el filtro no mejora esa métrica; produce un mapa consistente y conserva un margen pequeño de error. Las pruebas ruidosas muestran que estos defaults no bastan para cualquier ruido del RosMaster. Es necesario calibrarlos o mejorar la propuesta y la compensación temporal antes de atribuir robustez real al sistema.

La IoU exacta de ocupados fue 0,3642 en la base. Esto expone engrosamientos y desalineaciones de pocas celdas que la tolerancia espacial de precision/recall admite. Por eso se conservan ambas métricas: el mapa reproduce la estructura y mantiene la puerta, pero sus paredes no coinciden celda por celda con una referencia ideal de espesor mínimo.

Se verificaron dos semillas adicionales sin reajustar parámetros:

| Semilla | Precision / recall, 1 celda | IoU exacta | RMSE SLAM / odometría | Error final | Conectividad | Aceptación |
| --- | --- | --- | --- | --- | --- | --- |
| 7 | 0,9032 / 1,0000 | 0,3642 | 2,46 / 2,01 cm | 1,82 cm | 1,0 | Aprueba |
| 11 | 0,9361 / 1,0000 | 0,4245 | 2,88 / 3,78 cm | 3,71 cm | 1,0 | Aprueba |
| 19 | 0,8976 / 1,0000 | 0,3512 | 2,37 / 3,84 cm | 2,04 cm | 1,0 | Aprueba |

Resultados: [comparación de semillas](../artifacts/seeds/comparison.json). Hay mejoras sobre odometría en dos de las tres semillas y una degradación pequeña en la base. Tres repeticiones del mismo mundo siguen siendo una validación inicial, no una prueba de robustez para otros entornos.

## Respuestas a las preguntas de la consigna

La integración se validó también en ROS 2 Humble: el publicador toy, el rosbag2 sintético con odometría por tópico y el mismo bag con odometría por TF integraron 118 scans sin descartes y exportaron PGM idénticos. Se verificaron los servicios, el reset, el cierre y la exclusión de TF de localización obsoletos y órdenes de velocidad grabadas. [Evidencia de integración](../artifacts/ros/README.md). Esto no sustituye un recorrido con Gazebo ni el bag físico del RosMaster.

### 1. ¿Qué tan sensible a hiperparámetros es el método elegido? ¿Por qué?

Es sensible al equilibrio entre incertidumbre del movimiento y selectividad de la medición. Si el ruido de odometría se subestima, las partículas pueden no cubrir la pose verdadera; aumentar la confianza del LIDAR no crea hipótesis ausentes. Si se sobreestima y hay pocas partículas, se dispersan demasiado. Una likelihood muy estrecha o con demasiados haces correlacionados concentra prematuramente los pesos. El número de partículas y el umbral ESS controlan diversidad y costo. La resolución, espesor de impacto y probabilidades inversas condicionan el detalle y la persistencia de errores del mapa.

### 2. ¿Tuvieron que hacer un proceso de tuneo de parámetros?

Sí. Los primeros valores generaron demasiadas paredes engrosadas; se compararon alternativas de ruido de movimiento, espesor de impacto, sigma y temperatura de likelihood. La selección final usa 5 cm por celda, 24 partículas, probabilidades inversas de impacto/libre de 0,70/0,35, espesor de 3 cm, `sigma_hit=0.10` y temperatura 1. Los coeficientes de varianza finales son `0.0001, 0.00001, 0.0001, 0.0001`. Los resultados y parámetros están en `artifacts`, incluyendo condiciones que fallan. El tuneo de laboratorio debe repetir el mismo bag y medir geometría; falta calibrar ruido y extrínsecas con el RosMaster físico.

### 3. ¿Observaron dependencia entre el recorrido o velocidad y el mapa?

El toy utiliza una ruta cerrada con observaciones desde ambos lados de la puerta; eso permite evaluar el cierre y la geometría interior. Sus scans son instantáneos: no simula distorsión intrabarrido y no permite concluir una velocidad física admisible. La dependencia específica de velocidad del robot real sigue pendiente de medición. Teóricamente, giros rápidos y desplazamientos durante un barrido introducen distorsión porque esta versión usa una pose por scan. Recorridos sin solapamiento, pasillos repetitivos o revisitas muy tardías hacen más difícil conservar una hipótesis correcta. Reproducir un bag más lentamente reduce carga de cómputo, pero no modifica cómo se adquirieron sus scans.

### 4. ¿Qué fuentes de error tuvieron mayor impacto y cómo lo determinaron?

En las dos perturbaciones sintéticas ejecutadas, el LIDAR con σ=6 cm redujo la precision de paredes de 0,9032 a 0,6087; la odometría más ruidosa y sesgada la redujo a 0,6705 y elevó más el RMSE de pose. Por tanto, ambos degradaron el mapa, con efectos distintos. No son perturbaciones de severidad equivalente y no prueban qué sensor dominará en el robot. Para aislar más causas faltan sensor ideal, odometría ideal, variantes del algoritmo y varias trayectorias. En el robot se agregan extrínseca, sincronización, deslizamiento y distorsión de barrido. La comparación SLAM/odometría de una corrida cuantifica una diferencia, pero no aísla por sí sola todas las causas.

### 5. ¿Qué ocurre al volver a una zona recorrida? ¿Corrige errores locales o lejanos?

La lectura se compara con el mapa histórico de cada partícula. Si algunas hipótesis conservan una trayectoria consistente, pueden ganar peso y ser seleccionadas, mejorando simultáneamente la pose y el mapa completo asociado. Esa selección puede cambiar qué representación de zonas lejanas se publica. Sin embargo, la implementación no corrige retrospectivamente las poses de una trayectoria ni optimiza un grafo; tampoco reconstruye una hipótesis eliminada. Si todas derivaron o el remuestreo perdió la correcta, revisitar no garantiza un cierre global. El error de cierre toy mide ese caso concreto y no prueba cierres arbitrarios.

### 6. ¿Cómo afecta el tamaño de grillado al mapa y a la convergencia?

Celdas menores distinguen puertas, esquinas y paredes delgadas, y reducen la cuantización espacial. También exigen más memoria y trabajo para el mismo entorno, y pueden hacer más evidente el ruido o una mala localización. En el toy, pasar de 5 a 10 cm redujo aproximadamente a la mitad el runtime, pero elevó el RMSE de 2,46 a 6,42 cm; la puerta de 2 m siguió transitable. Esto no demuestra qué pasaría con una puerta estrecha. Celdas grandes mezclan libre y ocupado y suavizan el campo que evalúa la likelihood, por lo que también afectan indirectamente la selección de partículas. El ruido y espesor de impacto deben ser coherentes con el detalle buscado. Se conservó el área en metros en la comparación.

### 7. ¿Qué tan exigente es en cómputo? ¿Funcionaría en una computadora menor?

El costo aumenta con las partículas y los haces, y el de las grillas densas con el área dividida por el cuadrado de la resolución. El ray tracing también crece con el alcance en celdas. El remuestreo requiere copiar los mapas elegidos. La corrida toy de 119 scans tardó unos 4,5 s sin ROS, con 12,29 MB decimales de grillas de partículas más memoria auxiliar. No es una medición de latencia sostenida en el robot. En un equipo menor se pueden reducir haces o partículas, aumentar moderadamente la celda, limitar alcance y procesar por movimiento. Cada reducción debe verificarse contra calidad y frecuencia. Tiles y copy-on-write son mejoras estructurales futuras.

### 8. ¿Es online u offline? ¿Qué beneficios tiene cada opción?

Es online: incorpora datos de forma incremental y publica el mapa durante la adquisición o reproducción. Procesar un bag con el mismo filtro no lo convierte en un optimizador offline. Online permite inspeccionar cobertura y calidad mientras se recorre el entorno; exige procesar a la tasa necesaria y tomar decisiones con la evidencia disponible. Un método offline puede utilizar toda la secuencia, volver sobre poses anteriores y gastar más cómputo en optimización global; no ofrece necesariamente un mapa final durante el recorrido. La consigna exige un bag propio si se elige ese enfoque offline.

### 9. ¿Funcionaría para otro robot, por ejemplo un cuadrúpedo?

La estructura RBPF y las grillas son reutilizables si la operación sigue siendo aproximadamente planar y se dispone de odometría y rangos coherentes. Habría que calibrar el modelo de movimiento, extrínsecas, sincronización y ruido de ese robot. En un cuadrúpedo, roll, pitch y altura cambiante pueden invalidar un LIDAR tratado como un plano horizontal. Sería necesario estimar orientación 3D y proyectar/compensar lecturas o cambiar el estado y el mapa. No basta con renombrar tópicos.

### 10. ¿Qué tan fácil sería modificarlo para mapear en 3D?

La factorización conceptual se conserva, pero el cambio es importante: estado SE(3), modelo de movimiento y propuesta tridimensionales, extrínsecas completas, mediciones 3D, modelo inverso volumétrico y nuevo formato de salida. Una grilla densa por partícula escala con volumen/resolución³, por lo que resulta conveniente una estructura dispersa u octree con independencia lógica entre hipótesis. También cambian likelihood y evaluación geométrica. La separación de interfaces facilita reemplazar componentes; la implementación actual no produce mapas 3D ni proporciona esa extensión automáticamente.

## Criterio de aceptación para completar A

La entrega debe acompañar el paquete ROS y los launchfiles, este informe actualizado con experimentos propios, el mapa exportado del entorno real y evidencia de su calidad. Hay que verificar la totalidad de las zonas, la geometría de paredes, puertas transitables tras considerar la huella, y ausencia de obstáculos fantasma. Los launchers y el benchmark permiten repetir el proceso; la aprobación sintética no reemplaza las mediciones de Gazebo, bag y laboratorio.
