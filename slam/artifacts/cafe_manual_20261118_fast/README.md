# Café manual rápido, 18/11/2026

El usuario confirmó que la última sesión corresponde a la corrida rápida.
La lenta fue sobrescrita al usar el mismo `output_path` en todas las sesiones;
la corrida del choque se excluye. La sesión manual anterior de las 13:14 y el
ciclo Café 01 no deben etiquetarse como la corrida lenta de esta comparación.

Se copiaron PGM/YAML/JSON desde
`yahboom_rosmaster:/home/alumno/rosmaster_ws/maps/cafe`, con fecha de archivo
21:42 según el contenedor, y los logs del launch de 21:37. Sólo se analizaron
archivos existentes. No se ejecutaron SLAM, replays, nodos ni simuladores, y no
se modificó ningún hiperparámetro. El análisis se ejecutó con el Python y las
dependencias científicas ya disponibles en el contenedor; SciPy falta en el host.

## Resultado

La estructura principal del mapa es coherente y la puerta inferior permanece
transitable en el análisis estático. La limitación más clara para navegación
es la pérdida de dos obstáculos finos: patas de mesas. La conectividad de un
mapa no garantiza seguridad si faltan obstáculos reales.

- [Mapa, referencia y conectividad](map_comparison.png).
- [Puerta y sectores central/superior](details.png).
- [Inspección de las cinco patas](table_legs.png).
- [Métricas completas](metrics.json), [configuración/diagnóstico original](cafe.json)
  y [consignas reportadas por el usuario](teleop_limits.json).

| Medida | Resultado | Alcance |
| --- | --- | --- |
| Resolución | 5 cm | PGM trinarío exportado. |
| Ocupados a ≤10 / ≤15 cm de la referencia | 95,12 / 98,14 % | Cajas SDF aproximadas, sin ajustar alineación. No es RMSE de pose ni certifica que estén todos los obstáculos. |
| Puerta inferior | 1,05 m libre frente a 1,09 m nominal | Banda y∈[-11,08; -10,88] m; ancho discretizado, también confirmado libre. |
| Espacio libre conectado al origen, inflación 0,27 m | 164,94 m²; una componente | Se bloquean desconocido y exterior. No se ejecutó planificador ni controlador; la huella requiere calibración física. |
| Integraciones / omisiones por movimiento / descartes | 653 / 351 / 42 | Total recibido: 1046 scans; cero partículas fuera de grilla. |
| ESS final | 11,85 / 16 | Snapshot final, no historia completa de degeneración. |
| Núcleo, mediana / P95 | 235 / 265 ms | Últimas 512 integraciones; reloj de pared. |
| Procesamiento completo `drain`, mediana / P95 | 265 / 317 ms | Últimas 512 integraciones; no es latencia extremo a extremo. |
| Consignas máximas de teleop | 1,73 m/s y 3,45 rad/s | Sin duración ni velocidad medida. |

Los 42 descartes se atribuyen a odometría: 35 por timeout sin muestras y siete
por scan anterior al historial. Los avisos disponibles aparecen al arranque;
sin la serie completa no se asigna cada descarte a un instante. El JSON no
informa overflow de cola o timeout TF. No permite detectar pérdida antes del
callback DDS ni cuantificar una distribución de retraso extremo a extremo.

## Obstáculos finos

Las cinco mesas comparten la misma malla visual. Se verificó su sección a
z=0,19 m: la pata mide aproximadamente **4,16 cm** de ancho, coherente con la
caja de colisión de 4,2 cm. El hash de la malla del host coincide con el instalado
en el contenedor. Por tanto, para estas patas la ausencia no se explica por la
diferencia general entre cajas y mallas del Café.

| Centro nominal (x,y), m | Distancia al ocupado más cercano | Observación |
| --- | --- | --- |
| (0,5; -1,6) | 3,5 cm | Representada cerca de la referencia. |
| (2,4; -5,5) | 12,7 cm | Una celda ocupada desplazada; representación frágil. |
| (-1,5; -5,5) | 2,73 m | No hay ocupado cercano: la zona está marcada libre. |
| (2,4; -9,0) | 1,87 m | No hay ocupado cercano; queda una celda incierta algo desplazada. |
| (-1,5; -9,0) | 14,6 cm | Una celda ocupada desplazada; representación frágil. |

Estas distancias comparan mapa y modelo; no son errores de localización medidos.
No se puede atribuir la pérdida exclusivamente a velocidad sin scans ni un bag.
Una hipótesis a comprobar es el submuestreo de 1080 rayos a 180 haces de mapa:
la separación angular resultante ronda 2°, unos 17 cm a 5 m, mayor que una pata.
La cuantización de 5 cm y la persistencia inversa también pueden influir.

## Cobertura y referencia

Los hashes del world y ambos SDF coinciden con los instalados. Se rasterizan
cajas de colisión a z=0,19 m; el GPU LiDAR observa mallas visuales. La referencia
no equivale a un mapa exacto del sensor y el piso incluye exterior.

Contra esa referencia se observa el 91,90 % del interior no ocupado: 99,92 % en
el sector inferior, 99,88 % en el central y 76,32 % en el superior. **No concluir
que faltan 16,43 m² reales por explorar**: la diferencia se concentra arriba,
donde las cajas y las mallas no representan igual el contorno. Tampoco se puede
certificar cobertura completa. Los ajustes de paredes largas no muestran un
desdoblamiento general; la pared derecha presenta una inclinación ajustada de
aproximadamente 0,69° y residuos locales mayores que la izquierda. La comparación
con centros de cajas incluye espesor de muro y no mide por sí sola deriva de pose.

## Qué refinar primero, sin aplicar cambios

1. Recuperar obstáculos finos. Conservar D y la celda de 5 cm como base y, sobre
   una misma grabación futura, comparar primero `sensor.max_beams` 180→360,
   manteniendo los haces del matcher y demás parámetros. Medir cada pata,
   geometría, libre falso y tiempo. No se afirma que ese cambio ya lo resuelva.
2. Medir margen de cómputo y dejar registrado el backend efectivo. El LIDAR está
   configurado a 5 Hz (200 ms de simulación entre scans); el núcleo supera ese
   tiempo en reloj de pared. El factor de tiempo real y las omisiones importan,
   por lo que esto no prueba una cola creciente ni demuestra procesamiento
   sostenido a 1×. Existe caché Numba generada a las 21:16, compatible con uso JIT
   en estas sesiones; el proceso terminado no registró su backend en el JSON.
3. Guardar cada corrida con prefijo único y grabar scans, odometría, TF, reloj,
   GT, velocidades y estado antes de conducir. Faltan RMSE de pose, velocidades
   efectivas y comparación lenta/rápida. Esta corrida no valida deskew físico.

Con estas limitaciones no se certifica un mapa definitivo para B ni la aceptación
completa de A. No se inició otro ciclo ni se modificó el núcleo.

## Reproducir únicamente el análisis estático

`source/` contiene las versiones de los dos módulos de evaluación usados. El
script requiere NumPy, SciPy, PyYAML, Pillow y Matplotlib; puede ejecutarse dentro
del contenedor existente con `CAFE_PACKAGE` apuntando al share de `custom_map`.

Desde la raíz del repositorio, en el host:

```bash
docker exec yahboom_rosmaster mkdir -p /tmp/tp-final-fast-map-analysis
docker cp slam/artifacts/cafe_manual_20261118_fast/. \
  yahboom_rosmaster:/tmp/tp-final-fast-map-analysis/
docker exec \
  -e CAFE_PACKAGE=/home/alumno/rosmaster_ws/install/custom_map/share/custom_map \
  yahboom_rosmaster /usr/bin/python3 \
  /tmp/tp-final-fast-map-analysis/analyze_saved_map.py
docker cp yahboom_rosmaster:/tmp/tp-final-fast-map-analysis/. \
  slam/artifacts/cafe_manual_20261118_fast/
```
