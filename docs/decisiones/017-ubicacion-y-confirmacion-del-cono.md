# 017. Ubicación, confirmación y aproximación al cono

Estado: implementada el 2026-11-23 en `mission/mission/core/cone_locator.py`, con tests sin ROS. Los parámetros del filtro y del tamaño del cono son supuestos hasta medirlos con `bag_conos` y los conos del aula.

## Problema

El detector (016) da manchas en píxeles. La misión (015) necesita la posición del cono en el mapa, saber cuándo creer que es un cono y no una falsa alarma, y un lugar adonde mandar al robot. La consigna agrega el caso del cono visto por un hueco en una pared: hay que llegar rodeando la pared, no atravesarla.

## De píxeles al mapa

La profundidad en el centro de la mancha (mediana de la mitad central del rectángulo, para no mezclar el fondo) da la distancia. Con la calibración de `camera_info` el píxel pasa a un punto 3D en el frame óptico de la cámara, y con su pose en el mapa (de TF) al mapa. Con la misma distancia se mide el tamaño real de la mancha: lo que no mide entre 8 y 50 cm de ancho y entre 10 y 70 cm de alto se descarta. Es el filtro de tamaño de la decisión 5 del plan.

El respaldo con el lidar cuando la profundidad no tiene valor no está implementado: se agrega si el bag muestra que hace falta.

## Confirmación

| Opción | A favor | En contra |
|---|---|---|
| A. Contar N detecciones en el mismo lugar | Simple | No usa lo que se sabe del detector; una falsa alarma que se repite confirma; no hay evidencia en contra |
| B. Filtro de Bayes binario por cono candidato, en log-odds | Es el mismo modelo que la grilla de la etapa A y el programa de la materia (filtros bayesianos). Usa las tasas medidas del detector y también la evidencia en contra | Hay que estimar las tasas |

Elegí B. Cada candidato tiene un log-odds que empieza en logit(previa) y:

- si el detector lo ve, suma log(P(detección | cono) / P(detección | no cono));
- si estaba en el campo de visión (80 % central), entre 0,3 y 4 m y sin una pared en medio en el mapa, y no lo vio, suma log((1 − P(detección | cono)) / (1 − P(detección | no cono))), que es negativo;
- se confirma con probabilidad 0,97 y se olvida por debajo de 0,05.

Con previa 0,2, acierto 0,7 y falsa alarma 0,05, una detección da 0,78 y dos seguidas en el mismo lugar 0,98. Si después de una detección el robot lo mira cuatro veces sin verlo, se olvida. Las detecciones a menos de 40 cm son el mismo cono, y su posición es el promedio de las medidas.

No contar como evidencia en contra lo que está detrás de una pared importa para el cono visto por un hueco: cuando el robot se mueve y la pared lo tapa, no se pierde.

## Dónde pararse

| Opción | A favor | En contra |
|---|---|---|
| A. Punto a cierta distancia del cono con línea de vista libre | Simple | Por un hueco en la pared hay línea de vista: el robot quedaría frente al hueco, del lado equivocado |
| B. Punto desde el que el robot podría avanzar en línea recta hasta el cono: holgura al menos igual al radio del robot a lo largo del segmento | Del lado equivocado de la pared el segmento pasa por el hueco, más angosto que el robot | Un cálculo más |

Elegí B. El primer intento fue A y su test falló justo en el caso de la consigna: eligió un punto pegado a la pared mirando por el hueco. Candidatos: 24 direcciones alrededor del cono a 0,45, 0,675 y 0,9 m. Se queda con los alcanzables desde el robot que cumplen B, y elige el de camino más corto (Dijkstra). El ángulo mira al cono. Si no hay ninguno, la misión lo anota como inalcanzable y sigue explorando.

## Datos

19 tests en `mission/tests/test_cone_locator.py`. Con una cámara a 500 px de focal: un cono centrado a 2 m queda a 2 m adelante del robot, uno 100 px a la derecha queda 0,40 m a la derecha, y la posición sigue el ángulo del robot. Pared con un hueco de 15 cm y una puerta de 1 m: el cono se ve por el hueco, el punto de aproximación queda del lado del cono y el camino del A* de la etapa B pasa por la puerta.

Pendiente: medir en `bag_conos` las tasas de acierto y falsa alarma y el tamaño de los conos, y el error de posición del cono contra una medida con cinta.
