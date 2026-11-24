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

Elegí B. El primer intento fue A y su test falló justo en el caso de la consigna: eligió un punto pegado a la pared mirando por el hueco. Candidatos: 24 direcciones alrededor del cono, en anillos cada 15 cm desde 0,45 hasta 1,5 m. Valen los alcanzables desde el robot que cumplen B y tienen la misma holgura que los puntos de observación (`viewpoint_clearance`, 014), para no pararse en pasillos angostos entre bancos. Se usa el anillo más cercano al cono que tenga alguno, y en él el de camino más corto (Dijkstra): con lugar libre queda a 0,45 m, y un cono cerca de un lugar angosto se aproxima desde más lejos, hasta 1,5 m. El ángulo mira al cono. Si no hay ninguno, la misión lo anota como inalcanzable y sigue explorando.

Primero los anillos iban hasta 0,9 m y se elegía el de camino más corto entre todos: podía quedar a 0,9 m teniendo lugar a 0,45 m. Al probar un cono dentro de un pasillo entre bancos, con la holgura no había ningún candidato a 0,9 m o menos.

Si ningún candidato tiene la holgura, se busca de nuevo sin exigirla, con la misma regla de alcanzable y segmento libre. Corregido el 2026-11-24 en Gazebo (`aula_lejos`, `mission/artifacts/gazebo_20261124/aula_lejos/`): con el cono a unos 0,5 m de las dos paredes de una esquina, la misión lo confirmó tras recorrer el aula y lo abandonó en el mismo instante como inalcanzable, sin mandar objetivos. Sin ROS, con el mismo mapa: según dónde cae la estimación del cono por milímetros había uno o ningún candidato con 0,5 m de holgura. Una esquina no deja lugar lejos de las dos paredes, y el navegador igual llega a un lugar más justo porque su mapa de costos ya incluye la huella del robot.

## Datos

21 tests en `mission/tests/test_cone_locator.py`. Con una cámara a 500 px de focal: un cono centrado a 2 m queda a 2 m adelante del robot, uno 100 px a la derecha queda 0,40 m a la derecha, y la posición sigue el ángulo del robot. Pared con un hueco de 15 cm y una puerta de 1 m: el cono se ve por el hueco, el punto de aproximación queda del lado del cono y el camino del A* de la etapa B pasa por la puerta. Un cono al fondo de un pasillo de 0,8 m se aproxima desde la boca del pasillo con holgura de 0,5 m, y desde adentro sin ella. Al fondo de un pasillo sin salida de 0,9 m, donde ningún lugar tiene esa holgura, se aproxima igual desde adentro.

Pendiente: medir en `bag_conos` las tasas de acierto y falsa alarma y el tamaño de los conos, y el error de posición del cono contra una medida con cinta.
