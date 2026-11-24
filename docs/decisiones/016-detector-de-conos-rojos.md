# 016. Detector de conos rojos

Estado: implementado el 2026-11-23 en `mission/mission/core/detector.py`, probado con imágenes sintéticas. Los umbrales son supuestos: se ajustan con las imágenes del aula (`bag_conos`, visita 1).

## Problema

La consigna pide reconocer los conos rojos e ignorar los de otros colores, y la pregunta 1 del informe pregunta cómo se redujeron los falsos positivos y negativos. El robot tiene una cámara RGB-D (Astra); el detector trabaja sobre la imagen de color.

## Método

| Opción | A favor | En contra |
|---|---|---|
| A. Visión clásica: umbral de color en HSV, morfología y filtros de forma | Sin entrenamiento, rápido en la PC del laboratorio, cada filtro se explica y se ajusta por separado | Sensible a la luz y a objetos rojos con forma parecida |
| B. Red neuronal de detección (por ejemplo YOLO) | Robusta a la luz y al fondo | Necesita imágenes etiquetadas del aula, GPU o una PC rápida, y resuelve el problema central con una biblioteca ajena, contra la regla del grupo |

Elegí A. OpenCV se usa solo para operaciones genéricas de imagen, como hace la cátedra en `custom_map/features.py` del TP3.

1. **Color.** HSV separa el tono del brillo: el umbral cambia menos con la luz que en RGB. El rojo está en los dos extremos del tono (0 y 179 en OpenCV), así que se usan dos rangos: hasta 8 y desde 170. Saturación mínima 120 y brillo mínimo 70 descartan grises, blancos, rosados y sombras. El naranja, el distractor más probable, tiene tono de 10 a 25 y queda afuera.
2. **Limpieza.** Apertura morfológica (saca puntitos sueltos) y cierre (rellena huecos chicos), con un elemento elíptico de 5 px.
3. **Forma**, por cada mancha:
   - área mínima de 80 px: menos es ruido o un cono demasiado lejos para ubicarlo bien;
   - alto sobre ancho entre 0,6 y 4: descarta postes y patas;
   - solidez (área sobre envolvente convexa) de al menos 0,75: un cono es convexo;
   - angostamiento: el ancho del tercio de arriba sobre el del tercio de abajo, como mucho 0,85. Es lo que distingue un cono de una caja o una mochila roja.

El detector devuelve, por cada mancha, el centro, el punto medio del borde de abajo (donde el cono toca el piso) y el rectángulo. El **tamaño real** en metros necesita la profundidad y la calibración de la cámara: ese filtro va en la ubicación del cono, junto con la confirmación con filtro de Bayes (paso 5).

## Datos

14 tests en `mission/tests/test_detector.py`, con imágenes sintéticas de 640 × 480 sobre un piso gris con ruido. Se detectan: un cono rojo (con el punto de apoyo a menos de 3 px), un rojo del otro extremo del tono (176), un rojo oscuro pero saturado, y dos conos ordenados por tamaño. Se ignoran: un cono naranja, conos amarillo, verde, azul y violeta, un rectángulo rojo, una forma roja demasiado fina, conos de pocos píxeles, rojos poco saturados o muy oscuros. Cada test de distractor comprueba además que, sin su filtro, el distractor sí se detectaría: así se sabe qué filtro lo descarta.

## Riesgos a revisar con `bag_conos`

- **Luz real**: reflejos y contraluz cambian el tono y el brillo. Los umbrales se ajustan con el bag.
- **Bandas blancas**: un cono de tránsito con bandas reflectivas queda partido en varias manchas, y cada una puede no parecer un cono. Si los conos del aula las tienen, hay que unir las manchas alineadas verticalmente.
- **Conos tapados en parte** por una pata o una silla.
- **Otros objetos rojos** con forma de cono. Los filtra después el tamaño real.

Pendiente: tasas de acierto y falsa alarma medidas en `bag_conos`, que además alimentan el filtro de Bayes de la confirmación.
