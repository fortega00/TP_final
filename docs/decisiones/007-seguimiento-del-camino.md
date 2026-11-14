# 007. Seguimiento del camino y ángulo final

Estado: decidida el 2026-11-14. Implementada en `nav/nav/core/follower.py` y probada en el simulador en vivo.

## Problema

El robot tiene que seguir el camino de forma suave y terminar en la posición y en el ángulo del objetivo. El RosMaster X3 tiene ruedas mecanum, así que puede moverse de costado.

## Alternativas

| Opción | A favor | En contra |
|---|---|---|
| A. Pure pursuit con velocidad de avance y velocidad angular | Visto en la materia y hecho en el TP2. Suave por construcción. Un parámetro principal, la distancia de lookahead | Corta las esquinas más o menos en un lookahead. Se comporta mal si el camino empieza detrás del robot |
| B. Girar y avanzar punto por punto | El más fácil de defender | Movimiento a los saltos, no cumple "suave" |
| C. Control holonómico (vx, vy, yaw) | Puede sostener cualquier orientación mientras avanza, el ángulo final sale gratis | Las mecanum patinan de costado, la odometría empeora y la localización trabaja más. No se vio en clase |

## Decisión

Opción A, con dos giros en el lugar:

- al empezar, si el error de orientación respecto del camino es grande. Sin eso pure pursuit abre un arco ancho al lado de las paredes;
- al llegar, para alcanzar el ángulo del objetivo.

## Datos

Valores usados, sin barrido todavía: lookahead 0,40 m, velocidad máxima 0,20 m/s y 0,80 rad/s, tolerancia de llegada 8 cm y 0,05 rad. El giro en el lugar empieza con más de 0,80 rad de error y termina por debajo de 0,15 rad.

Simulador en vivo, café, odometría con deriva. Error al llegar medido contra la pose real:

| Objetivo | Distancia recorrida (m) | Tiempo (s) | Error de posición (cm) | Error de ángulo (°) | Lo que creía el robot |
|---|---|---|---|---|---|
| (3,1, -2,4) | 4 | 32 | 13,1 | 4,9 | 10 cm, 2,7° |
| (3,19, -8,17) | 6 | 35 | 22,2 | 4,3 | 8 cm, 1,8° |
| (0, 0) | 9 | 61 | 18,2 | 4,8 | 14 cm, 2,6° |

Lecturas:

- El robot llega a los tres objetivos sin tocar nada.
- La mayor parte del error de posición es de localización, no de control: el robot cree estar a 8 o 14 cm y está a 18 o 22. Con el mapa de los profesores la localización tiene un piso de 6 a 12 cm ([004](004-localizacion-en-mapa-conocido.md)).
- El robot cree terminar a más de los 8 cm de tolerancia porque se sigue moviendo después de entrar en la tolerancia, mientras frena y hace el giro final. No vuelve a corregir la posición.
- El ángulo final queda a 4 o 5°, más que la tolerancia de 2,9°: el giro corta al entrar en la tolerancia y el robot sigue girando un poco.

Pendiente: barrido de lookahead, error lateral respecto del camino, y decidir si después del giro final se corrige de nuevo la posición.
