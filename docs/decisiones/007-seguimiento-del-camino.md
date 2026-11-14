# 007. Seguimiento del camino y ángulo final

Estado: decidida el 2026-11-14. Sin implementar.

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

Pendiente: error lateral respecto del camino, error de posición y de ángulo al llegar, y el lookahead elegido con su barrido.
