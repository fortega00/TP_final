# 015. Máquina de estados de la misión

Estado: decidida e implementada el 2026-11-23 en `mission/mission/core/mission.py`, con tests sin ROS contra un navegador simulado. Falta probarla en Gazebo y en el robot.

## Problema

La misión de la etapa C tiene que explorar el aula, frenar ante algo rojo, confirmar que es un cono e ir hasta él, usando el navegador de la etapa B sin modificarlo. El navegador recibe objetivos por `/goal_pose` y publica su estado en `/nav/state`; no tiene un "cancelar" ni avisa con un código si un objetivo terminó bien o mal.

## Estados

| Estado | Qué hace | Sale cuando |
|---|---|---|
| `WAIT_LOCALIZATION` | Nada | El navegador está ubicado |
| `EXPLORE` | Va al próximo punto de observación (014) | Llega: `LOOK_AROUND`. No llega: siguiente punto |
| `LOOK_AROUND` | Gira en el punto hacia lo que la cámara no vio | No queda nada por ver: siguiente punto |
| `CONFIRM` | Frena mirando un candidato rojo | Se confirma: `APPROACH`. Pasan `confirm_timeout` s: vuelve a lo que hacía |
| `APPROACH` | Va a la pose frente al cono | Llega: `DONE`. Falla `approach_attempts` veces: anota el cono como inalcanzable y sigue explorando |
| `DONE` | Nada: llegó al cono | — |
| `EXHAUSTED` | Nada: recorrió todos los puntos | — |

Un cono confirmado interrumpe `EXPLORE`, `LOOK_AROUND` y `CONFIRM`. Un candidato interrumpe `EXPLORE` y `LOOK_AROUND`. Si el navegador pierde la localización (`LOST`, `WAIT_INITIAL_POSE`, `LOCALIZING`, `WAIT_SCAN`) la misión espera, y ese tiempo no cuenta para los timeouts.

## Cómo saber si un objetivo terminó bien

| Opción | A favor | En contra |
|---|---|---|
| A. Leer el texto de los eventos de `/nav/state` ("goal reached", "goal rejected") | Exacto | Depende de un texto pensado para personas: si alguien lo cambia, la misión se rompe sin aviso |
| B. Agregar a `nav` un tópico con el resultado | Lo más limpio | Toca la etapa B, cerrada |
| C. Geometría: cuando el navegador vuelve a `IDLE`, se llegó si el robot está cerca del objetivo | No depende de textos ni toca `nav` | Un `IDLE` viejo, justo después de mandar el objetivo, podría tomarse como el final |

Elegí C. Para evitar el `IDLE` viejo, se exige haber visto al navegador en otro estado o que pasen `ack_timeout` (3 s) sin moverse; en ese caso el objetivo se considera rechazado. Tolerancias: 0,30 m y 0,35 rad.

## Girar, frenar e interrumpir

Como el navegador no tiene "cancelar", todo se hace con objetivos:

- **Girar en un punto** o **frenar a mirar un candidato**: objetivo en la posición actual, con el ángulo deseado. El navegador lo resuelve con su giro final.
- **Interrumpir la exploración**: mandar otro objetivo. El navegador replanifica con el robot en marcha ([009](009-maquina-de-estados.md)).

## Decisiones menores

- **Saltear puntos ya vistos.** Antes de ir a un punto se consulta la cobertura (`CoverageTracker`, 014): si lo que se ve desde ahí ya se vio en el camino, se pasa al siguiente.
- **Frenar ante un candidato** en lugar de confirmarlo en marcha: el robot puede estar girando y perderlo de vista, y la ubicación del cono es más precisa con el robot quieto.
- **Candidatos descartados.** Un candidato que no se confirmó no vuelve a frenar al robot si aparece en el mismo lugar (a menos de 0,5 m). Sin esto, un objeto rojo que no es un cono lo frenaría cada vez que lo ve.
- **Cono inalcanzable.** Si no hay pose de aproximación o el navegador falla dos veces, el cono se anota y se sigue explorando. Es el caso del cono visto por un hueco sin paso.
- **Cono sobre un punto de observación.** No choca: el lidar marca el cono como obstáculo nuevo, el navegador no encuentra camino a un objetivo ocupado y lo abandona a los 15 s, y la misión saltea el punto. Pero solo lo encuentra si la cámara lo ubica desde otro lugar. En Gazebo, con el cono sobre el segundo punto, desde el primero quedaba a 2,07 m, fuera de la profundidad simulada (1,4 m), y la misión siguió sin verlo (`mission/artifacts/gazebo_20261124/aula_punto_cono_en_punto/`). Con el alcance de 3 m del perfil real esa distancia entraría, sin medir todavía en el aula. No se agregó otro manejo: si en el aula pasa, buscar un punto libre cercano al que quedó bloqueado. El test de este caso usa un navegador simulado que no ve el cono como obstáculo, así que no cubre el bloqueo.
- **Un solo cono.** La misión termina en el primer cono alcanzado (decisión 7 del plan de la etapa C).

## Datos

17 tests en `mission/tests/test_mission.py`, contra un navegador simulado que pasa por `PLANNING` y `FOLLOWING` y puede rechazar objetivos o perder la localización. Cubren cada transición y un caso de punta a punta con el plan y la cobertura reales en una sala sintética, que termina con 90 % del piso visto.

Pendiente: tiempo total de la misión y comportamiento con el navegador real en Gazebo; ajustar `confirm_timeout` con la frecuencia real del detector.
