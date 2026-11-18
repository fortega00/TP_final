# Informe de la etapa B

**Borrador del 2026-11-18.** Todo lo que dice "pendiente" necesita al robot real y sale de las dos visitas de la [guía de laboratorio](robot_real_checklist.md). Los números vienen de los documentos de [decisiones](decisiones/); acá va el resumen.

## Desarrollo

### Método de localización

Elegí un filtro de partículas (MCL) con campo de verosimilitud, implementado en `nav/nav/core/localization.py`. Motivos:

- usa la grilla del SLAM directamente, sin extraer landmarks;
- tolera un click inicial impreciso, porque arranca con muchas hipótesis;
- reusa el modelo de movimiento y el de sensor del SLAM de la etapa A, así el mapa se lee con el mismo modelo que lo construyó.

Descarté un EKF porque es unimodal (un click malo no se recupera) y necesita un scan matcher adelante. Detalle en [004](decisiones/004-localizacion-en-mapa-conocido.md).

### Modificaciones al filtro básico

| Modificación | Para qué |
|---|---|
| Ráfaga inicial con el robot quieto | Después del click, pesa y remuestrea varias veces con el mismo scan hasta que la nube es chica y la estimación deja de moverse. Queda a 2 a 8 cm en unos 10 ms |
| Compuerta de movimiento | Con el robot quieto no actualiza: scans repetidos vuelven al filtro demasiado seguro |
| Remuestreo solo con ESS bajo | No destruir hipótesis sin necesidad |
| Detección de perdido | La nube se puede cerrar en una pose falsa. Si más del 25 % del scan cae lejos de toda pared (15 % justo después del click), el nodo informa `LOST` |
| Inyección de partículas al azar | Implementada. No mostró beneficio medible |
| Scan matching sobre la odometría | Implementado y apagado: no mejora y cuesta el doble |

### Dificultades

- **La ráfaga cortaba en la primera iteración.** Con la nube ancha todo el peso cae en una partícula y la dispersión da cero. Agregué la condición de que la estimación esté estable.
- **Convergido no es lo mismo que bien ubicado.** Con un click a 90° la nube se cierra en una pose falsa. De ahí salió la detección de perdido, que cambió dos veces con datos: la distancia media daba `LOST` al rodear una caja.
- **El SLAM derivaba con el robot quieto** (17 cm en 215 s) y el origen del mapa no coincidía con el lugar físico de partida ([012](decisiones/012-integracion-con-el-robot-quieto.md)).
- **Liberar celdas del mapa con el lidar borraba paredes** (15 a 38 % en los bags del aula). Los obstáculos nuevos van en una capa aparte ([008](decisiones/008-obstaculos-no-mapeados.md)).
- **La parada de seguridad frenaba contra paredes del mapa** y el robot no llegaba a objetivos cercanos a una pared (81 % de llegada, 96 % al acortarla a 25 cm).
- **Nombres del robot real**: `base_footprint`, `ROS_DOMAIN_ID`, lidar montado a 180°. Un nombre mal y el nodo descarta todos los scans.

## Preguntas

### 1. ¿Qué tan sensible a hiperparámetros es el método que eligieron? ¿Por qué?

Poco. En el simulador, cambiar las partículas de 1000 a 3000, el ruido de movimiento 20 veces o `sigma_hit` 4 veces mueve el RMSE menos de 2 cm (11 a 12 cm en un tramo, 6 a 8 cm en el otro). El motivo: el lidar ve 360° y cada scan alcanza para ubicar al robot, así que el filtro casi no depende del modelo de movimiento. El parámetro que sí importa es la cantidad de partículas en el arranque: con 300, una de tres corridas arrancó 90 cm corrida.

### 2. ¿Tuvieron que hacer un proceso de tuneo de parámetros?

Casi no. Los parámetros del sensor son los del SLAM de la etapa A y el barrido de la pregunta 1 mostró que no había nada que ganar. Lo que sí ajusté con datos: el fin de la ráfaga inicial, los umbrales de perdido (25 % y 15 %) y los parámetros de la capa de obstáculos. El criterio fue siempre el mismo: RMSE contra la pose real en simulación, y cantidad de falsas alarmas y de casos no detectados sobre los bags del aula.

### 3. ¿Qué tan importante es la estimación inicial? ¿Qué ocurre si el filtro comienza con una posición incorrecta?

Hasta cierto error no importa. Sobre los bags del aula, 60 de 60 clicks con hasta 1,5 m o 45° de error terminan igual que el click correcto. Con 90° de error, 3 de 6 se cierran en una pose falsa a metros de la real, y el filtro dice que convergió. En el café, con 2 m de error, la mitad cae en un falso encaje a lo largo de un pasillo. La inyección de partículas no los recupera. Por eso el nodo revisa cuánto del scan queda sin explicar y rechaza el click.

### 4. ¿Qué sucede cuando el robot se desplaza a una zona que no está correctamente representada en el mapa?

Depende de qué falta. Objetos nuevos que el mapa no tiene (sillas, cajas) suman 1 a 3 cm de error: el campo de verosimilitud cobra un costo acotado por cada punto sin explicar. Si el robot sale del área mapeada, todas las partículas quedan fuera y el filtro falla con un error explícito en lugar de seguir. Robot real: pendiente (bag con sillas movidas).

### 5. ¿Qué fuente de información resulta más importante: la odometría o el sensor?

El sensor. Con odometría sola el RMSE es de 1,7 a 1,8 m en 20 m de recorrido en el simulador; con el lidar queda en 6 a 12 cm. En los bags del aula la odometría termina 1,5 a 1,9 m corrida. La odometría importa entre dos scans y cuando el lidar informa poco: un pasillo largo sin detalles, o el robot pegado a una pared (en el aula, 40 % de los haces sin retorno). Ahí la pose se sostiene con las ruedas.

### 6. ¿Cómo afecta la velocidad del robot a la precisión de la localización?

En el simulador, nada hasta 2,2 m/s equivalentes (RMSE de 11 cm en todos los casos). Se rompe a 4,4 m/s, cuando el robot avanza casi 1 m entre scans: 21 a 35 cm de RMSE y picos de más de 1 m. El robot navega a 0,2 m/s, lejos de ese límite. La prueba no incluye la deformación de un scan tomado en movimiento: en el aula, a 0,4 m/s un scan abarca 7 cm y 8°, y es la causa más probable de las paredes de 20 cm de grosor del mapa. Robot real: pendiente (bag a 0,15 m/s contra los de 0,4 m/s).

### 7. ¿Qué sucede si el mapa contiene errores? ¿Cómo se propagan a la estimación?

El filtro se ubica respecto del mapa, entonces un error del mapa pasa directo a la pose. En el café el mapa está corrido respecto del mundo simulado y la pose queda con 8,5 cm de corrimiento lateral estable, que ningún parámetro del filtro saca. Con sillas que se movieron respecto del mapa, el error sube 2 a 8 cm porque un objeto nuevo cerca de uno viejo se asocia con él y tira de la pose. El filtro no se perdió en ninguna de esas corridas.

### 8. ¿Cómo evaluaron cuantitativamente la calidad de la localización?

En simulación, RMSE de posición y de yaw contra la pose real. En el robot no hay pose real, entonces uso tres medidas: error al llegar a una marca del piso medido con cinta, repetibilidad al estacionar varias veces sobre la misma marca, y acuerdo con la trayectoria del SLAM sobre el mismo bag (3,7 a 4,9 cm en los bags del aula). Probé el residuo del scan contra el mapa como métrica única y lo descarté: no sigue al error real (correlación 0,41 y -0,73). Sirve solo para separar ubicado de perdido ([010](decisiones/010-metricas-de-evaluacion.md)).

### 9. ¿Qué ocurre si se aumenta o disminuye la frecuencia de actualización?

Casi nada, en un rango amplio. Actualizando cada 0,02 m o cada 1,28 m el RMSE va de 11 a 12,5 cm. Más seguido no mejora: cuesta cómputo y, con scans casi iguales, el filtro cuenta la misma evidencia varias veces y se vuelve demasiado seguro. Menos seguido deja crecer la deriva entre correcciones, y recién molesta cerca de 1 m entre actualizaciones. Uso 0,08 m o 0,06 rad.

### 10. ¿Podría funcionar con un mapa generado por otro algoritmo de SLAM?

Sí, y ya lo hace: las pruebas en el café y con el bag del Parcial usan mapas de los profesores. Condiciones: una grilla de ocupación con resolución y origen conocidos, que distinga libre, ocupado y desconocido, hecha con un lidar a la misma altura, y con paredes no mucho más gruesas que `sigma_hit`.

### 11. ¿Qué diferencias observaron entre simulación y robot real?

Pendiente: la etapa B todavía no corrió en el robot. Lo medido hasta ahora, con los bags del 16/11: la localización sigue al SLAM a 4 a 5 cm, la odometría real deriva parecido a la simulada, y el lidar real tiene haces sin retorno y scans deformados por el movimiento que el simulador no tiene.

### 12. ¿Cómo afecta la calidad del mapa a la precisión?

Es el piso del error. En el café, el RMSE de 6 a 12 cm viene del mapa y no del filtro: desde la pose estimada el scan encaja mejor en el mapa (3,4 cm de residuo) que desde la pose real (5,3 cm). Paredes gruesas, como las de 20 cm del mapa del aula, dejan ambigua la posición en ese ancho. Robot real: pendiente, con el mapa nuevo hecho a 0,15 m/s contra `intento3_corregido`.
