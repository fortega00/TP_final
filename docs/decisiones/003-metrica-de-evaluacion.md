# 003. Cómo se mide el error de trayectoria en el bag real

Estado: adoptada el 2026-11-14. Reemplaza la alineación usada en las tablas originales de 001 y 002.

## Problema

En el bag real no hay pose verdadera. La referencia es mi localización del TP3 sobre el mapa del laboratorio de los profesores, que tiene su propio error. El SLAM arranca en `(0,0,0)` en su propio frame, así que para comparar hay que llevar su trayectoria al frame del mapa de referencia.

Hasta ahora esa transformación se fijaba con un solo dato: la pose de referencia en el instante del primer scan. Si esa pose tiene un error chico de ángulo, toda la trayectoria queda girada, y a 10 m del inicio medio grado ya son 9 cm. El error medido dependía entonces de qué tan buena era la referencia en un único instante.

Lo detecté al pasar el bag por el nodo ROS en tiempo real. El nodo descarta los primeros 7 scans mientras llega el TF del láser, o sea que arranca un segundo más tarde, y con semilla 7 dio 11,5 cm contra 8,0 cm del replay offline. El mapa y el filtro eran los mismos.

## Alternativas

| Opción | A favor | En contra |
|---|---|---|
| A. Anclar en la primera pose (lo que había) | Simple, sin ajuste | Depende del error de la referencia en un solo instante |
| B. Mejor ajuste rígido de toda la trayectoria | Usa todos los puntos, es el error absoluto de trayectoria estándar en SLAM | El ajuste absorbe un error global de rotación o traslación del SLAM |
| C. Anclar en el promedio de los primeros segundos | Menos sensible que A | Sigue dependiendo del arranque, y agrega una ventana para elegir |

## Decisión

Opción B. La transformación es la rotación más traslación, sin escala, que minimiza la distancia cuadrática entre la trayectoria del SLAM y la de referencia (algoritmo de Kabsch). Con esa misma transformación se comparan los mapas. El RMSE con la primera pose se sigue calculando y se informa aparte.

Lo que absorbe el ajuste no es un problema acá: el frame del SLAM es arbitrario, entonces una rotación o traslación global no es un error del mapa. La deformación interna de la trayectoria sí queda en el número.

En simulación no cambia nada. Ahí la referencia es la pose real de Gazebo, exacta y en el mismo frame.

## Datos

Bag real, semillas 7 / 1 / 2, scan matching con 8 partículas:

| Corrida | RMSE con primera pose (cm) | RMSE con mejor ajuste (cm) |
|---|---|---|
| Replay offline | 8,0 / 7,2 / 8,0 | 5,8 / 5,7 / 5,4 |
| Nodo ROS en tiempo real | 11,5 / 6,2 / 7,8 | 5,5 / 5,3 / 5,4 |
| Replay offline arrancando en el mismo scan que el nodo (semilla 7) | 13,7 | 5,5 |

Con mejor ajuste el nodo ROS y el replay coinciden dentro de 0,4 cm en las tres semillas. Con la primera pose la misma trayectoria da entre 8,0 y 13,7 cm según en qué scan arranca.

Todas las corridas que seguían disponibles, con la métrica nueva:

| Configuración | RMSE (cm) | Error máximo (cm) | Recall a 10 cm | Precisión a 10 cm | Área libre (referencia 121 m²) |
|---|---|---|---|---|---|
| Scan matching, 8 partículas, offline | 5,8 / 5,7 / 5,4 | 22,5 / 23,1 / 15,2 | 0,97 / 0,97 / 0,98 | 0,96 / 0,96 / 0,96 | 116 m² |
| Scan matching, 8 partículas, nodo ROS en tiempo real | 5,5 / 5,3 / 5,4 | 17,0 / 14,6 / 20,5 | 0,97 / 0,97 / 0,98 | 0,96 / 0,96 / 0,97 | 116 m² |
| Odometría ajustada, 128 partículas | 7,4 / 7,7 / 6,9 | 19,8 / 23,8 / 18,3 | 0,96 / 0,96 / 0,97 | 0,93 / 0,92 / 0,95 | 116 m² |
| Odometría ajustada, 128 partículas, `+inf` libera (regla vieja) | 7,4 / 8,1 / 7,9 | 20,7 / 24,3 / 23,6 | 0,80 / 0,80 / 0,80 | 0,99 / 0,99 / 0,99 | 325 m² |
| Perfil default con sensor 0,03 / 0,10 (valores viejos) | 6,6 / 6,4 / 6,1 | 21,7 / 19,3 / 17,9 | 0,87 / 0,88 / 0,88 | 0,95 / 0,96 / 0,95 | 124 m² |
| Perfil default con sensor 0,08 / 0,15 (valores actuales) | 5,7 / 6,0 / 5,7 | 20,7 / 20,6 / 19,2 | 0,94 / 0,93 / 0,94 | 0,96 / 0,95 / 0,96 | 120 m² |

En las tres corridas del nodo ROS el tiempo por actualización fue de 0,071 a 0,074 s de mediana, sin cola después del arranque, con 856 scans integrados y 7 descartados en el primer segundo.

## Qué cambia en las decisiones anteriores

- **001 (lecturas `+inf`).** La decisión se mantiene, por el mapa: con la regla vieja el recall de paredes es 0,80 contra 0,96 y el área libre es 325 m² contra 116 m². Lo que no se sostiene es que la regla vieja deformara la trayectoria. Los 28,2 cm de la semilla 2 eran un efecto de la alineación: con mejor ajuste da 7,9 cm.
- **002 (scan matching).** La decisión se mantiene. Scan matching con 8 partículas da 5,4 a 5,8 cm contra 6,9 a 7,7 cm de la propuesta de odometría con 128, con mejor precisión del mapa y unas 6 veces menos tiempo por actualización. La ventaja en precisión es más chica de lo que mostraba la tabla original, y la propuesta de odometría con 128 partículas no tiene una semilla mala.
- **Valores de sensor del default.** Se mantienen. La diferencia está sobre todo en el recall del mapa (0,94 contra 0,88).

Las tablas originales de 001 y 002 quedan como registro. Sirven para comparar configuraciones entre sí, pero sus valores absolutos de RMSE no.

## Pendiente

Las corridas del barrido de partículas, de la ventana y de `scan_match.sigma` de 002 no se pudieron recalcular porque sus salidas ya no están. Para el informe conviene repetir las que se vayan a citar.
