# 001. Lecturas sin retorno (`+inf`) del lidar

Estado: implementada en la rama `fix/slam-review` (2026-11-11).

## Problema

El lidar real devuelve `+inf` cuando no recibe eco. En el bag del Parcial eso pasa en el 10,6 % de las lecturas. El código trataba cada `+inf` como espacio libre hasta `max_range` (8 m). En el robot real un `+inf` no garantiza que el rayo esté vacío: aparece con vidrio, superficies oscuras o rasantes, y también fuera de alcance. Entonces el mapa borraba paredes reales y marcaba como libre zonas fuera de la sala.

En Gazebo es distinto: el lidar simulado devuelve `+inf` solo cuando no hay nada hasta el alcance máximo, o sea que ahí sí es espacio libre.

## Alternativas

| Opción | A favor | En contra |
|---|---|---|
| A. `+inf` siempre libera (lo que había) | Mapas de simulación con más área libre | Borra paredes reales en el robot |
| B. `+inf` nunca aporta evidencia | La más simple, segura en cualquier sensor | En simulación no marca como libres las zonas abiertas más allá del alcance |
| C. Liberar solo un tramo corto (2 a 3 m) | Compromiso | Sigue borrando paredes si el no retorno viene de un vidrio cercano, y agrega un umbral para ajustar |
| D. Flag por perfil, por defecto no aporta evidencia | Fiel a los dos sensores, un solo campo de config | Un parámetro más |

## Decisión

Opción D. El campo `sensor.no_return_is_free` vale `false` por defecto (robot real y bag) y `true` en `config/cafe.yaml` (Gazebo). Las lecturas finitas más allá del alcance útil siguen liberando hasta ese límite, porque ahí sí hubo un eco.

## Datos

Usé como referencia independiente el mapa del laboratorio de los profesores (`my_map3`) y mi localización del TP3 sobre ese mapa, con el mismo bag. Ninguno de los dos entra al SLAM.

Mapeo con poses conocidas (aísla el modelo de mapeo del filtro):

| Regla | Recall de paredes a 10 cm | Precisión a 10 cm | Área libre fuera de la referencia |
|---|---|---|---|
| `+inf` libera | 0,80 | 0,99 | 229 m² |
| `+inf` ignorado | 0,97 | 0,97 | 2 m² |

SLAM completo con `parcial_bag_tuned.yaml`, semillas 7 / 1 / 2:

| Regla | RMSE de posición (cm) | Recall a 10 cm | Precisión a 10 cm | Área libre (referencia 121 m²) |
|---|---|---|---|---|
| `+inf` libera | 10,5 / 11,8 / 28,2 | 0,80 / 0,76 / 0,53 | 0,99 / 0,96 / 0,73 | 325 m² |
| `+inf` ignorado | 9,5 / 12,8 / 7,9 | 0,96 / 0,88 / 0,98 | 0,91 / 0,84 / 0,95 | 116 m² |

Corrección del 2026-11-14: esta tabla alinea la trayectoria con una sola pose de referencia, y eso infla el RMSE y arrastra al recall y a la precisión (ver [003](003-metrica-de-evaluacion.md)). Los 28,2 cm de la semilla 2 con la regla vieja eran ese efecto, la trayectoria no estaba deformada. Con el mejor ajuste rígido de toda la trayectoria:

| Regla | RMSE de posición (cm) | Recall a 10 cm | Precisión a 10 cm | Área libre (referencia 121 m²) |
|---|---|---|---|---|
| `+inf` libera | 7,4 / 8,1 / 7,9 | 0,80 / 0,80 / 0,80 | 0,99 / 0,99 / 0,99 | 325 m² |
| `+inf` ignorado | 7,4 / 7,7 / 6,9 | 0,96 / 0,96 / 0,97 | 0,93 / 0,92 / 0,95 | 116 m² |

La regla casi no cambia la trayectoria. Cambia el mapa: con la regla vieja se pierde el 20 % de las paredes y aparecen 200 m² de espacio libre que no existe. Por eso se mantiene la decisión.

## Pendiente

No medí cuánta área libre se pierde en simulación con la opción B, porque el perfil de Gazebo conserva el comportamiento anterior.
