# Evidencia existente sobre ruido de odometría

Revisión de archivos del 18/11/2026. No se ejecutaron nuevos filtros, replays,
pruebas ROS ni simuladores durante esta revisión.

## Dos perturbaciones diferentes

1. Cambiar la odometría de entrada: simular medidas más ruidosas o sesgadas.
2. Cambiar la incertidumbre asumida por el filtro: aumentar `motion.alpha*`,
   modificar el prior del matcher o su dispersión de muestreo, conservando los datos.

El segundo cambio amplía las hipótesis consideradas; no empeora las lecturas de
entrada. En la propuesta actual con scan matching, `motion.alpha*` interviene
en el arranque y en los casos que vuelven a la propuesta odométrica. La búsqueda
normal depende de `scan_match.prior_*`, `sample_*` y las ventanas. Las pruebas
históricas con la propuesta odométrica no sustituyen una evaluación del perfil D.

## Evidencia primaria persistida

### Odómetro sintético más ruidoso y sesgado

Fuentes: [base](../slam/artifacts/tuning/baseline/metrics.json) y
[odometría ruidosa](../slam/artifacts/tuning/noisy_odometry/metrics.json).
Ambas corridas corresponden al toy anterior sin scan matching, semilla 7,
24 partículas y 118 integraciones; mantienen los mismos `alpha`.

| Magnitud | Base | Odómetro perturbado |
| --- | --- | --- |
| Ruido de traslación de entrada | 0,004 m | 0,020 m |
| Ruido angular de entrada | 0,003 rad | 0,015 rad |
| Factor de escala odométrica | 1,002 | 1,010 |
| RMSE de posición de la odometría | 2,01 cm | 10,23 cm |
| RMSE de posición del SLAM | 2,46 cm | 5,37 cm |
| Precisión de ocupados a una celda (5 cm) | 90,32 % | 67,05 % |
| Recall de ocupados a una celda | 100 % | 97,12 % |
| Criterio de aceptación sintético | Aprueba | Falla precisión |

El filtro redujo el error de la entrada perturbada, pero empeoró respecto del
escenario base y perdió calidad geométrica. Se cambiaron ruido y sesgo juntos:
esta comparación no aísla sus contribuciones individuales.

### Mayor incertidumbre del filtro sobre el bag físico

Fuentes: [screen_64](../slam/artifacts/entrega_a_20261110/screen_64/metrics.json),
[su configuración](../slam/artifacts/entrega_a_20261110/screen_64/config.yaml),
[screen_128](../slam/artifacts/entrega_a_20261110/screen_128/metrics.json) y
[su configuración](../slam/artifacts/entrega_a_20261110/screen_128/config.yaml).
Son replays históricos del núcleo sobre las mismas entradas del bag, sin GT.

| Configuración | Separación inicio–fin | Extremos del scan a ≤10 cm del mapa final |
| --- | --- | --- |
| 64 partículas; alpha `(0,03; 0,003; 0,03; 0,03)` | 2,365 m | 16,19 % |
| 128 partículas; alpha `(0,10; 0,01; 0,10; 0,10)` | 0,880 m | 54,94 % |

Se modificaron partículas y `alpha` simultáneamente, por lo que no es una
ablación exclusiva de ruido. La separación inicio–fin no es error absoluto;
la consistencia de extremos reutiliza las mediciones que hicieron el mapa.
Estos datos muestran ensayos con incertidumbre mayor, no una calibración física
ni una aprobación de calidad del mapa.

### Búsqueda preliminar

[exploratory_search.json](../slam/artifacts/exploratory_search.json) incluye
`alpha` mayores, por ejemplo `(0,0025; 0,0004; 0,0025; 0,0004)`.
El archivo se declara anterior a corregir la discretización de impactos y mezcla
cambios de sensor/movimiento. Se conserva como antecedente; no se usa para
comparar causalmente el código actual.

## Resultados documentados por Iván

La [decisión 002](decisiones/002-propuesta-con-scan-matching.md) registra un
barrido de `alpha` entre 0,003 y 0,1 y una simulación Café Classic con deriva.
Para esa simulación informa RMSE odométrico de 6,67 m, divergencia de la propuesta
odométrica por defecto y resultados de 12,8–16,8 cm con scan matching en tres
semillas. Los ejecutores originales se referencian allí en su repositorio
personal. No se reejecutaron ni se verificaron sus fuentes en esta revisión;
no equivalen a una corrida nueva de Fortress con el perfil D.

Los RMSE del bag de esa decisión usan como referencia otro estimador y tienen
correcciones posteriores de alineación. No se presentan aquí como exactitud GT.

## Qué falta para el perfil Café D

Hay antecedentes de ambas clases de perturbación. La evidencia Café 01 usa
odometría ideal y compara partículas, prior y muestreo; no es un ensayo de
robustez de D con odometría degradada. Una comparación futura debería conservar
scans, trayectoria y semilla, variar una fuente a la vez y medir pose contra GT,
geometría y transitabilidad. No se inició ese experimento.
