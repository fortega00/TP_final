# 018. Perfil real con 8 partículas

Estado: adoptada el 2026-11-25. Reemplaza a la [013](013-perfil-real-e16.md).

## Problema

La 013 puso 16 partículas y `sample_xy` 0,005 (E16) en `slam/config/parcial_bag_scan_match.yaml`. Sus métricas eran de una sola semilla y anteriores a la compuerta solo por movimiento (012), y la propia 013 lo marcaba como pendiente. Mientras tanto el mapa final (`sesiones/20261116/mapa/intento3_corregido`) y el informe usan 8 partículas y `sample_xy` 0,01, así que el perfil que indica la guía no era el que generó el mapa entregado.

## Datos

Replay offline de los dos bags del laboratorio con el código actual, 6 semillas por caso (`informe/scripts/experimentos_a.py`, resumen en `informe/datos/resumen_a.csv`). El tiempo es por actualización en una laptop (Apple M4), con una corrida por vez.

| Caso | Cierre, mediana (cm) | Distancia scan a pared (cm) | Tiempo (ms) |
|---|---|---|---|
| 8 partículas, `sample_xy` 0,01, `bag_mapa2` | 11,7 | 1,1 | 32 |
| 16 partículas, `sample_xy` 0,01, `bag_mapa2` | 11,6 | 1,3 | 65 |
| E16, `bag_mapa2` | 11,9 | 1,1 | no medido aparte |
| 8 partículas, `bag_mapa` | 13,5, con 2 de 6 semillas a más de 50 cm | 1,6 | no medido aparte |
| E16, `bag_mapa` | 10,5, con 1 de 6 semillas a más de 50 cm | 1,6 | no medido aparte |

El robot se estacionó con hasta 10 cm de error, entonces las diferencias de cierre por debajo de ese valor no distinguen un caso de otro.

## Decisión

`parcial_bag_scan_match.yaml` vuelve a 8 partículas y `sample_xy` 0,01, igual que la configuración del mapa final. Sobre el recorrido que generó el mapa, 16 partículas no mejoran nada medible y cuestan el doble. Fran lo acordó por chat el 2026-11-24.

Lo que queda a favor de 16: en `bag_mapa` falla una semilla menos. Con 6 semillas esa diferencia (1 contra 2) no alcanza para afirmar que es más robusto. Si un mapeo nuevo sale mal con 8, probar 16 es el primer cambio a intentar.
