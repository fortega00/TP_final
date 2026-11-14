# 010. Métricas de la etapa B

Estado: decidida el 2026-11-14 y corregida el mismo día con los primeros datos de simulación.

## Problema

La pose real del robot solo existe en el simulador. Para comparar simulación y robot real (pregunta 11 del informe) hace falta medir lo mismo en los dos.

## Alternativas

| Opción | A favor | En contra |
|---|---|---|
| A. Conjunto completo: RMSE contra la pose real en simulación, más medidas que funcionen en los dos mundos | Algo se mide igual en simulación y en el robot | Más trabajo de registro |
| B. RMSE en simulación y error de llegada en el robot | Menos métricas | Nada se mide igual en los dos, la comparación queda cualitativa |
| C. Solo el residuo del scan contra el mapa | Una métrica que sirve en todos lados | Indirecta |

Elegí A.

## Qué se mide

| Métrica | Simulación | Robot real | Qué mide |
|---|---|---|---|
| RMSE de posición y de yaw contra la pose real | sí | no | Exactitud de la localización |
| Tiempo e iteraciones desde `/initialpose` hasta converger | sí | sí | Eficiencia de la localización inicial |
| Error al llegar al objetivo | contra la pose real | cinta métrica hasta una marca en el piso | Resultado de punta a punta |
| Repetibilidad en marcas del piso | sí | sí | Dispersión de la estimación al estacionar el robot varias veces sobre la misma marca |
| Trayectoria de la localización contra la del SLAM sobre el mismo bag | sí | sí | Acuerdo a lo largo de todo el recorrido |
| Tiempo de planificación, largo del camino y distancia mínima a obstáculos | sí | sí | Calidad del planificador |
| Tiempo por actualización del filtro | sí | sí | Si alcanza el cómputo del robot |

En Gazebo Classic `/odom` es la pose real, así que el filtro recibe la odometría con deriva de `/calc_odom`.

## Corrección: el residuo del scan no mide exactitud

El plan original usaba el residuo del scan contra el mapa como métrica común, calibrado contra el RMSE en simulación. Lo medí y, con el filtro ya convergido, no sigue al error real: la correlación entre los dos dio 0,41 en un tramo y -0,73 en el otro ([004](004-localizacion-en-mapa-conocido.md)). Lo reemplazan la repetibilidad en marcas del piso y la comparación con el SLAM.

Límites de las dos medidas nuevas:

- la repetibilidad mide precisión, no exactitud. Un corrimiento fijo del mapa no aparece;
- la localización y el SLAM usan el mismo modelo de sensor, entonces pueden coincidir y estar los dos equivocados.

El residuo se sigue registrando. Queda por medir si sirve para separar ubicado de perdido.

## Datos

Pendiente: todas las filas de la tabla, en simulación con mapa propio y en el robot real.
