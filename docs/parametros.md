# Parámetros del SLAM y cómo ajustarlos

## 6. Hiperparámetros del filtro

Crear una copia de `config/default.yaml` y pasar su ruta absoluta con `config_file:=/ruta/a/config.yaml`. Es un YAML del núcleo con grupos `map`, `sensor`, `motion`, `filter`, `update` y `scan_match` (opcional, ver `docs/decisiones/002-propuesta-con-scan-matching.md`), compartido por CLI y ROS. Los campos desconocidos se rechazan para evitar que un error de escritura deje un parámetro sin aplicar. Ejemplo de una configuración parcial válida:

```yaml
map:
  resolution: 0.05
  width: 16.0
  height: 16.0
  origin_x: -8.0
  origin_y: -8.0
sensor:
  p_hit: 0.70
  p_free: 0.35
filter:
  particles: 24
  seed: 7
```

Los campos omitidos toman los valores de las tablas. Son un punto de partida para el toy, no una calibración del robot real.

Los valores finales se eligieron después de comparar alternativas en el toy. La corrida base de semilla 7 obtuvo precision de obstáculos de 0,9032, recall de 1,0 —con tolerancia de una celda de 5 cm— y RMSE de posición de 2,46 cm. El detalle, las variantes que fallaron y las limitaciones están en [el informe](entrega_a.md) y en `artifacts/tuning`. La odometría de ese toy ya es precisa: su RMSE fue 2,01 cm, menor que el de SLAM; no se presenta como una mejora general de localización.

### Geometría y prior

| Campo | Default | Unidad y efecto |
| --- | ---: | --- |
| `map.resolution` | `0.05` | m/celda. Permite representar puertas y paredes finas con más detalle que 10 cm. |
| `map.width` | `16.0` | Ancho en metros, redondeado hacia arriba a celdas completas. |
| `map.height` | `16.0` | Alto en metros. |
| `map.origin_x` | `-8.0` | Coordenada x del extremo inferior izquierdo de la grilla, en m. |
| `map.origin_y` | `-8.0` | Coordenada y del extremo inferior izquierdo, en m. |
| `map.prior` | `0.5` | Probabilidad inicial; la máscara de observación mantiene unknown separado. |

La extensión es fija. Antes de mapear un entorno mayor, ampliar dimensiones y elegir el origen de modo que la trayectoria y las paredes quepan. No se rellena automáticamente el exterior ni se usa la frontera como obstáculo. El consumo de grillas crece aproximadamente con `partículas × ancho × alto / resolución²`.

### Modelo inverso del sensor

| Campo | Default | Unidad y efecto |
| --- | ---: | --- |
| `sensor.p_hit` | `0.70` | Probabilidad inversa de ocupación para un impacto. |
| `sensor.p_free` | `0.35` | Probabilidad inversa de ocupación para espacio atravesado. |
| `sensor.log_odds_min` | `-4.0` | Límite inferior de evidencia; corresponde aproximadamente a p=0,018. |
| `sensor.log_odds_max` | `4.0` | Límite superior; corresponde aproximadamente a p=0,982. |
| `sensor.hit_thickness` | `0.03` | Espesor radial total de la zona de impacto, en m. |
| `sensor.min_range` | `0.12` | Rango mínimo utilizable, en m, combinado con el del mensaje. |
| `sensor.max_range` | `8.0` | Alcance máximo de mapeo, en m, combinado con el del mensaje. |
| `sensor.max_beams` | `180` | Máximo de haces integrados, seleccionados a lo largo del campo angular. |

Se exige `0 < p_free < prior < p_hit < 1`. La actualización suma el logit de la probabilidad inversa **menos el logit del prior** y luego satura. Una observación de la misma celda no se acumula decenas de veces sólo porque varios haces del mismo scan la atraviesen; se deduplican las celdas por lectura y el impacto tiene precedencia sobre evidencia libre de esa lectura.

Las lecturas `NaN`, `-inf`, no positivas, demasiado cercanas o finitas fuera del rango físico declarado se descartan. Un `+inf` representa ausencia de retorno. Por defecto (`sensor.no_return_is_free: false`) no aporta evidencia, porque en el lidar real un `+inf` aparece con vidrio, superficies oscuras o rasantes y no garantiza espacio libre. Con `no_return_is_free: true` (perfil de Gazebo, `cafe.yaml`) libera el tramo observable sin crear un obstáculo en el extremo. Un valor finito igual al alcance útil libera el tramo y tampoco crea obstáculo. Si se limita el alcance configurado por debajo de un retorno válido, sólo se usa evidencia libre hasta ese límite.

### Modelo de likelihood

| Campo | Default | Unidad y efecto |
| --- | ---: | --- |
| `sensor.likelihood_beams` | `60` | Máximo de extremos usados para ponderar partículas. |
| `sensor.sigma_hit` | `0.10` | Desvío espacial, en m, de la concordancia con obstáculos. |
| `sensor.z_hit` | `0.95` | Peso de la componente de impacto. |
| `sensor.z_rand` | `0.05` | Peso de la componente uniforme; debe ser positivo. |
| `sensor.likelihood_max_dist` | `0.60` | Truncamiento de distancia al obstáculo, en m. |
| `sensor.likelihood_temperature` | `1.0` | Multiplica la suma de log-likelihoods y permite moderar la confianza. |
| `sensor.occupied_threshold` | `0.65` | Umbral para considerar una celda obstáculo en la ponderación. |

La mezcla exige `z_hit + z_rand = 1`. Para un impacto a distancia `d` del obstáculo más próximo se usa una puntuación proporcional a `z_hit·exp(-d²/(2·sigma_hit²)) + z_rand/rango_máximo`. Se evalúa sobre el mapa de cada partícula antes de integrar el scan. El primer scan arranca sin obstáculos previos y no discrimina hipótesis por concordancia artificial con su propio mapa.

Reducir `sigma_hit` o aumentar cantidad de haces/temperatura vuelve la ponderación más selectiva; con odometría ruidosa puede concentrar el peso en una hipótesis equivocada. Aumentarlos en sentido opuesto puede suavizar diferencias hasta que las partículas casi no se corrijan con el LIDAR.

### Movimiento, partículas y actualización

| Campo | Default | Unidad y efecto |
| --- | ---: | --- |
| `motion.alpha1` | `0.0001` | Coeficiente de varianza del ruido de traslación por traslación. |
| `motion.alpha2` | `0.00001` | Varianza de traslación por rotación. |
| `motion.alpha3` | `0.0001` | Varianza de rotación por rotación. |
| `motion.alpha4` | `0.0001` | Varianza de rotación por traslación. |
| `filter.particles` | `24` | Cantidad fija de hipótesis; aumenta costo y diversidad. |
| `filter.seed` | `7` | Semilla entera no negativa. |
| `filter.resample_ess_ratio` | `0.5` | Remuestreo si ESS es menor que esta fracción de N. |
| `filter.initial_x` | `0.0` | Posición inicial del robot en `map`, en m. |
| `filter.initial_y` | `0.0` | Posición inicial y, en m. |
| `filter.initial_yaw` | `0.0` | Orientación inicial en radianes. |
| `filter.initial_std_xy` | `0.0` | Desvío de muestreo inicial por eje, en m. |
| `filter.initial_std_yaw` | `0.0` | Desvío angular inicial, en radianes. |
| `update.min_translation` | `0.08` | Desplazamiento neto mínimo desde el último scan integrado, en m. |
| `update.min_rotation` | `0.06` | Giro neto mínimo desde el último integrado, en radianes. |
| `update.max_interval` | `1.0` | Máximo intervalo entre integraciones, en segundos de datos. |

Los coeficientes `alpha` parametrizan **varianzas**, no desvíos estándar. Para un incremento relativo `(dx,dy,dtheta)`, la varianza por eje de traslación es `alpha1·(dx²+dy²) + alpha2·dtheta²`; la angular es `alpha3·dtheta² + alpha4·(dx²+dy²)`. El modelo conserva movimiento lateral; no supone una base exclusivamente diferencial. Se integra un scan al superar cualquiera de los umbrales de desplazamiento/giro/tiempo. Los scans omitidos no reinician la referencia, de modo que movimientos pequeños pueden acumularse hasta superar el umbral. La semilla no compensa datos perdidos o diferencias en el orden temporal de ROS.

## 7. Cómo ajustar y evaluar

1. Verificar unidades, frames, timestamps, extrínseca y odometría antes de tocar la likelihood. Una extrínseca equivocada genera paredes dobles que no se arreglan aumentando partículas.
2. Elegir extensión y resolución para cubrir el mundo completo y las puertas relevantes. Mantener la referencia geométrica y el recorrido cuando se comparan resoluciones.
3. Estimar ruido y deriva de la odometría, incluyendo desplazamientos laterales, giros y retroceso; ajustar `alpha` para representar la incertidumbre observada.
4. Ajustar el modelo inverso y el espesor de impacto sin ocultar aperturas reales. Una probabilidad de ocupación extrema satura rápido y dificulta corregir falsos obstáculos.
5. Ajustar `sigma_hit`, haces y temperatura observando ESS, pérdida de diversidad, consistencia geométrica y costo. Probar más de una semilla.
6. Guardar parámetros, bag/trayectoria, mapa y métricas de cada corrida. Comparar precisión/recall de paredes con tolerancia métrica, error de trayectoria cuando exista ground truth, ancho de puertas y conectividad después de inflar por el robot.

Para B, comprobar caminos sobre celdas conocidas libres e inflar obstáculos por la huella y un margen. Las métricas de conectividad del toy hacen una comprobación conservadora con disco; no sustituyen el planificador ni certifican cualquier ruta del entorno real. La zona desconocida no debe convertirse en libre para conseguir que una ruta exista.
