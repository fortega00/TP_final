# Sesión con el robot real, 2026-11-16

Datos crudos de la sesión en el laboratorio con el RosMaster X3. Plan seguido: [robot_real_checklist.md](../../docs/robot_real_checklist.md). Se completaron los pasos 2, 5 y 6 (verificar datos, grabar bags, construir el mapa). Los pasos 7 a 10 no se hicieron en esta tanda.

Nada de esto está comparado contra medidas con cinta: los mapas se revisaron solo a ojo.

## Entorno

- PC del laboratorio con ROS 2 Humble nativo. Workspace propio en `~/ivnafra/ws` con `slam` y `nav` enlazados.
- Robot en `172.23.16.78`, `ROS_DOMAIN_ID=11`. El SLAM corrió en la PC, con los datos por wifi.
- `scripts/env.sh` prepara una terminal; `scripts/session.sh` arranca y baja bag, SLAM, RViz y log de estado. Las rutas de los dos son las de esa PC.

## Lo medido en el paso 2

| Dato | Valor |
|---|---|
| `/scan` | 7,3 Hz, 1080 haces, frame `laser_link`, QoS best-effort |
| `/odom` | 9,9 Hz, `frame_id: odom`, `child_frame_id: base_footprint` |
| TF `odom → base_footprint` | 10 Hz |
| TF `base_footprint → laser_link` | x = 0,043 m, z = 0,181 m, yaw 180° (del URDF, sin verificar con cinta) |
| Scan de muestra | 933 haces válidos, 147 `inf`, mínimo 0,42 m |
| Diferencia de reloj PC y robot | 0,16 a 0,33 s |
| Batería al empezar | 10,6 V |

La base es `base_footprint`, no `base_link` como dicen los comandos del checklist.

## Corridas

Todas con `slam/config/parcial_bag_scan_match.yaml`, `mode:=real`, `odom_source:=topic`, `base_frame:=base_footprint`.

| Bag | Mapa | Duración | Scans | Scans integrados | Notas |
|---|---|---|---|---|---|
| `bags/bag_intento_descartado` | no tiene | 109 s | 816 | | El robot se movió sin querer y había dos nodos SLAM a la vez. Sin `/cmd_vel` |
| (sin bag, se regrabó encima) | `mapa/intento1` | | | 200 | Cobertura parcial, mapa torcido. Descartado |
| `bags/bag_mapa` | `mapa/intento2` | 734 s | 5381 | 1104 | Recorrido completo. Paredes gruesas, contorno algo inclinado |
| `bags/bag_mapa2` | `mapa/intento3` | 475 s | 3614 | 962 | Recorrido completo. Paredes rectas y finas. El elegido para la etapa B |

En las dos corridas completas: `update_seconds` entre 0,08 y 0,09 s, cero scans descartados, cero en cola y ninguna partícula fuera del mapa. Zona libre 108,5 m² y 108,2 m². Los dos mapas están girados 180° entre sí porque el robot arrancó mirando para lados opuestos.

Cada carpeta de mapa tiene `aula.pgm`, `aula.yaml`, `aula.json` (configuración usada) y `status.log` (`/slam/status` de la corrida). `aula_vista.png` es el mapa recortado y ampliado.

Los bags se grabaron en la PC por wifi, con `/scan` best-effort: puede faltar algún scan.
