# Entrega de la parte A

Qué pide la consigna de la parte A (`consignas/PRA_TPFinal_Parte_A.pdf`, sección 4) y dónde está cada cosa. Actualizado el 2026-11-25.

El informe de la parte A está en [`informe/`](../informe/README.md), en LaTeX, junto con los de las partes B y C. Este archivo contenía una versión anterior de las diez respuestas, escrita antes del scan matching y de la sesión en el laboratorio, con resultados del mundo sintético y 24 partículas. Esa versión queda en el historial de git (hasta el commit `250ec46`) y no debe citarse.

## Entregables

| Qué pide la consigna | Dónde está | Estado |
|---|---|---|
| Paquetes de ROS creados | `slam/` (paquete `ament_python`) | Hecho |
| Launch files configurados | `slam/launch/slam.launch.py`, un launch para toy, simulación, bag y robot real | Hecho |
| Documentación clara para ejecutarlos | [`guia_ejecucion.md`](guia_ejecucion.md) y [`parametros.md`](parametros.md) | Hecho |
| Mapa del espacio real exportado | `sesiones/20261116/mapa/intento3_corregido/` (`map.pgm`, `map.yaml` y la configuración que lo generó) | Hecho |
| Informe: método elegido, modificaciones al filtro, dificultades | `informe/parte_a.tex`, secciones 1.1 a 1.3 | Borrador completo |
| Informe: verificación del mapa (criterios 3.1 a 3.3) | `informe/parte_a.tex`, sección 1.4 | Borrador completo, sin medidas con cinta |
| Informe: las diez preguntas | `informe/parte_a.tex`, sección 1.5 | Borrador completo |

La consigna de la parte A no pide video. El formato de entrega (un zip, el repositorio o el PDF aparte) no figura en el PDF.

## Pendiente

- Medir con cinta las ocho paredes marcadas en `informe/figuras/medidas_aula.png` y sacar las dos fotos, para comparar el mapa con el laboratorio real.
- Revisión del informe por los tres integrantes.

## Cómo se obtuvieron los números del informe

Todos salen de repetir offline los bags grabados, con los scripts de `informe/scripts/` y los datos de `informe/datos/`. Los comandos están en [`informe/README.md`](../informe/README.md).
