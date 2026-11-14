# 005. Arquitectura de nodos de la etapa B

Estado: decidida el 2026-11-14. Sin implementar, hoy solo existe el núcleo de localización.

## Problema

La etapa B junta localización, planificación, seguimiento del camino, evasión y una máquina de estados. Hay que decidir en cuántos nodos ROS se reparte.

## Alternativas

| Opción | A favor | En contra |
|---|---|---|
| A. Dos nodos, `localization` y `navigator` | La localización se prueba sola y sirve igual para la etapa C. El planificador y el seguidor son módulos de Python sin ROS, con tests propios. Es el patrón de orquestador de la Parte 0 | Dos procesos para lanzar |
| B. Un solo nodo | El launch más simple | Una actualización lenta del filtro frena el lazo de control. Más difícil de probar |
| C. Cuatro nodos, al estilo Nav2 | Lo más parecido a la práctica de la industria | Mucha conexión de tópicos y tiempos sin beneficio para la evaluación |

## Decisión

Opción A. El nodo `localization` recibe `/scan`, la odometría y `/initialpose`, y publica la pose en el mapa y el TF `map → odom`. El nodo `navigator` tiene la máquina de estados ([009](009-maquina-de-estados.md)) y llama al planificador y al seguidor como funciones.

Igual que en `slam/`, los algoritmos van en `nav/nav/core/` sin importar ROS y los nodos en `nav/nav/ros/`.

## Datos

Pendiente: tiempo por actualización de cada nodo en la computadora del robot.
