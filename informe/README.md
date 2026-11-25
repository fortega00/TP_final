# Informe del TP Final

Un solo informe para las partes A, B y C, escrito en LaTeX.

## Archivos

| Archivo | Contenido |
|---|---|
| `informe.tex` | Preámbulo, título y autores. Incluye las tres partes |
| `parte_a.tex`, `parte_b.tex`, `parte_c.tex` | El texto de cada parte |
| `figuras/` | Las imágenes que usa el informe |
| `scripts/` | Los scripts que corren los experimentos y generan las figuras |
| `datos/` | Entradas (bags del aula ya convertidos a `.npz`) y resultados de los experimentos |
| `build.sh` | Compila y deja `informe.pdf` en esta carpeta |

`informe.pdf` e `informe.log` no se suben a git mientras se escribe (están en `.gitignore`).

## Compilar

Se usa [tectonic](https://tectonic-typesetting.github.io): un solo ejecutable que descarga solo los paquetes de LaTeX que hacen falta. No hay que instalar TeX Live.

**Linux**, una sola vez:

```bash
curl --proto '=https' --tlsv1.2 -fsSL https://drop-sh.fullyjustified.net | sh
sudo mv tectonic /usr/local/bin/
tectonic --version
```

El primer comando deja el ejecutable `tectonic` en la carpeta actual y el segundo lo pone en el `PATH`. Con conda también sirve `conda install -c conda-forge tectonic`. En macOS, `brew install tectonic`.

**Generar el PDF**, desde la raíz del repo:

```bash
./informe/build.sh
```

El resultado queda en `informe/informe.pdf`. La primera vez tarda unos minutos porque descarga paquetes y fuentes, y necesita internet. Si se queda trabada, cortar con Ctrl+C y volver a correr. Después tarda segundos.

Los fuentes son LaTeX estándar. Quien prefiera Overleaf puede subir esta carpeta y compilar `informe.tex` con pdfLaTeX. Esa vía no está probada.

## Rehacer experimentos y figuras

Hace falta solo para cambiar una figura o un número. No usa ROS. Desde la raíz del repo, con las dependencias de `slam/requirements-evaluation.txt` y `nav/requirements.txt` instaladas:

```bash
PYTHONPATH=slam python informe/scripts/experimentos_a.py    # unos 30 minutos, 6 procesos
PYTHONPATH=slam python informe/scripts/experimentos_a.py --workers 1 --seeds 7 \
  --output informe/datos/tiempos_a                          # tiempos, una corrida por vez
python informe/scripts/analisis_a.py                        # escribe datos/resumen_a.csv y tiempos_a.csv
python informe/scripts/etapas_a.py                          # error en simulación y en el bag del Parcial
PYTHONPATH=slam:nav python informe/scripts/localizacion_a.py  # localización de la parte B sobre el mapa final
PYTHONPATH=slam:nav python informe/scripts/figuras_a.py     # escribe figuras/a_*.png
python informe/scripts/medidas_aula.py                      # plano de medidas con cinta
```

Las corridas quedan en `datos/experimentos_a/`, que no se sube a git por tamaño.

## Editar

- El texto de cada parte va en su `parte_X.tex`. Para cambiar un párrafo alcanza con editar ese archivo y volver a correr `build.sh`.
- Una figura nueva se guarda en `figuras/` y se incluye con su nombre, sin la ruta:

  ```latex
  \begin{figure}[t]
    \centering
    \includegraphics[width=\columnwidth]{mapa_aula}
    \caption{Mapa del aula.}
    \label{fig:mapa_aula}
  \end{figure}
  ```

- Cada sección termina con `\FloatBarrier` para que sus figuras no pasen a la sección siguiente.
- `\code{nombre}` es para nombres de archivos, tópicos y parámetros.
- Si la compilación falla, el error con su número de línea está en `informe.log`.

## Estilo

- Español, primera persona del plural ("elegimos", "medimos").
- Formal y directo. Sin preguntas retóricas y sin rayas largas.
- Los términos habituales del área quedan en inglés: "scan matching", "resampling", "lookahead".
- Cada número del informe sale de una corrida guardada en el repo.
