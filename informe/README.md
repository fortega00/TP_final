# Informe del TP Final

Un solo informe para las partes A, B y C, escrito en LaTeX.

## Archivos

| Archivo | Contenido |
|---|---|
| `informe.tex` | Preámbulo, título y autores. Incluye las tres partes |
| `parte_a.tex`, `parte_b.tex`, `parte_c.tex` | El texto de cada parte |
| `figuras/` | Las imágenes que usa el informe |
| `build.sh` | Compila y deja `informe.pdf` en esta carpeta |

`informe.pdf` e `informe.log` no se suben a git mientras se escribe (están en `.gitignore`).

## Compilar

Se usa [tectonic](https://tectonic-typesetting.github.io): un solo ejecutable que descarga los paquetes de LaTeX que hacen falta la primera vez. No hace falta instalar TeX Live.

Instalación, una sola vez:

```bash
# macOS
brew install tectonic

# Linux, con conda
conda install -c conda-forge tectonic

# Linux, sin conda: deja el ejecutable `tectonic` en la carpeta actual
curl --proto '=https' --tlsv1.2 -fsSL https://drop-sh.fullyjustified.net | sh
sudo mv tectonic /usr/local/bin/
```

Compilar:

```bash
./informe/build.sh
```

La primera compilación tarda unos minutos porque descarga los paquetes. Las siguientes tardan segundos.

Los fuentes son LaTeX estándar. Quien prefiera Overleaf o TeX Live puede subir esta carpeta y compilar `informe.tex` con pdfLaTeX. Esa vía no está probada.

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
- `\resp{la pregunta 3 de la parte A}` agrega una nota gris que indica qué pregunta de la consigna responde un párrafo.
- `\code{nombre}` es para nombres de archivos, tópicos y parámetros.
- Si la compilación falla, el error con su número de línea está en `informe.log`.

## Estilo

- Español, primera persona del plural ("elegimos", "medimos").
- Formal y directo. Sin preguntas retóricas y sin rayas largas.
- Los términos habituales del área quedan en inglés: "scan matching", "resampling", "lookahead".
- Cada número del informe sale de una corrida guardada en el repo.
