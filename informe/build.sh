#!/usr/bin/env bash
# Compila el informe con tectonic y deja informe.pdf en esta carpeta.
set -euo pipefail
cd "$(dirname "$0")"
tectonic --keep-logs informe.tex
echo "listo: $(pwd)/informe.pdf"
