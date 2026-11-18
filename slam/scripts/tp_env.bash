#!/usr/bin/env bash
# Funciones de uso interactivo; este archivo se copia dentro del workspace.

_TP_ENV_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

_tp_env_on() {
  local nounset_enabled=0
  if [[ -n "${VIRTUAL_ENV:-}" && "$VIRTUAL_ENV" != "$_TP_ENV_WS/.venv" ]]; then
    echo "Deactivate the current virtual environment first." >&2
    return 1
  fi
  if [[ ! -x "$_TP_ENV_WS/.venv/bin/python" ]]; then
    echo "Environment is not installed. Run container_env.sh setup on the host." >&2
    return 1
  fi
  if [[ "${_TP_ENV_ACTIVE:-0}" != 1 ]]; then
    # ROS Humble puede consultar variables opcionales al cargar sus overlays.
    [[ $- != *u* ]] || { nounset_enabled=1; set +u; }
    source /opt/ros/humble/setup.bash
    if [[ -f "$_TP_ENV_WS/install/setup.bash" ]]; then
      source "$_TP_ENV_WS/install/setup.bash"
    fi
    (( ! nounset_enabled )) || set -u
    _TP_ENV_OLD_PYTHONPATH_SET="${PYTHONPATH+x}"
    _TP_ENV_OLD_PYTHONPATH="${PYTHONPATH:-}"
    _TP_ENV_OLD_CACHE_SET="${NUMBA_CACHE_DIR+x}"
    _TP_ENV_OLD_CACHE="${NUMBA_CACHE_DIR:-}"
    _TP_ENV_OLD_DISABLE_SET="${SLAM_DISABLE_NUMBA+x}"
    _TP_ENV_OLD_DISABLE="${SLAM_DISABLE_NUMBA:-}"
    _TP_ENV_OLD_JIT_SET="${NUMBA_DISABLE_JIT+x}"
    _TP_ENV_OLD_JIT="${NUMBA_DISABLE_JIT:-}"
    source "$_TP_ENV_WS/.venv/bin/activate"
    # Los console_scripts de colcon conservan /usr/bin/python3 en su shebang.
    # PYTHONPATH permite que esos procesos también vean las ruedas del venv.
    export PYTHONPATH="$_TP_ENV_WS/.venv/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
    export NUMBA_CACHE_DIR="$_TP_ENV_WS/.cache/numba"
    _TP_ENV_ACTIVE=1
  fi
  unset SLAM_DISABLE_NUMBA
  unset NUMBA_DISABLE_JIT
}

tp-env() {
  case "${1:-status}" in
    on) _tp_env_on ;;
    off)
      if [[ "${_TP_ENV_ACTIVE:-0}" == 1 ]]; then
        deactivate
        if [[ -n "$_TP_ENV_OLD_PYTHONPATH_SET" ]]; then
          export PYTHONPATH="$_TP_ENV_OLD_PYTHONPATH"
        else
          unset PYTHONPATH
        fi
        if [[ -n "$_TP_ENV_OLD_CACHE_SET" ]]; then
          export NUMBA_CACHE_DIR="$_TP_ENV_OLD_CACHE"
        else
          unset NUMBA_CACHE_DIR
        fi
        if [[ -n "$_TP_ENV_OLD_DISABLE_SET" ]]; then
          export SLAM_DISABLE_NUMBA="$_TP_ENV_OLD_DISABLE"
        else
          unset SLAM_DISABLE_NUMBA
        fi
        if [[ -n "$_TP_ENV_OLD_JIT_SET" ]]; then
          export NUMBA_DISABLE_JIT="$_TP_ENV_OLD_JIT"
        else
          unset NUMBA_DISABLE_JIT
        fi
        unset _TP_ENV_ACTIVE
      fi
      ;;
    numpy)
      _tp_env_on || return
      export SLAM_DISABLE_NUMBA=1
      ;;
    status)
      python3 - <<'PY'
import os
import sys
import numpy
import scipy
import yaml

print(f"Python: {sys.executable}")
print(f"Virtual environment: {os.environ.get('VIRTUAL_ENV', 'inactive')}")
print(f"NumPy: {numpy.__version__}; SciPy: {scipy.__version__}; PyYAML: {yaml.__version__}")
try:
    import numba
    import llvmlite
    print(f"Numba: {numba.__version__}; llvmlite: {llvmlite.__version__}")
except ImportError:
    print("Numba: not installed")
try:
    from slam.core.acceleration import BACKEND
    print(f"SLAM backend: {BACKEND}")
except ModuleNotFoundError as error:
    if error.name not in ('slam', 'slam.core', 'slam.core.acceleration'):
        raise
    print("SLAM backend: unavailable; sync and build the current slam package")
print(f"JIT cache: {os.environ.get('NUMBA_CACHE_DIR', 'default')}")
PY
      ;;
    *)
      echo "Usage: tp-env {on|off|numpy|status}" >&2
      return 2
      ;;
  esac
}

_tp_env_setup() {
  set -e
  local env_dir="$_TP_ENV_WS/.venv"
  if [[ -e "$env_dir" && ! -f "$env_dir/.tp_final_env" ]]; then
    echo "Refusing to modify an unmanaged environment: $env_dir" >&2
    return 1
  fi
  /usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), "Python 3.10 required"'
  # --without-pip funciona incluso si Ubuntu no incluye ensurepip.
  /usr/bin/python3 -m venv --system-site-packages --without-pip --prompt tp-final "$env_dir"
  touch "$env_dir/.tp_final_env" "$env_dir/COLCON_IGNORE"
  PYTHONPATH="$_TP_ENV_WS/.tp_env/pip-24.0-py3-none-any.whl" \
    "$env_dir/bin/python" -m pip install --no-index --no-deps \
    --find-links "$_TP_ENV_WS/.tp_env" pip==24.0
  "$env_dir/bin/python" -m pip install --no-index --no-deps \
    --find-links "$_TP_ENV_WS/.tp_env" \
    -r "$_TP_ENV_WS/.tp_env/requirements-acceleration-humble.txt"
  mkdir -p "$_TP_ENV_WS/.cache/numba"
  local startup_line
  printf -v startup_line 'source %q' "$_TP_ENV_WS/.tp_env/scripts/tp_env.bash"
  if ! grep -Fxq "$startup_line" "$HOME/.bashrc"; then
    printf '\n# Entorno del TP Final (ROS Humble + Numba).\n%s\n' "$startup_line" >> "$HOME/.bashrc"
  fi
  _tp_env_on
  # Sólo imports; no se inicializan nodos ni se ejecuta un replay.
  /usr/bin/python3 -c 'import numba, llvmlite, rclpy; assert numba.__version__ == "0.57.1"; assert llvmlite.__version__ == "0.40.1"'
  tp-env status
  echo 'Ready. New container shells enable this environment. Use tp-env off to leave it.'
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  case "${1:-}" in
    setup) _tp_env_setup ;;
    *) echo "Usage: tp_env.bash setup (inside the container)" >&2; exit 2 ;;
  esac
else
  _tp_env_on
fi
