#!/usr/bin/env bash
# =============================================================================
# RL eğitim ortamı: Python sanal ortamı + PyTorch (CPU) + Stable-Baselines3
# =============================================================================
# Sistem Python'una dokunmaz (Ubuntu 24.04+ buna zaten izin vermiyor,
# PEP 668); her şey ~/hexapod_venv içine kurulur. sudo GEREKMEZ.
#
# --system-site-packages: numpy, yaml ve ROS'un gz.sim Python bağları
# (source /opt/ros/lyrical/setup.bash ile gelir) sanal ortamdan görünsün.
#
# PyTorch'un CPU sürümü: politika küçük bir MLP, Stable-Baselines3 PPO için
# CPU'yu öneriyor; CUDA sürümü birkaç GB, CPU sürümü ~200 MB.
# Sürümler sabit: Görkem ve Samet'in ortamları aynı olsun.
#
# Kullanım (depo klasöründen):
#     bash tools/wsl/rl_kurulum.sh
# Sonra her terminalde:
#     source ~/hexapod_venv/bin/activate
# =============================================================================

set -eo pipefail

VENV="${HEXAPOD_VENV:-$HOME/hexapod_venv}"
TORCH="torch==2.14.0"
SB3="stable-baselines3==2.9.0"
GYM="gymnasium==1.3.0"

if [ ! -x "$VENV/bin/pip" ]; then
    rm -rf "$VENV"
    echo "==> sanal ortam: $VENV"
    if ! python3 -m venv --system-site-packages "$VENV" >/dev/null 2>&1; then
        # python3-venv (ensurepip) kurulu değilse: sudo istemeden, pip'siz
        # ortam + PyPA'nın resmi get-pip.py'si. (ros_kurulum.sh python3-venv
        # kurar; bu yol yalnızca onsuz kurulmuş makineler için.)
        rm -rf "$VENV"
        echo "    python3-venv yok; pip get-pip.py ile kuruluyor"
        python3 -m venv --system-site-packages --without-pip "$VENV"
        curl -fsSL https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
        "$VENV/bin/python" /tmp/get-pip.py --quiet
    fi
fi
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -m pip install --quiet --upgrade pip

echo "==> PyTorch (CPU) — ~200 MB"
python -m pip install --quiet "$TORCH" --index-url https://download.pytorch.org/whl/cpu
echo "==> Stable-Baselines3 + Gymnasium"
python -m pip install --quiet "$SB3" "$GYM"

echo "==> kontrol"
python - <<'EOF'
import gymnasium, stable_baselines3, torch
print(f"torch {torch.__version__} (CUDA: {torch.cuda.is_available()}), "
      f"stable-baselines3 {stable_baselines3.__version__}, gymnasium {gymnasium.__version__}")
EOF

echo
echo "TAMAM. Kullanmak için:  source $VENV/bin/activate"
