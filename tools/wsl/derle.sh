#!/usr/bin/env bash
# =============================================================================
# ROS 2 paketlerini derle (WSL / Linux)
# =============================================================================
# Çalışma alanı WSL'in kendi diskinde (~/hexapod_ws) kurulur; kaynaklar
# depodaki src/ paketlerine sembolik bağlantıyla bağlanır. Neden depo
# klasöründe derlenmiyor: depo OneDrive'da; build/ install/ log/ OneDrive'a
# senkronlanır ve /mnt/c üzerinden derleme yavaştır.
#
# --symlink-install: Python dosyalarında yaptığınız değişiklik yeniden
# derlemeden görünür. Yeni dosya/paket eklenince betiği tekrar çalıştırın.
#
# Kullanım (depo klasöründen):
#     bash tools/wsl/derle.sh
# Sonra her yeni terminalde:
#     source ~/hexapod_ws/install/setup.bash
# =============================================================================

set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${HEXAPOD_WS:-$HOME/hexapod_ws}"

[ -f /opt/ros/lyrical/setup.bash ] || { echo "HATA: ROS 2 Lyrical yok. Önce: bash tools/wsl/ros_kurulum.sh" >&2; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash

mkdir -p "$WS/src"
# Depodaki her ROS paketi (package.xml olan klasör) için bağlantı.
for pkg in "$REPO"/src/*/; do
    [ -f "$pkg/package.xml" ] || continue
    name="$(basename "$pkg")"
    ln -sfn "${pkg%/}" "$WS/src/$name"
done
# Depodan silinmiş paketlerin kırık bağlantılarını temizle.
find "$WS/src" -maxdepth 1 -xtype l -delete

cd "$WS"
echo "==> derleniyor: $WS  (kaynak: $REPO/src)"
colcon build --symlink-install --event-handlers console_cohesion+ "$@"

# ~/.bashrc'ye ekle (bir kez)
satir="source $WS/install/setup.bash"
grep -qxF "$satir" ~/.bashrc || echo "$satir" >> ~/.bashrc

echo
echo "TAMAM. Bu terminalde kullanmak için:  source $WS/install/setup.bash"
echo "(yeni açılan terminallerde otomatik)"
