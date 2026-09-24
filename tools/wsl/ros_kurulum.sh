#!/usr/bin/env bash
# =============================================================================
# ROS 2 Lyrical + Gazebo kurulumu — WSL2, Ubuntu 26.04 (GOREVLER.md: G4)
# =============================================================================
# Ne yapar: resmi ROS 2 Lyrical kurulum adımlarını (docs.ros.org, "Installing
# on Ubuntu") sırayla uygular, üstüne simülasyon için gereken paketleri
# (Gazebo köprüsü, ros2_control, RViz eklem kaydırıcıları) kurar ve sonunda
# kurulumu kendisi test eder.
#
# Kullanım (WSL terminalinde, depo klasöründen):
#     bash tools/wsl/ros_kurulum.sh
#
# sudo şifresini BİR KEZ sorar; şifreyi kendiniz girin. Birkaç GB indirir,
# internet hızına göre 15-40 dakika sürer. Yarıda kesilirse tekrar
# çalıştırmak güvenli: yapılmış adımları atlar.
# =============================================================================

set -eo pipefail

DISTRO=lyrical
LOG=/tmp/ros_kurulum.log

adim() { printf '\n\033[1;36m[%s] %s\033[0m\n' "$1" "$2"; }
tamam() { printf '\033[1;32m    tamam\033[0m\n'; }
hata() {
    printf '\n\033[1;31mHATA: %s\033[0m\n' "$1" >&2
    printf 'Ayrıntı: %s dosyasının son satırları:\n' "$LOG" >&2
    tail -n 15 "$LOG" >&2 || true
    exit 1
}
calistir() { "$@" >>"$LOG" 2>&1 || hata "şu komut başarısız oldu: $*"; }

: >"$LOG"

# --- 0. Ön kontroller ----------------------------------------------------------
. /etc/os-release
[ "${UBUNTU_CODENAME:-}" = "resolute" ] \
    || hata "Bu betik Ubuntu 26.04 (resolute) için. Bulunan: ${PRETTY_NAME:-?}"
[ "$(id -u)" -ne 0 ] \
    || hata "Betiği 'sudo' ile değil, doğrudan çalıştırın: bash tools/wsl/ros_kurulum.sh"

adim 0/7 "sudo şifresi (yalnızca bir kez sorulur)"
sudo -v || hata "sudo şifresi kabul edilmedi."
# Uzun indirmeler sırasında sudo süresi dolmasın.
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &
tamam

# --- 1. Dil ayarı ---------------------------------------------------------------
adim 1/7 "Dil ayarı (UTF-8)"
calistir sudo apt-get update
calistir sudo apt-get install -y locales
calistir sudo locale-gen en_US en_US.UTF-8
calistir sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
tamam

# --- 2. ROS 2 depoları ------------------------------------------------------------
adim 2/7 "ROS 2 paket depoları"
calistir sudo apt-get install -y software-properties-common curl
calistir sudo add-apt-repository -y universe
if dpkg -s ros2-apt-source >/dev/null 2>&1; then
    echo "    zaten ekli, atlandı"
else
    ver=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}' || true)
    [ -n "$ver" ] || hata "ros-apt-source sürümü alınamadı. İnternet bağlantısını kontrol edin."
    calistir curl -fL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${UBUNTU_CODENAME}_all.deb"
    calistir sudo dpkg -i /tmp/ros2-apt-source.deb
fi
tamam

# --- 3. Sistem güncellemesi --------------------------------------------------------
adim 3/7 "Sistem güncellemesi (ROS paketleri güncel bir sistem ister)"
calistir sudo apt-get update
calistir sudo env DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
tamam

# --- 4. ROS 2 + Gazebo -----------------------------------------------------------
adim 4/7 "ROS 2 $DISTRO + Gazebo + araçlar (en uzun adım, birkaç GB)"
PAKETLER=(
    "ros-$DISTRO-desktop"                     # ROS 2, RViz, örnekler
    ros-dev-tools                              # colcon, rosdep, ...
    "ros-$DISTRO-ros-gz"                      # Gazebo (Jetty) + ROS köprüsü
    "ros-$DISTRO-gz-ros2-control"             # Gazebo'da eklem kontrolü
    "ros-$DISTRO-ros2-control"
    "ros-$DISTRO-ros2-controllers"
    "ros-$DISTRO-joint-state-publisher-gui"   # RViz'de eklem kaydırıcıları
    "ros-$DISTRO-xacro"
    python3-yaml python3-pytest python3-numpy python3-matplotlib
)
calistir sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "${PAKETLER[@]}"
tamam

# --- 5. rosdep -------------------------------------------------------------------
adim 5/7 "rosdep (paket bağımlılıklarını çözen araç)"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    calistir sudo rosdep init
fi
calistir rosdep update
tamam

# --- 6. Ortam ----------------------------------------------------------------------
adim 6/7 "Her yeni terminalde ROS'un hazır gelmesi"
satir="source /opt/ros/$DISTRO/setup.bash"
if grep -qxF "$satir" ~/.bashrc; then
    echo "    ~/.bashrc'de zaten var"
else
    echo "$satir" >>~/.bashrc
    echo "    ~/.bashrc'ye eklendi"
fi
tamam

# --- 7. Test ------------------------------------------------------------------------
adim 7/7 "Kurulum testi"
# shellcheck disable=SC1090
source "/opt/ros/$DISTRO/setup.bash"
# talker kendi kendine kapansın diye timeout içinde
timeout 30 ros2 run demo_nodes_cpp talker >>"$LOG" 2>&1 &
if timeout 25 ros2 topic echo --once /chatter >>"$LOG" 2>&1; then
    echo "    ROS 2 mesajlaşması çalışıyor"
else
    hata "talker/listener testi başarısız."
fi
gz_surum=$(gz sim --version 2>/dev/null | head -n1 || true)
[ -n "$gz_surum" ] || hata "Gazebo (gz sim) bulunamadı."
echo "    Gazebo: $gz_surum"
tamam

printf '\n\033[1;32mKURULUM TAMAM.\033[0m\n'
printf 'Bu terminali kapatıp yeni bir WSL terminali açın (ROS ortamı otomatik yüklenir).\n'
printf 'Gazebo penceresini denemek için:  gz sim shapes.sdf\n'
