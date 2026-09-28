#!/usr/bin/env bash
# =============================================================================
# Raspberry Pi 4 kurulumu — Ubuntu Server 26.04 arm64 (GOREVLER.md D3, D11)
# =============================================================================
# Ne yapar: robotta gereken her şeyi kurar ve kendini test eder.
#   - ROS 2 Lyrical'ın HAFİF sürümü (ros-base; Gazebo, RViz, masaüstü YOK —
#     bilgisayardaki tools/wsl/ros_kurulum.sh simülasyon için, Pi'ye ağır)
#   - robotun Python kütüphaneleri: numpy, yaml, smbus2 (I2C), lgpio (GPIO)
#   - klavyeyle sürmek için teleop_twist_keyboard, i2c-tools
#   - I2C ve GPIO izinleri (kullanıcı sudo'suz erişsin)
#   - depodaki robot paketlerini ~/hexapod_ws'te derler
#   - test: ROS mesajlaşması + robot testleri (servoya/sensöre YAZMADAN,
#     deneme modunda; robot.launch.py'nin uçtan uca testi dahil)
#
# Kullanım (Pi'de, depo klasöründen; önce: git clone <depo>):
#     bash tools/pi/pi_kurulum.sh
#
# sudo şifresini BİR KEZ sorar. Tekrar çalıştırmak güvenli: yapılmış
# adımları atlar. Sonunda "KURULUM TAMAM" der; I2C ayarı yeni açıldıysa ya da
# kullanıcı gruba yeni eklendiyse bir kez yeniden başlatın (sudo reboot).
#
# Sonraki adım (D3): python3 tools/hwcheck.py  — iki PCA9685'i ve IMU'yu görmeli.
# =============================================================================

set -eo pipefail

DISTRO=lyrical
LOG=/tmp/pi_kurulum.log
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
YENIDEN_BASLAT=0
KULLANICI="$(id -un)"

adim() { printf '\n\033[1;36m[%s] %s\033[0m\n' "$1" "$2"; }
tamam() { printf '\033[1;32m    tamam\033[0m\n'; }
hata() {
    printf '\n\033[1;31mHATA: %s\033[0m\n' "$1" >&2
    printf 'Ayrıntı: %s dosyasının son satırları:\n' "$LOG" >&2
    tail -n 20 "$LOG" >&2 || true
    exit 1
}
calistir() { "$@" >>"$LOG" 2>&1 || hata "şu komut başarısız oldu: $*"; }

: >"$LOG"

# --- 0. Ön kontroller ----------------------------------------------------------
. /etc/os-release
[ "${UBUNTU_CODENAME:-}" = "resolute" ] \
    || hata "Bu betik Ubuntu 26.04 (resolute) için. Bulunan: ${PRETTY_NAME:-?}"
[ "$(id -u)" -ne 0 ] \
    || hata "Betiği 'sudo' ile değil, doğrudan çalıştırın: bash tools/pi/pi_kurulum.sh"
[ -f "$REPO/config/robot.yaml" ] || hata "Depo bulunamadı ($REPO). Betiği depodan çalıştırın."
if [ "$(uname -m)" != "aarch64" ]; then
    echo "UYARI: bu makine Raspberry Pi (aarch64) değil ($(uname -m)); deneme amaçlı devam ediliyor."
fi

adim 0/8 "sudo şifresi (yalnızca bir kez sorulur)"
sudo -v || hata "sudo şifresi kabul edilmedi."
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &
tamam

# --- 1. Dil ayarı ---------------------------------------------------------------
adim 1/8 "Dil ayarı (UTF-8)"
calistir sudo apt-get update
calistir sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y locales
calistir sudo locale-gen en_US en_US.UTF-8
calistir sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
tamam

# --- 2. ROS 2 depoları ------------------------------------------------------------
adim 2/8 "ROS 2 paket depoları"
calistir sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common curl
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
adim 3/8 "Sistem güncellemesi"
calistir sudo apt-get update
calistir sudo env DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
tamam

# --- 4. Paketler ---------------------------------------------------------------------
adim 4/8 "ROS 2 $DISTRO (hafif sürüm) + robot kütüphaneleri"
PAKETLER=(
    "ros-$DISTRO-ros-base"                    # ROS 2 çekirdeği (rclpy, mesajlar, launch)
    ros-dev-tools                              # colcon (paketleri derlemek için)
    "ros-$DISTRO-teleop-twist-keyboard"       # klavyeyle /cmd_vel
    python3-numpy python3-yaml python3-pytest
    python3-smbus2                             # I2C (PCA9685, VL53L0X, BNO055)
    python3-lgpio                              # GPIO (VL53L0X XSHUT pinleri)
    i2c-tools                                  # i2cdetect: veri yolunda kim var
    git
)
calistir sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "${PAKETLER[@]}"
tamam

# --- 5. I2C ve GPIO izinleri ---------------------------------------------------------
adim 5/8 "I2C ve GPIO izinleri"
calistir sudo groupadd -f i2c
calistir sudo groupadd -f gpio
KURAL=/etc/udev/rules.d/99-hexapod-i2c-gpio.rules
if [ ! -f "$KURAL" ]; then
    printf '%s\n' \
        '# hexapod (tools/pi/pi_kurulum.sh): I2C ve GPIO sudo gerektirmesin' \
        'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"' \
        'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' \
        | sudo tee "$KURAL" >/dev/null
    # udev yoksa (kapsayıcıda deneme) hata sayılmaz; Pi'de yeniden başlatınca da uygulanır
    sudo udevadm control --reload-rules >>"$LOG" 2>&1 && sudo udevadm trigger >>"$LOG" 2>&1 || true
    YENIDEN_BASLAT=1
fi
for grup in i2c gpio dialout; do
    if ! id -nG "$KULLANICI" | tr ' ' '\n' | grep -qx "$grup"; then
        calistir sudo usermod -aG "$grup" "$KULLANICI"
        YENIDEN_BASLAT=1
    fi
done
CONFIG_TXT=/boot/firmware/config.txt
if [ -f "$CONFIG_TXT" ]; then
    if grep -qE '^\s*dtparam=i2c_arm=on' "$CONFIG_TXT"; then
        echo "    I2C zaten açık ($CONFIG_TXT)"
    else
        echo 'dtparam=i2c_arm=on' | sudo tee -a "$CONFIG_TXT" >/dev/null
        echo "    I2C açıldı ($CONFIG_TXT); yeniden başlatma gerekecek"
        YENIDEN_BASLAT=1
    fi
else
    echo "    $CONFIG_TXT yok (Pi değil); I2C ayarı atlandı"
fi
tamam

# --- 6. Ortam ----------------------------------------------------------------------
adim 6/8 "Her yeni terminalde ROS'un hazır gelmesi"
satir="source /opt/ros/$DISTRO/setup.bash"
grep -qxF "$satir" ~/.bashrc || echo "$satir" >>~/.bashrc
tamam

# --- 7. Robot paketlerini derle ---------------------------------------------------
adim 7/8 "Robot paketlerini derle (~/hexapod_ws)"
# Yalnız robotta gerekenler; simülasyon ve eğitim paketleri (hexapod_rl,
# hexapod_gazebo) bağlanır ama derlenmez.
calistir bash "$REPO/tools/wsl/derle.sh" --packages-up-to hexapod_bringup hexapod_teleop
tamam

# --- 8. Test --------------------------------------------------------------------------
adim 8/8 "Kurulum testi (servoya/sensöre yazmadan)"
# shellcheck disable=SC1090
source "/opt/ros/$DISTRO/setup.bash"
# shellcheck disable=SC1090
source "$HOME/hexapod_ws/install/setup.bash"
export ROS_DOMAIN_ID=$((20 + RANDOM % 70))   # açık başka bir ROS ağıyla karışmasın
# ros-base'de demo düğümleri yok: ros2 topic ile yayınla-dinle
timeout 30 ros2 topic pub -r 2 /kurulum_testi std_msgs/msg/String "{data: merhaba}" >>"$LOG" 2>&1 &
YAYIN=$!
if timeout 25 ros2 topic echo --once /kurulum_testi std_msgs/msg/String >>"$LOG" 2>&1; then
    echo "    ROS 2 mesajlaşması çalışıyor"
else
    hata "ROS 2 yayınla/dinle testi başarısız."
fi
kill "$YAYIN" 2>/dev/null || true
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
calistir python3 -c "import numpy, yaml, smbus2, lgpio, rclpy
import hexapod_policy.node, hexapod_sensors.node, hexapod_hardware.node, hexapod_teleop.node"
echo "    kütüphaneler ve robot paketleri yükleniyor"
TESTLER=(
    tests/test_servo_layer.py tests/test_driver_controller.py tests/test_sensors.py
    tests/test_kinematics.py tests/test_tripod_gait.py tests/test_teleop_controller.py
    tests/test_policy_mlp.py tests/test_policy_lift_reflex.py tests/test_policy_ranges.py
    tests/test_stop_signals.py tests/test_ros_nodes.py tests/test_bringup.py
)
( cd "$REPO" && python3 -m pytest -q -p no:cacheprovider "${TESTLER[@]}" ) >>"$LOG" 2>&1 \
    || hata "robot testleri geçmedi (günlük: $LOG)"
echo "    robot testleri geçti: $(grep -E '[0-9]+ passed' "$LOG" | tail -n1)"
if [ -e /dev/i2c-1 ]; then
    echo "    I2C veri yolu 1'deki cihazlar (i2cdetect):"
    sudo i2cdetect -y 1 | sed 's/^/      /' || true
fi
tamam

printf '\n\033[1;32mKURULUM TAMAM.\033[0m\n'
if [ "$YENIDEN_BASLAT" = 1 ]; then
    printf 'I2C/GPIO ayarları değişti: bir kez yeniden başlatın:  sudo reboot\n'
fi
printf 'Sonraki adım (D3):  python3 tools/hwcheck.py\n'
printf 'Robotu başlatmak (kablolama ve kalibrasyon girildikten sonra):\n'
printf '    ros2 launch hexapod_bringup robot.launch.py\n'
