#!/usr/bin/env bash
# =============================================================================
# ROS'lu simde politika düğümünü ölç (GOREVLER.md G8)
# =============================================================================
# Gazebo'yu (sim.launch.py, pencere yok) ve politika düğümünü başlatır,
# tools/politika_ros_olcum.py ile ileri, geri, iki yana, iki dönüş, karışık
# ve sıfır komutu sırayla verip gövde hızını ölçer, sonra her şeyi kapatır.
#
# Kullanım (WSL, depo klasöründen; sudo gerekmez):
#     bash tools/wsl/politika_ros_olcum.sh                                 # models/ppo_omni_250k
#     bash tools/wsl/politika_ros_olcum.sh models/ppo_res_250k/policy.npz  # başka politika
#
# Başka bir simülasyon ya da eğitimle çakışmasın diye kendi ROS_DOMAIN_ID'si
# ve GZ_PARTITION'ı var (PROJE_DEVIR ders 22).
# =============================================================================

set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
POLICY="$(realpath "${1:-$REPO/models/ppo_omni_250k/policy.npz}")"
source /opt/ros/lyrical/setup.bash
source ~/hexapod_ws/install/setup.bash
export ROS_DOMAIN_ID=57
export GZ_PARTITION=politika_olcum_$$
LOGS=$(mktemp -d)

ros2 launch hexapod_gazebo sim.launch.py gui:=false >"$LOGS/sim.log" 2>&1 &
LAUNCH=$!
ok=0
for _ in $(seq 1 60); do
    # ros2 control servis yoksa sonsuza kadar bekler: timeout şart (ders 22)
    if timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "leg_controller.*active"; then
        ok=1; break
    fi
    sleep 2
done
if [ "$ok" = 1 ]; then
    ros2 run hexapod_policy policy --ros-args -p policy:="$POLICY" -p use_sim_time:=true \
        >"$LOGS/policy.log" 2>&1 &
    POL=$!
    sleep 3
    python3 "$REPO/tools/politika_ros_olcum.py"
    kill -INT "$POL" 2>/dev/null
else
    echo "HATA: kontrolcüler 2 dakikada açılmadı; günlük: $LOGS/sim.log" >&2
fi
kill -INT "$LAUNCH" 2>/dev/null
sleep 5
# Kalanlar (gz sunucusu SIGINT'te bazen kalıyor). Desen bu dosyada, komut
# satırında değil: pgrep kendi kabuğunu bulmaz (ders 19).
for p in $(pgrep -f "gz sim|parameter_bridge|robot_state_publisher|hexapod_policy"); do
    kill -KILL "$p" 2>/dev/null
done
echo "--- politika düğümü durumları ($LOGS/policy.log)"
grep -E "hazır|durum:|Error|Traceback" "$LOGS/policy.log" 2>/dev/null | sed 's/^\[[^]]*\] //' | cut -c1-140
[ "$ok" = 1 ]
