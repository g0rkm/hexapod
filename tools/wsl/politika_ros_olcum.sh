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
#     bash tools/wsl/politika_ros_olcum.sh models/ppo_kaldirma35_250k/policy.npz  # başka politika
#     WORLD=/yol/dunya.sdf WORLD_NAME=zemin KOMUTLAR="0.1,0,0" SURE=15 \
#         bash tools/wsl/politika_ros_olcum.sh models/ppo_lift50_3750k/policy.npz
#     (deneme zemini dünyası: terrain_probe.world_sdf(...))
#     ZEMIN=basamak:3 KOMUTLAR="0.1,0,0" SURE=12 bash tools/wsl/politika_ros_olcum.sh ...
#     (S5 zemini: dünya dosyası hexapod_terrain ile üretilir, WORLD gerekmez)
#     REFLEKS=1 ZEMIN=basamak:3 ... : politika düğümü -p reflex:=true ile,
#     DENEYSEL yerleşimli bir config KOPYASIYLA (hexapod_rl.deneysel_yerlesim;
#     depodaki robot.yaml'a dokunulmaz) başlar; ölçüm aracı simdeki pozdan
#     /range<kimlik> yayınlar (robottaki sensör düğümünün yerine).
#     Komutlar robot sıfırlanmadan art arda koşar: zeminde her komutu ayrı
#     çalıştırın (PROJE_DEVIR ders 38).
#
# Başka bir simülasyon ya da eğitimle çakışmasın diye kendi ROS_DOMAIN_ID'si
# ve GZ_PARTITION'ı var (PROJE_DEVIR ders 22).
# =============================================================================

set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
POLICY="$(realpath "${1:-$REPO/models/ppo_omni_250k/policy.npz}")"
source /opt/ros/lyrical/setup.bash
source ~/hexapod_ws/install/setup.bash
# Her koşu kendi ROS_DOMAIN_ID'sinde: arka arkaya koşularda bir öncekinin
# kapanmakta olan controller_manager'ı "aktif" cevabı verip yeni sim hazır
# olmadan ölçümü başlatıyordu (2026-09-26, zemin dünyaları denemesi).
export ROS_DOMAIN_ID=${OLCUM_DOMAIN_ID:-$((20 + RANDOM % 80))}
export GZ_PARTITION=politika_olcum_$$
LOGS=$(mktemp -d)

# S5 zemini: dünya dosyasını üret (dünya adı = zemin türü).
if [ -n "${ZEMIN:-}" ]; then
    python3 -m hexapod_terrain dunya "${ZEMIN%%:*}" "${ZEMIN#*:}" -o "$LOGS/dunya.sdf" >/dev/null \
        || { echo "HATA: zemin üretilemedi: $ZEMIN" >&2; exit 1; }
    WORLD="$LOGS/dunya.sdf"
    WORLD_NAME="${ZEMIN%%:*}"
fi
# Refleks: DENEYSEL yerleşimli config kopyası (robotta kullanılmaz).
POLICY_ARGS=()
OLCUM_ARGS=()
if [ -n "${REFLEKS:-}" ]; then
    CFG="$LOGS/robot_deneysel.yaml"
    python3 -m hexapod_rl.deneysel_yerlesim "$CFG" >/dev/null || exit 1
    POLICY_ARGS=(-p reflex:=true -p config:="$CFG")
    OLCUM_ARGS=(--mesafe "$CFG" ${ZEMIN:+--zemin "$ZEMIN"})
fi

# Sim ve düğüm kendi süreç gruplarında (setsid): kapatırken bütün grup gider.
setsid ros2 launch hexapod_gazebo sim.launch.py gui:=false ${WORLD:+world:=$WORLD} \
    >"$LOGS/sim.log" 2>&1 &
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
    setsid ros2 run hexapod_policy policy --ros-args -p policy:="$POLICY" -p use_sim_time:=true \
        "${POLICY_ARGS[@]}" >"$LOGS/policy.log" 2>&1 &
    POL=$!
    sleep 3
    python3 "$REPO/tools/politika_ros_olcum.py" --world "${WORLD_NAME:-flat}" \
        --seconds "${SURE:-8}" ${KOMUTLAR:+--commands "$KOMUTLAR"} "${OLCUM_ARGS[@]}"
    kill -INT -- -"$POL" 2>/dev/null
else
    echo "HATA: kontrolcüler 2 dakikada açılmadı; günlük: $LOGS/sim.log" >&2
fi
kill -INT -- -"$LAUNCH" 2>/dev/null
sleep 5
# Kalanlar (gz sunucusu SIGINT'te bazen kalıyor): bütün grup.
kill -KILL -- -"$LAUNCH" 2>/dev/null
[ -n "${POL:-}" ] && kill -KILL -- -"$POL" 2>/dev/null
echo "--- politika düğümü durumları ($LOGS/policy.log)"
grep -E "hazır|durum:|refleks:|HATA|Error|Traceback" "$LOGS/policy.log" 2>/dev/null \
    | sed 's/^\[[^]]*\] //' | cut -c1-160
[ "$ok" = 1 ]
