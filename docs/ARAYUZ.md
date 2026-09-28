# Eklem Komut Arayüzü

Simülasyon, gerçek robot, tripod yürüyüşü ve RL politikası **aynı konuları** konuşur. Simülasyondan robota geçişte yalnızca dinleyen taraf değişir (Gazebo'daki kontrolcü yerine gerçek sürücü düğümü); komut yayınlayan kod aynı kalır.

| Taraf | Görev | Rolü |
|---|---|---|
| Gazebo + ros2_control | G5 (Görkem) | komutu dinler, eklemleri sürer |
| Gerçek robot sürücü düğümü | S4 (Samet) | komutu dinler, `ServoBus.set_angle`'a taşır |
| Tripod yürüyüş düğümü | S3 (Samet) | komut yayınlar |
| RL politika düğümü | G8 (Görkem) | komut yayınlar (`hexapod_policy`; `/imu` + `/cmd_vel` dinler; `-p reflex:=true` ile `/range<kimlik>` de) |
| Sensör düğümü | S7 (Samet) | `/imu` ve `/range<kimlik>` yayınlar (`hexapod_sensors`; robotta) |

Sözleşmenin kodu: [`hexapod_description/interface.py`](../src/hexapod_description/hexapod_description/interface.py). Sabitleri ve dönüşümleri **oradan import edin**, kendi kodunuzda tekrar yazmayın. Testleri: `tests/test_interface.py`.

## Konular

| Konu | Mesaj | Yön | Not |
|---|---|---|---|
| `/leg_controller/commands` | `std_msgs/msg/Float64MultiArray` | → robot | 18 eklem hedefi, **radyan**, aşağıdaki sırayla |
| `/joint_states` | `sensor_msgs/msg/JointState` | robot → | ad listesiyle; sıra garanti değil, adla eşleyin |
| `/imu` | `sensor_msgs/msg/Imu` | robot → | çerçeve `imu_link` = `base_link` yönelimi |
| `/range{i}` | `sensor_msgs/msg/Range` | robot → | mesafe sensörü i (robot.yaml `sensors.range_finders.devices[*].id`), m; görmüyorsa `range = max_range` (REP-117'deki +inf da kabul); kaldırma refleksi için. ROS'lu simde sensör yok: `tools/politika_ros_olcum.py --mesafe` pozdan hesaplayıp yayınlar |
| `/leg{i}/foot_contact` | `ros_gz_interfaces/msg/Contacts` | sim → | **yalnız simülasyon**, i = 0..5 |
| `/clock` | `rosgraph_msgs/msg/Clock` | sim → | simülasyonda `use_sim_time: true` |

Komut hızı **50 Hz** (`COMMAND_RATE_HZ`). MG996R 50 Hz PWM ile sürülür, daha sık komutun anlamı yok.

## Eklem sırası

Bacak bacak, her bacakta coxa, femur, tibia (`interface.joint_names()`):

```
 0 leg0_coxa_joint   1 leg0_femur_joint   2 leg0_tibia_joint
 3 leg1_coxa_joint   4 leg1_femur_joint   5 leg1_tibia_joint
 ...
15 leg5_coxa_joint  16 leg5_femur_joint  17 leg5_tibia_joint
```

Bacak kimlikleri `config/robot.yaml`'daki `legs[*].id` (0 sol orta, 1 sol ön, 2 sağ ön, 3 sağ orta, 4 sağ arka, 5 sol arka; ön keyfi seçildi, bkz. PROJE_DEVIR §5.5).

## Birim, sıfır ve işaret

- ROS tarafında **radyan**. `hexapod_kinematics` **derece** kullanır; dönüşüm yalnızca `to_command()` / `from_command()` ile.
- Sıfır duruşu ve pozitif yönler IK ↔ kalibrasyon sözleşmesiyle aynı ([CLAUDE.md](../CLAUDE.md)):

| Eklem | 0 | + yönü |
|---|---|---|
| coxa | bacak gövdeden dümdüz dışarı | yukarıdan bakınca saat yönünün tersi |
| femur | femur yere paralel | bacak yukarı kalkar |
| tibia | tibia femura dik | diz açılır, ayak dışarı gider |

- Limitler: kalibrasyon limitleri (`joints[*].limits_deg`) varsa onlar, yoksa geçici ±90° (`simulation.provisional_joint_limits_deg`). Limit dışı komut gerçek robotta `LimitError` verir (servoya gitmez). Simülasyonda eklem limitte durur (doğrulandı, iki servo modelinde de: femur'a 2.0 rad → 1.5708'de durdu) ve günlüğe "out of limits" yazar. İkisi farklı davrandığı için **yayınlayan taraf limit içinde kalmalı.**

## Örnek: komut yayınlamak (S3, G8)

```python
from hexapod_description.interface import COMMAND_TOPIC, to_command
from hexapod_kinematics import HexapodKinematics
from std_msgs.msg import Float64MultiArray

angles = kin.inverse(feet)                     # {bacak: JointAngles}, derece
msg = Float64MultiArray(data=to_command(angles))  # 18 değer, radyan, doğru sırada
publisher.publish(msg)                         # publisher: create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
```

## Örnek: komut dinlemek (S4)

```python
from hexapod_description.interface import from_command

def on_command(msg):
    for leg, a in from_command(msg.data).items():   # derece
        for part, deg in a.as_dict().items():
            bus.set_angle(leg, part, deg)            # hexapod_driver.ServoBus
```

`from_command` 18 değer ve sonlu sayı bekler, değilse `ValueError` fırlatır. Bozuk komut servoya gitmemeli.

## Gerçek robot farkları (sim-to-real için ÖNEMLİ)

1. **MG996R konum geri bildirimi vermez.** Gerçek robotta `/joint_states` ölçüm değil, **son gönderilen komuttur.** RL politikası eklem açısını gözlem olarak kullanacaksa simülasyonda da komut edilen açıyı (ya da gürültülü, gecikmeli hâlini) görmeli; yoksa simülasyonda öğrendiği bilgiyi gerçekte bulamaz.
2. **Ayak temas sensörü yok.** `/leg{i}/foot_contact` yalnız simülasyonda. Politikanın **gözlemine girmemeli**; ödül ve değerlendirme (S6) için var.
3. **IMU yönelimi:** `/imu` her zaman `base_link` yöneliminde yayınlanır. Gerçek sürücü (S7/D8), BNO055'in ham verisini montaj yönelimine (`sensors.imu.mount_rotation_deg`, henüz bilinmiyor) göre döndürür. Bu yüzden politika IMU'nun robotta nasıl takıldığından habersizdir.
4. **Servo tepkisi:** iki simülasyon da servoyu tork tabanlı modelliyor: tork = sertlik × hata − sönüm × hız, durma torkuyla sınırlı (`simulation.servo.stiffness_nm_per_rad`, `damping_nm_s_per_rad`, `effort_nm`). RL simülasyonunda (`hexapod_rl.sim`) Python'da, her fizik adımında, DC motorun tork-hız doğrusuyla. ROS'lu simülasyonda (sim.launch.py, 2026-09-26'dan beri varsayılan `servo:=torque`) `leg_controller` bir `pid_controller`'a (`servo_controller`) zincirli: P kazancı sertlik, çıkış ±durma torku, 1 kHz; sönüm URDF'te eklem sönümü. Tork-hız doğrusu yok; yerine gz_ros2_control eklem hız sınırında (7.48 rad/s) torku sıfırlıyor (günlükte "out of limits" uyarıları bundan, beklenen). Komut konusu ve sırası iki modelde aynı. Ölçüldü (düz zemin): tripod 0.08 m/s komutta 0.079 m/s, politika 0.10'da 0.096 m/s; eski model (`servo:=velocity`, konum komutu hız kontrolüyle, zaman sabiti `time_constant_s`) ayak kaydırıyordu: 0.067 ve 0.083 m/s. Gerçek servo farklıysa (D9'da ölçülür) değerler güncellenir.

## Değiştirme kuralı

Bu arayüzü değiştirmek dört tarafı birden etkiler. Değişiklik:
1. `interface.py`, bu belge ve `tests/test_interface.py` **aynı commit'te** değişir,
2. Görkem ve Samet'in ikisi de haberdar edilir.
