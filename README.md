# Hexapod

Altı bacaklı robotun yazılımı. Bağlam ve kararlar için [hexapod-proje-brifi.md](hexapod-proje-brifi.md).

Hedef mimari, TÜBİTAK Kulüp Geliştirme Desteği başvurusunda tanımlanan yön:
**ROS 2 + Gazebo**, üzerine pekiştirmeli öğrenme (Stable-Baselines3 / PPO) ile
adaptif yürüyüş. Bu depo o mimarinin en alt katmanıyla başlıyor.

## Durum

| Katman | Durum |
|---|---|
| Servo sürücü katmanı | ✅ yazıldı, testli |
| Kalibrasyon aracı | ✅ yazıldı, çalışıyor |
| Ters kinematik (IK) + gövde pozu | ✅ yazıldı, testli |
| URDF modeli | ⏭ sıradaki |
| Gazebo simülasyonu | ⛔ |
| Gait motoru (tripod) | ⛔ |
| RL (PPO) | ⛔ |
| Pi 4'e aktarma | ⛔ |

Önce yazılım: her şey CAD geometrisiyle simülasyonda geliştiriliyor.
Kablolama ve kalibrasyon robotu kuran kişi tarafından sonra yapılacak;
yazılımın önünde engel değil, sadece config'e girilecek değerler.

## Kurulum

```bash
pip install pyyaml smbus2 pytest
```

`smbus2` yalnızca gerçek donanımda gerekli. Donanımsız geliştirme için
`--dry-run` yeterli.

## Yapı

```
config/
  robot.yaml          # robotun fiziksel tanımı (geometri, kablolama, limitler)
  calibration.yaml    # servo merkez/yön değerleri — calibrate.py üretir
src/hexapod_driver/   # ROS 2 (ament_python) paketi, çekirdeği saf Python
  hexapod_driver/
    config.py         # robot.yaml yükleyici + eksik alan raporu
    calibration.py    # calibration.yaml okuma/yazma
    pca9685.py        # PCA9685 I2C PWM sürücüsü
    backends.py       # gerçek I2C / dry-run arka uçları
    servo_bus.py      # eklem adı -> kart/kanal -> darbe
src/hexapod_kinematics/  # ters/düz kinematik + gövde pozu, saf Python
  hexapod_kinematics/
    leg.py            # tek bacak IK/FK, sıfır duruşu ve yön tanımları
    body.py           # altı bacak, gövde çerçevesi, gövde pozu
tools/
  map_channels.py     # hangi servo hangi kanalda — servoları kıpırdatıp sorar
  calibrate.py        # etkileşimli servo kalibrasyon aracı
  hwcheck.py          # I2C tarama + robot.yaml karşılaştırma
  cad_extract.py      # CAD'den geometri türetme (sayıların kaynağı)
tests/
```

`hexapod_driver` bir ROS 2 paketi olarak derlenir, ama ROS'a **bağımlı değildir**.
Kalibrasyon aracı ve testler ROS kurulu olmadan çalışır; bu, tezgâh üstü
kalibrasyonu Pi'de ROS ortamı ayağa kaldırmadan yapabilmek için bilinçli bir seçim.

## Kanal haritası

```bash
python3 tools/map_channels.py
```

Servo kartlarını I2C'de kendisi bulur, her kanaldaki servoyu sırayla
kıpırdatır; kullanıcı hangi bacağın hangi ekleminin kıpırdadığını yazar
(ör. `1c` = bacak 1 coxa). Kabloların hangi sırayla takıldığı önemli değil.
Robot bir kutunun üstünde, bacaklar havada olmalı.

## Kalibrasyon

```bash
python tools/calibrate.py --dry-run
```

Kablolama bilgisi `config/robot.yaml` içinde dolu değilse araç başlamaz ve
tam olarak hangi alanların eksik olduğunu listeler.

Kablolama dolduktan sonra gerçek donanımda:

```bash
python tools/calibrate.py
```

Aynı anda yalnızca seçili servo beslenir; diğerleri serbesttir. Bu, 18 servonun
birlikte tork uygulayıp beslemeyi çökertmesini (brifteki brownout riski)
önlemek için.

Akış: bir eklem seç → `+`/`-` ile gözünle ortala → `c` ile merkezi kaydet →
araç otomatik sıradakine geçer → sonunda `save`.

Başlangıç darbesi 1500 µs'dir. Bu bir robot parametresi değil, hobi RC
servolarının ortak nötr darbesi — sadece bir yerden başlamak için.

## Donanım kontrolü

```bash
python tools/hwcheck.py
```

I2C veri yolunu tarar, bulunan adresleri `robot.yaml` beklentisiyle
karşılaştırır ve adres alanları boşsa yapıştırılabilir öneri üretir.

## Geometrinin kaynağı

```bash
python tools/cad_extract.py
```

`robot.yaml`'daki coxa=50, femur=80, tibia=126.6, yarıçap=100 değerlerinin
STEP assembly'si ve basılan STL'lerden türetilmesini yeniden çalıştırır.
CAD dosyaları depoda yok (344 MB); `Kerem Baltacı/` altına yerel olarak konmalı. Kendi tutarlılık
kontrolünü yapar (altı bacak da aynı yarıçapta çıkmalı).

## Testler

```bash
python -m pytest -q
```

## Açık kararlar ve ölçümler

`config/robot.yaml` içindeki her `null`, bilinmeyen bir değerdir. Hepsini
listelemek için:

```bash
python -c "import sys; sys.path.insert(0,'src/hexapod_driver'); from hexapod_driver import RobotConfig; [print(v.path, '<-', v.source) for v in RobotConfig.load().unknowns()]"
```

Özetle bekleyenler:

**Geometri — kumpasla ölçülmüyor**

coxa 50, femur 80, tibia 126.6 mm CAD'den alındı (`measured: false`). Brif
kumpasla ölçmeyi öneriyordu, ama hata büyüklükleri buna değmediğini gösteriyor:

| Hata kaynağı | Ayak ucunda etkisi | Nasıl giderilir |
|---|---|---|
| Baskı toleransı | < 0.5 mm | gerek yok |
| Yuvarlak ayak ucunun kayması | 1–3 mm | gerek yok |
| Servo horn'unun mil dişlisine oturması (25 diş, diş başına 14.4°) | **~10 mm'ye kadar** | kalibrasyon |

Doğruluk kalibrasyondan gelir. Robot yürürken tutarlı bir sapma görülürse
ölçüme dönülür.

**Robotun kendisiyle belirlenecekler**
- Eklem açı limitleri (`joints[*].limits_deg`) — `calibrate.py` içinde `limit`
- `body.standing_height` — ölçüm değil, IK çalışınca seçilecek bir hedef
- `body.total_mass_kg` — terazi; ilk yürüyüş için şart değil

**Kablolama**
- İki PCA9685'in I2C adresleri
- 18 eklemin kart/kanal haritası
- VL53L0X'lerin XSHUT GPIO'ları ve bakış yönleri (üçü de 0x29'da doğar)
- IMU adresi ve montaj yönelimi

**Konvansiyon**
- `frames.body.forward_offset_deg` — hangi yön burun. Geometriden simetri
  düzlemi biliniyor (x=0), yani bu değer 90 ya da −90 olacak.

## Kaynak notları

- Mekanik CAD: Sir Kuhnhero, "3D Printed Hexapod", Printables model 606030,
  CC BY-SA 4.0. Geometri buradan alındı.
- **CAD'in elektroniği bu robotu tanımlamaz.** Donör tasarım STM32 BluePill +
  4×18650 kullanıyor; bu robot Raspberry Pi 4 + 2S LiPo kullanıyor.
- Donanım envanterinin tek güvenilir kaynağı `yavuz selim/*.pdf` faturalarıdır.
  İki adet "gömülü malzemeler listesi.txt" birbiriyle ve faturalarla çelişiyor
  (PCA9685 sayısı, VL53L0X sayısı, batarya kapasitesi).

## Çalışma ortamı

| | Nerede | Sistem |
|---|---|---|
| Geliştirme + simülasyon + RL eğitimi | PC, WSL2 | Ubuntu 26.04 |
| Robot | Raspberry Pi 4 | Ubuntu Server 26.04 (arm64) |

ROS 2 dağıtımı: **Lyrical Luth** (LTS, Mayıs 2031'e kadar destekli), Ubuntu
26.04'ün birincil ROS 2 sürümü. Simülatör: **Gazebo Jetty**,
`ros-lyrical-desktop` ile birlikte geliyor. PC'de ve robotta aynı sistemin
çalışması simülasyondan robota geçişi kolaylaştırıyor.

(İlk taslakta Jazzy / Ubuntu 24.04 varsayılmıştı. Geliştirme bilgisayarındaki
WSL'de Ubuntu 26.04 kurulu olduğu için Lyrical'a geçildi.)
