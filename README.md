# Hexapod

Altı bacaklı robotun yazılımı. Projenin tüm bağlamı, kararları ve durumu:
[docs/PROJE_DEVIR.md](docs/PROJE_DEVIR.md). İlk brif: [docs/hexapod-proje-brifi.md](docs/hexapod-proje-brifi.md).
Görev dağılımı ve takibi: [GOREVLER.md](GOREVLER.md).

Hedef mimari, TÜBİTAK Kulüp Geliştirme Desteği başvurusunda tanımlanan yön:
**ROS 2 + Gazebo**, üzerine pekiştirmeli öğrenme (Stable-Baselines3 / PPO) ile
adaptif yürüyüş. Bu depo o mimarinin en alt katmanıyla başlıyor.

## Durum

| Katman | Durum |
|---|---|
| Servo sürücü katmanı | ✅ yazıldı, testli |
| Kalibrasyon aracı | ✅ yazıldı, çalışıyor |
| Ters kinematik (IK) + gövde pozu | ✅ yazıldı, testli |
| URDF modeli | ✅ üretiliyor, testli, RViz'de açılıyor |
| Gazebo simülasyonu | ✅ robot doğuyor, ayağa kalkıyor; sensörler yayında |
| Gait motoru (tripod) | ⛔ |
| RL (PPO) | 🔄 hızlı simülasyon çekirdeği hazır (ROS'suz, ~21x gerçek zaman) |
| Pi 4'e aktarma | ⛔ |

Önce yazılım: her şey CAD geometrisiyle simülasyonda geliştiriliyor.
Yazılım işi Görkem ve Samet arasında bölüşüldü; kablolama, kalibrasyon ve
gerçek robotta denemeler şimdilik durduruldu, ayrı bir donanım vardiyasında
yapılacak ([GOREVLER.md](GOREVLER.md)). Yazılımın önünde engel değiller,
sadece config'e sonradan girilecek değerler.

## Kurulum

```bash
pip install pyyaml smbus2 pytest
```

`smbus2` yalnızca gerçek donanımda gerekli. Donanımsız geliştirme için
`--dry-run` yeterli.

## Yapı

```
config/
  robot.yaml          # robotun fiziksel tanımı (geometri, kablolama, limitler, simülasyon)
  calibration.yaml    # servo merkez/yön değerleri — calibrate.py üretir
src/                  # ROS 2 (ament_python) paketleri; çekirdekleri saf Python
  hexapod_driver/     # servo sürücü katmanı
    config.py         #   robot.yaml yükleyici + eksik alan raporu
    calibration.py    #   calibration.yaml okuma/yazma
    pca9685.py        #   PCA9685 I2C PWM sürücüsü
    backends.py       #   gerçek I2C / dry-run arka uçları
    servo_bus.py      #   eklem adı -> kart/kanal -> darbe
  hexapod_kinematics/ # ters/düz kinematik + gövde pozu
    leg.py            #   tek bacak IK/FK, sıfır duruşu ve yön tanımları
    body.py           #   altı bacak, gövde çerçevesi, gövde pozu
  hexapod_description/  # simülasyon modeli (URDF'in girdisi)
    model.py          #   kütle/atalet/çarpışma/limitler, SI birimlerinde
    urdf.py           #   RobotModel -> URDF
    launch/, rviz/    #   RViz'de görüntüleme
    meshes.yaml       #   görsel mesh yerleşimi (cad_sim_model.py üretir)
tools/                # komut satırı araçları
  map_channels.py     # hangi servo hangi kanalda — servoları kıpırdatıp sorar
  calibrate.py        # etkileşimli servo kalibrasyon aracı
  hwcheck.py          # I2C tarama + robot.yaml karşılaştırma
  cad_extract.py      # CAD'den geometri türetme (sayıların kaynağı)
  cad_sim_model.py    # CAD'den kütle, atalet, çarpışma kutuları
  make_urdf.py        # robot.yaml -> URDF (ROS'suz)
  preview_urdf.py     # URDF'i PNG'ye çizer (ROS'suz önizleme)
  wsl/ros_kurulum.sh  # WSL'e ROS 2 Lyrical + Gazebo kurulumu
  cadlib/             # iki CAD aracının ortak kütüphanesi (STEP, STL, çerçeveler)
tests/
docs/
  PROJE_DEVIR.md      # projenin bütün bağlamı, kararlar, dersler
  hexapod-proje-brifi.md
  malzeme/            # malzeme listeleri (güvenilmez; faturalar esas)
cad/                  # CAD ve baskı dosyaları (depoda, ~345 MB)
  Hexapod/            #   STEP/STL/3MF/F3Z (Printables 606030) + lisans PDF'i
  Baskı Dosyaları/    #   basılan STL'ler
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
CAD dosyaları `cad/Hexapod/` altında (depoda). Kendi tutarlılık
kontrolünü yapar (altı bacak da aynı yarıçapta çıkmalı).

## Simülasyon verisi

```bash
python tools/cad_sim_model.py --copy-meshes
```

Basılan parçaların STL'lerinden link başına kütle, ağırlık merkezi, atalet ve
çarpışma kutularını hesaplar (`robot.yaml` → `simulation`), görsel mesh
yerleşimini yazar. Robot tartılmadığı için bunlar **tahmin**: PETG yoğunluğu ×
doluluk oranı + katalog servo kütlesi. Tartım yapılınca girdiler düzeltilip araç
yeniden çalıştırılır.

## URDF

```bash
python tools/make_urdf.py -o hexapod.urdf
python tools/preview_urdf.py
```

URDF elle yazılmaz; her seferinde `robot.yaml`'dan üretilir. Eklem zinciri IK
ile birebir aynı (test). `preview_urdf.py` ROS olmadan robotun bir resmini
`onizleme.png`'ye çizer. ROS kurulunca: `ros2 launch hexapod_description display.launch.py`.

## Eklem komut arayüzü

Simülasyon, gerçek robot, yürüyüş ve RL politikası aynı konuları konuşur:
[docs/ARAYUZ.md](docs/ARAYUZ.md). Kod: `hexapod_description.interface`.

## Gazebo (ROS kurulduktan sonra)

Önce paketleri derle (WSL, depo klasöründen):

```bash
bash tools/wsl/derle.sh
```

Sonra:

```bash
ros2 launch hexapod_gazebo sim.launch.py
```

```bash
ros2 run hexapod_gazebo stand
```

## ROS 2 kurulumu (WSL)

WSL terminalinde, depo klasöründen:

```bash
bash tools/wsl/ros_kurulum.sh
```

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

## Kaynak notları

- Mekanik CAD: Sir Kuhnhero, "3D Printed Hexapod", Printables model 606030,
  CC BY-SA 4.0. Geometri buradan alındı.
- **CAD'in elektroniği bu robotu tanımlamaz.** Donör tasarım STM32 BluePill +
  4×18650 kullanıyor; bu robot Raspberry Pi 4 + 2S LiPo kullanıyor.
- Donanım envanterinin tek güvenilir kaynağı faturalardır. Faturalar kişisel
  veri (TCKN, adres) içerdiği için depoda değil; `.gitignore` bütün PDF'leri
  ve ekran görüntülerini dışarıda tutuyor. İki malzeme listesi
  (`docs/malzeme/gömülü malzemeler listesi.txt` ve `... (alternatif).txt`) birbiriyle ve
  faturalarla çelişiyor (PCA9685 sayısı, VL53L0X sayısı, batarya kapasitesi).

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
