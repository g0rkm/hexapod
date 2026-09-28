# CLAUDE.md

> **Yeni bir oturumdaysan önce [docs/PROJE_DEVIR.md](docs/PROJE_DEVIR.md)'yi baştan
> sona oku.** Projenin bütün bağlamı orada: kullanıcının çalışma tarzı ve
> git tercihleri (Türkçe detaylı commit, **Claude imzası yok**), alınan
> kararlar ve gerekçeleri, bulunan hatalar, donanım özeti, açık işler ve
> sıradaki adımın planı. Bu dosya onun kısa özetidir.
>
> **Yeni bilgisayardaysan** (2026-09-26'da RTX 5070'li PC'ye geçildi):
> kurulum ve doğrulama adımları PROJE_DEVIR §0'da.

Bu depoda çalışırken bilmen gerekenler. Ayrıntılı bağlam:
[docs/PROJE_DEVIR.md](docs/PROJE_DEVIR.md), ilk brif:
[docs/hexapod-proje-brifi.md](docs/hexapod-proje-brifi.md), kullanım: [README.md](README.md),
görev dağılımı (kim neyi yapıyor, ne neyi bekliyor): [GOREVLER.md](GOREVLER.md).

## Proje

Altı bacaklı robot. 3 DOF × 6 bacak = 18 servo. Raspberry Pi 4 üzerinde
ROS 2, Gazebo'da pekiştirmeli öğrenme (PPO) ile adaptif yürüyüş, sim-to-real.

## Temel kural: eksik değer uydurulmaz

`config/robot.yaml` içindeki her sayı `{value, source, measured}` üçlüsüyle
tutulur. `value: null` ise o değer **bilinmiyor**.

- Hiçbir katman `null` bir alana varsayılan koyamaz. Yükleyici `MissingValue`
  fırlatır ve değerin nereden geleceğini söyler.
- `measured: false`, değerin CAD'den türetildiği ama monte robottan
  doğrulanmadığı anlamına gelir. Çalışır, ama "doğru" sayılmaz.

Sebebi brifin risk maddesi: eksik parametre yerine makul görünen bir sayı
koymak, sonraki her katmanı sessizce tutarsız hâle getirir ve hata
donanım arızası gibi görünür.

Eksikleri görmek için:

```bash
python tools/hwcheck.py
```

veya kalibrasyon aracı içinde `gaps`.

## Kaynak hiyerarşisi

Donanım hakkında çelişki çıkarsa sıra şu:

1. **Faturalar** — gerçekte ne alındığının tek kaydı. Depoda DEĞİL: TCKN
   ve adres içeriyor. `.gitignore` bütün PDF'leri ve ekran görüntülerini
   klasörden bağımsız olarak dışarıda tutuyor; bu kuralı gevşetme.
2. **CAD** (`cad/Hexapod/`, `cad/Baskı Dosyaları/`) — yalnızca mekanik geometri.
   2026-09-24'ten beri depoda (345 MB; depo gizli, kullanıcının kararı).
3. Malzeme listesi txt'leri (`docs/malzeme/`) — **güvenilmez**, ikisi birbiriyle çelişiyor:
   `gömülü malzemeler listesi.txt` (faturalara daha yakın) ve
   `gömülü malzemeler listesi (alternatif).txt` (TF-Luna, LM2596 —
   bunlar hiç alınmadı).

Bilinen çelişkiler: PCA9685 (liste 1, gerçek **2**), VL53L0X (liste 2,
gerçek **3**), batarya (liste 2200 mAh, gerçek **2800 mAh**).

## CAD donör bir tasarım

`cad/Hexapod/` ve `cad/Baskı Dosyaları/` altındaki her şey Sir Kuhnhero'nun
"3D Printed Hexapod" tasarımından (Printables 606030, CC BY-SA 4.0).

- Mekanik geometri geçerli.
- **Elektronik geçerli değil.** Donör tasarım STM32 BluePill + 4×18650 +
  XL4016 kullanıyor. Bu robot Raspberry Pi 4 + 2S LiPo kullanıyor.
- CAD'deki kart yerleşimine bakarak güç veya kontrolcü varsayımı yapma.

## Geometri nereden geldi

`config/robot.yaml`'daki coxa=50, femur=80, tibia=126.6, yarıçap=100 değerleri
STEP assembly'sinden ve basılan STL'lerden türetildi. Türetme yeniden çalıştırılabilir:

```bash
python tools/cad_extract.py
```

Yöntem: her eklemin dönme ekseni, servo horn'u ile karşısındaki bushing'i
birleştiren doğru. Araç kendi tutarlılık kontrolünü yapar (altı bacak da
aynı yarıçapta çıkmalı).

**STEP'te parça adı ↔ geometri eşlemesine güvenme.** tibia_tip ile
tibia_main için ters çıkıyor; ilk türetmede tibia bu yüzden 121 yazıldı
(doğrusu 126.6). Tibia artık adı doğru olan STL'lerden hesaplanıyor.
Ayrıca B-spline kontrol noktaları yüzeyin dışında durur: bir parçanın
en uç noktasını STEP noktalarından değil, mesh köşelerinden al.

**Aynalı bacaklar tuzak.** Bacak 2, 3, 4 aynalı basılmış ve Fusion bunları
ayrı gövde olarak dışa aktarmış: dönüşüm matrisi düzgün bir dönme, yansıma
geometride. Yerel Z ekseni ters yönde uzanır. Bu, eklem açı işaretlerinin
o bacaklarda ters olabileceği anlamına gelir — varsayma, kalibrasyonda
`dir` komutuyla belirle.

## Depo yapısı

```
config/robot.yaml         robotun fiziksel tanımı (+ simulation: tahminler)
config/calibration.yaml   servo merkez/yön/limit — calibrate.py üretir
src/hexapod_driver/       ROS 2 (ament_python) paketi, çekirdeği saf Python
src/hexapod_kinematics/   ters/düz kinematik + gövde pozu, saf Python
src/hexapod_gait/         tripod yürüyüş çekirdeği (gövde hızı -> eklem açısı), saf Python
src/hexapod_teleop/       /cmd_vel -> hexapod_gait -> eklem komut arayüzü, ROS 2 düğümü
src/hexapod_hardware/     gerçek robot sürücü düğümü: eklem komutu -> ServoBus (dry-run destekli)
src/hexapod_description/  simülasyon modeli, URDF, eklem arayüzü (interface.py)
src/hexapod_gazebo/       Gazebo dünyaları, sim.launch.py, stand komutu
src/hexapod_rl/           RL: süreç içi Gazebo (sim), ortam (env, task), taklit (demo, pretrain),
                          PPO (train), ölçüm (evaluate, baseline, robustness, reflex_probe),
                          mesafe sensörü simi (rangefinder), aktarma (export)
src/hexapod_policy/       politika düğümü: torch'suz (numpy) MLP, /imu + /cmd_vel -> komut
                          + mesafe sensörlü ayak kaldırma refleksi (lift_reflex)
src/hexapod_terrain/      RL/ölçüm zeminleri: eğim, basamak, merdiven, engebe, çukur (SDF + yükseklik), saf Python
src/hexapod_sensors/      VL53L0X x3 + BNO055 sürücüleri ve düğümü (/range*, /imu); donanımda denenmedi
src/hexapod_bringup/      gerçek robotu tek komutla başlatır (sensör + sürücü + politika; dry_run ile robotsuz)
                          ölçüm aracı: hexapod_rl.olcum (zeminlerde hız/enerji/devrilme tablosu)
tools/map_channels.py     hangi servo hangi kanalda — kıpırdatıp sorar
tools/calibrate.py        etkileşimli servo kalibrasyonu
tools/hwcheck.py          I2C tarama + config karşılaştırma
tools/cad_extract.py      CAD'den geometri türetme
tools/cad_sim_model.py    CAD'den kütle/atalet/çarpışma (simülasyon)
tools/make_urdf.py        robot.yaml -> URDF (ROS'suz)
tools/preview_urdf.py     URDF'i PNG'ye çizer (ROS'suz önizleme)
tools/wsl/ros_kurulum.sh  WSL'e ROS 2 Lyrical + Gazebo kurulumu
tools/wsl/derle.sh        ROS paketlerini ~/hexapod_ws'te derler
tools/wsl/rl_kurulum.sh   ~/hexapod_venv: torch (CPU), SB3, Gymnasium
tools/wsl/politika_ros_olcum.sh  ROS'lu simde politika düğümünü komut komut ölçer
tools/pi/pi_kurulum.sh    Raspberry Pi'ye robotta gerekenler (hafif ROS 2, I2C/GPIO) + derleme + test
tools/cadlib/             CAD araçlarının ortak kütüphanesi
tests/
docs/                     PROJE_DEVIR.md, ARAYUZ.md (eklem arayüzü), brif, malzeme/
cad/                      CAD + baskı dosyaları (depoda)
models/                   kayda değer modeller (model.zip + policy.npz) ve karşılaştırma tabloları
egitim_kayitlari/         bütün eğitimlerin progress.csv/ayarlar kayıtları
```

`hexapod_driver` ROS 2 paketi olarak derlenir ama **ROS'a bağımlı değildir**.
Bu bilinçli: tezgâh üstü kalibrasyon Pi'de ROS ortamı ayağa kaldırmadan
yapılabilsin diye. Yeni kod eklerken bu ayrımı koru — donanıma dokunan
katman saf Python kalsın, ROS sarmalayıcısı ayrı olsun.

## Servo katmanı sözleşmesi

- `set_pulse_us()` — ham darbe. Yalnızca kalibrasyon aracı kullanır.
- `set_angle()` — derece. IK ve gait bunu kullanır. Kalibrasyon tamamlanmadan
  çalışmaz, `MissingValue` fırlatır.
- `servo.pulse_us_hard_limits` mutlak güvenlik sınırıdır; katman bu aralığın
  dışına asla darbe göndermez.
- Kalibrasyon sırasında **aynı anda tek servo beslenir**. 18 servo birlikte
  tork uygularsa besleme çöker, Pi resetlenir ve yazılım hatası gibi görünür.

## Donanımsız geliştirme

`DryRunBackend` gerçek I2C yerine yazmaları kaydeder. Testlerin tamamı ve
`--dry-run` bayrağı bunu kullanır, robot olmadan çalışır.

```bash
python -m pytest -q
```

## Geometri CAD'den alınır, kumpasla ölçülmez

Brif segment uzunluklarının kumpasla ölçülmesini öneriyordu. Bu karar
2026-09-24'te değişti: coxa, femur ve tibia CAD'den alınıyor
(`measured: false`), fiziksel ölçüm yapılmıyor.

Gerekçe hata büyüklüklerinin karşılaştırması:
- Baskı toleransı segment uzunluğunda ~0.5 mm'den az hata yaratır.
- Yuvarlak ayak ucunun kayması ~r·sin(açı), yani 1-3 mm.
- Servo horn'unun mil dişlisine oturma hatası (MG996R, 25 diş, diş başına
  14.4°) femur ucunda **~10 mm'ye kadar** sapma yaratır.

Asıl hata kaynağı sonuncusu, onu da kumpas göremez: kalibrasyon
(`center_us`) giderir. Yani doğruluk kalibrasyondan gelir, ölçümden değil.

Robot yürürken ayak konumunda tutarlı bir sapma görülürse, ancak o zaman
ölçüme dönülür.

## Sıfır duruşu ve pozitif yönler (IK ↔ kalibrasyon sözleşmesi)

Kalibrasyonda kaydedilen "merkez" (`center_us`) ile IK'daki 0 açısı AYNI
duruştur. Biri değişirse öteki de değişmeli, yoksa her ayak kayık basar.

| Eklem | 0 açısı | + yönü |
|---|---|---|
| coxa  | bacak gövdeden dümdüz dışarı | yukarıdan bakınca saat yönünün tersi |
| femur | femur yere paralel | bacak yukarı kalkar |
| tibia | tibia femura dik | diz açılır, ayak dışarı gider |

Kaynak: `src/hexapod_kinematics/hexapod_kinematics/leg.py` modül açıklaması.
`tools/calibrate.py` yardım metni aynı tabloyu gösterir.

## Hedef ve yol haritası

Kullanıcının hedefi (2026-09-24): **otonom** bir hexapod. ROS 2 + Gazebo
simülasyonunda geliştirilip Raspberry Pi 4'e aktarılacak; farklı zemin ve
zorluklarda kendi çözümünü üreten (RL ile öğrenilmiş) bir sistem.

Önce yazılım, sonra donanım. Yazılım görevleri Görkem (G) ve Samet (S)
arasında bölüşüldü; donanım işleri **durduruldu**, ayrı bir vardiyada
(D1–D12) yapılacak — bkz. GOREVLER.md. Görkem robotu kurmadı; ondan
donanım işi isteme. Yazılım CAD geometrisiyle simülasyonda ilerler;
donanım bilgileri config'e sonradan girilir.

1. ✅ Servo sürücü katmanı, kalibrasyon ve kanal haritası araçları
2. ✅ Ters/düz kinematik + gövde pozu (`hexapod_kinematics`)
3. ✅ URDF modeli (`hexapod_description`), RViz ve check_urdf'ten geçti
4. ✅ Gazebo dünyası + eklem komut arayüzü (`hexapod_gazebo`, docs/ARAYUZ.md)
5. ✅ Klasik yürüyüş (tripod) — çekirdek (`hexapod_gait`) + ROS düğümü
   (`hexapod_teleop`), Gazebo'da doğrulandı
6. ✅ RL ortamı + PPO eğitimi (G7 2026-09-28'de kapandı): düz zeminde en iyi `models/ppo_omni_250k`
   (tripod + öğrenilmiş düzeltme, her yöne); zeminde `models/ppo_lift50_3750k`
   (taban 50 mm ayak kaldırma, deneme zeminlerinde eğitildi, 60 mm engeller);
   arada `models/ppo_kaldirma35_250k` (öğrenilmiş kaldırma ~35 mm: düzde
   +%20 güçle 45 mm engeller). Mesafe sensörlü kaldırma refleksiyle
   (`hexapod_policy.lift_reflex`, S7/D8'i bekliyor) düzde 25 mm'nin
   enerjisi + engelde 50 mm'den iyi geçiş; refleks açıkken eğitilen
   `models/ppo_refleks_1500k` engelde en iyisi. **2026-09-28:** S5 zeminleri
   eğitime bağlı (`--terrains s5`); S6 "bitti" ölçümü (docs/olcumler/) iki
   ölçüm hatası düzeltilerek yapıldı (güç artık kontrol adımı ortalaması,
   ders 48; tripod:50 kırpılıyordu, ders 49): `ppo_kaldirma35_250k` +
   refleks ("egim" kipi, ders 50) tripod'u her zemin türünde geçiyor ya da
   eşit
7. 🔄 Pi 4'e aktarma: politika düğümü (`hexapod_policy`; mesafe sensörlü
   refleks `-p reflex:=true`, yerleşim robot.yaml'da null, D8), sensör
   düğümü (`hexapod_sensors`) ve sürücü düğümü (`hexapod_hardware`)
   yazıldı, simde çalışıyor; Pi'de denenmedi. **Yazılım aşamasının bütün
   görevleri bitti (2026-09-28); sırada donanım vardiyası.**

Gerçekçi beklenti: RL politikası eğitimde gördüğü zorluk türlerine karşı
sağlam olur, "her koşula" değil. Eğitim senaryoları neyi kapsarsa sistem
onu çözer. Eğitim bir PC'de yapılır; Pi 4 sadece eğitilmiş politikayı
çalıştırır.
