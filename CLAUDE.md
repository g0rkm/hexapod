# CLAUDE.md

Bu depoda çalışırken bilmen gerekenler. Ayrıntılı bağlam:
[hexapod-proje-brifi.md](hexapod-proje-brifi.md), kullanım: [README.md](README.md).

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

1. **Faturalar** (`yavuz selim/*.pdf`) — gerçekte ne alındığının tek kaydı
2. **CAD** (`Kerem Baltacı/`) — yalnızca mekanik geometri
3. Malzeme listesi txt'leri — **güvenilmez**, ikisi birbiriyle çelişiyor

Bilinen çelişkiler: PCA9685 (liste 1, gerçek **2**), VL53L0X (liste 2,
gerçek **3**), batarya (liste 2200 mAh, gerçek **2800 mAh**).

## CAD donör bir tasarım

`Kerem Baltacı/` altındaki her şey Sir Kuhnhero'nun "3D Printed Hexapod"
tasarımından (Printables 606030, CC BY-SA 4.0).

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
config/robot.yaml         robotun fiziksel tanımı
config/calibration.yaml   servo merkez/yön/limit — calibrate.py üretir
src/hexapod_driver/       ROS 2 (ament_python) paketi, çekirdeği saf Python
tools/calibrate.py        etkileşimli servo kalibrasyonu
tools/hwcheck.py          I2C tarama + config karşılaştırma
tools/cad_extract.py      CAD'den geometri türetme
tests/
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

## Şu an yazılmayacak olanlar

IK ve gait için geometrik bir engel kalmadı, ama kullanıcı söylemeden
başlanmaz. Kablolama (kart adresleri + kanal haritası) ve kalibrasyon
tamamlanmadan IK donanımda denenemez zaten.
