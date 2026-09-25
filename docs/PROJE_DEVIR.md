# Hexapod — Proje Devir Belgesi

> **Bu belgeyi okuyan Claude için:** Projeye yeni bir hesaptan/oturumdan
> devam ediyorsun. Önceki oturumların bütün bağlamı, kararları, bulunan
> hataları ve kullanıcının çalışma tercihleri burada. Herhangi bir iş
> yapmadan önce bu belgenin tamamını oku. `CLAUDE.md` bunun kısa
> özetidir; çelişki görürsen bu belge + koddaki güncel durum esastır.
>
> Son güncelleme: **2026-09-25** (2. oturum) · Testler: **114/114** (Linux; Windows'ta 102, Gazebo/Gymnasium testleri atlanır)
>
> Bu belgeyi güncel tut: önemli bir karar, bulunan bir hata ya da biten bir
> aşama olduğunda ilgili bölümü güncelle ve "Son güncelleme"yi değiştir.

---

## İçindekiler

1. [Kullanıcı ve çalışma tarzı](#1-kullanıcı-ve-çalışma-tarzı)
2. [Proje özeti ve hedef](#2-proje-özeti-ve-hedef)
3. [Mevcut durum ve yol haritası](#3-mevcut-durum-ve-yol-haritası)
4. [Donanım](#4-donanım)
5. [Geometri ve koordinat çerçeveleri](#5-geometri-ve-koordinat-çerçeveleri)
6. [IK ↔ kalibrasyon sözleşmesi](#6-ik--kalibrasyon-sözleşmesi)
7. [Yazılım mimarisi ve depo](#7-yazılım-mimarisi-ve-depo)
8. [Araçlar](#8-araçlar)
9. [Çalışma ortamı](#9-çalışma-ortamı)
10. [Git, güvenlik ve kişisel veri](#10-git-güvenlik-ve-kişisel-veri)
11. [Karar günlüğü](#11-karar-günlüğü)
12. [Bilinen tuzaklar ve çıkarılan dersler](#12-bilinen-tuzaklar-ve-çıkarılan-dersler)
13. [Açık kalan işler](#13-açık-kalan-işler)
14. [Sıradaki iş için hazır plan: URDF + Gazebo](#14-sıradaki-iş-için-hazır-plan-urdf--gazebo)
15. [Depoda olmayan kaynakların özeti](#15-depoda-olmayan-kaynakların-özeti)
16. [Yeni oturumda ilk adımlar](#16-yeni-oturumda-ilk-adımlar)

---

## 1. Kullanıcı ve çalışma tarzı

**Kim:** Görkem Mutlu. Depo sahibi, GitHub kullanıcı adı `g0rkm`, commit
e-postası `gorkemmutlu227@gmail.com`. Proje bir üniversite kulübü takım
projesi (TÜBİTAK Milli Teknoloji Kulüpler Birliği, Kulüp Geliştirme
Desteği başvurusu). Görkem yazılım tarafını Claude ile yürütüyor.
Takım arkadaşı **Samet** de yazılım tarafında: yazılım görevleri Görkem
ile Samet arasında bölüşüldü (2026-09-24, kullanıcının kararı). Donanım
işleri **durduruldu** ve ayrı bir donanım vardiyasına taşındı. Görev
sahipleri ve bağımlılıklar [GOREVLER.md](../GOREVLER.md)'de.

**Önemli:** Görkem robotu **kendisi monte etmedi**. Donanım, lehim, kablo
ve terminal işlerinde deneyimli değil ve bu tür işler istenince bunalıyor
(bir keresinde açıkça "ben nasıl yapacam bilmiyorum, ben birleştirmedim
aleti, yazılımı elimizdeki bilgilerle yapamaz mıyız" dedi). Donanım işi
robotu kuran takım arkadaşına kalıyor.

**İletişim tercihleri:**
- Türkçe konuş. Samimi ve doğrudan olabilirsin.
- **Kısa, madde madde** cevap ("kısaca yaz madde madde"). Uzun tablolu,
  jargonlu talimatları anlamadığını söyledi.
- Bir şey anlatman gerekiyorsa **çok basit** anlat ("mala anlatır gibi"):
  her adım tek eylem, komutlar ayrı kod bloklarında, terim kullanırsan tek
  cümleyle açıkla.
- Ondan bir şey istemeden önce: **bu işi bir araçla otomatikleştirebilir
  miyim?** Otomatikleştirilebiliyorsa kullanıcıya talimat yazmak yerine
  aracı yaz (örnek: kabloları elle takip ettirmek yerine `map_channels.py`).
- Gereksiz soru sorma. Makul varsayımı yap, söyle, devam et. Robot simetrik
  olduğu için "ön neresi" gibi sorular gereksizdi, sordum ve kullanıcı
  bunu belirtti.
- Donanım/ölçüm işi isteme. Geometri CAD'den geliyor (bkz. §11).

**Git tercihleri (kesin):**
- Commit mesajları **Türkçe ve detaylı**: ne değiştiğini değil, **neden**
  öyle yapıldığını da anlatsın.
- Commit'lerde **Claude imzası YOK**: `Co-Authored-By`, "Generated with
  Claude Code" ve benzeri satırlar yasak. Yazar yalnızca Görkem.
- Değişiklikleri **mantıklı, ayrı commit'lere** böl (düzeltme, özellik,
  belge ayrı).
- **Push'u kullanıcı istediğinde** yap. Şimdiye kadar hep "commit atıp
  pushla" diyerek istedi; istemeden push etme, gerekirse tek satırla sor.

---

## 2. Proje özeti ve hedef

Altı bacaklı (hexapod) bir robot. Her bacakta 3 eklem (coxa, femur,
tibia), toplam **18 servo**.

**Hedef (kullanıcının 2026-09-24'te doğruladığı hâliyle):**
> Otonom bir sistem. ROS 2 üzerinden Gazebo simülasyonu ile geliştirilip
> Raspberry Pi 4'e aktarılacak; her koşulda ve her zorlukta çözüm üreten
> bir sistem.

TÜBİTAK başvurusundaki yöntem (bkz. §15): Gazebo'da sanal IMU ve temas
sensörleriyle donatılmış bir hexapod modeli; Python tabanlı bir RL ajanı
(**Stable-Baselines3, PPO**) robotun devrilmeden, en az enerjiyle ve en
hızlı ilerlemesini ödüllendiren bir fonksiyonla eğitilecek. Sabit
yürüyüş desenleri (CPG) yerine değişken zemine uyum sağlayan, öğrenilmiş
bir yürüyüş hedefleniyor. Eğitim simülasyonda yapılıp gerçek robota
aktarılacak (sim-to-real).

**Gerçekçi beklenti (kullanıcıya söylendi):** RL politikası eğitimde
gördüğü zorluk türlerine karşı sağlam olur, "her koşula" değil. Eğitim
senaryolarının kapsamı (eğim, engebe, kaygan zemin, itme, kütle/sürtünme
değişimi) sistemin neyi çözebileceğini belirler. Eğitim PC'de yapılır;
Pi 4 yalnızca eğitilmiş politikayı çalıştırır.

**Strateji: önce yazılım, sonra donanım.** Her şey CAD geometrisiyle
simülasyonda geliştiriliyor. Kablolama ve kalibrasyon yazılımın önünde
engel değil; robotu kuran kişi tarafından yapılıp config'e girilecek.

**Asıl brif:** `docs/hexapod-proje-brifi.md`. Başlangıç belgesi. Şu kısımları
artık geçersiz ya da güncellendi:
- Mimari: brifteki düz, katmanlı Python yığını yerine **ROS 2 + Gazebo +
  RL** (TÜBİTAK başvurusu esas alındı). Katman sırası yine geçerli.
- "Segment uzunluklarını kumpasla ölç" notu: **iptal**, CAD kullanılıyor.
- Brifteki "kesinleşmemiş" maddeler (DOF, servo arayüzü) artık kesin:
  3 DOF, PCA9685 üzerinden PWM.

---

## 3. Mevcut durum ve yol haritası

| # | Aşama | Durum |
|---|---|---|
| 1 | Servo sürücü katmanı (`hexapod_driver`) | ✅ bitti, testli |
| 1b | Kalibrasyon, kanal haritası, donanım kontrolü, CAD çıkarım araçları | ✅ bitti |
| 2 | Ters/düz kinematik + gövde pozu (`hexapod_kinematics`) | ✅ bitti, testli |
| 3 | **URDF modeli** (robot.yaml'dan, STL'ler görsel mesh) | ✅ testli; RViz ve check_urdf'ten geçti |
| 4 | Gazebo dünyası + ROS 2 kontrol arayüzü | ✅ robot Gazebo'da doğuyor ve ayağa kalkıyor (G5) |
| 5 | Klasik yürüyüş (tripod) — RL için referans ve yedek | ⛔ |
| 6 | RL ortamı (Gymnasium) + PPO eğitimi, değişken zeminler | ⛔ |
| 7 | Pi 4'e aktarma: politika + ROS 2 düğümleri + gerçek sürücü | ⛔ |

**Görev dağılımı:** [GOREVLER.md](../GOREVLER.md). İki bölüm: şimdiki
**yazılım aşaması** (G = Görkem, S = Samet) ve **⏸ durdurulmuş donanım
vardiyası** (D1–D12). Her görev hangi görevi beklediğini ve hangisini
açtığını söyler. Bir görev bittiğinde durumunu orada güncelle.

**Durdurulmuş donanım işleri:** bkz. §13.2 ve GOREVLER.md D1–D12.

---

## 4. Donanım

### 4.1 Gerçekte alınanlar (faturalarla doğrulandı)

Faturalar **tek güvenilir kaynak**. Depoda değiller (kişisel veri
içeriyorlar, bkz. §10); içerikleri burada özetlendi. Alımlar Temmuz sonu –
Ağustos 2026.

| Parça | Adet | Not |
|---|---|---|
| Raspberry Pi 4, 4 GB | 1 | Ana kontrolcü |
| Raspberry Pi Camera Module V2 | 1 | CSI |
| MG996R servo, **180 derece** sürüm | 20 | 18 kullanım + 2 yedek |
| PCA9685 16 kanal I2C PWM sürücü | **2** | 18 servo tek karta sığmaz |
| Gravity BNO055 + BMP280 10DOF AHRS (IMU) | 1 | I2C |
| VL53L0X uçuş süresi mesafe sensörü | **3** | I2C, üçü de 0x29'da doğar |
| 7.4V 2S LiPo, **2800 mAh**, 40C | 2 | |
| 300W 20A DC-DC buck (voltaj düşürücü) | 3 | |
| Çift USB çıkışlı 5V 3A regülatör | 1 | Muhtemelen Pi beslemesi |
| JST konnektör takımı, 12 AWG silikon kablo | — | |
| M3 ve M5 pirinç insert somun | — | Montaj |
| PETG filament (siyah ×2, beyaz ×2, 1 kg) | 4 | Parçalar **PETG** basıldı, PLA değil |

MG996R katalog değerleri (simülasyon için kullanılabilir, `measured:
false` olarak işaretle): kütle ~55 g; tork ~9.4 kg·cm @4.8 V, ~11 kg·cm
@6 V; hız ~0.17 s/60° @4.8 V, ~0.14 s/60° @6 V; çalışma gerilimi 4.8–7.2 V.
Mil dişlisi **25 diş** (diş başına 14.4°).

### 4.2 Malzeme listeleri güvenilmez

Depoda (`docs/malzeme/`) iki malzeme listesi var ve **ikisi de hatalı**:
- `gömülü malzemeler listesi.txt`: faturalara daha yakın (VL53L0X, 300W
  buck) ama PCA9685'i **1**, VL53L0X'i **2**, bataryayı **2200 mAh**
  gösteriyor. Doğrusu 2, 3 ve 2800 mAh.
- `gömülü malzemeler listesi (alternatif).txt`: TF-Luna LiDAR, LM2596,
  3 batarya. **Bunlar hiç alınmadı.**
- `Malzeme Listesi.txt`: mekanik montaj listesi (vida, insert, rulman).
  "Total: 18x MG996R Servo Horn" satırı 3 DOF'nin ilk kanıtıydı.

Çelişki olursa sıra: **fatura > CAD > listeler.**

### 4.3 CAD donör bir tasarım

`cad/Hexapod/` ve `cad/Baskı Dosyaları/` altındaki her şey **Sir Kuhnhero, "3D
Printed Hexapod", Printables model 606030, CC BY-SA 4.0** tasarımından.
Lisans ve atıf belgesi: `cad/Hexapod/606030-*.pdf` (depodaki tek PDF).

- **Mekanik geometri geçerli**, robot bu parçalarla basıldı.
- **Elektronik geçerli DEĞİL.** Donör tasarım STM32 BluePill + 4×18650
  hücre + XL4016 8A buck kullanıyor (`body-v35.step` içinde görünür). Bu
  robot Pi 4 + 2S LiPo kullanıyor. CAD'deki kart yerleşimine bakarak güç
  ya da kontrolcü varsayımı yapma. Donör tasarımdaki 2× PCA9685 ise
  bizimkiyle örtüşüyor.
- Donör tasarımcının kendi belirttiği sorunlar (PDF'te): **akım
  sıçramalarında servo buck'ının yetmemesi** ve **STM32 ile servo
  sürücüler arasındaki I2C hattının çökmesi** (ekranlı kabloyla
  çözülmüş). İkisi de bu robot için gerçek risk.

### 4.4 Elektrik ve güvenlik notları

- **Servo gerilimi:** dolu 2S LiPo 8.4 V verir, MG996R en fazla 7.2 V
  kaldırır. Servolar **voltaj düşürücü üzerinden ~6 V** ile beslenmeli.
  Bağlamadan önce multimetreyle ayarlanmalı.
- **Brownout riski (brifin 1 numaralı riski):** 18 servo yük altındayken
  gerilim çökebilir ve Pi resetlenir; bu yazılım hatası gibi görünür ve
  günlerce kovalanır. Servo ve Pi beslemesi ayrı olmalı, toprak ortak
  olmalı.
- **PCA9685 VCC Pi'nin 3.3 V'una (pin 1) bağlanmalı, 5 V'a DEĞİL.** Kart
  I2C hatlarını VCC'ye çeker; 5 V verilirse Pi'nin GPIO'ları zarar görür.
  Bağlantı: VCC→pin 1, SDA→pin 3, SCL→pin 5, GND→pin 6.
- **PCA9685 adresleri:** iki kart da fabrikadan **0x40** gelir. Birinin
  **A0** lehim noktası birleştirilince **0x41** olur. Her PCA9685 ayrıca
  **0x70**'te (ALLCALL, ortak çağrı adresi) cevap verir; bu kartın kendi
  adresi değildir.
- **VL53L0X:** üçü de 0x29'da doğar; ayrı XSHUT pinleriyle sırayla
  uyandırılıp yeniden adreslenmeleri gerekir. BNO055 de ADR yüksekken
  0x29'dadır, çakışmaya dikkat.
- **MG996R kablo renkleri:** kahverengi = GND, kırmızı = V+, turuncu =
  sinyal (PWM).

---

## 5. Geometri ve koordinat çerçeveleri

### 5.1 Değerler (`config/robot.yaml`)

Hepsi CAD'den türetildi, hepsi `measured: false`. Yeniden üretmek için
`python tools/cad_extract.py`.

| Parametre | Değer | Kaynak |
|---|---|---|
| coxa | **50.0 mm** | STEP: J1 ekseninden J2 eksenine yatay (tam 50.000) |
| femur | **80.0 mm** | STEP: J2→J3 (79.997, tasarım değeri 80) |
| tibia | **126.6 mm** | STL + STEP montaj dönüşümü: J3 → ayak kapağının en uç noktası (126.635) |
| femur_joint_z_offset | **−10.05 mm** | Femur eklemi, bacak montaj referans düzleminin 10.05 mm altında |
| coxa_axis_radius | **100.0 mm** | Coxa eksenleri gövde merkezinden 100 mm'de (montaj orijinleri 120'de, eksen 20 mm içeride) |
| Bacak azimutları (CAD) | 0, ±60, ±120, 180° | Radyal altıgen yerleşim |
| Ayak ucu yuvarlaklık yarıçapı | ~5.1 mm | STL'e daire uydurma (bilgi amaçlı, config'de alan yok) |

Diğer CAD bulguları: tibia gövdesi (`pla_tibia_main`) J3'ten 118.62 mm'de
bitiyor, ayak kapağı (`pla_tibia_tip`) onun ucuna geçip 8 mm taşıyor ve
yere değen parça o. Gövdenin kendi bacak montaj vida çemberi 80 mm
yarıçapta (`body-v35.step`).

### 5.2 Türetme yöntemi

- STEP assembly ağacı çözüldü (`NEXT_ASSEMBLY_USAGE_OCCURRENCE` +
  `ITEM_DEFINED_TRANSFORMATION`); her parçanın montaj çerçevesindeki
  konumu hesaplandı (`tools/cadlib/assembly.py`).
- Her eklemin dönme ekseni = o eklemin **servo horn'u ile karşısındaki
  bushing'i birleştiren doğru**. Bacak yerel çerçevesinde (`leg-v2-v20.step`;
  +Y yukarı, bacak −Z yönünde dışarı uzanıyor):
  - J1 coxa (yaw): yerel Y boyunca, (x=0, z=+20)
  - J2 femur (pitch): yerel X boyunca, (y=−10.05, z=−30)
  - J3 tibia (pitch): yerel X boyunca, (y=+10.66, z=−107.27)
- Tibia: **basılan STL'ler** (`pla_tibia_tip.stl`) kullanıldı. STL'ler
  parçanın kendi çerçevesinde; STEP'teki montaj dönüşümü uygulanınca
  bacak çerçevesine oturuyorlar (STEP yüzey köşeleriyle 0.004 mm örtüşme
  doğrulandı).
- Tam montaj (`hexapod-v8.step`): bacak orijinleri 120 mm yarıçapta; coxa
  eksenleri 100 mm'de. Altı bacağın da tam 100.0 çıkması araçta
  tutarlılık kontrolü olarak duruyor.

### 5.3 Neden kumpasla ölçülmüyor

Brif ölçmeyi öneriyordu; kullanıcı "çizimler varken fiziksel ölçüm şart
mı" diye sordu ve vazgeçildi. Hata bütçesi:

| Hata kaynağı | Ayak ucunda etkisi | Nasıl giderilir |
|---|---|---|
| Baskı toleransı | < 0.5 mm | gerek yok |
| Yuvarlak ayak ucunun kayması (~r·sin açı) | 1–3 mm | gerek yok |
| Servo horn'unun mile oturduğu diş (25 diş, 14.4°/diş) | **~10 mm'ye kadar** | **kalibrasyon** |

Doğruluk kalibrasyondan gelir. Robot yürürken ayakta tutarlı bir sapma
görülürse ancak o zaman ölçüme dönülür. **Kullanıcıdan kumpas ölçümü
isteme.**

### 5.4 Koordinat çerçeveleri

- **CAD çerçevesi** (`hexapod-v8.step`): +Y yukarı, X ve Z yatay. Bacak
  azimutu α = atan2(z, x). Bu açı **yukarıdan bakınca saat yönünde** artar.
- **Gövde çerçevesi** (ROS REP-103): +x ileri, +y sol, +z yukarı. Orijin,
  coxa eksenlerinin geçtiği çemberin merkezi, bacak montaj referans
  düzleminin yüksekliğinde. Açılar yukarıdan bakınca **saat yönünün
  tersine** artar.
- **Dönüşüm:** `gövde_azimut = forward_offset_deg − cad_azimut`.
  (İşaret ters, çünkü iki çerçevede açılar zıt yönlerde artıyor. İlk
  yazılan formül `cad − offset` idi ve **yanlıştı**; hiçbir kod
  kullanmadan düzeltildi. Yanlış hâli sol ve sağ bacakları yer
  değiştirirdi.)
- **`forward_offset_deg = 90`**: CAD +Z ileri kabul edildi. **Keyfi**,
  çünkü robot simetrik: altı bacağın geometrisi aynı ve 60° aralıklı.
  Kullanıcı da kamera yerinin fark etmediğini söyledi.
- **Bacak çerçevesi** (IK): orijin coxa'nın dikey ekseni üzerinde, femur
  ekleminin yüksekliğinde; +x bacak dümdüz dışarı (coxa=0 iken), +z yukarı.

### 5.5 Bacak tablosu

| id | CAD az. | Gövde az. | Konum | Fiziksel bant no.* | Aynalı |
|---|---|---|---|---|---|
| 0 | 0° | +90° | sol orta | 6 | hayır |
| 1 | 60° | +30° | sol ön | 1 | hayır |
| 2 | 120° | −30° | sağ ön | 2 | evet |
| 3 | 180° | −90° | sağ orta | 3 | evet |
| 4 | −120° | −150° | sağ arka | 4 | evet |
| 5 | −60° | +150° | sol arka | 5 | hayır |

\* Fiziksel bant no.: robotu kuran kişi iki bacak arasına "ÖN" bandı
yapıştırıp yukarıdan bakarak saat yönünde 1–6 numaralandıracak (1 sol ön,
2 sağ ön, 3 sağ orta, 4 sağ arka, 5 sol arka, 6 sol orta).
`tools/map_channels.py` bu numaraları soruyor.

**Aynalı bacaklar:** kinematik olarak normal bacakla aynıdır (eklem
eksenleri bacak düzleminde aynı yerde). Fark yalnızca servo yön
işaretlerinde olabilir; o da kalibrasyonda eklem eklem belirlenir.
Fiziksel robot başka bir "ön" ile etiketlenirse "aynalı" bilgisi yanlış
tarafı gösterebilir. IK ve kalibrasyon buna dayanmaz; yalnızca
simülasyon modeli dayanır (aynalı bacaklarda kütle dağılımı y'de çevrilir,
bkz. §7.7).

---

## 6. IK ↔ kalibrasyon sözleşmesi

Kalibrasyonda kaydedilen **merkez** (`center_us`) ile IK'daki **0 açısı
aynı duruştur**. Biri değişirse öteki de değişmeli, yoksa her ayak
sistematik olarak kayık basar.

| Eklem | 0 açısı | + yönü |
|---|---|---|
| coxa | bacak gövdeden dümdüz dışarı | yukarıdan bakınca saat yönünün tersi |
| femur | femur yere paralel | bacak yukarı kalkar |
| tibia | tibia femura dik (femur yataysa dümdüz aşağı) | diz açılır, ayak dışarı gider |

Kaynaklar: `src/hexapod_kinematics/hexapod_kinematics/leg.py` modül
açıklaması ve `tools/calibrate.py` yardım metni (ikisi aynı tabloyu
gösterir).

Sıfır duruşunda ayak bacak çerçevesinde **(130, 0, −126.6) mm**'de.
Servo sinyali: `pulse_us = center_us + direction × açı × us_per_deg`.

---

## 7. Yazılım mimarisi ve depo

### 7.1 Temel ilkeler

1. **Eksik değer uydurulmaz.** `robot.yaml`'da her sayı `{value, source,
   measured}` üçlüsü. `value: null` bilinmiyor demektir; okunmaya
   çalışılırsa `MissingValue` fırlar ve değerin nereden geleceğini söyler.
   Hiçbir katman null'a varsayılan koyamaz.
2. **`measured: false`** = CAD'den ya da veri sayfasından türetildi,
   robottan doğrulanmadı. Çalışır ama "doğru" sayılmaz.
3. **Donanıma dokunan çekirdek saf Python, ROS'a bağımlı değil.** Paketler
   ROS 2 `ament_python` olarak derlenir ama tezgâh üstü araçlar ve
   testler ROS kurulu olmadan çalışır. ROS sarmalayıcıları ayrı olmalı.
4. **Aynı anda tek servo** (kalibrasyon ve kanal haritasında). Brownout
   riskine karşı.
5. **Mutlak darbe sınırı:** `servo.pulse_us_hard_limits` (500–2500 µs)
   dışına hiçbir koşulda darbe gitmez.
6. **Kırpma yok:** IK erişilemeyen hedefi en yakın noktaya kırpmaz,
   `ReachError` fırlatır.
7. **Donanımsız geliştirme:** `DryRunBackend` I2C yazmalarını kaydeder;
   bütün testler ve `--dry-run` bayrakları bunu kullanır.

### 7.2 Katman sırası

```
Otonomi / davranış          (ileride: RL politikası, sensör füzyonu)
Kumanda + telemetri         (ileride)
Gait motoru                 (ileride: tripod, sonra RL)
Ters kinematik + gövde pozu (hexapod_kinematics)   ✅
Servo sürücü katmanı        (hexapod_driver)       ✅
```

Her katman altındakine bağımlı, üstündekinden habersiz.

### 7.3 Depo yapısı

```
hexapod/
├── CLAUDE.md               kısa özet + kurallar (Claude Code otomatik okur)
├── README.md               insanlar için genel bakış
├── docs/
│   ├── PROJE_DEVIR.md      bu belge
│   ├── hexapod-proje-brifi.md  ilk brif (bir kısmı geçersiz, bkz. §2)
│   └── malzeme/            Malzeme Listesi.txt, gömülü malzemeler listesi*.txt (güvenilmez, §4.2)
├── config/
│   ├── robot.yaml          robotun fiziksel tanımı (geometri, kablolama, limitler, simulation)
│   └── calibration.yaml    (henüz yok) calibrate.py üretecek; GİT'TE TUTULMALI
├── src/
│   ├── hexapod_driver/     servo sürücü katmanı (ament_python)
│   │   └── hexapod_driver/
│   │       ├── config.py       robot.yaml yükleyici, Value, unknowns()/unverified()/wiring_gaps()
│   │       ├── calibration.py  calibration.yaml okuma/yazma (atomik)
│   │       ├── pca9685.py      NXP PCA9685 sürücüsü (25 MHz, 12 bit, 50 Hz -> prescale 121)
│   │       ├── backends.py     SMBusBackend (smbus2) ve DryRunBackend
│   │       ├── servo_bus.py    eklem adı -> kart/kanal -> darbe
│   │       └── errors.py       HexapodError, ConfigError, MissingValue, LimitError, BackendError
│   ├── hexapod_kinematics/ ters/düz kinematik + gövde pozu (ament_python)
│   │   └── hexapod_kinematics/
│   │       ├── leg.py      tek bacak IK/FK, sıfır duruşu ve yön tanımları
│   │       ├── body.py     altı bacak, gövde çerçevesi, BodyPose
│   │       └── errors.py   ReachError
│   └── hexapod_description/ simülasyon modeli, URDF'in girdisi (ament_python)
│       ├── hexapod_description/
│       │   ├── model.py    RobotModel: kütle/atalet/çarpışma/limit, SI birimleri
│       │   └── meshes.yaml görsel mesh yerleşimi (cad_sim_model.py üretir)
│       └── meshes/         STL kopyaları (cad_sim_model.py --copy-meshes üretir)
├── tools/                  komut satırı araçları
│   ├── map_channels.py     hangi servo hangi kanalda — kıpırdatıp sorar
│   ├── calibrate.py        etkileşimli servo kalibrasyonu
│   ├── hwcheck.py          I2C tarama + robot.yaml karşılaştırma
│   ├── cad_extract.py      CAD'den geometri türetme
│   ├── cad_sim_model.py    CAD'den kütle, atalet, çarpışma kutuları, mesh yerleşimi
│   └── cadlib/             iki CAD aracının ortak kütüphanesi
│       ├── step.py         minimal STEP AP214 ayrıştırıcı
│       ├── assembly.py     montaj ağacı -> Assembly / Occurrence
│       ├── transform.py    3x4 katı dönüşüm cebiri
│       ├── mesh.py         STL okuma, Box, MassProps
│       └── frames.py       CAD <-> IK/gövde çerçeveleri
├── tests/
│   ├── test_servo_layer.py   30 test
│   ├── test_kinematics.py    17 test
│   ├── test_map_channels.py   9 test
│   └── test_description.py    7 test
├── conftest.py             src/ paketlerini sys.path'e ekler (ROS'suz test için)
├── pytest.ini, .gitignore, .gitattributes
└── cad/                    CAD ve baskı dosyaları (depoda, ~345 MB)
    ├── Hexapod/            STEP/3MF/F3Z/STL + 606030-*.pdf (Printables lisans/atıf)
    └── Baskı Dosyaları/    basılan STL'ler

Git'e GİRMEYENLER: faturalar, ekran görüntüleri (kişisel veri), Python
önbellekleri. CAD 2026-09-24'ten beri depoda (§10).
```

### 7.4 `hexapod_driver`

- `RobotConfig.load(path=None)`: `config/robot.yaml`'ı bulur
  (`HEXAPOD_CONFIG_DIR` ortam değişkeni ya da yukarı doğru arama).
  `segments`, `coxa_axis_radius`, `femur_joint_z_offset`,
  `forward_offset_deg`, `standing_height`, `total_mass_kg` (hepsi `Value`),
  `legs` (`LegSpec`), `drivers` (`DriverSpec`), `joints` (`JointSpec`:
  driver, channel, limit_min, limit_max). Yardımcılar: `joint(leg, name)`,
  `unknowns()`, `unverified()`, `wiring_is_complete()`, `wiring_gaps()`.
  Not: `sensors` bölümündeki null'lar (IMU adresi, VL53L0X pinleri)
  `unknowns()` tarafından henüz izlenmiyor, sadece `raw` içinde.
- `Calibration`: `config/calibration.yaml`. Eklem anahtarı `leg{N}_{eklem}`
  (ör. `leg0_coxa`). Alanlar: `center_us`, `direction` (+1/−1),
  `us_per_deg`, `limit_min_us`, `limit_max_us`, `calibrated_at`.
  `set_center`, `set_direction`, `set_limit`, `set_span(pulse, derece)`,
  `limits_deg()` (eksik bilgiyle None döner, uydurmaz), `save()` (.tmp
  sonra rename).
- `ServoBus(config, calibration, backend=None, dry_run=False)`: `start()`
  kart adreslerini `require()` eder. `set_pulse_us(leg, eklem, us)` ham
  darbe (sadece kalibrasyon araçları). `set_angle(leg, eklem, derece)`
  kalibrasyon ve limitler tam değilse `MissingValue`/`LimitError` fırlatır.
  `release`, `release_all`, `active_joints`.

### 7.5 `hexapod_kinematics`

- `LegGeometry(coxa, femur, tibia)`, `JointAngles(coxa, femur, tibia)`
  (derece), `ZERO`.
- `forward(geom, angles) -> (x, y, z)` ve `inverse(geom, x, y, z) ->
  JointAngles`, bacak çerçevesinde.
  - Diz-yukarıda çözüm dalı.
  - Erişim dışında `ReachError` (kırpmaz).
  - Femur açısı (−180, 180] aralığına sarılır (bkz. §12, 300° hatası).
  - Varsayım: ayak coxa ekseninin dışında. Eksenin arkasındaki hedef
    "bacak 180° dönmüş" pozdan geometrik olarak ayırt edilemez; gerçek
    robotta o bölge gövdenin altı.
- `HexapodKinematics.from_config(config)`: `mounts` (her bacak için
  `LegMount(x, y, z, yaw)`, gövde çerçevesinde; z = −10.05).
  `to_leg_frame`, `to_body_frame`, `neutral_stance()` (bütün eklemler 0
  iken ayaklar; yürüyüş duruşu DEĞİL), `forward(açılar)`,
  `inverse(ayaklar, pose=BodyPose())`. Hata bacak numarasını söyler.
- `BodyPose(x, y, z, roll, pitch, yaw)`: mm ve derece. Ayaklar dünyada
  sabit, gövde kayar/döner. Dönme sırası ZYX (REP-103).

### 7.6 Testler

```bash
python -m pytest -q          # depo kökünden; 88 test, ~2.5 sn, donanım gerekmez
```

Öne çıkan testler: eksik değerde `MissingValue`; PCA9685 prescale (50 Hz
→ 121) ve darbe→sayaç (1500 µs → 307); iki kart arası yönlendirme; her
kanalın kıpırdatma sonrası kapatıldığı (yazmaç düzeyinde); IK gidiş-dönüş
(270 poz, 1e-6°); "sol" etiketli bacakların gerçekten +y'de olması; gövde
pozu gidiş-dönüşü; simülasyon ataletlerinin fiziksel olması (pozitif
tanımlı, üçgen eşitsizliği); aynalı bacakta y'nin çevrilmesi.

### 7.7 `hexapod_description`

Simülasyon modeli: veri katmanı (`model.py`) + URDF üreticisi (`urdf.py`).
Gazebo/RL ortamı da sayıları buradan alacak.

- `build_urdf(model, meshes=None)`: URDF metni. Ağaç: `base_link` (ataletsiz;
  KDL ataletli kökü desteklemiyor) → sabit `body` (gövde kütlesi) ve her
  bacak için `leg{i}_coxa|femur|tibia` + sabit `leg{i}_foot` (yalnız
  çerçeve). Eklemler `leg{i}_coxa_joint` vb. Ayak küresi tibia linkinde.
  `meshes` yoksa görseller çarpışma kutularından.
- `MeshSet.load(uri_prefix)`: `meshes.yaml` + `package://...` ya da `file://...`.
- CLI: `python tools/make_urdf.py -o x.urdf [--meshes none|package|file]`
  (ROS'ta `ros2 run hexapod_description make_urdf`).
- `launch/display.launch.py`: robot_state_publisher + eklem kaydırıcıları
  + RViz; URDF açılışta robot.yaml'dan üretilir. **Henüz denenmedi** (ROS yok).
- `tests/test_urdf.py`: URDF'i üreticiden bağımsız bir zincir hesaplayıcıyla
  okur; ayak konumları `hexapod_kinematics.forward` ile 300 pozda 1e-9 m
  içinde aynı. Kasıtlı bozulmalar (eksen ters, tibia +1 mm) yakalanıyor.

Veri katmanı:


- `RobotModel.from_config(config)`: SI birimlerinde (m, kg, kg·m², rad).
  Geometri ve bacak montajları **doğrudan `HexapodKinematics`'ten**; robot.yaml
  ikinci kez okunmuyor ki simülasyon ile IK ayrışmasın.
- `links`: body/coxa/femur/tibia için `Inertial` (kütle, ağırlık merkezi,
  atalet) + `CollisionBox` listesi; normal bacak için. `leg_link(id, ad)`
  aynalı bacaklarda y'yi çevirir (com.y, ixy, iyz, kutu merkezi).
- `limits[(bacak, eklem)]`: kalibrasyon limitleri varsa onlar, yoksa
  `simulation.provisional_joint_limits_deg` (`provisional=True`).
  `provisional_joints()` hangilerinin geçici olduğunu söyler.
- `effort` / `velocity`: MG996R katalog (6 V).
- `meshes.yaml`: görsel mesh yerleşimi, `tools/cad_sim_model.py` üretir.
  STL'ler `--copy-meshes` ile `meshes/` altına kopyalanır (depoda).
- Link çerçeveleri (URDF ile aynı olacak): coxa = bacak çerçevesi; femur
  orijini J2'de, +x femur boyunca; tibia orijini J3'te, tibia −z boyunca.
  Femur ve tibia eksenleri **−y** (femur + = yukarı, tibia + = ayak dışarı).

---

## 8. Araçlar

Hepsi `python tools/<araç>.py` ile çalışır. Hepsinde `--dry-run` var
(donanıma yazmaz). Windows konsolunda Türkçe karakter bozulmasın diye
stdout'u UTF-8'e zorlarlar.

### `map_channels.py` — kanal haritası
Servo kartlarını I2C'de kendisi bulur (0x70 hariç), her kanaldaki servoyu
sırayla ±100 µs kıpırdatır; kullanıcı kıpırdayan eklemi yazar:
`1c` = bacak 1 coxa, `3f` = femur, `6t` = tibia, Enter = boş kanal,
`t` = tekrar, `q` = bitir. Sonunda haritayı basar. Kabloların hangi
sırayla takıldığı önemli değil. Robot kutu üstünde, bacaklar havada
olmalı (servo ilk sinyalde orta konuma zıplar). Tek kart bulursa
çalışmayı reddeder (iki kart aynı adresteyse harita yanlış çıkar).
**Çıktısı `robot.yaml`'a henüz otomatik yazılmıyor**; çıktı geldiğinde
bacak numaraları §5.5 tablosuyla id'lere çevrilip elle girilecek.

### `calibrate.py` — kalibrasyon
Kablolama (`drivers[*].address`, `joints[*].driver/channel`) dolu
değilse başlamaz ve eksikleri listeler. Aynı anda tek servo beslenir.
Başlangıç darbesi 1500 µs (RC nötrü, robot parametresi değil).
Komutlar: `+`/`-`, `+N`/`-N`, `=N`, `step N`, `c` (merkez = sıfır duruşu,
kaydedip sıradakine geçer), `dir +`/`dir -`, `off`, `span D` (şu anki
konum merkezden D derece → us/derece), `limit min`/`limit max`,
`n`/`p`/`go L J`, `list`, `gaps`, `limits` (robot.yaml formatında
basar), `save`, `q`, `?`.
Limit bulma sırası: `c` → `dir` → `span` → `limit min`/`limit max` →
`limits` çıktısını robot.yaml'a yapıştır.

### `hwcheck.py` — I2C kontrolü
Veri yolunu tarar, bilinen adresleri tahmin eder (BNO055 0x28/0x29,
VL53L0X 0x29, BMP280 0x76/0x77, PCA9685 0x40–0x7F; 0x70 ALLCALL olarak
etiketlenir), robot.yaml ile karşılaştırır, adres önerir. İki kart aynı
adresteyse A0'ın lehimlenmesini söyler.

### `cad_extract.py` — geometri türetme
`--leg-only`, `--body-only`, `--verbose`. CAD'i `cad/Hexapod/` (ya da eski
`Hexapod/`, `Kerem Baltacı/Hexapod/`) altında arar. Çıktı: coxa 50.000, femur
80.000, tibia 126.635 mm, altı bacakta coxa yarıçapı 100.0; iki
tutarlılık kontrolü. Tam montaj çözümlemesi (85 MB STEP) ~5 sn.

### `cad_sim_model.py` — simülasyon verisi
Basılan parçaların STL'lerinden link başına kütle, ağırlık merkezi,
atalet ve çarpışma kutularını hesaplar; `robot.yaml`'a yapıştırılacak
`simulation.links` bloğunu basar, `meshes.yaml`'ı yazar. `--copy-meshes`
STL'leri pakete kopyalar. Girdiler `simulation.mass_inputs`'ta (PETG
yoğunluğu, doluluk oranı, servo kütlesi/boyutu, elektronik kütlesi).
Kontroller: segment uzunlukları cad_extract ile aynı çıkmalı; CAD coxa
eksenleri robot.yaml montajlarıyla 0.000 mm örtüşmeli. Parça→link
ataması ve gerekçesi aracın başındaki açıklamada (coxa servosunun
gövdesi bacakla döner; Coxa_top gövdeye bağlı). ~8 sn.

Her iki CAD aracı da ortak kodu `tools/cadlib/`'den alır (STEP
ayrıştırma, montaj ağacı, dönüşümler, STL/kütle, çerçeveler).

### `make_urdf.py` ve `preview_urdf.py` — URDF
`make_urdf.py` robot.yaml'dan URDF yazar (ROS'suz). `preview_urdf.py`
URDF'i okuyup STL'leri link konumlarına yerleştirerek PNG çizer (numpy +
matplotlib; tek görünüş ~1 dk, `--collision` ~1 sn). RViz'in yerini
tutmaz; mesh yerleşimini gözle kontrol etmek için.

### `wsl/ros_kurulum.sh` — ROS 2 kurulumu (G4)
WSL Ubuntu 26.04'e ROS 2 Lyrical + Gazebo + ros2_control + RViz eklem
kaydırıcılarını kurar. Adımlar resmi belgeden (ros2_documentation,
`lyrical` dalı, Ubuntu-Install-Debs.rst; docs.ros.org bot korumasıyla
erişimi engelliyor). Paketlerin resolute deposunda var olduğu doğrulandı.
sudo'yu bir kez sorar, tekrar çalıştırmak güvenli, sonunda kendini test
eder (talker → /chatter, `gz sim --version`). Günlük: /tmp/ros_kurulum.log.

---

## 9. Çalışma ortamı

| | Nerede | Sistem |
|---|---|---|
| Geliştirme + simülasyon + RL eğitimi | Kullanıcının PC'si, **WSL2** | **Ubuntu 26.04** |
| Robot | Raspberry Pi 4 | **Ubuntu Server 26.04 arm64** (planlanan) |

- **ROS 2: Lyrical Luth** (LTS, Mayıs 2031'e kadar), Ubuntu 26.04'ün
  birincil sürümü. **Gazebo Jetty** `ros-lyrical-desktop` ile birlikte
  geliyor. (İlk taslakta Jazzy/24.04 varsayılmıştı; PC'de 26.04 olduğu
  için değiştirildi.)
- **RL sanal ortamı** `~/hexapod_venv` (torch 2.14 CPU, SB3 2.9, Gymnasium 1.3;
  `tools/wsl/rl_kurulum.sh`). Kullanım: `source /opt/ros/lyrical/setup.bash;
  source ~/hexapod_ws/install/setup.bash; source ~/hexapod_venv/bin/activate`.
  python3-venv sistemde yoktu (sudo ister); betik get-pip.py ile sudo'suz kurdu.
  Eğitim çıktıları `~/hexapod_runs/<ad>/` (depoda değil).
- **WSL'de ROS 2 Lyrical + Gazebo 10.5 (Jetty) KURULU** (2026-09-24,
  `tools/wsl/ros_kurulum.sh`). Python 3.14, 8 çekirdek, 7 GB RAM (PC'nin yarısı).
  Kullanıcı adı `gorkem`. Paketler: `bash tools/wsl/derle.sh` → `~/hexapod_ws`
  (kaynaklar depoya sembolik bağlı, `--symlink-install`).
- Claude WSL'de sudo gerektirmeyen her şeyi çalıştırabilir: `wsl -e bash <betik>`.
  Tırnaklı uzun komutlar PowerShell→wsl geçişinde bozuluyor; betiği dosyaya yazıp
  çalıştır. Aynı anda birkaç simülasyon koşacaksa farklı `ROS_DOMAIN_ID` ver.
- **colcon derlemesini OneDrive klasöründe yapma**: `build/ install/ log/`
  OneDrive'a senkronlanır ve /mnt/c yavaştır. Çalışma alanı WSL'in kendi
  diskinde (ör. `~/hexapod_ws`) olmalı, kaynaklar depodan bağlanmalı.
- **PC:** Windows 11 Pro, Intel i5-10300H (8 thread), 16 GB RAM, NVIDIA
  GTX 1650 + Intel UHD. Gazebo ve PPO eğitimi için yeterli.
- **Depo yolu (Windows):** `C:\Users\gorke\OneDrive\Masaüstü\hexapod`
  (OneDrive altında). **GitHub:** https://github.com/g0rkm/hexapod
  (varsayılan dal `main`).
- **WSL'de `sudo` şifre ister.** Kurulum komutlarını kullanıcıya ver,
  şifreyi asla sen girme. Kullanıcı terminalde deneyimsiz; komutları tek
  tek, açıklamalı ver.
- **Ubuntu 24.04+ sistem Python'una `pip install` engelli** (PEP 668).
  Paketleri `apt` ile kur (`python3-yaml`, `python3-smbus2`) ya da venv
  kullan.
- **Pi'de I2C:** Ubuntu'da genelde açık gelir (`ls /dev/i2c-1`); kullanıcı
  `i2c` grubuna eklenmeli (`sudo usermod -aG i2c $USER`). Raspberry Pi OS'ta
  `raspi-config` ile açılır.
- **Satır sonları:** `.gitattributes` `eol=lf` zorluyor. Windows'ta CRLF ile
  commit'lenen bir betiğin shebang'i Linux'ta `python3\r` olur ve "bad
  interpreter" verir.
- **Kodlama:** Windows konsolu cp1254/cp857; Türkçe çıktı basan
  betiklerde `sys.stdout.reconfigure(encoding="utf-8")` kullanılıyor.
  Kabuktan uzun ve özel karakterli heredoc'lar bazen bozuluyor; o
  durumda dosyayı doğrudan yaz.

---

## 10. Git, güvenlik ve kişisel veri

- **Faturalar kesinlikle depoya girmez.** İçlerinde TCKN, ev adresi,
  telefon ve e-posta var (üçüncü kişilerin). GitHub'a giden kişisel veri
  fork/cache/indeks yüzünden geri alınamaz.
- `.gitignore` bunu **klasör adından bağımsız** koruyor: bütün `*.pdf`,
  `Ekran görüntüsü*`, `Screenshot*`, `yavuz selim/` dışarıda. Tek istisna
  `!cad/Hexapod/606030-3d-printed-hexapod-*.pdf`. **Bu kuralı gevşetme.**
  Paylaşılabilir bir PDF eklenecekse açık bir `!` istisnası yaz ve önce
  içinde kişisel veri olmadığından emin ol.
- **CAD binary'leri** (`*.step`, `*.f3z`, `*.3mf`, `*.stl`, toplam 345 MB)
  **2026-09-24'ten beri depoda** (`cad/`). Depo gizli olduğu ve iki kişi
  aynı dosyalarla çalışacağı için kullanıcı istedi. En büyük iki STEP 85 ve
  67 MB: GitHub 50 MB'ın üstünü uyarıyla kabul ediyor, 100 MB'ı reddediyor.
  Depo herkese açılacaksa (TÜBİTAK: açık kaynak) lisans (CC BY-SA 4.0)
  uygun ama Git LFS düşünülmeli. Faturalar İSE HÂLÂ depoya girmez.
- **`config/calibration.yaml` git'te TUTULMALI** (brifteki risk:
  kalibrasyon kayıt altına alınmazsa her seferinde sıfırdan başlanır).
  Yalnızca `.tmp` hâli yoksayılıyor.
- Push öncesi alışkanlık: `git diff --cached` ile eklenecek dosyaları ve
  boyutlarını gözden geçir; TCKN/telefon deseni tara; commit'lerde imza
  satırı olmadığını kontrol et.
- Commit stili: `tür(kapsam): özet` + Türkçe gövde. Türler: `feat`,
  `fix`, `config`, `docs`, `test`, `chore`.

---

## 11. Karar günlüğü

| Tarih | Karar | Gerekçe |
|---|---|---|
| 09-23 | 3 DOF, coxa–femur–tibia | 18 servo horn (liste), bacak CAD'inde 3 servo, faturada 20 MG996R |
| 09-23 | Servo arayüzü: PCA9685 ×2, PWM | Faturalar; 18 servo > 16 kanal |
| 09-23 | Mimari: ROS 2 + Gazebo + RL (TÜBİTAK başvurusu), brifteki düz Python değil | Kullanıcı "TÜBİTAK başvurusunda ne diyorsa onu yap" dedi |
| 09-23 | robot.yaml CAD değerleriyle şimdi yazılsın, `measured: false` | Kullanıcı seçti; şema sabit kalır, sayılar sonra değişir |
| 09-23 | Donanım çekirdeği ROS'tan bağımsız saf Python | Kalibrasyon ROS ayağa kaldırmadan yapılabilsin |
| 09-23 | Faturalar ve CAD binary'leri depoya alınmaz | Kişisel veri; 344 MB |
| 09-23 | Satır sonları LF | Pi'de shebang kırılması |
| 09-24 | **Kumpas ölçümü yok, geometri CAD'den** | Hata bütçesi (§5.3); kullanıcı sordu |
| 09-24 | tibia = 126.6 mm (121 değil) | STL'den doğru parça ölçüldü (§12) |
| 09-24 | **Ön keyfi**, forward_offset = 90 | Robot simetrik; kullanıcı "kamera yeri fark etmez" dedi |
| 09-24 | Kanal haritası elle değil, araçla | Kullanıcı donanımda deneyimsiz |
| 09-24 | **Önce yazılım/simülasyon, donanım işi kullanıcıdan istenmez** | Kullanıcı robotu kurmadı; "yazılımı elimizdeki bilgilerle yapamaz mıyız" |
| 09-24 | IK'ya başlandı ve bitti | Kullanıcının yukarıdaki cümlesi onay sayıldı |
| 09-24 | ROS 2 Lyrical + Gazebo Jetty, Ubuntu 26.04 | PC'deki WSL Ubuntu 26.04; LTS 2031'e kadar; Pi'de de aynı sistem |
| 09-24 | Klasörler düzleştirildi (`Kerem Baltacı/`, `yavuz selim/` kalktı) | Kullanıcı yaptı; ezilen liste geri kondu, alternatif ayrı adla saklandı |
| 09-24 | .gitignore desen tabanlı (tüm PDF'ler) | Klasör kalkınca eski koruma boşa düşmüştü |
| 09-24 | Simülasyon kütle/atalet CAD'den, `robot.yaml` → `simulation` altında, hepsi TAHMİN işaretli | Robot tartılmadı; uydurmak yerine STL hacmi × PETG × doluluk (0.6) + katalog; araçla tekrar üretilebilir |
| 09-24 | Geçici eklem limitleri ±90° (yalnız simülasyon) | Servo aralığının yarısı; kalibrasyon limitleri gelince otomatik onlar kullanılır |
| 09-24 | Gövde kapağı kütleye dahil değil (~43 g) | STL baskı tablası konumunda, CAD'deki yeri bilinmiyor; yer tahmin edilmedi |
| 09-24 | Tibia çarpışması iki kutu + ayak küresi (r = 5.1 mm) | Tek kutu, tibia ~25°'den fazla eğilince ayaktan önce yere değiyordu |
| 09-24 | Aynalı bacaklarda simülasyon kütlesi y'de çevrilir; görsel mesh çevrilmez | URDF mesh yansıtamaz; fark yalnız görünüşte |
| 09-24 | Klasör düzeni: `docs/` (devir, brif, `malzeme/`), `cad/` (yerel CAD), `tools/cadlib/` | Kullanıcı istedi: kök dağınıktı, kütüphane kodu script'lerin içindeydi |
| 09-24 | Görev dağılımı GOREVLER.md'de, bağımlılıklarla (ilk hâli: Görkem yazılım, Samet donanım) | Kullanıcı istedi |
| 09-24 | **Donanım durduruldu, ayrı vardiyaya (D1–D12) taşındı; yazılım Görkem (G) ve Samet (S) arasında bölüşüldü** | Kullanıcı istedi. Vardiya için önerilen başlama şartı: S3 (tripod sim) + S4 (sürücü düğümü) |
| 09-24 | CAD (~345 MB) ve mesh'ler depoya alındı; faturalar hâlâ dışarıda | Depo gizli, iki kişi aynı dosyalarla çalışacak; kullanıcı istedi |
| 09-24 | **Eklem komut arayüzü:** `/leg_controller/commands`, Float64MultiArray, 18 değer, radyan, bacak bacak coxa-femur-tibia (docs/ARAYUZ.md) | ros2_control'ün standart ForwardCommandController'ı; gerçek sürücü (S4) aynı konuyu dinlerse sim → robot geçişinde yayınlayan kod değişmez |
| 09-24 | Ayak temas sensörü yalnız simülasyonda; politika gözlemine girmez | Gerçek robotta yok; sim-to-real'de olmayan bilgiye dayanmasın |
| 09-24 | `/joint_states` gerçek robotta ölçüm değil son komut | MG996R geri bildirim vermiyor; politika da simde komut edilen açıyı görmeli |
| 09-24 | Simde servo = birinci derece sistem, T = 0.05 s (TAHMİN) | gz_ros2_control konum komutunu hız kontrolüyle uyguluyor; kazanç = 1/(T x 100 Hz) = 0.2 |
| 09-24 | **RL simülasyonu ROS'suz, süreç içi Gazebo (`hexapod_rl.sim`)** | ROS'lu sim ~1.3x; süreç içi 2 ms adım 4.5x/süreç, 8 süreç ~21x. Aynı URDF, fizik ve servo modeli |
| 09-24 | RL fizik adımı 2 ms (ROS simi 1 ms) | 1 ms 1.8x, 2 ms 4.5x, 4 ms 7.7x; üçünde de robot 100.0 mm'de duruyor. Yürüyüşte temas doğruluğu için 2 ms; yürüyüş gelince tekrar bakılacak |
| 09-24 | PyTorch CPU sürümü (venv'de) | Politika küçük MLP, SB3 PPO için CPU öneriyor; CUDA sürümü GB'larca, CPU 196 MB |
| 09-24 | RL gözlemi yalnız gerçek robotta da olanlar: IMU (yerçekimi yönü, açısal hız), son eklem komutları, hız komutu, adım saati | Sim-to-real: ölçülen açı ve ayak teması gerçekte yok; ödülde kullanılabilir, gözlemde değil |
| 09-24 | RL eylemi: ayakta duruş + 0.5 rad x [-1,1], limitlere kırpılır | Politika sıfırdan değil, dengeli bir duruştan başlasın |
| 09-24 | İlk PPO (1M adım, 34 dk): robot yürümedi (10 s'de -1.5 cm), devrilmedi; ödül 550 → 810 | Ödül yerinde durmayı fazla ödüllendiriyor. Kullanıcı: ödülü düzelt + uzun eğitim, S3 gelince tripod üstüne öğrenme |
| 09-25 | Ödül v2 ile 10M eğitim 5.75M'de DURDURULDU | Robot ritmi öğrendi (%93) ama yerinde saydı (10 s'de 0.7 cm). Sebep servo modeli çıktı (alttaki satır); o fizikle eğitmek boşa |
| 09-25 | **RL simi tork tabanlı servo: tork = Kp·hata − Kd·hız, DC motor tork-hız doğrusuyla sınırlı (Kp 20, Kd 0.05, TAHMİN)** | Hız komutlu model (gz_ros2_control'ünki) ile elle yazılmış tripod bile beklenenin %12'siyle yürüyordu (ayaklar kayıyor); tork modeliyle aynı 1.08 N·m'de %94–97. Hareketsiz tripod iki modelde de 0.6 N·m'de sağlam: sorun dinamik |
| 09-25 | Enerji cezası artık gerçek mekanik güç Σ\|τ·ω\| (W) | Tork modeliyle tork biliniyor; TÜBİTAK'taki "en az enerji" tanımına uygun |
| 09-25 | **İlk yürüyen politika** (tork modeli + ödül v2, 10M adım, 8.1 sa): ~0.087 m/s, hiç devrilmiyor; hız komutunu yok sayıyor, ~12°/s sağa dönüyor | Model depoda (`models/tork_v2_10M/`). Sıradaki ödül v3: dönüş izleme ağırlığı/toleransı ve hız toleransı |
| 09-25 | Kayda değer eğitilmiş modeller depoda `models/<ad>/` (zip ~0.5 MB); tam eğitim çıktıları depoda değil | Samet ve politika düğümü (G8) aynı modeli kullanabilsin |

---

## 12. Bilinen tuzaklar ve çıkarılan dersler

1. **STEP'te parça adı ↔ geometri eşlemesine güvenme.** `SHAPE_DEFINITION_
   REPRESENTATION` üzerinden yapılan eşleme `PLA_Tibia_tip` ile
   `PLA_Tibia_main` için **ters** çıktı. İlk türetmede tibia bu yüzden
   121 yazıldı. Montaj konumları (occurrence/NAUO adları) güvenilir; brep
   → ürün adı eşlemesi değil.
2. **B-spline kontrol noktaları yüzeyin dışında durur.** Bir parçanın en
   uç noktası STEP `CARTESIAN_POINT`'lerinden bulunamaz; mesh köşelerini
   (STL/3MF) kullan. Ayak ucu STEP'te analitik küre değil, serbest form.
3. **Aynalı bacaklar:** Fusion bunları ayrı gövde olarak dışa aktarmış;
   dönüşüm matrisi düzgün bir dönme (det +1), yansıma geometride. Yerel Z
   ters yönde uzanıyor. Bunu hesaba katmayınca aynalı bacaklar 140 mm,
   normaller 100 mm yarıçapta çıkıyordu (simetrik robotta imkânsız).
   Eklem ofseti aynalılarda ters işaretle alınmalı.
4. **CAD azimut yönü:** CAD'de +Y yukarıyken atan2(z, x) yukarıdan saat
   yönünde artar. Gövde çerçevesine geçişte işaret ters:
   `gövde = offset − cad`.
5. **IK açı sarması:** ayak femur ekleminin gerisine düştüğünde
   `atan2 + α` 180°'yi aşıyor ve femur −60 yerine 300° çıkıyordu. Servoya
   giderse ters döner. (−180, 180]'e sarılıyor; gidiş-dönüş testi yakalar.
6. **PCA9685 0x70 (ALLCALL):** her kart burada da cevap verir; kart adresi
   sanma. `hwcheck.py` bir ara bunu yanlışlıkla ikinci kart önerebiliyordu.
7. **Aynı adlı dosya çakışması:** klasörler düzleştirilirken iki "gömülü
   malzemeler listesi.txt" çakıştı, biri ötekini ezdi. Klasör taşımaları
   sonrası `git status`'u dikkatle oku; beklenmeyen `M`/`D` varsa
   dokunmadan önce sor ya da incele.
8. **Dosyalar dışarıdan değişebilir.** Kullanıcı çalışırken klasör
   düzenleyebiliyor. Commit'lerde `git add -A` yerine dosyaları açıkça
   seç; kullanıcının değişikliğini kendi commit'ine katma, önce söyle.
9. **Tahmin etme, uydurma.** İlk oturumda kullanıcı açıkça "tahmin etme,
   varsayılan değer uydurma" dedi; bu, `MissingValue` tasarımının kökeni.
   Simülasyon için tahmini değer gerekirse (kütle, atalet), kaynağıyla ve
   `measured: false` / "simülasyon tahmini" notuyla gir.
10. **Yansıtılan mesh'in hacmi negatif çıkar.** Bir STL'i yansıtınca
    (ör. aynalı bacak için y → −y) üçgenlerin dönüş yönü tersine döner;
    dörtyüzlü toplamıyla hesaplanan hacim ve kütle negatif olur, parça
    kütle eklemek yerine çıkarır. İlk hesapta gövde ağırlık merkezi bu
    yüzden 9 mm kaymıştı. Yansıtınca üçgen sırasını da çevir
    (`cadlib.mesh.flip_winding`).
11. **Her STL, STEP parça çerçevesinde değil.** `pla_body-lid.stl`
    (yalnız `Baskı Dosyaları/`'nda) baskı tablası konumunda dışa
    aktarılmış; montaj dönüşümü uygulanınca gövdenin 100 mm dışına
    düşüyordu. Yeni bir STL kullanmadan önce dönüştürülmüş sınır
    kutusunun beklenen yerde olduğunu kontrol et.
12. **Uzun heredoc'lar Bash aracında bozuluyor** ("unexpected EOF").
    Uzun dosyaları Write aracıyla yaz; kısa betikler için heredoc olur.
13. **ROS paket adlarını ezbere yazma, Lyrical'da doğrula.** `position_controllers`
    Lyrical'da YOK (ForwardCommandController + interface_name: position kullan).
    Doğrulama yolu: ROS apt deposunun paket listesi
    (packages.ros.org/ros2/ubuntu/dists/resolute/main/binary-amd64/Packages.gz)
    ve ilgili GitHub deposunun `lyrical` dalı. docs.ros.org bot korumalı.
14. **sdformat çarpışma adları:** URDF → SDF'te çarpışma `<ad>_collision` (sırası 0
    değilse `_<sıra>` eki) olur. Temas sensörü bu adı ister; ayak küresi bu yüzden
    tibia'nın ilk çarpışması. gz-sim temas sensörünün konusu `<contact><topic>`
    içinde (sensör düzeyinde değil).
15. **RViz WSLg'de Wayland'de çöker** ("Invalid parentWindowHandle ... GLXWindow",
    100 denemeden sonra abort). `QT_QPA_PLATFORM=xcb` ile XWayland'de çalışıyor;
    display.launch.py bunu Wayland varsa kendisi veriyor. Gazebo penceresi etkilenmiyor.
16. **ament_python'da `setup.cfg` şart:** yoksa console_scripts `bin/`'e kurulur ve
    `ros2 run paket komut` "No executable found" der. `[develop] script_dir` ve
    `[install] install_scripts` `$base/lib/<paket>` olmalı.
17. **gz.sim Python bağları:** `import gz.math` yapılmadan `Link.world_pose` çağrılırsa
    pybind11 Pose3d'yi çeviremez ve süreç ÇÖKER (yakalanamaz). `reset_all()`
    isteği bir adım gecikmeyle işlenir ve sıfırlama `run(n)`'in adımlarından birini
    yer; `HexapodSim.reset()` bunu tek adımlık çağrılarla çözüyor. Hız komutu
    (`Joint.set_velocity`) adımlar arasında kalıcı: her fizik adımında değil, servo
    döngüsünde vermek yetiyor ve ~%30 hızlandırıyor. Kamera sensörü, konusunu
    dinleyen yoksa kare üretmiyor (`<save>` olsa bile).
18. **Servo modeli yürümeyi belirliyor; önce fiziği doğrula.** Hız komutlu servo
    modelinde robot yürüyemiyordu ama bu RL eğitiminden anlaşılmadı: iki eğitim
    (6+ saat) "ödül yanlış" sanılarak harcandı. Teşhis sırası şuydu ve işe yaradı:
    (1) RL'siz, elle yazılmış açık döngü yörüngeyle yürüyor mu? (2) değilse
    parametreleri tek tek değiştir (tork, sürtünme, tepki süresi) (3) hareketsiz
    duruşla statik/dinamik ayrımı. Artık `test_simulasyon_yurumeye_izin_veriyor`
    bunu her test koşusunda denetliyor. Ayrıca: gz.sim'de `Joint.transmitted_wrench`
    Python'da bozuk (gz::msgs::Wrench çevrilemiyor); tork gerekiyorsa servo
    modelinin uyguladığı tork kullanılır (`SimState.joint_effort`).

---

## 13. Açık kalan işler

### 13.1 `robot.yaml`'da boş alanlar (76)

- `drivers[0].address`, `drivers[1].address` — kart adresleri (muhtemelen
  0x40 ve 0x41, A0 lehimlenince). `map_channels.py`/`hwcheck.py` bulur.
- `joints[*].driver`, `joints[*].channel` (36) — kanal haritası,
  `map_channels.py` çıkarır.
- `joints[*].limits_deg.min/max` (36) — `calibrate.py` içinde `span` +
  `limit` ile bulunur. Simülasyon şimdilik `simulation.provisional_joint_
  limits_deg` (±90°) kullanıyor. İlk RL eğitimi (G7) bununla yapılır;
  **gerçek limitlerle yeniden eğitim şart** (D10), coxa ±90'da komşu
  bacağa girer.
- `body.standing_height` — ölçüm değil, IK/gait çalışınca seçilecek
  hedef.
- `body.total_mass_kg` — terazi (D7, donanım vardiyası). Simülasyon şimdilik CAD tahmini
  kullanıyor: 2.13 kg (`simulation.links`). Tartım gelince doluluk oranı
  ve elektronik kütlesi (`simulation.mass_inputs`) buna göre düzeltilip
  `tools/cad_sim_model.py` yeniden çalıştırılır.
- `sensors.imu.address`, `sensors.imu.mount_rotation_deg`,
  `sensors.range_finders.devices[*]` (XSHUT GPIO, adres, bakış yönü) —
  otonomi katmanında gerekecek.

### 13.1b Yazılım tarafı

- **ROS'lu simülasyon (sim.launch.py) hâlâ hız komutlu servo modelinde.** RL simi
  tork modeline geçti (karar günlüğü 09-25); ROS'lu simde tripod ayakları kayar.
  Çözüm seçenekleri: gz_ros2_control'ün effort arayüzü + tork modelini uygulayan
  özel bir GazeboSimSystem eklentisi (C++), ya da gz-sim JointPositionController
  (PID + kuvvet sınırı; tork-hız eğrisi yok). G8 (politika düğümü) simde
  denenmeden önce yapılmalı.

### 13.2 Donanım tarafı (⏸ durduruldu; GOREVLER.md D1–D12)

1. Bacaklara "ÖN" + 1–6 bandı (§5.5).
2. Kartlardan birinin A0'ını lehimle.
3. Kartları Pi'ye bağla (VCC 3.3 V!), servoları kartlara tak (sıra fark
   etmez), servo gücü ~6 V buck'tan.
4. Pi'ye Ubuntu Server 26.04 kur; `git clone`; `python3
   tools/map_channels.py` → çıktıyı robot.yaml'a işle.
5. `python3 tools/calibrate.py` ile 18 eklemin merkez/yön/span/limitleri.
6. Güç bağlantısını kontrol et: servo hattı Pi'den ayrı mı, topraklar
   ortak mı, sigorta nerede.

---

## 14. Sıradaki iş için hazır plan: URDF + Gazebo

**Durum (2. oturum sonu): 1–7 BİTTİ.** URDF RViz ve check_urdf'ten geçti; robot
Gazebo'da doğuyor, kontrolcüler açılıyor, sensörler yayında, `stand` ile
istenen 100 mm'ye kalkıyor (ayrıntı GOREVLER.md G3–G5). **Sıradaki: 8 — ama
tripod Samet'in (S2/S3); Görkem'in sıradaki işi G6 (RL ortamı).** G6'nın
tasarımını belirleyen hız ölçümü GOREVLER.md G6'da: ROS'lu simülasyon ~1.3x,
yalın Gazebo ~3–5x gerçek zaman; öneri ROS'suz, süreç içi `gz.sim` ile adımlama
ve paralel ortamlar. Yalın Gazebo'da adımı 1→2 ms yapmak hızlandırmadı; darboğaz
fizik değil, incelenmedi (G6'nın ilk işi).

Asıl plan:

1. **Yeni paket `src/hexapod_description`** (ament_python ya da
   ament_cmake + xacro). URDF'i elle yazma; `robot.yaml`'dan **üreten** bir
   betik/xacro kullan ki geometri tek kaynaktan gelsin.
2. **Link/joint yapısı IK sözleşmesiyle birebir aynı olmalı** (§6):
   - `base_link` = gövde çerçevesi (REP-103).
   - Her bacak için: `coxa_joint` (revolute, eksen +z) → `femur_joint`
     (+x yönünde coxa=50 mm ötede; femur + = yukarı olduğu için eksen
     **−y**) → `tibia_joint` (+x yönünde femur=80 mm ötede, eksen **−y**,
     0'da tibia dümdüz aşağı) → ayak ucu tibia linkinde (0, 0, −126.6).
   - Bacak montajı: (100·cos φ, 100·sin φ, −10.05), yaw = φ (§5.5 gövde
     azimutları).
3. **Görsel mesh:** `cad/Hexapod/leg/*.stl` ve `cad/Hexapod/body/*.stl`
   (mm → `scale="0.001 0.001 0.001"`). STL'ler parçanın kendi çerçevesinde;
   STEP'teki montaj dönüşümleri `tools/cadlib/assembly.py` ile alınır
   (✅ `tools/cad_sim_model.py` yapıyor, sonuç `meshes.yaml`). Çarpışma için basit geometri (silindir,
   kutu, ayakta küre r≈5 mm) daha hızlı ve kararlı.
4. **Kütle/atalet:** config'de yok. Simülasyon için tahmin gerekiyor:
   MG996R 55 g (katalog), parçalar STL hacmi × PETG yoğunluğu × doluluk
   oranı. Bunları "simülasyon tahmini" olarak açıkça işaretle, gerçek
   değer gibi sunma.
5. **Eklem limitleri/hız/tork (sim):** MG996R katalogundan hız ~0.14 s/60°
   (@6 V), tork ~11 kg·cm; açı limitleri kalibrasyon gelene kadar geçici
   ve işaretli.
6. **Doğrulama testi (ROS'suz yazılabilir):** URDF'i Python'da parse edip
   ileri kinematiği hesapla, `hexapod_kinematics.forward` ile aynı ayak
   konumlarını verdiğini test et. Simülasyon ile IK'nın ayrışmasını
   baştan engeller.
7. **Gazebo:** Jetty + `ros_gz` köprüsü; eklem pozisyon kontrolü için
   `gz_ros2_control` ya da Gazebo'nun joint position controller
   eklentisi. IMU ve ayak temas sensörleri (RL gözlemi için).
8. **Sonra:** tripod gait (`hexapod_gait`, IK üzerinden) → RL ortamı
   (Gymnasium, ros_gz üzerinden; gözlem: IMU + eklem açıları + temaslar;
   eylem: eklem hedefleri ya da gait parametre düzeltmeleri; ödül:
   ileri hız − enerji − devrilme cezası) → PPO (Stable-Baselines3) →
   alan rastgeleleştirme (zemin, sürtünme, kütle, itme, gecikme).
   Not: Gazebo RL için yavaş olabilir; headless ve paralel ortamlar
   gerekebilir. Başka bir simülatöre geçiş TÜBİTAK başvurusundan sapma
   olur, kullanıcıya sormadan yapma.

**Çalıştırmak için WSL'e kurulum gerekecek** (şifre ister, kullanıcı
çalıştırır): ROS 2 Lyrical apt deposunu ekle, `sudo apt install
ros-lyrical-desktop` (Gazebo Jetty dahil). Güncel resmi kurulum adımlarını
kullanmadan önce doğrula.

---

## 15. Depoda olmayan kaynakların özeti

**TÜBİTAK başvurusu** (Milli Teknoloji Kulüpler Birliği, Kulüp Geliştirme
Desteği Başvuru Formu; ekran görüntüsü olarak vardı, depoda değil):
- Konu: altı bacaklı robotların karmaşık ve değişken zeminlerde (engebeli,
  kumlu, eğimli) stabil ve verimli hareketi.
- Sorun: geleneksel kontrol önceden programlanmış sabit desenlere (CPG)
  dayanır; kuma saplanma, gevşek taşa basma, eğim gibi durumlarda enerji
  verimsizliği ve devrilme olur. IMU ve temas verisindeki anlık
  değişimlere dinamik tepki vermez.
- Öneri: kural tabanlı değil, **Pekiştirmeli Öğrenme (RL)** tabanlı, veri
  güdümlü ve modelden bağımsız (model-free) bir kontrol politikası. Ajan
  sanal ortamı (değişken araziler) ve iç durumunu (sensörler) gözleyip
  deneme-yanılmayla en yüksek ödülü (ör. devrilmeden ileri gitme)
  öğrenir; kaygan yüzeyde küçük adım atmak ya da engel tırmanırken farklı
  bacak sırası kullanmak gibi stratejileri kendisi keşfeder.
- Yöntem: **ROS + Gazebo**; sanal IMU ve temas sensörleriyle hexapod
  modeli; Python, **Stable-Baselines3, PPO**; ödül: devrilmeden, en az
  enerjiyle, en hızlı ilerleme.
- Plan: **12 ay**, dört ana başlık: simülasyon ortamının kurulması, RL
  algoritmasının kodlanması, robotun montajı, simülasyondaki en iyi
  sonuçlarla gerçek arazide deneme.
- Yaygın etki: adaptif kontrol algoritması; ulusal/uluslararası konferans
  sunumu; **kodların açık kaynak paylaşılması** (bu yüzden depoda kişisel
  veri olmaması ayrıca önemli).
- İlgili öncelikli alanlar: "Robotik-mekatronik", "Modelleme ve
  simülasyon teknolojileri".

**Diğer ekran görüntüleri** (referans amaçlı YouTube kareleri): başka bir
hexapod tasarımı ("I Redesigned My Hexapod In Fusion 360") ve DS3225
servo karşılaştırması (25 kg·cm). **DS3225 alınmadı**, robot MG996R ile.

**Faturalar:** içerik §4.1'de. Kişisel veri nedeniyle depoda değil.

---

## 16. Yeni oturumda ilk adımlar

1. Bu belgeyi (`docs/PROJE_DEVIR.md`), `CLAUDE.md`'yi ve `GOREVLER.md`'yi oku.
2. Durumu doğrula:
   ```bash
   git log --oneline | head -5
   python -m pytest -q
   ```
   73 test geçmeli.
3. CAD depoda (`cad/Hexapod/`). Eksikse `git status` ile bak; yine yoksa
   Printables 606030'dan indirilebileceğini söyle (indirme için izin al).
4. `git status`'ta beklenmeyen değişiklik varsa kullanıcının olabilir;
   dokunmadan incele (§12, madde 7–8).
5. Kullanıcı başka bir şey istemediyse: ROS kurulu mu bak
   (`wsl -e bash -lc "ls /opt/ros"`). Değilse kullanıcıdan
   `bash tools/wsl/ros_kurulum.sh`'ı çalıştırmasını iste (G4). Kuruluysa
   G3'ün son kontrolü (colcon build + RViz), sonra G5 (Gazebo). Hangi işin
   kimde olduğu ve neyi beklediği GOREVLER.md'de.
   Kullanıcıdan donanım/ölçüm işi isteme.
6. Önemli bir karar ya da biten aşama olduğunda bu belgeyi güncelle.
