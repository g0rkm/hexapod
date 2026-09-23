# Hexapod Robot — Yazılım Proje Brifi

Bu doküman, yazılım geliştirmeye başlamadan önce alınan kararları, eksik
bilgileri ve planlanan mimariyi içerir. Claude Code oturumunun başlangıç
bağlamı olarak kullanılmak üzere hazırlanmıştır.

---

## 1. Proje Özeti

Altı bacaklı (hexapod) bir robot üretiliyor. Mekanik ve donanım tarafı ekip
tarafından yürütülüyor; yazılım tarafı Claude ile birlikte yazılacak.

Hedeflenen kapsam dört başlıkta toplanıyor ve hepsi planın içinde:

1. Yürüyüş (gait) motoru
2. Uzaktan kumanda ve telemetri
3. Ters kinematik (IK) ve gövde pozu kontrolü
4. Sensör destekli otonom hareket

---

## 2. Donanım Kararları

### Kesinleşen

| Konu | Karar |
|---|---|
| Ana kontrolcü | Raspberry Pi |

### Henüz Netleşmemeyen (kod yazımını bloke eden)

| Konu | Durum | Neden kritik |
|---|---|---|
| Bacak başına DOF | 2 mi 3 mü belli değil | 2 DOF ve 3 DOF tamamen farklı iki kod tabanı demek. IK ve gait motoru bu karar verilmeden yazılamaz. |
| Servo arayüzü | Belirlenmedi | PWM (PCA9685 gibi bir sürücü üzerinden) ile akıllı seri servo (Dynamixel, LX-16A, SCS) sürücü katmanını baştan ayırır. |
| Güç mimarisi | Belirlenmedi | Servo beslemesi ve Pi beslemesi ayrılmalı, ortak toprak çekilmeli. |

---

## 3. Kod Yazımı İçin Gereken Bilgiler

### 3.1 Zorunlu — bunlar olmadan tek satır yazılamaz

- **Bacak başına DOF ve eklem sırası**
  (coxa–femur–tibia mı, yoksa coxa–femur mü)
- **Segment uzunlukları (mm)**: coxa, femur, tibia.
  IK'nın tek gerçek girdisi budur.
- **Gövde geometrisi**: her bacağın gövde merkezine göre x/y bağlantı
  koordinatları ve montaj açısı
  (0°/60°/120° radyal yerleşim mi, dikdörtgen yerleşim mi)
- **Servo modeli ve arayüzü**: PWM + sürücü kartı mı, seri/akıllı servo mu
- **Eklem açı limitleri** ve her eklemde pozitif dönüş yönünün hangi tarafa
  olduğu

### 3.2 Çok işe yarar

- CAD dosyası veya ölçülendirilmiş teknik çizim — yukarıdakilerin çoğunu tek
  seferde verir
- Servo besleme değerleri: gerilim, akım kapasitesi, Pi'nin ayrı beslenip
  beslenmediği
- Sensörler: IMU var mı, modeli ne
- Ayakta duruş sırasında gövde yüksekliği ve robotun toplam ağırlığı
  (gait tempo ve duty cycle hesabı buna bağlı)

### 3.3 Sonraya bırakılabilir

- Kumanda arayüzü tercihi (gamepad, telefon, web arayüzü)
- Ayak uçlarında temas sensörü olup olmayacağı

> **Ölçüm notu:** Segment uzunluklarını CAD'den değil, monte edilmiş
> robottan kumpasla ölçmek gerekiyor. Baskı toleransı ve servo horn
> kalınlığı yüzünden birkaç mm sapma olur; bu sapma IK'da "ayak yere
> oturmuyor" olarak geri döner.

---

## 4. Yazılım Mimarisi

Sistem katmanlı kurulacak. Her katman altındakine bağımlı, üstündekinden
habersiz olmalı:

```
┌─────────────────────────────────────────┐
│  Otonomi / davranış katmanı             │  (sensör füzyonu, engelden kaçınma)
├─────────────────────────────────────────┤
│  Kumanda + telemetri                    │  (komut girişi, durum yayını)
├─────────────────────────────────────────┤
│  Gait motoru                            │  (tripod, wave, ripple, geçişler)
├─────────────────────────────────────────┤
│  Ters kinematik (IK)                    │  (bacak IK + gövde pozu)
├─────────────────────────────────────────┤
│  Servo sürücü katmanı                   │  (açı → sinyal, kalibrasyon, limitler)
└─────────────────────────────────────────┘
```

### Önerilen repo yapısı

```
hexapod/
├── config/
│   ├── robot.yaml          # segment uzunlukları, gövde geometrisi, limitler
│   └── calibration.yaml    # servo offset değerleri (elle ayarlanan)
├── hexapod/
│   ├── servo/              # sürücü katmanı
│   ├── kinematics/         # IK, gövde pozu
│   ├── gait/               # yürüyüş motoru
│   ├── control/            # kumanda, telemetri
│   └── autonomy/           # sensör, davranış
├── tools/
│   └── calibrate.py        # servo kalibrasyon yardımcısı
└── tests/
```

Kalibrasyon değerleri ve robot geometrisi koda gömülmeyip config
dosyalarında tutulacak. Bu, donanım değiştiğinde kodun yeniden yazılmasını
önler.

---

## 5. Geliştirme Planı ve Süre Tahmini

Aşağıdaki tahminler, kodun Claude ile birlikte yazıldığı senaryoya göredir.
Kod yazımı hızlanır; donanım hata ayıklaması hızlanmaz.

| Aşama | Süre | Notlar |
|---|---|---|
| Servo katmanı + kalibrasyon | ~1 hafta | Kod hızlı yazılır, ancak 18 servonun offset'ini tek tek oturtmak elle yapılan iş. Burada acele edilirse sonraki her katman tutarsız davranır. |
| Ters kinematik | 2–3 gün | Matematik tek seferde çıkar, kalanı donanımda doğrulama. |
| Gait motoru | ~1 hafta | Tripod hızlı yürür; wave/ripple ve geçişlerin pürüzsüz olması zaman alır. |
| Kumanda + telemetri | 2–3 gün | Saf yazılım; en çok hız kazanılan kısım. |
| Sensör / otonomi | 2–3 hafta+ | Ucu açık. Hedefin IMU ile gövde dengelemesi mi, engelden kaçınma mı olduğuna göre büyük ölçüde değişir. |

**Toplam:**

- Part-time (~12 saat/hafta): **4–5 hafta**
- Tam zamanlı: **10–12 gün**
- İlk yürüyen + kumandayla sürülebilen sürüm: part-time **~2.5 hafta**

### Asıl darboğaz

Yazılan kodun ne kadar hızlı test edilebildiği. Robotun başındayken geri
bildirim döngüsü dakikalarla ölçülür; kod alınıp hafta sonu deneniyorsa
takvim test fırsatlarına göre şekillenir.

---

## 6. Bilinen Riskler

- **Brownout / besleme çökmesi.** 18 servo aynı anda yük altındayken Pi'yi
  resetleten gerilim düşmeleri, yazılım hatası gibi görünür ve günlerce
  kovalanır. Ayrı besleme ve ortak toprak baştan halledilmeli.
- **DOF kararının gecikmesi.** IK ve gait yazımını doğrudan bloke eder.
- **Kalibrasyonun kayıt altına alınmaması.** Offset değerleri yazılı
  tutulmazsa her oturumda sıfırdan başlanır.
- **Bağlamın kaybolması.** Repo ve kalibrasyon değerleri sürüm kontrolünde
  tutulmalı; aksi halde aynı kararlar tekrar tekrar konuşulur.

---

## 7. İlk Adımlar

1. Bacak başına DOF kararını kesinleştir.
2. Monte edilmiş bacaktan coxa / femur / tibia uzunluklarını kumpasla ölç.
3. Gövde bağlantı noktalarının koordinatlarını ve montaj açılarını çıkar.
4. Servo modeli ve sürücü arayüzünü netleştir.
5. `config/robot.yaml` dosyasını bu değerlerle doldur.
6. Servo sürücü katmanı ve kalibrasyon aracıyla kodlamaya başla.
