# Yürüyüş ölçümleri (S6)

Denetleyicilerin S5 zeminlerindeki karşılaştırması. Üreten araç:

```bash
python -m hexapod_rl.olcum tripod tripod:50 models/<ad>/model.zip models/<ad>/model.zip+refleks \
    --tohum 3 --saniye 10 --temiz --paralel 8 \
    --csv docs/olcumler/tripod_vs_politika.csv -o docs/olcumler/tablo.md
```

`+refleks`: mesafe sensörlü kaldırma refleksi açık (DENEYSEL yerleşim: gövde
kenarında ileri ve ±90°, 20° aşağı; robot.yaml'da yok, D8). Tekrarlarda sensöre
%5 gürültü + %10 düşen okuma, temiz ölçümde ideal sensör. `--paralel N`: N
süreçte (eski PC'de 8; tam tablo ~10 dk).

## Hücreler nasıl okunur

`0.088 m/s · 42.1 J/m · 3/3`

| Alan | Anlamı |
|---|---|
| m/s | komut yönünde ortalama hız (tekrarların ortalaması) |
| J/m | **metre başına mekanik enerji**: ortalama güç × süre ÷ gidilen yol |
| 3/3 | kaç tekrarda devrilmeden tamamladı |
| "en az X m" | devrilme olduysa, düşmeden gidilen en az mesafe |

Neden J/m: yavaş ama verimli bir yürüyüşle hızlı ama savurgan olanı yalnız
bu ayırır. Toplam enerji tek başına yavaş yürüyüşü haksız yere ödüllendirir.

## Tekrarlar neden rastgeleleştirme açık koşuyor

Süreç içi Gazebo deterministik: rastgeleleştirme kapalıyken farklı tohumlar
**birebir aynı** sonucu verir (ölçüldü 2026-09-28: üç tohum da 0.0869 m/s),
yani tekrar hiçbir bilgi taşımaz. Açıkken servo gücü, gecikme, itme ve IMU
gürültüsü her bölümde tohumdan çekilir; tohumlar farklılaşır ve sınırdaki bir
engelin "bazen geçiliyor" olduğu görünür.

Bu, tek deterministik ölçümün sınır durumlarda yanıltıcı olduğu dersinin
(PROJE_DEVIR §12.32) ölçüm tarafındaki karşılığı: tabloda ortalamanın yanında
`3/3` sütunu tam da bunun için var.

`--temiz` ayrıca rastgeleleştirmesiz tek ölçüm alır; CSV'de `rastgele=0`
satırı odur, tablodaki ortalamaya karışmaz.

## Sonuçlar (2026-09-28 öğleden sonra, düzeltilmiş ölçüm)

Komut ileri 0.1 m/s, 10 s, 3 tohum, rastgeleleştirme açık. Tablo:
[tablo.md](tablo.md).

> [!IMPORTANT]
> Sabahki ilk tabloda iki ölçüm hatası vardı, ikisi de düzeltildi ve tablo
> baştan ölçüldü (eskisi git geçmişinde):
> 1. **Güç tek anlık örnekti** (PROJE_DEVIR ders 48): kontrol adımının yalnız
>    son fizik adımından okunuyordu; servo gecikmesine göre 4 kata kadar
>    oynuyor, temiz koşuda gerçeğin yarısını gösteriyordu. Artık adım boyunca
>    ortalama. İlk tablodaki "tohumlar arasında güç ikiye katlanıyor"
>    gözleminin sebebi buydu.
> 2. **`tripod:50` 50 mm kaldırmıyordu** (ders 49): tripod sarmalayıcısında
>    eylem ±0.5 rad'a kırpılıyordu, 50 mm femurda 49° ister; ~30 mm
>    kaldırıyordu. İlk tablodaki "yüksek adım düzde %31 daha ucuz" bulgusu
>    bu ikisinin birleşiminden çıkmıştı: **düzeltilmiş ölçümde tersi doğru**,
>    50 mm adımlı tripod düzde %17 daha pahalı (42.9'a karşı 36.7 J/m).

**1. Refleksli politika tripod'u her engelde geçiyor, düzde aynı enerjide.**
`ppo_kaldirma35_250k+refleks` düzde tripod'la aynı hız ve enerji (0.102 m/s,
36.5 J/m; tripod 36.7), 45 mm basamakta 0.095 m/s (tripod 0.019, tripod:50
0.087), 40 mm çukurda 0.099 (0.014 / 0.088), merdivende 0.097 (0.064 /
0.090). 60 mm basamakta `ppo_refleks_1500k+refleks` en iyi: 0.091 m/s, 73 J/m
(tripod:50 0.050 m/s, 137 J/m); `ppo_kaldirma35_250k+refleks` orada sınırda
(rastgelede 0.075 ama temizde takılıyor).

**2. Refleksin zayıf yeri eğim.** Yokuşu ve yan eğimi önündeki "engel" sanıp
ayağı yükseltiyor: `ppo_kaldirma35_250k+refleks` 10° yokuşta 47.3 J/m (düzde
36.5), tripod 39.4. Engebe 40/60'ta da %14 pahalı (44.0'a karşı 38.7). Hız
buralarda tripod'dan biraz yüksek. Sebep: engel yüksekliği yerçekimine göre
ölçülüyor, düzgün bir eğim de 20° aşağı bakan ışının çarptığı yerde
"yükselmiş zemin" gibi görünüyor.

**3. Sensörsüz politikalarda ödünleşim sürüyor** (§12.33): `ppo_omni_250k`
düzde ve eğimde en verimli (36.2 J/m; 20° yokuşta 46.5, tripod 49.7) ama
engelde tripod kadar takılıyor; `ppo_lift50_3750k` engelde iyi (45 mm 0.107,
çukur 0.107) ama düzde %31 pahalı (48.2 J/m).

**4. Kaygan eğimde politikalar tutunuyor.** 10° μ0.3'te politikalar
0.071-0.077 m/s ve 53-65 J/m, tripod 0.046 m/s ve 82.5 J/m. 15° μ0.3'te
kimse çıkamıyor (sürtünme payı yok): tripod 0.77 m/s geri kayıp 3/3
devriliyor, politikalar yerinde duruyor, hiç devrilmiyor.

**5. Düzde herkes başa baş** (düz tripod tavanı, §12.28): hız 0.098-0.114,
en iyi enerji tripod, `ppo_omni_250k` ve refleksli 35 mm'lik modelde (36-37
J/m).

**G7 için:** refleksle "politika tripod'u geçiyor" şartı eğim, engebe ve
kaygan zeminde hızda karşılanıyor, engellerde (basamak, çukur, merdiven,
yayla) hem hızda hem enerjide açık ara; düzde enerji eşit. Eğim ve engebede
enerjide geride (refleksin eğimi engel sanması). Sensörsüz en iyi seçenek
zemine göre değişiyor.

## Dosyalar

| Dosya | Ne |
|---|---|
| `tablo.md` | karşılaştırma tablosu (zemin satır, denetleyici sütun) |
| `tripod_vs_politika.csv` | ham veri, her koşu bir satır (grafik çizmek için) |

## DİKKAT: bu tablo Görkem'in eski tablolarıyla karşılaştırılamaz

Şu an depoda **iki ayrı zemin tanımı** var ve aynı adı taşıyan zeminler
aslında farklı:

| | Görkem (`hexapod_rl.terrain_probe`) | S5 (`hexapod_terrain`) |
|---|---|---|
| basamak yeri | x = 0.30 m | x = 0.40 m |
| engebe alanı | 3 x 3 m kare, hücre 12 cm | 5 x 2 m koridor, hücre 15 cm |
| engebe girişi | düzlükten ani | 0.5 m rampa |
| engebe x=2 m'de | düz (alan bitmiş) | hâlâ engebeli |

Bu tablo (`docs/olcumler/`) **S5 zeminlerinde**, `models/README.md`'deki
tablolar ise **Görkem'in deneme zeminlerinde** ölçüldü. "engebe 60 mm"
satırları aynı adı taşısa da aynı zemin değil; iki tablodaki sayıları yan
yana koyup yorum çıkarmayın.

G7 eğitimi 2026-09-28'den beri S5 zeminlerinde (`train.py --terrains s5`,
`terrain_probe.TRAIN_SETS["s5"]`). `terrain_probe`'un kendi zeminleri
kaldırılmadı: depodaki modeller onlarla eğitildi ve models/README tabloları
onlarla ölçüldü, yeniden üretilebilsinler diye kalıyorlar. Hangi tablonun
hangi zeminle ölçüldüğü başlıkta yazmalı.

## Ölçümün sınırları

- Yol, gidilen **düz mesafe**; robot daire çizerse gerçek patikadan kısa
  görünür ve J/m yüksek çıkar. Komutlar düz olduğu için pratikte sorun değil.
- Güç, simülasyonun servo modelinin uyguladığı tork üzerinden (`Σ|τ·ω|`);
  elektriksel tüketim değil, mekanik güç. Gerçek akü tüketimi donanım
  vardiyasında ölçülür.
- Zeminler CAD geometrisiyle ve tahmini kütleyle koşuyor; gerçek robotta
  farklar çıkabilir (D9, D10).
