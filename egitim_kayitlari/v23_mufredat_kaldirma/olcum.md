# v22–v23 müfredat denemeleri: ham ölçümler (2026-09-27)

Özet ve yorum: `models/README.md` ("Müfredat"), PROJE_DEVIR ders 40.
Müfredat: `train.py --curriculum deneme` (terrain_probe.CURRICULA; çukur ve
yayla 10–60 mm, eğim x 5–25°, eğim y 5–20°). Seviye indeksi: çukur/yayla 0=10,
1=20, 2=30, 3=35, 4=40, 5=45, 6=50, 7=55, 8=60 mm.

- v22_mufredat: ppo_omni_250k'dan (taban 25 mm, sabit), v12 ile aynı ayarlar
  (std 0.1, lr 1e-4, target_kl 0.02); 2.25M'de durduruldu (çukur seviyesi
  500k'dan beri ~3).
- v23_mufredat_kaldirma: ppo_omni_250k genişletildi (öğrenilmiş kaldırma
  20–60 mm, 25 mm'den, kaldırma std 0.5), v20 ile aynı ayarlar; 2.5M'de soket
  sızıntısıyla çöktü (`cokus.txt`, PROJE_DEVIR ders 39).
- v23b_mufredat_kaldirma: düzeltmeden sonra v23'ün 2.5M kaydından 1.5M devam
  (toplam 4M; müfredat seviyeleri 0'dan yeniden başladı).

## Eğitimde ortalama müfredat seviyesi (250k'lık pencereler, progress.csv)

Seviyeye eğitimdeki gürültülü (stokastik) politikanın başarısı karar verir;
deterministik politikanın yeteneğinden yüksek görünür (aşağıda).

| Pencere | v22 çukur | v22 yayla | v23 çukur | v23 yayla | v23b çukur | v23b yayla |
|---|---|---|---|---|---|---|
| 0–250k | 2.40 | 3.96 | 3.16 | 3.71 | 3.65 | 4.31 |
| 250–500k | 3.17 | 6.40 | 4.62 | 5.61 | 5.76 | 4.22 |
| 500–750k | 3.50 | 5.81 | 4.89 | 4.74 | 5.59 | 5.16 |
| 750k–1M | 2.68 | 6.12 | 4.52 | 5.64 | 5.31 | 5.17 |
| 1–1.25M | 3.10 | 4.22 | 4.89 | 4.57 | 5.69 | 4.66 |
| 1.25–1.5M | 3.04 | 6.71 | 4.90 | 4.52 | 5.87 | 4.89 |
| 1.5–1.75M | 3.02 | 5.70 | 4.92 | 5.80 | 6.12 | 5.50 |
| 1.75–2M | 3.27 | 5.51 | 5.17 | 5.16 | | |
| 2–2.25M | 3.22 | 5.41 | 5.46 | 5.03 | | |
| 2.25–2.5M | | | 5.10 | 5.04 | | |

## Deterministik ve gürültülü politika, 45 mm çukur (v23 1M, 10 s)

```
çukur 40 geri  deterministik yol 0.43 m  kaldırma ort 27.4 / %90 29.0 mm  güç 3.58 W
çukur 45 geri  deterministik yol 0.12 m  kaldırma ort 27.7 / %90 28.9 mm  güç 2.68 W
çukur 45 yana  deterministik yol 0.11 m  kaldırma ort 27.8 / %90 29.3 mm  güç 2.72 W
düz ileri      deterministik yol 1.01 m  kaldırma ort 27.9 / %90 29.0 mm  güç 2.27 W
çukur 40 geri  stokastik     yol 0.58 m  kaldırma ort 27.6 / %90 39.4 mm  güç 6.84 W
çukur 45 geri  stokastik     yol 0.47 m  kaldırma ort 27.7 / %90 39.4 mm  güç 7.14 W
çukur 45 yana  stokastik     yol 0.11 m  kaldırma ort 27.8 / %90 39.3 mm  güç 6.19 W
düz ileri      stokastik     yol 0.91 m  kaldırma ort 28.0 / %90 37.8 mm  güç 6.15 W
```

(Stokastik koşunun üç tohumu birebir aynı çıktı: eylem gürültüsü torch'tan
geliyor, çatallanan süreçlerde aynı başlıyor; tek koşu sayılmalı.)

## Kaldırma (deterministik; düz / basamak 45 / engebe 40, 0.1 m/s)

| Kayıt | Kaldırma | Düz yol / güç | Basamak 45 yol (kaldırma önce → engelde) | Engebe 40 |
|---|---|---|---|---|
| v23 250k | 25.2 mm | 1.00 m / 1.98 W | 0.08 m | 0.56 m |
| v23 500k | 25.3 mm | 1.01 m / 2.01 W | 0.08 m (25.0 → 25.4) | 0.64 m |
| v23 2M | 30.6 mm | 1.02 m / 2.66 W | 0.43 m (30.1 → 30.7) | 0.76 m |
| v23 2.5M | 30.5 mm | 1.04 m / 2.82 W | 0.08 m (29.5 → 30.5) | 0.65 m |
| v23b best (toplam 3.75M) | 32.6 mm | 1.05 m / 3.11 W | 0.79 m (31.1 → 32.9) | 0.67 m |

## Düz zemin (her yön seti, rastgeleleştirme kapalı)

| model | her yön ödül | her yön güç (W) | ileri 0.10 hız (m/s) | ileri güç (W) | kaldırma (mm) |
|---|---|---|---|---|---|
| ppo_omni_250k | 2.868 | 1.98 | 0.100 | 2.00 | 25.0 |
| v12_zemin 2M | 2.757 | 2.42 | 0.096 | 2.39 | 25.0 |
| v22 500k | 2.838 | 2.21 | 0.099 | 2.22 | 25.0 |
| v22 1M | 2.852 | 2.20 | 0.099 | 2.24 | 25.0 |
| v22 2M | 2.815 | 2.40 | 0.100 | 2.65 | 25.0 |
| ppo_kaldirma35_250k | 2.855 | 2.43 | 0.102 | 2.39 | 35.2 |
| ppo_lift50_3750k | 2.693 | 4.02 | 0.114 | 3.99 | 50.0 |
| v23 2M | 2.819 | 2.72 | 0.102 | 2.66 | 30.6 |
| v23b best (3.75M) | 2.760 | 3.07 | 0.105 | 3.11 | 32.6 |
| v23b son (4M) | 2.744 | 3.17 | 0.108 | 3.15 | 33.6 |

## Bütün zemin türleri (rastgeleleştirme açık, 3 tohum, 10 s)

| model | basamak 45 | basamak 60 | çukur 60 geri | çukur 45 yana | yokuş 20 | kaygan yokuş 10 μ0.25 | kaygan yokuş 15 μ0.3 | kaygan yokuş 20 μ0.4 | kaygan yan 15 μ0.3 | engebe 40 | engebe 60 | engebe 40 yana | düz | engel skoru |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ppo_omni_250k | 0.08 (0/3) | 0.08 (0/3) | 0.12 (0/3) | 0.11 (0/3) | 0.85 (3/3) | 0.47 (2/3) | -0.45 (0/3) | -0.52 (0/3) | 1.01 (3/3) | 0.58 (3/3) | 0.07 (0/3) | 0.36 (0/3) | 1.02 (3/3) | 0.338 |
| v12_zemin 2M | 0.08 (0/3) | 0.08 (0/3) | 0.13 (0/3) | 0.11 (0/3) | 0.80 (3/3) | 0.44 (2/3) | -0.26 (0/3) | -0.26 (0/3) | 0.94 (3/3) | 0.60 (3/3) | 0.14 (0/3) | 0.23 (0/3) | 0.99 (3/3) | 0.316 |
| v22 500k | 0.08 (0/3) | 0.08 (0/3) | 0.12 (0/3) | 0.11 (0/3) | 0.88 (3/3) | 0.54 (2/3) | -0.24 (0/3) | -0.24 (0/3) | 1.03 (3/3) | 0.43 (2/3) | 0.07 (0/3) | 0.33 (0/3) | 1.02 (3/3) | 0.329 |
| v22 1M | 0.08 (0/3) | 0.08 (0/3) | 0.13 (0/3) | 0.11 (0/3) | 0.86 (3/3) | 0.52 (2/3) | -0.34 (0/3) | -0.35 (0/3) | 1.02 (3/3) | 0.63 (3/3) | 0.17 (0/3) | 0.35 (1/3) | 1.01 (3/3) | 0.353 |
| v22 2M | 0.08 (0/3) | 0.08 (0/3) | 0.12 (0/3) | 0.14 (0/3) | 0.88 (3/3) | 0.58 (3/3) | 0.04 (0/3) | 0.08 (0/3) | 1.06 (3/3) | 0.56 (3/3) | 0.19 (1/3) | 0.36 (0/3) | 1.04 (3/3) | 0.371 |
| ppo_kaldirma35_250k | 0.80 (3/3) | 0.08 (0/3) | 0.12 (0/3) | 0.14 (0/3) | 0.87 (3/3) | 0.57 (3/3) | -0.26 (0/3) | -0.32 (0/3) | 1.04 (3/3) | 0.83 (3/3) | 0.65 (3/3) | 0.48 (3/3) | 1.03 (3/3) | 0.496 |
| v23 2M | 0.08 (0/3) | 0.08 (0/3) | 0.12 (0/3) | 0.11 (0/3) | 0.86 (3/3) | 0.52 (2/3) | -0.11 (0/3) | -0.15 (0/3) | 1.00 (3/3) | 0.82 (3/3) | 0.50 (3/3) | 0.46 (3/3) | 1.03 (3/3) | 0.411 |
| v23b best (3.75M) | 0.63 (3/3) | 0.08 (0/3) | 0.12 (0/3) | 0.14 (0/3) | 0.87 (3/3) | 0.44 (2/3) | -0.31 (0/3) | -0.34 (0/3) | 1.02 (3/3) | 0.71 (3/3) | 0.62 (3/3) | 0.47 (3/3) | 1.05 (3/3) | 0.457 |
| v23b son (4M) | 0.78 (3/3) | 0.08 (0/3) | 0.12 (0/3) | 0.23 (0/3) | 0.88 (3/3) | 0.43 (2/3) | -0.48 (0/3) | -0.58 (0/3) | 1.05 (3/3) | 0.77 (3/3) | 0.76 (3/3) | 0.48 (3/3) | 1.08 (3/3) | 0.499 |

(v12'nin gorev.json'ı yok, `:residual` ile yüklendi; taban 25 mm varsayılan.)
