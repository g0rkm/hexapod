# v18–v21 öğrenilmiş ayak kaldırma: ham ölçümler (2026-09-27)

Özet ve yorum: `models/README.md` ("Öğrenilmiş ayak kaldırma"), PROJE_DEVIR
ders 35–36. Hepsi RL simi (HexapodEnv), deterministik, 10 s.

## Ara kayıtlarda kaldırma (düz / basamak 45 / engebe 40, 0.1 m/s, rastgeleleştirme kapalı)

"önce/engelde/üstte": basamak 45'te gövde x < 0.05, 0.05–0.35, > 0.35 m iken
ortalama kaldırma. Güç ortalama mekanik güç (W).

| Eğitim / adım | Kaldırma std (çıkış) | Düz yol / kaldırma / güç | Basamak 45 yol / kaldırma (önce, engelde) | Engebe 40 yol / kaldırma |
|---|---|---|---|---|
| v19 250k | 0.407 | 1.01 m / 25.70 / 2.05 | 0.08 m / 25.71, 25.70 | 0.57 m / 25.70 |
| v19 500k | 0.409 | 1.02 m / 26.12 / 2.12 | 0.08 m / 26.35, 26.11 | 0.57 m / 26.10 |
| v20 250k | 0.511 | 1.00 m / 25.55 / 2.03 | 0.08 m / 25.55, 25.54 | 0.56 m / 25.55 |
| v20 500k | 0.522 | 1.01 m / 26.58 / 2.13 | 0.08 m / 26.84, 26.55 | 0.65 m / 26.56 |
| v20 750k | 0.520 | 0.97 m / 27.58 / 2.27 | 0.08 m / 28.16, 27.54 | 0.68 m / 27.58 |
| v20 1M | 0.519 | 1.00 m / 27.80 / 2.25 | 0.08 m / 28.12, 27.95 | 0.68 m / 27.94 |
| v21 250k | 0.513 | 1.02 m / 35.21 / 2.39 | 0.80 m / 35.10, 35.19 | 0.80 m / 35.14 |
| v21 500k | 0.521 | 1.04 m / 36.41 / 2.52 | 0.82 m / 36.35, 36.31 | 0.87 m / 36.28 |
| v21 750k | 0.515 | 1.02 m / 36.80 / 2.59 | 0.66 m / 37.32, 36.60 | 0.89 m / 36.81 |
| v21 1M | 0.512 | 1.03 m / 37.81 / 2.65 | 0.75 m / 38.64, 37.63 | 0.91 m / 37.81 |
| v21 1.5M | — | 1.08 m / 38.38 / 2.93 | 0.87 m / 39.64, 38.12 | 0.94 m / 38.32 |
| v21 2M | — | 1.06 m / 40.32 / 3.07 | 0.87 m / 41.12, 40.23 | 0.95 m / 40.26 |

v19 her adım seçim (eski kod); v20/v21 salınım başı seçim. v19'un ölçümü
eski kodla alındı (eğitildiği gibi).

## Kaldırma elle sabit (v20 250k, aynı politika, 0.1 m/s)

```
düz        kaldırma 25  yol 1.00  güç 2.00
düz        kaldırma 35  yol 1.01  güç 2.36
düz        kaldırma 45  yol 1.04  güç 2.80
düz        kaldırma 55  yol 1.03  güç 3.81
basamak45  kaldırma 25  yol 0.08  güç 2.45
basamak45  kaldırma 35  yol 0.78  güç 3.52
basamak45  kaldırma 45  yol 0.89  güç 3.99
basamak45  kaldırma 55  yol 0.94  güç 4.81
çukur45    kaldırma 25  yol 0.13  güç 2.43
çukur45    kaldırma 35  yol 0.77  güç 3.35
çukur45    kaldırma 45  yol 0.84  güç 4.10
çukur45    kaldırma 55  yol 0.93  güç 4.90
engebe40   kaldırma 25  yol 0.57  güç 3.86
engebe40   kaldırma 35  yol 0.78  güç 3.65
engebe40   kaldırma 45  yol 0.93  güç 4.16
engebe40   kaldırma 55  yol 0.95  güç 5.01
```

## Düz zemin, v21 adayları (her yön seti, rastgeleleştirme kapalı)

| model | her yön ödül | her yön güç (W) | ileri 0.10 hız (m/s) | ileri güç (W) | kaldırma (mm) |
|---|---|---|---|---|---|
| ppo_omni_250k | 2.868 | 1.98 | 0.100 | 2.00 | 25.0 |
| ppo_lift50_3750k | 2.693 | 4.02 | 0.114 | 3.99 | 50.0 |
| ppo_250000_steps | 2.855 | 2.43 | 0.102 | 2.39 | 35.2 |
| ppo_1000000_steps | 2.830 | 2.69 | 0.103 | 2.65 | 37.8 |
| ppo_2000000_steps | 2.816 | 2.93 | 0.106 | 3.07 | 40.3 |
| best_model | 2.803 | 3.02 | 0.110 | 3.19 | 42.3 |
| ppo_3000000_steps | 2.758 | 3.55 | 0.111 | 3.79 | 46.5 |
| model | 2.716 | 4.31 | 0.108 | 4.65 | 52.7 |

## Bütün zemin türleri, v21 adayları (rastgeleleştirme açık, 3 tohum)

| model | basamak 45 | basamak 60 | çukur 60 geri | çukur 45 yana | yokuş 20 | kaygan yokuş 10 μ0.25 | kaygan yokuş 15 μ0.3 | kaygan yokuş 20 μ0.4 | kaygan yan 15 μ0.3 | engebe 40 | engebe 60 | engebe 40 yana | düz | engel skoru |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ppo_omni_250k | 0.08 (0/3) | 0.08 (0/3) | 0.12 (0/3) | 0.11 (0/3) | 0.85 (3/3) | 0.47 (2/3) | -0.45 (0/3) | -0.52 (0/3) | 1.01 (3/3) | 0.58 (3/3) | 0.07 (0/3) | 0.36 (0/3) | 1.02 (3/3) | 0.338 |
| ppo_lift50_3750k | 1.04 (3/3) | 0.70 (3/3) | 0.98 (3/3) | 0.63 (3/3) | 0.94 (3/3) | 0.57 (3/3) | -0.20 (0/3) | -0.14 (0/3) | 1.14 (3/3) | 0.99 (3/3) | 0.97 (3/3) | 0.62 (3/3) | 1.14 (3/3) | 0.763 |
| ppo_250000_steps | 0.80 (3/3) | 0.08 (0/3) | 0.12 (0/3) | 0.14 (0/3) | 0.87 (3/3) | 0.57 (3/3) | -0.26 (0/3) | -0.32 (0/3) | 1.04 (3/3) | 0.83 (3/3) | 0.65 (3/3) | 0.48 (3/3) | 1.03 (3/3) | 0.496 |
| ppo_1000000_steps | 0.83 (3/3) | 0.08 (0/3) | 0.12 (0/3) | 0.14 (0/3) | 0.86 (3/3) | 0.61 (3/3) | -0.05 (0/3) | -0.05 (0/3) | 1.06 (3/3) | 0.91 (3/3) | 0.71 (3/3) | 0.56 (3/3) | 1.04 (3/3) | 0.523 |
| ppo_2000000_steps | 0.81 (3/3) | 0.25 (1/3) | 0.12 (0/3) | 0.30 (1/3) | 0.93 (3/3) | 0.60 (3/3) | -0.05 (0/3) | 0.04 (0/3) | 1.09 (3/3) | 0.79 (3/3) | 0.86 (3/3) | 0.54 (3/3) | 1.07 (3/3) | 0.567 |
| best_model | 0.79 (3/3) | 0.46 (2/3) | 0.12 (0/3) | 0.29 (1/3) | 0.95 (3/3) | 0.62 (3/3) | -0.06 (0/3) | -0.00 (0/3) | 1.14 (3/3) | 1.02 (3/3) | 0.90 (3/3) | 0.53 (3/3) | 1.11 (3/3) | 0.599 |
| ppo_3000000_steps | 0.83 (3/3) | 0.41 (2/3) | 0.58 (3/3) | 0.23 (1/3) | 0.96 (3/3) | 0.57 (3/3) | -0.04 (0/3) | 0.08 (0/3) | 1.13 (3/3) | 0.96 (3/3) | 0.98 (3/3) | 0.58 (3/3) | 1.11 (3/3) | 0.643 |
| model | 0.97 (3/3) | 0.31 (1/3) | 0.49 (3/3) | 0.39 (2/3) | 0.94 (3/3) | 0.46 (2/3) | -0.26 (0/3) | -0.17 (0/3) | 1.08 (3/3) | 1.01 (3/3) | 1.02 (3/3) | 0.62 (3/3) | 1.10 (3/3) | 0.652 |

## ROS'lu sim, `models/ppo_kaldirma35_250k/policy.npz` (gerçek politika düğümü)

Düz (`bash tools/wsl/politika_ros_olcum.sh models/ppo_kaldirma35_250k/policy.npz`):

| Komut (vx, vy, wz) | gövde vx | gövde vy | dönüş rad/s | izleme | yol (tüm süre) | yükseklik |
|---|---|---|---|---|---|---|
| (0.1, 0.0, 0.0) | +0.102 | -0.001 | +0.004 | %102 | 0.82 m | 98 mm (z 98) |
| (-0.1, 0.0, 0.0) | -0.102 | +0.002 | -0.002 | %102 | 0.79 m | 98 mm (z 98) |
| (0.0, 0.06, 0.0) | -0.002 | +0.063 | -0.002 | %105 | 0.49 m | 99 mm (z 98) |
| (0.0, -0.06, 0.0) | +0.002 | -0.062 | +0.005 | %104 | 0.48 m | 98 mm (z 96) |
| (0.0, 0.0, 0.4) | -0.000 | +0.004 | +0.413 | %103 | 0.02 m | 99 mm (z 96) |
| (0.0, 0.0, -0.4) | -0.001 | -0.002 | -0.408 | %102 | 0.01 m | 98 mm (z 96) |
| (0.1, 0.04, 0.25) | +0.100 | +0.043 | +0.259 | %100 / %107 / %104 | 0.72 m | 99 mm (z 98) |
| (0.0, 0.0, 0.0) | -0.000 | -0.000 | +0.000 | hareket 0.0 mm | 0.00 m | 99 mm (z 99) |

Deneme zemini dünyaları (`WORLD=... WORLD_NAME=zemin SURE=12`):

| Dünya | Komut | gövde vx | gövde vy | izleme | yol | yükseklik |
|---|---|---|---|---|---|---|
| basamak45 | (0.1, 0.0, 0.0) | +0.074 | -0.003 | %74 | 0.93 m | 129 mm (z 143) |
| basamak60 | (0.1, 0.0, 0.0) | -0.001 | -0.002 | %-1 | 0.08 m | 100 mm (z 100) |
| cukur45 | (-0.1, 0.0, 0.0) | -0.063 | -0.003 | %63 | 0.85 m | 127 mm (z 144) |
| cukur45 | (0.0, 0.06, 0.0) | -0.002 | +0.063 | %105 | 0.75 m | 144 mm (z 144) |

**Düzeltme (2026-09-27):** son satır geçersiz. İki komut aynı dünyada robot
sıfırlanmadan art arda koşuldu; yana komutu robot geri komutuyla çukurdan
çıktıktan sonra başladı (ortalama yükseklik 144 mm: baştan dışarıda). Yana
komutu tek başına (`KOMUTLAR="0,0.06,0"`):

| Model | gövde vx | gövde vy | izleme | yol | yükseklik |
|---|---|---|---|---|---|
| ppo_kaldirma35_250k | -0.001 | +0.013 | %21 | 0.20 m | 99 mm (z 99), çukurda kaldı |
| ppo_lift50_3750k | -0.009 | +0.054 | %90 | 0.68 m | 119 mm (z 143), çıktı |

RL simde aynı durum (yana, 45 mm çukur): ppo_kaldirma35_250k 0.11–0.21 m
(rastgeleleştirme açık/kapalı, fizik 1/2 ms, 10/12 s; hepsi çukurda);
geri 0.68–1.00 m (çıkıyor).
