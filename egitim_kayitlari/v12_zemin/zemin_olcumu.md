# v12_zemin — deneme zeminlerinde ara kayıtlar (2026-09-26)

`ppo_omni_250k`'dan devam, taban ayak kaldırma 25 mm, `--terrains deneme`,
std 0.1. 2.25M'de durduruldu. Ölçüm: deterministik, rastgeleleştirmesiz,
10 s; 10 s'de alınan yol (komut yönünde). Tek ölçüm (rastgeleleştirme
kapalıyken simülasyon belirlenimci).

| model | basamak 30 ileri | basamak 45 ileri | basamak 60 ileri | çukur 45 geri | çukur 45 yana | yayla 50 ileri (iniş) | yokuş yukarı 20 | düz ileri |
|---|---|---|---|---|---|---|---|---|
| tripod | 0.71 m | 0.09 m | 0.09 m | 0.13 m | 0.12 m | 0.58 m | 0.77 m | 0.98 m |
| phase | 0.76 m | 0.08 m | 0.08 m | 0.12 m | 0.11 m | 0.54 m | 0.79 m | 0.96 m |
| ppo_omni_250k | 0.81 m | 0.08 m | 0.08 m | 0.12 m | 0.11 m | 0.48 m | 0.84 m | 1.00 m |
| ppo_250000_steps | 0.81 m | 0.08 m | 0.08 m | 0.11 m | 0.11 m | 0.48 m | 0.83 m | 1.00 m |
| ppo_500000_steps | 0.82 m | 0.08 m | 0.08 m | 0.12 m | 0.11 m | 0.47 m | 0.85 m | 1.02 m |
| ppo_750000_steps | 0.84 m | 0.08 m | 0.08 m | 0.12 m | 0.11 m | 0.48 m | 0.81 m | 1.00 m |
| ppo_1000000_steps | 0.81 m | 0.08 m | 0.08 m | 0.12 m | 0.11 m | 0.48 m | 0.81 m | 1.01 m |
| ppo_1250000_steps | 0.81 m | 0.08 m | 0.08 m | 0.12 m | 0.11 m | 0.48 m | 0.79 m | 1.00 m |
| ppo_1500000_steps | 0.81 m | 0.08 m | 0.08 m | 0.12 m | 0.11 m | 0.48 m | 0.79 m | 0.99 m |
| ppo_1750000_steps | 0.83 m | 0.08 m | 0.08 m | 0.12 m | 0.11 m | 0.49 m | 0.81 m | 0.99 m |
| ppo_2000000_steps | 0.80 m | 0.08 m | 0.08 m | 0.13 m | 0.11 m | 0.47 m | 0.79 m | 0.96 m |

Sonuç: 2M adımda hiçbir engelde iyileşme yok; keşif gürültüsüyle (std 0.1)
ayağı yükseğe kaldırmak bulunamadı. Taban ayak kaldırma 50 mm'ye çıkarılınca
(v13_lift50) aynı engeller geçildi.
