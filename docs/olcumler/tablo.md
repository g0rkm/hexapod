<!-- ÜRETİLDİ: python -m hexapod_rl.olcum tripod tripod:50 models/ppo_omni_250k/model.zip models/ppo_lift50_3750k/model.zip models/ppo_kaldirma35_250k/model.zip models/ppo_kaldirma35_250k/model.zip+refleks models/ppo_refleks_1500k/model.zip+refleks -->
zemin kaynağı: hexapod_terrain (S5) · komut vx=0.1 vy=0 wz=0 · 10 s · 3 tohum, rastgeleleştirme açık · temiz ölçüm ayrıca alındı
enerji: güç kontrol adımındaki bütün fizik adımlarının ortalaması (2026-09-28 öncesi tablolar tek anlık örnekti, karşılaştırılamaz)
+refleks: DENEYSEL mesafe sensörü yerleşimi (gövde kenarında ileri ve ±90°, 20° aşağı; robot.yaml'da yok, D8) · engel yüksekliği 'egim' kipinde · tekrarlarda %5 gürültü + %10 düşen okuma

| Zemin | tripod | tripod:50 | models/ppo_omni_250k/model.zip | models/ppo_lift50_3750k/model.zip | models/ppo_kaldirma35_250k/model.zip | models/ppo_kaldirma35_250k/model.zip+refleks | models/ppo_refleks_1500k/model.zip+refleks |
|---|---|---|---|---|---|---|---|
| düz | 0.098 m/s · 36.7 J/m · 3/3 | 0.100 m/s · 42.9 J/m · 3/3 | 0.102 m/s · 36.2 J/m · 3/3 | 0.114 m/s · 48.2 J/m · 3/3 | 0.103 m/s · 43.5 J/m · 3/3 | 0.102 m/s · 36.5 J/m · 3/3 | 0.106 m/s · 38.7 J/m · 3/3 |
| yokuş yukarı 10° | 0.091 m/s · 39.4 J/m · 3/3 | 0.095 m/s · 45.6 J/m · 3/3 | 0.096 m/s · 39.4 J/m · 3/3 | 0.107 m/s · 52.3 J/m · 3/3 | 0.098 m/s · 46.9 J/m · 3/3 | 0.097 m/s · 39.3 J/m · 3/3 | 0.099 m/s · 43.3 J/m · 3/3 |
| yokuş yukarı 20° | 0.076 m/s · 49.7 J/m · 3/3 | 0.078 m/s · 57.7 J/m · 3/3 | 0.085 m/s · 46.5 J/m · 3/3 | 0.094 m/s · 61.3 J/m · 3/3 | 0.087 m/s · 54.6 J/m · 3/3 | 0.086 m/s · 46.5 J/m · 3/3 | 0.087 m/s · 57.1 J/m · 3/3 |
| yokuş aşağı 10° | 0.100 m/s · 37.6 J/m · 3/3 | 0.103 m/s · 44.0 J/m · 3/3 | 0.103 m/s · 36.4 J/m · 3/3 | 0.117 m/s · 47.3 J/m · 3/3 | 0.106 m/s · 43.8 J/m · 3/3 | 0.104 m/s · 36.7 J/m · 3/3 | 0.109 m/s · 38.7 J/m · 3/3 |
| yan eğim 10° | 0.098 m/s · 37.4 J/m · 3/3 | 0.101 m/s · 43.6 J/m · 3/3 | 0.102 m/s · 37.3 J/m · 3/3 | 0.114 m/s · 47.8 J/m · 3/3 | 0.103 m/s · 44.9 J/m · 3/3 | 0.102 m/s · 37.5 J/m · 3/3 | 0.106 m/s · 40.6 J/m · 3/3 |
| yokuş yukarı 10° μ0.3 | 0.046 m/s · 82.5 J/m · 3/3 | 0.056 m/s · 78.4 J/m · 3/3 | 0.071 m/s · 52.7 J/m · 3/3 | 0.076 m/s · 65.2 J/m · 3/3 | 0.077 m/s · 58.4 J/m · 3/3 | 0.072 m/s · 53.0 J/m · 3/3 | 0.069 m/s · 57.8 J/m · 3/3 |
| yokuş yukarı 15° μ0.3 | **geri kaydı** -0.773 m/s, 0/3 | **geri kaydı** -0.645 m/s, 1/3 | **geri kaydı** -0.043 m/s | **geri kaydı** -0.019 m/s | **geri kaydı** -0.026 m/s | **geri kaydı** -0.038 m/s | **geri kaydı** -0.017 m/s |
| basamak 30 mm | 0.070 m/s · 62.8 J/m · 3/3 | 0.093 m/s · 54.0 J/m · 3/3 | 0.078 m/s · 57.7 J/m · 3/3 | 0.108 m/s · 56.4 J/m · 3/3 | 0.090 m/s · 59.3 J/m · 3/3 | 0.097 m/s · 51.6 J/m · 3/3 | 0.102 m/s · 53.8 J/m · 3/3 |
| basamak 45 mm | 0.019 m/s · 222.2 J/m · 3/3 | 0.087 m/s · 64.1 J/m · 3/3 | 0.018 m/s · 264.4 J/m · 3/3 | 0.107 m/s · 59.4 J/m · 3/3 | 0.074 m/s · 82.2 J/m · 3/3 | 0.098 m/s · 57.2 J/m · 3/3 | 0.097 m/s · 65.1 J/m · 3/3 |
| basamak 60 mm | 0.019 m/s · 226.1 J/m · 3/3 | 0.050 m/s · 136.9 J/m · 3/3 | 0.018 m/s · 264.1 J/m · 3/3 | 0.052 m/s · 199.1 J/m · 3/3 | 0.018 m/s · 307.4 J/m · 3/3 | 0.089 m/s · 70.8 J/m · 3/3 | 0.080 m/s · 90.4 J/m · 3/3 |
| merdiven 6x35 mm | 0.064 m/s · 75.1 J/m · 3/3 | 0.090 m/s · 61.4 J/m · 3/3 | 0.069 m/s · 73.9 J/m · 3/3 | 0.105 m/s · 63.0 J/m · 3/3 | 0.086 m/s · 68.5 J/m · 3/3 | 0.099 m/s · 56.9 J/m · 3/3 | 0.102 m/s · 61.0 J/m · 3/3 |
| engebe 40 mm | 0.097 m/s · 38.7 J/m · 3/3 | 0.099 m/s · 45.1 J/m · 3/3 | 0.100 m/s · 39.3 J/m · 3/3 | 0.113 m/s · 50.4 J/m · 3/3 | 0.102 m/s · 46.1 J/m · 3/3 | 0.101 m/s · 43.3 J/m · 3/3 | 0.105 m/s · 45.1 J/m · 3/3 |
| engebe 60 mm | 0.095 m/s · 40.5 J/m · 3/3 | 0.098 m/s · 46.9 J/m · 3/3 | 0.098 m/s · 40.9 J/m · 3/3 | 0.112 m/s · 52.0 J/m · 3/3 | 0.100 m/s · 48.0 J/m · 3/3 | 0.102 m/s · 46.1 J/m · 3/3 | 0.106 m/s · 49.6 J/m · 3/3 |
| çukur 40 mm | 0.014 m/s · 304.0 J/m · 3/3 | 0.088 m/s · 61.6 J/m · 3/3 | 0.013 m/s · 371.6 J/m · 3/3 | 0.107 m/s · 60.3 J/m · 3/3 | 0.069 m/s · 89.1 J/m · 3/3 | 0.099 m/s · 54.1 J/m · 3/3 | 0.105 m/s · 55.2 J/m · 3/3 |
| yayla 50 mm | 0.054 m/s · 86.4 J/m · 3/3 | 0.087 m/s · 64.1 J/m · 3/3 | 0.046 m/s · 105.3 J/m · 3/3 | 0.099 m/s · 67.1 J/m · 3/3 | 0.073 m/s · 80.4 J/m · 3/3 | 0.086 m/s · 64.8 J/m · 3/3 | 0.091 m/s · 64.6 J/m · 3/3 |
