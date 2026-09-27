# Eğitilmiş politikalar

Her klasör bir PPO eğitiminin son modeli (`model.zip`, Stable-Baselines3) ve
değerlendirmesi. `policy.npz`: aynı politikanın torch'suz hâli (robotta
`hexapod_policy` düğümü okur; `python -m hexapod_rl.export <model.zip>` üretir). Eğitimlerin tam çıktıları (ara kayıtlar, `progress.csv`)
eğitimi yapan makinede `~/hexapod_runs/<ad>/` altında; depoda yalnızca
kayda değer modeller tutulur.

Değerlendirmek için (WSL, ortam: `tools/wsl/rl_kurulum.sh`):

```bash
python -m hexapod_rl.evaluate models/ppo_omni_250k/model.zip --residual --vx -0.1   # geri
python -m hexapod_rl.evaluate models/ppo_omni_250k/model.zip --residual --vx 0 --wz 0.4
python -m hexapod_rl.evaluate models/ppo_v4_4M/model.zip --vx 0.1 --noise 0.1
python -m hexapod_rl.evaluate tripod --vx 0.1          # karşılaştırma: Samet'in tripod'u
python -m hexapod_rl.terrain_probe tripod models/ppo_omni_250k/model.zip:residual
```

## Mesafe sensörlü kaldırma refleksi (2026-09-27)

Kör politika ayak kaldırmayı zemine göre seçemiyor (ders 33, 35). Soru: önüne
bakan mesafe sensörleri (robotta 3 VL53L0X var) engeli görürse kaldırma
zemine göre ayarlanabilir mi, sensör nereye bakmalı?

`python -m hexapod_rl.reflex_probe` — eğitim yok: `ppo_kaldirma35_250k`'nın
eklemleri, kaldırma çıkışı ise her adımda ya sabit ya da
`hexapod_policy.lift_reflex.LiftReflex`'in kararı. Refleks: her sensörün
ölçümünden ışının çarptığı noktanın robotun durduğu zemine göre yüksekliği
(mesafe + sensör yeri + IMU'nun yerçekimi yönü; robotta da hesaplanabilir).
Bir sensör art arda 2 okumada 20 mm'yi aşarsa engel sayılır; son 3 s'de
görülen en yüksek engel + 15 mm kaldırma olur (25–60 mm), yoksa 25 mm.

**Sensör yerleşimi DENEYSEL** (robot.yaml'da `null`, D8): gövde önünde üç
sensör (x 0.10 m, z +0.02 m; bakış 0° ve ±25°, menzil 1 m), aşağı bakış
açısı taranıyor. Işın idealdir (koni, yansıma yok); gürültü ve okuma
düşmesi taranan büyüklükler. Rastgeleleştirme açık, 3 tohum, 0.1 m/s ileri,
10 s. Hücre: yol, geçen tohum, ortalama güç, ortalama kaldırma.

İdeal sensör:

| Kaldırma | düz | basamak 45 | basamak 60 | çukur 45 ileri | engebe 40 | engebe 60 | yokuş 20 |
|---|---|---|---|---|---|---|---|
| sabit 25 mm | 1.02 (3/3) 3.95 W 25 mm | 0.08 (0/3) 5.46 W 25 mm | 0.08 (0/3) 5.47 W 25 mm | 0.13 (0/3) 5.39 W 25 mm | 0.57 (3/3) 5.58 W 25 mm | 0.12 (0/3) 6.57 W 25 mm | 0.86 (3/3) 4.34 W 25 mm |
| sabit 35 mm | 1.03 (3/3) 4.38 W 35 mm | 0.81 (3/3) 5.87 W 35 mm | 0.08 (0/3) 5.89 W 35 mm | 0.68 (3/3) 6.12 W 35 mm | 0.86 (3/3) 5.99 W 35 mm | 0.64 (3/3) 6.28 W 35 mm | 0.87 (3/3) 4.71 W 35 mm |
| sabit 50 mm | 1.04 (3/3) 4.45 W 50 mm | 0.88 (3/3) 6.08 W 50 mm | 0.41 (2/3) 7.16 W 50 mm | 0.94 (3/3) 6.00 W 50 mm | 0.91 (3/3) 6.15 W 50 mm | 0.91 (3/3) 6.59 W 50 mm | 0.88 (3/3) 4.83 W 50 mm |
| refleks, 20° aşağı | 1.02 (3/3) 3.95 W 25 mm | 0.97 (3/3) 5.44 W 47 mm | **0.86 (3/3)** 6.19 W 55 mm | 0.98 (3/3) 5.48 W 49 mm | 0.76 (3/3) 5.92 W 36 mm | 0.75 (3/3) 6.13 W 45 mm | 0.87 (3/3) 4.44 W 59 mm |
| refleks, 25° aşağı | 1.02 (3/3) 3.95 W 25 mm | 0.97 (3/3) 5.38 W 47 mm | 0.54 (3/3) 6.77 W 58 mm | 0.98 (3/3) 5.51 W 47 mm | 0.80 (3/3) 5.86 W 37 mm | 0.77 (3/3) 5.99 W 45 mm | 0.87 (3/3) 4.44 W 59 mm |
| refleks, 30° aşağı | 1.02 (3/3) 3.95 W 25 mm | 0.98 (3/3) 5.45 W 47 mm | 0.61 (3/3) 6.86 W 58 mm | 0.97 (3/3) 5.51 W 46 mm | 0.84 (3/3) 5.70 W 34 mm | 0.71 (3/3) 6.04 W 42 mm | 0.87 (3/3) 4.44 W 59 mm |
| refleks, 45° aşağı | 1.02 (3/3) 3.95 W 25 mm | 0.08 (0/3) 5.46 W 25 mm | 0.08 (0/3) 5.47 W 25 mm | 0.13 (0/3) 5.39 W 25 mm | 0.76 (3/3) 5.85 W 33 mm | 0.64 (3/3) 6.18 W 38 mm | 0.87 (3/3) 4.44 W 59 mm |

Gürültülü sensör (bağıl %5 gürültü, okumaların %10'u gelmiyor, iki kontrol
adımında bir okuma; `--noise 5 --drop 10 --every 2`):

| Kaldırma | düz | basamak 45 | basamak 60 | çukur 45 ileri | engebe 40 | engebe 60 | yokuş 20 |
|---|---|---|---|---|---|---|---|
| refleks, 20° aşağı | 1.02 (3/3) 3.95 W 25 mm | 0.98 (3/3) 5.34 W 49 mm | **0.86 (3/3)** 6.11 W 57 mm | 0.98 (3/3) 5.49 W 51 mm | 0.89 (3/3) 5.91 W 39 mm | 0.78 (3/3) 6.05 W 46 mm | 0.87 (3/3) 4.44 W 59 mm |
| refleks, 25° aşağı | 1.02 (3/3) 3.95 W 25 mm | 0.98 (3/3) 5.39 W 49 mm | 0.65 (3/3) 6.60 W 59 mm | 0.99 (3/3) 5.50 W 50 mm | 0.91 (3/3) 5.98 W 41 mm | 0.83 (3/3) 6.31 W 48 mm | 0.87 (3/3) 4.44 W 59 mm |
| refleks, 30° aşağı | 1.02 (3/3) 3.95 W 25 mm | 0.98 (3/3) 5.35 W 48 mm | 0.69 (2/3) 6.28 W 56 mm | 0.98 (3/3) 5.50 W 48 mm | 0.89 (3/3) 5.92 W 39 mm | 0.76 (3/3) 5.98 W 44 mm | 0.87 (3/3) 4.44 W 59 mm |

- **Kör politikanın ödünleşimi sensörle çözülüyor.** Refleks düzde sabit 25
  mm'nin gücünde (3.95 W; rastgeleleştirme açıkken ölçüldü, sabit 50 mm
  4.45 W), engellerde sabit 50 mm'den iyi: 45 mm basamak 0.97–0.98 (sabit
  50: 0.88), 60 mm basamak 20°'de 0.86 3/3 (sabit 50: 0.41 2/3), çukur
  0.98 (0.94). Engebe 60'ta sabit 50 hâlâ önde (0.91'e 0.75–0.83):
  refleks engeli görene kadar 25 mm'de yürüyor.
- **Bakış açısı önemli:** 45° çok dik; ışın gövdeye çok yakın düşüyor,
  basamağı ön ayaklar dayandığında ancak alt kenarından görüyor (eşiği
  aşmıyor), refleks hiç tetiklenmiyor. 20°'de ışın sensörden ~0.33 m ileride; 60 mm
  basamakta en iyisi bu.
- **Gürültü:** ilk sürüm (eşik 12 mm, tek okuma) %5 gürültüde düzde yanlış
  alarmla kaldırmayı 25 → 32 mm'ye, gücü 3.95 → 4.35 W'a çıkarıyordu.
  Eşik 20 mm + art arda 2 okuma bunu bitirdi (düzde 25 mm), engel
  becerisi aynı kaldı.
- **Kalibrasyon hatasına karşı da dayanıklı** (`--offset σ`, eklem başına
  rastgele ofset; gürültülü sensör, 20°): σ2°'de sabit 35 mm'nin 45 mm
  basamağı 0.44 m'ye (2/3) düşüyor, refleks 0.96 m (3/3); 60 mm basamak
  refleks 0.71 (3/3), sabit 50 mm 0.55 (2/3); düz güç refleks 4.31 W, sabit
  50 mm 4.85 W. σ4°'te basamak 45 / 60 / çukur 45: refleks 0.94 / 0.51 /
  0.94, sabit 50 mm 0.78 / 0.36 / 0.68; düzde refleksin hızı 0.87'ye iniyor
  (25 mm'de ayak ofsetle sürtüyor; sabit 50 mm 1.00). Dayanıklılık
  taramasında (yukarıda) 35 mm'lik modeli kırılgan yapan şey engelde payın
  dar olmasıydı; refleks engelde payı açıyor.
- **Sınırlar:** sensörler yalnız ileri bakıyor; yana/geri yürürken refleks
  engeli görmez (25 mm). Işın ideal (VL53L0X'in görüş konisi, yansıtıcılık
  ve ölçüm süresi modellenmedi). Kaldırma çıkışlı modelle çalışıyor
  (öğrenilmiş kaldırmalı eğitimde eklemler 25–60 mm'ye alıştı). Robot
  düğümüne bağlanması S7 (sürücü) ve D8'i (yerleşim) bekliyor.
- **D8'e öneri:** mesafe sensörlerinden en az biri gövdenin önünde, yerin
  ~12 cm üstünde, 20–25° aşağı bakacak şekilde. Yan sensörler ±25°.
  Kesin yerleşim robotta ölçülüp robot.yaml'a girince bu tablo yeniden
  koşulur (`--pitch`).

**Gerçek robotta DENENMEDİ.**

## Müfredat: kolaydan zora zemin (2026-09-27, v22–v23)

`train.py --curriculum deneme` (`terrain_probe.CURRICULA`): her ortam bir zemin
türünde (çukur, yayla, eğim x/y, düz) en kolay seviyeden başlar; bölümde 0.5 m
ilerlerse zorlaşır, devrilir ya da takılırsa kolaylaşır. Çukur ve yayla 10 →
60 mm, eğim 5 → 25°. Soru: sabit setin (v12, v20) takıldığı yerden müfredat
çıkarır mı? Ham ölçümler ve seviye eğrileri:
`egitim_kayitlari/v23_mufredat_kaldirma/olcum.md`.

| Eğitim | Başlangıç | Karşılaştırma | Sonuç |
|---|---|---|---|
| v22_mufredat | ppo_omni_250k, taban 25 mm sabit | v12 (sabit set) | çukur seviyesi 500k'dan beri ~35 mm; 45 mm'lik hiçbir engel yok; engel skoru 0.338 → 0.371 (v12 0.316); 2.25M'de durduruldu |
| v23 + v23b | ppo_omni_250k genişletildi, öğrenilmiş kaldırma 25 mm'den | v20 (sabit set, ~28 mm'de takıldı) | kaldırma 25 → 30.6 (2M) → 33.6 mm (4M); 45 mm basamak 3/3; engel skoru 0.499 |

| Model | Kaldırma | Düz güç (ileri 0.1) | Engel skoru | Basamak 45 |
|---|---|---|---|---|
| ppo_omni_250k | 25 mm | 2.00 W | 0.338 | 0/3 |
| v22 2M | 25 mm | 2.65 W | 0.371 | 0/3 |
| **ppo_kaldirma35_250k** (35 mm'den, müfredatsız) | 35.2 mm | **2.39 W** | 0.496 | 3/3 |
| v23b son (4M, müfredatlı) | 33.6 mm | 3.15 W | 0.499 | 3/3 |

- **Müfredat öğrenilmiş kaldırmayı yerel en iyiden çıkardı** (v20 aynı
  başlangıçla 28 mm'de durmuştu), ama vardığı nokta 35 mm'den müfredatsız
  başlayan modelle aynı engel skorunda ve daha çok güçle. Model alınmadı.
- **Sabit 25 mm tabanda müfredat sınırlı:** eklem düzeltmesi 35–40 mm'lik
  çukurlara kadar taşıyor, ötesine geçemiyor.
- **Kaldırma yine zeminden bağımsız:** basamağa yaklaşınca 31.1 → 32.9 mm
  (ilk kez küçük bir tepki); düzde 32.6 mm.
- **Seviye gürültülü politikayla ölçülüyor:** eğitimde çukur seviyesi 45–50
  mm'ye çıktı; deterministik politika aynı anda 45 mm'de takılıyordu (v23
  1M: gürültülüyle 45 mm çukurdan 0.47 m, deterministikle 0.12 m).
  Kaldırma gürültüsü (std 0.5, salınım boyunca sabit) yüksek salınımlarla
  engeli geçiriyor.
- Altyapı S5 için hazır: türler ve seviyeler `CURRICULA`'ya, S5'in üreteci
  yazılır.

## Dayanıklılık taraması: robota geçişte beklenen hatalar (2026-09-27)

`python -m hexapod_rl.robustness tripod models/<ad>/model.zip ...`
(`task.Perturbation`). Eğitimde rastgeleleştirilmeyen ama gerçek robotta
kesin olacak hatalar sabit olarak eklenip modeller ölçüldü:

- **ofset σ:** her eklemde servo sıfırının kalibrasyon hatası, N(0, σ); eklem
  komut + ofset'e gider, politika bunu görmez.
- **IMU θ:** IMU gövdeye θ kadar eğik takılı (yönü rastgele).
- **gecikme:** komutun servoya ulaşması; eğitimde 0–18 ms.
- **servo ×k:** durma torku simin katalog değerinin (1.08 N·m @6 V,
  ölçülmedi) k katı; akü gerilimi düşünce ya da zayıf servoda. Eğitimde ×0.8–1.1.

Değerler ölçüm değil, taranan büyüklükler. 0.1 m/s ileri, 10 s,
deterministik, rastgeleleştirme kapalı; ofset ve IMU için ortalama (ilk
tabloda 3, ikincide 5 tohum). Hücre: alınan yol (m), D = devrilen koşu sayısı.

| Model | Zemin | yok | ofset σ1° | σ2° | σ4° | IMU 3° | 6° | 10° | gecikme 20 ms | 40 | 60 | 80 | servo ×0.7 | ×0.6 | ×0.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| tripod | düz | 0.98 | 0.97 | 0.95 | 0.87 | 0.98 | 0.98 | 0.98 | 0.98 | 0.97 | 0.97 | 0.97 | 0.98 | 0.98 | 0.98 |
| tripod | basamak 45 | 0.09 | 0.09 | 0.09 | 0.12 | 0.09 | 0.09 | 0.09 | 0.09 | 0.09 | 0.10 | 0.10 | 0.09 | 0.10 | 0.10 |
| tripod | engebe 40 | 0.57 | 0.55 | 0.55 | 0.50 | 0.57 | 0.57 | 0.57 | 0.44 | 0.55 | 0.47 | 0.47 | 0.41 | 0.57 | 0.16 |
| ppo_omni_250k | düz | 1.00 | 1.00 | 0.98 | 0.88 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.99 | 0.99 | 1.00 | 1.01 | 1.01 |
| ppo_omni_250k | basamak 45 | 0.08 | 0.08 | 0.08 | 0.11 | 0.08 | 0.08 | 0.08 | 0.08 | 0.08 | 0.08 | 0.08 | 0.08 | 0.09 | 0.09 |
| ppo_omni_250k | engebe 40 | 0.57 | 0.59 | 0.59 | 0.41 | 0.56 | 0.55 | 0.60 | 0.61 | 0.55 | 0.55 | 0.55 | 0.56 | 0.59 | 0.58 |
| ppo_kaldirma35_250k | düz | 1.02 | 1.01 | 0.99 | 0.95 | 1.02 | 1.02 | 1.02 | 1.02 | 1.02 | 1.01 | 1.01 | 1.02 | 1.03 | 1.02 |
| ppo_kaldirma35_250k | basamak 45 | 0.80 | **0.62** | **0.54** | **0.51** | 0.80 | 0.79 | 0.80 | 0.81 | 0.77 | 0.83 | 0.80 | 0.81 | 0.73 | **0.43** |
| ppo_kaldirma35_250k | engebe 40 | 0.80 | 0.85 | 0.84 | 0.81 | 0.74 | 0.68 | 0.74 | 0.71 | 0.87 | 0.65 | 0.66 | 0.86 | 0.92 | 0.70 |
| ppo_lift50_3750k | düz | 1.14 | 1.12 | 1.10 | 1.06 | 1.13 | 1.13 | 1.14 | 1.13 | 1.12 | 1.12 | 1.11 | 1.14 | 1.14 | 1.15 |
| ppo_lift50_3750k | basamak 45 | 1.02 | 0.99 | 1.00 | 0.98 | 1.02 | 1.02 | 1.02 | 1.00 | 0.99 | 1.00 | 0.99 | 1.08 | 0.93 | **0.49** |
| ppo_lift50_3750k | engebe 40 | 1.05 | 1.01 | 1.03 | 1.00 | 1.03 | 1.03 | 1.04 | 1.02 | 0.97 | 0.98 | 1.02 | 1.07 | 1.11 | 0.92 |

**Kırılma noktaları** (daha büyük bozulmalar, ofset ve IMU 5 tohum):

| Model | Zemin | yok | ofset σ3° | σ6° | σ8° | IMU 15° | 20° | gecikme 120 ms | 160 | 240 | servo ×0.45 | ×0.4 | ×0.3 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| tripod | düz | 0.98 | 0.88 | 0.73 | 0.59 | 0.98 | 0.98 | 0.97 | 0.96 | 0.95 | 0.98 | 0.98 | 0.23 D1 |
| tripod | basamak 45 | 0.09 | 0.10 | 0.16 | 0.13 | 0.09 | 0.09 | 0.10 | 0.10 | 0.09 | 0.10 | 0.10 | 0.07 |
| tripod | engebe 40 | 0.57 | 0.46 | 0.34 | 0.31 | 0.57 | 0.57 | 0.49 | 0.40 | 0.45 | 0.12 | 0.09 | 0.02 |
| ppo_omni_250k | düz | 1.00 | 0.94 | 0.76 | 0.53 | 1.00 | 1.00 | 0.99 | 0.98 | 0.98 | 1.01 | 1.00 | 0.17 D1 |
| ppo_omni_250k | basamak 45 | 0.08 | 0.14 | 0.15 | 0.09 | 0.08 | 0.08 | 0.09 | 0.08 | 0.08 | 0.10 | 0.09 | 0.06 |
| ppo_omni_250k | engebe 40 | 0.57 | 0.50 | 0.32 | 0.24 | 0.56 | 0.61 | 0.55 | 0.60 | 0.52 | 0.58 | 0.22 | 0.01 |
| ppo_kaldirma35_250k | düz | 1.02 | 0.95 | 0.82 | 0.78 | 1.02 | 1.02 | 1.01 | 1.00 | 1.00 | 1.02 | 1.02 | 0.05 D1 |
| ppo_kaldirma35_250k | basamak 45 | 0.80 | **0.35** | 0.35 | 0.25 | 0.80 | 0.81 | 0.79 | 0.78 | 0.74 | 0.29 | 0.40 | 0.05 D1 |
| ppo_kaldirma35_250k | engebe 40 | 0.80 | 0.80 | 0.66 | 0.47 | 0.69 | 0.68 | 0.81 | 0.70 | 0.66 | 0.32 | 0.24 | 0.06 |
| ppo_lift50_3750k | düz | 1.14 | 1.06 | 1.00 | 0.96 | 1.13 | 1.13 | 1.11 | 1.11 | 1.09 | 1.15 | **0.15 D1** | 0.02 D1 |
| ppo_lift50_3750k | basamak 45 | 1.02 | 0.97 | 0.92 | 0.72 | 1.01 | 1.01 | 1.00 | 0.99 | 0.97 | **0.41 D1** | 0.06 D1 | 0.02 D1 |
| ppo_lift50_3750k | engebe 40 | 1.05 | 0.99 | 0.92 | 0.85 | 1.02 | 1.01 | 1.01 | 1.01 | 0.99 | 0.30 D1 | 0.05 | 0.06 |

- **Hassas olunan iki hata var: kalibrasyon ve servo gücü.**
  - Düzde ofset σ2°'ye kadar herkes en çok %4 kaybediyor; σ4°'de %7–12,
    σ6°'de %12–26, σ8°'de %16–47. 25 mm'li politika tripod kadar
    kaybediyor, yani bu politikanın zaafı değil, ayakların yanlış yere
    basması (politika ofseti göremez). Ayağı yüksek kaldıran modeller daha
    az kaybediyor (`ppo_lift50_3750k` σ8°'de %16).
  - `ppo_kaldirma35_250k`'nın basamak becerisinin payı dar: σ1°'de bile
    0.80 → 0.62 m, σ3°'te 0.35 m. 35 mm kaldırma engeli geçme eşiğine
    (28–35 mm) yakın; birkaç derecelik hata ayağı eşiğin altına indiriyor.
    `ppo_lift50_3750k` σ4°'te basamakta hâlâ 0.98 m.
  - Servo gücü: ×0.5'te iki zemin modeli de basamakta yarıya iniyor.
    `ppo_lift50_3750k` ×0.45'te engelde, ×0.4'te düzde de devriliyor;
    25 mm'li modeller ×0.4'te düzde hâlâ yürüyor. ×0.3'te herkes
    devriliyor. **Yüksek kaldırmanın tork payı en dar.**
- **IMU eğikliği (20°'ye kadar) ve gecikme (240 ms'ye kadar) neredeyse
  etkisiz.** Açık döngü taban tripod yürüyüşü taşıyor, politikanın
  düzeltmeleri küçük; yön ve IMU'ya bağımlılık az. (Kör politikanın
  IMU'dan zemin çıkaramadığı ders 35 ile tutarlı.)
- **Donanım vardiyası için:** kalibrasyonda eklem başına ~2°'lik doğruluk
  hedeflenmeli (D6); servo beslemesinin yük altındaki gerilimi ve gerçek
  tork ölçülmeli, önce 25 mm'lik modelle başlanmalı (D9). Gerçek robotta
  ofset ve tork ölçülürse o aralıklar eğitimin rastgeleleştirmesine
  eklenir (D10).

**Gerçek robotta DENENMEDİ.**

## Öğrenilmiş ayak kaldırma — `ppo_kaldirma35_250k` (2026-09-27 gece)

Politika taban tripod'un ayak kaldırmasını kendisi seçiyor: 19. çıkış,
20–60 mm (`TaskConfig.lift_action`, `PolicyContract.lift_range`). Her
salınımın ilk adımında seçilir, salınım boyunca sabit. Soru: kör politika
düzde az, engelde çok kaldırmayı öğrenebilir mi (25 mm verim ↔ 50 mm
engel ödünleşimi, PROJE_DEVIR ders 33)?

**Cevap: hayır.** Kaldırma bütün zeminlerde aynı kalıyor; politika onu
eğitim zemin karışımı için tek bir değere ayarlıyor. Engeli ancak
gördüğünde (ileri bakan mesafe sensörü, S7) zemine göre seçebilir;
altyapı buna hazır.

**Denemeler** (hepsi deneme zeminlerinde, her yöne, rastgeleleştirme açık,
lr 1e-4; kayıtlar `egitim_kayitlari/`):

| Eğitim | Başlangıç | Kaldırma seçimi | Sonuç |
|---|---|---|---|
| v18_kaldirma | ppo_lift50_3750k, 50 mm, std 0.05 | her adım | 1M'de 50–51.5 mm; KL sınırına her güncellemede takıldı, durduruldu |
| v19_kaldirma25 | ppo_omni_250k, 25 mm, kaldırma std 0.4 | her adım | 750k'da 25.7 → 26.1 mm, basamakta da artmıyor, durduruldu |
| v20_kaldirma_salinim | ppo_omni_250k, 25 mm, kaldırma std 0.5 | salınım başı | 1M'de 27.8 mm'de yavaşladı (yerel en iyi), durduruldu |
| v21_kaldirma35 | ppo_omni_250k, **35 mm**, kaldırma std 0.5 | salınım başı | 4M: 35 → 38 (1M) → 40 (2M) → 53 mm (4M), her zeminde aynı |

**Kaldırma elle sabitlenince** (v20'nin 250k ara kaydı, aynı politika,
0.1 m/s, 10 s, deterministik): engeller 28–35 mm arasında bir eşikte
geçilmeye başlıyor. 25 mm'den başlayan eğitim eşiğin altında kalıyor:
orada engelde ödül farkı yok, düzde güç cezası aşağı itiyor.

| Kaldırma | Düz yol / güç | Basamak 45 | Çukur 45 | Engebe 40 |
|---|---|---|---|---|
| 25 mm | 1.00 m / 2.00 W | 0.08 m | 0.13 m | 0.57 m |
| 35 mm | 1.01 m / 2.36 W | 0.78 m | 0.77 m | 0.78 m |
| 45 mm | 1.04 m / 2.80 W | 0.89 m | 0.84 m | 0.93 m |
| 55 mm | 1.03 m / 3.81 W | 0.94 m | 0.93 m | 0.95 m |

**v21 ara kayıtları, düz zemin** (her yön seti, 10 s, deterministik):

| Model | Her yön ödül | Her yön güç | İleri 0.10 hız | İleri güç | Kaldırma |
|---|---|---|---|---|---|
| ppo_omni_250k | **2.868** | **1.98 W** | 0.100 | 2.00 W | 25 mm |
| **ppo_kaldirma35_250k** (v21 250k) | 2.855 | 2.43 W | 0.102 | 2.39 W | 35.2 mm |
| v21 1M | 2.830 | 2.69 W | 0.103 | 2.65 W | 37.8 mm |
| v21 2M | 2.816 | 2.93 W | 0.106 | 3.07 W | 40.3 mm |
| v21 best_model (2.25M) | 2.803 | 3.02 W | 0.110 | 3.19 W | 42.3 mm |
| v21 3M | 2.758 | 3.55 W | 0.111 | 3.79 W | 46.5 mm |
| v21 4M (son) | 2.716 | 4.31 W | 0.108 | 4.65 W | 52.7 mm |
| ppo_lift50_3750k | 2.693 | 4.02 W | 0.114 | 3.99 W | 50 mm |

**Bütün zemin türleri** (aşağıdaki "Bütün zemin türleri" tablosuyla aynı
ölçüm: rastgeleleştirme açık, 3 tohum, 10 s):

| Model | Basamak 45 / 60 | Çukur 60 geri | Çukur 45 yana | Yokuş 20° | Kaygan 10° μ0.25 | Kaygan 15° μ0.3 | Kaygan 20° μ0.4 | Kaygan yan 15° | Engebe 40 / 60 | Engebe 40 yana | Engel skoru |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ppo_omni_250k | 0.08 / 0.08 | 0.12 (0/3) | 0.11 (0/3) | 0.85 | 0.47 (2/3) | −0.45 | −0.52 | 1.01 | 0.58 / 0.07 | 0.36 (0/3) | 0.338 |
| **ppo_kaldirma35_250k** | 0.80 (3/3) / 0.08 | 0.12 (0/3) | 0.14 (0/3) | 0.87 | 0.57 (3/3) | −0.26 | −0.32 | 1.04 | 0.83 / 0.65 (3/3) | 0.48 (3/3) | 0.496 |
| v21 1M | 0.83 / 0.08 | 0.12 (0/3) | 0.14 (0/3) | 0.86 | 0.61 (3/3) | −0.05 | −0.05 | 1.06 | 0.91 / 0.71 | 0.56 | 0.523 |
| v21 2M | 0.81 / 0.25 (1/3) | 0.12 (0/3) | 0.30 (1/3) | 0.93 | 0.60 (3/3) | −0.05 | 0.04 | 1.09 | 0.79 / 0.86 | 0.54 | 0.567 |
| v21 best_model | 0.79 / 0.46 (2/3) | 0.12 (0/3) | 0.29 (1/3) | 0.95 | 0.62 (3/3) | −0.06 | −0.00 | 1.14 | 1.02 / 0.90 | 0.53 | 0.599 |
| v21 3M | 0.83 / 0.41 (2/3) | 0.58 (3/3) | 0.23 (1/3) | 0.96 | 0.57 (3/3) | −0.04 | 0.08 | 1.13 | 0.96 / 0.98 | 0.58 | 0.643 |
| v21 4M (son) | 0.97 / 0.31 (1/3) | 0.49 (3/3) | 0.39 (2/3) | 0.94 | 0.46 (2/3) | −0.26 | −0.17 | 1.08 | 1.01 / 1.02 | 0.62 | 0.652 |
| **ppo_lift50_3750k** | **1.04 / 0.70 (3/3)** | **0.98 (3/3)** | **0.63 (3/3)** | 0.94 | 0.57 (3/3) | −0.20 | −0.14 | 1.14 | 0.99 / 0.97 | 0.62 | **0.763** |

- **v21'in son modeli ppo_lift50_3750k'dan iki yönden de kötü** (engel
  skoru 0.652 < 0.763, düz güç 4.65 > 3.99 W). Kaydedilmedi.
- **`ppo_kaldirma35_250k` iki model arasında bir orta nokta:** düzde
  ppo_omni_250k'dan %20 fazla güç (ppo_lift50_3750k %100 fazla), 45 mm
  basamak, 60 mm engebe ve 10° kaygan yokuş 3/3; 45 mm çukurdan geri
  çıkıyor (RL simde 10 s'de 0.68–0.79 m, rastgeleleştirme açık, 3 tohum);
  60 mm basamak, 60 mm çukurdan geri ve 45 mm çukurdan yana çıkış yok.
  Kaldırması sabit sayılır (35.1–35.2 mm, zeminden bağımsız).
- **ROS'lu simde gerçek düğümle** (`tools/wsl/politika_ros_olcum.sh`,
  düğüm kaldırmayı salınım başında seçiyor): düzde her komutta %100–107,
  sıfır komutta hareket yok. Deneme dünyalarında (12 s): 45 mm basamak
  0.93 m (üstte, z 143 mm), 45 mm çukurdan geri 0.85 m (dışarıda, z 144
  mm), 60 mm basamakta 0.08 m'de takılı. 45 mm çukurdan yana (komut tek
  başına) 0.20 m, çukurda kalıyor (z 99 mm); RL simle aynı.
  `ppo_lift50_3750k` aynı ölçümde yana çıkıyor (0.68 m, z 143 mm).
  **Düzeltme (2026-09-27):** ilk ölçümde "yana 0.75 m, dışarıda" yazılmıştı
  ve bu RL simle çelişiyordu. Sebep ölçüm aracı: komutlar aynı dünyada
  robot sıfırlanmadan art arda koşuyor; yana komutu, robot geri komutuyla
  çukurdan çıktıktan sonra başlamıştı (başlangıç yüksekliği zaten 144 mm).
  Araç artık başlangıç ve bitiş z'sini yazıyor; zeminde her komut ayrı
  koşulmalı (PROJE_DEVIR ders 38).
- `gorev.json`'daki `lift_mm: 25` bu modelde kullanılmıyor (kaldırma
  `lift_action` aralığından geliyor); widen'ın kaynak modelinden kalma.

**Gerçek robotta DENENMEDİ.**

## Zeminli eğitim — `ppo_lift50_3750k` ve `ppo_lift50_2250k` (2026-09-26 öğleden sonra)

İlk zeminde eğitilmiş politika. Her yöne, artık eylem; **taban tripod ayağı
50 mm kaldırıyor** (öncekiler 25 mm). Eğitim 16 ortamda, deneme zeminleriyle
(`terrain_probe.TRAIN_SETS["deneme"]`: düz, 20–60 mm çukurlar, yaylalar,
10–25° eğimler; S5'in yerine geçmez). Görev ayarı `gorev.json`'da; `evaluate`,
`export`, `terrain_probe` oradan okur.

**Önce tek başına taban** (düzeltmesiz PhaseTripod, 0.1 m/s, 10 s): ayak
kaldırma 25 mm'de 45 mm basamak ve 50 mm yayladan iniş takılıyor; 40 mm'de
ikisi de geçiliyor; 60 mm'de 60 mm basamak da. Düz zeminde ödül değişmiyor
(3.20–3.24), güç 1.9 → 3.2–3.3 W.

**Zemin ölçümü** (rastgeleleştirme açık, 3 tohum, deterministik, 10 s; alınan
yol ve engeli geçen tohum sayısı; "geçti" = 0.4 m'den fazla ilerledi).
"Zemin skoru": her durumda en fazla beklenen yol (1 m, yanda 0.6 m) sayılır,
yani hedef hızı aşmak puan getirmez.

| Model | 45 mm basamak | 60 mm basamak | çukur 45 geri | çukur 45 yana | çukur 60 geri | yayla 50 iniş | yokuş yukarı 20° | düz | Zemin skoru |
|---|---|---|---|---|---|---|---|---|---|
| tripod (Samet) | 0.09 (0/3) | 0.09 (0/3) | 0.14 (0/3) | 0.12 (0/3) | 0.13 (0/3) | 0.54 (3/3) | 0.76 | 0.98 | 0.404 |
| tripod (Samet), adım 50 mm | 0.62 (2/3) | 0.09 (0/3) | 0.41 (2/3) | 0.12 (0/3) | 0.14 (0/3) | 0.86 (3/3) | 0.70 | 1.00 | 0.548 |
| ppo_omni_250k (25 mm) | 0.08 (0/3) | 0.08 (0/3) | 0.12 (0/3) | 0.11 (0/3) | 0.12 (0/3) | 0.46 (3/3) | 0.85 | 1.02 | 0.407 |
| taban 50 mm (düzeltme 0) | 0.90 (3/3) | 0.27 (1/3) | 0.91 (3/3) | 0.58 (3/3) | 0.45 (2/3) | 0.86 (3/3) | 0.82 | 1.00 | 0.791 |
| ppo_lift50_2250k | 1.00 (3/3) | 0.30 (1/3) | 1.03 (3/3) | 0.59 (3/3) | 0.79 (3/3) | 0.93 (3/3) | 0.91 | 1.10 | 0.879 |
| **ppo_lift50_3750k** | **1.04 (3/3)** | **0.70 (3/3)** | **1.07 (3/3)** | **0.63 (3/3)** | **0.98 (3/3)** | **0.99 (3/3)** | **0.94** | 1.14 | **0.957** |

**Düz zemin** (her yön seti, 10 s, deterministik):

| Model | Her yön ortalama ödül | İleri 0.05/0.10/0.15 hız | Güç (her yön ort.) | Rastgeleleştirmede | Eklem gürültüsü | Gövde ×1.6 |
|---|---|---|---|---|---|---|
| tripod | 2.848 | 0.049 / 0.098 / 0.146 | 1.85 W | 3.08–3.21 | 1.46 | 3.22 |
| taban 50 mm | 2.829 | 0.049 / 0.100 / 0.150 | 3.13 W | 3.09–3.16 | 1.82 | 3.20 |
| ppo_omni_250k | **2.868** | 0.050 / 0.100 / 0.150 | 1.98 W | 3.08–3.24 | 1.84 | 3.25 |
| ppo_lift50_2250k | 2.764 | 0.053 / 0.109 / 0.165 | 3.82 W | 3.05–3.08 | **1.88** | 3.13 |
| ppo_lift50_3750k | 2.693 | — / 0.114 / — | 4.02 W | — | 1.87 | 3.07 |

- **Zeminde RL tabanını geçiyor:** aynı 50 mm tabana göre engellerde +%15
  (0.726 → 0.834), zemin skoru 0.791 → 0.879. 60 mm çukurdan geri çıkışı
  3/3 yapıyor (taban 2/3). 60 mm basamağı ileri çıkmayı güvenilir
  öğrenmedi (ara kayıtlar 0/3–2/3 arasında dalgalanıyor).
- **Samet'in tripod'u ve 25 mm'li politikalar** 45 mm'lik hiçbir engeli
  geçemiyor. **Aynı adım yüksekliğinde de RL önde:** `TripodGait` 50 mm
  adımla (`GaitParams.step_height_mm`; `terrain_probe tripod:50`) 45 mm
  basamağı 2/3, çukurdan yana ve 60 mm'yi 0/3 geçiyor; zemin skoru 0.548
  (RL 0.879). Düzde 50 mm adımlı tripod 0.10 m/s'de ödül 3.25, 2.57 W.
- **Bedeli düz zemin:** hedef hızı %10 aşıyor (ders 25), güç 3.8 W
  (25 mm'li politika 2.0 W). Düz zeminde `ppo_omni_250k` daha iyi.
- ROS'lu simde gerçek düğümle (`tools/wsl/politika_ros_olcum.sh`): bütün
  komutlarda yürüyor, ileri/geri/yana %106–111 (aşma burada da), dönüş
  %98–99, sıfır komutta ayakta.
- **Ara kayıt seçimi:** eğitim içindeki otomatik seçim düz zemine baktığı
  için 250k'yı seçti (düzde en az aşan). Zemin skoruna göre en iyisi
  2.25M; elle seçildi (`egitim_kayitlari/v13_lift50/zemin_olcumu.md`).
  Zeminli eğitimde seçim ölçütüne zemin de girmeli.
- Önceki deneme `v12_zemin` (taban 25 mm, aynı zeminler, 2.25M adım):
  hiçbir engelde iyileşme yok; keşif gürültüsü ayağı yükseğe kaldırmayı
  bulamadı (`egitim_kayitlari/v12_zemin/zemin_olcumu.md`).

**Eğitim:** taklit `bc_lift50` (zeminlerde, 128 bölüm, std 0.1) → PPO
`v13_lift50` (lr 1e-4, target_kl 0.02, std 0.1, 3M, 34 dk, 1461 adım/s)
→ 2.25M ara kaydı `ppo_lift50_2250k`. Ondan devam `v14_lift50_std05` (std
0.05, 2M, 22 dk; ara kayıt seçimi artık zemin durumlarını da ölçüyor):
1.5M ara kaydı `ppo_lift50_3750k` (bc'den toplam 3.75M adım).

**v14 (std 0.05) sonucu:** amaç düzdeki hız aşmasını ve enerjiyi azaltmaktı,
**olmadı** (1.5M'de aşma %14, 4.0 W). Ama zeminde belirgin ilerleme: 60 mm
basamak 1/3 → **3/3**, zemin skoru 0.879 → **0.957**. Eğitim içi seçim
(deterministik, tek ölçüm) 500k'yı seçti; rastgeleleştirmeli 3 tohumlu
ölçüm 1.5M'yi açıkça öne koydu (geç/geçeme durumları tek ölçümde
gürültülü, `egitim_kayitlari/v14_lift50_std05/zemin_olcumu.md`). ROS'lu
simde gerçek düğümle bütün komutlarda yürüyor (ileri/geri/yana %106–115,
dönüş %89–97), sıfır komutta ayakta.

**Aşmayı ödülle düzeltme denemesi (ödül v7, `--overshoot`, 2026-09-26):**
`progress` komutu aşan hızı da kaybettirsin diye `ppo_lift50_3750k`'dan 2M
adım devam (std 0.05), iki katsayıyla. Önce ders 20: v7, tabanları ve
politikaları doğru sıralıyor (düzde aşan politika cezalanıyor, zeminde
tırmanan yine önde).

| | 0.10'da hız | Güç (her yön) | Her yön ödülü (v6) | 60 mm basamak | çukur 45 yana | çukur 60 geri |
|---|---|---|---|---|---|---|
| ppo_lift50_3750k (başlangıç) | 0.114 | 4.02 W | 2.693 | 3/3 | 3/3 | 3/3 |
| v15, katsayı 1, 2M | 0.110 | 3.90 W | 2.676 | 3/3 | 3/3 | 3/3 |
| v16, katsayı 3, 2M | 0.107 | 4.01 W | 2.701 | 3/3 | **0/3** | 2/3 |

Aşma biraz azaldı ama **enerji azalmadı**; katsayı büyüyünce zemin becerisi
kayboldu. Fazla enerji hızdan değil, engel geçiren düzeltmelerden geliyor;
kör politika engel önünde olup olmadığını bilmediği için bu yürüyüşü her
yerde kullanıyor. Depoya model alınmadı; zeminde en iyisi `ppo_lift50_3750k`.
Kayıtlar `egitim_kayitlari/v15_odul_v7`, `v16_odul_v7_k3`.

**Gerçek robotta DENENMEDİ.** Gerçek robotta 50 mm ayak kaldırmanın
erişilebilirliği ve servo yükü eklem limitleri (D6) ile kontrol edilmeli.

### Bütün zemin türleri: eğim, engebe, kaygan, basamak (2026-09-26 akşam)

G7'nin "bitti" şartındaki üç tür (eğim, engebe, kaygan) + basamaklar, kendi
deneme zeminlerimde (S5'in yerine geçmez). Rastgeleleştirme açık, 3 tohum,
deterministik, 10 s; hücre: ortalama yol (m) ve 0.4 m'den fazla ilerleyen
tohum; D = devrilme. Engel skoru: düz hariç, her durumda en fazla beklenen
yol (1 m, yanda 0.6 m). Kaygan yokuşta çekiş için en az μ = tan θ gerekir
(10° 0.18, 15° 0.27, 20° 0.36): μ 0.3 / 0.4 fiziksel sınıra %10 yakın.

| Model | Basamak 45 / 60 | Çukur 60 geri | Çukur 45 yana | Yokuş 20° | Kaygan yokuş 10° μ0.25 | Kaygan yokuş 15° μ0.3 | Kaygan yokuş 20° μ0.4 | Engebe 40 / 60 | Engebe 40 yana | Engel skoru |
|---|---|---|---|---|---|---|---|---|---|---|
| tripod (Samet) | 0.09 / 0.09 (0/3) | 0.13 (0/3) | 0.12 (0/3) | 0.76 | 0.13 (0/3) | **−3.60, D3** | **−3.48, D3** | 0.47 / 0.10 (0/3) | 0.27 (0/3) | 0.281 |
| tripod, 50 mm adım | 0.62 (2/3) / 0.09 | 0.14 (0/3) | 0.12 (0/3) | 0.70 | 0.16 (0/3) | −3.57, D3 | −3.47, D3 | 0.77 / 0.81 | 0.51 | 0.443 |
| taban 50 mm (düzeltme 0) | 0.90 / 0.27 (1/3) | 0.45 (2/3) | 0.58 | 0.82 | 0.48 (2/3) | −1.02 | −1.26 | 0.91 / 0.87 | 0.52 | 0.628 |
| **ppo_lift50_3750k** | **1.04 / 0.70 (3/3)** | **0.98 (3/3)** | **0.63** | **0.94** | **0.57 (3/3)** | −0.20 | −0.14 | **0.99 / 0.97** | **0.62** | **0.763** |

- **Politika her zemin türünde tripod'dan iyi.** Engebe ve kaygan eğim
  eğitimde yoktu; politika genelleşiyor. Samet'in tripod'u kaygan yokuşta
  geriye kayıp 3/3 devriliyor, politika devrilmeden tutunuyor (−0.1…−0.2 m).
- Payı dar kaygan yokuşları (15° μ 0.3, 20° μ 0.4) kimse çıkamıyor.
- **ROS'lu simde, robotta koşacak düğümle** (deneme zemini dünyası,
  `terrain_probe.world_sdf` + `tools/wsl/politika_ros_olcum.sh`, 12 s):
  `ppo_lift50_3750k` 45 mm basamakta 1.14 m (üstte, z 144 mm), 60 mm
  basamakta 0.96 m (z 159 mm), 45 mm çukurdan geri 1.17 m; `ppo_omni_250k`
  üçünde de 0.08–0.13 m'de takılıyor. Zemin becerisi 1 ms fizikli, ros2_control
  servo zincirli simde de aynı.
- **Deneme (v17):** eğitim setine kaygan eğim ve engebe eklendi
  (`TRAIN_SETS["deneme2"]`), `ppo_lift50_3750k`'dan 3M adım (49 dk, 1014
  adım/s; engebe simi ~2 kat yavaş). İyileştirmedi: engel skoru ara
  kayıtlarda 0.675–0.719 (başlangıç 0.763). Çukur ortamı 8'den 3'e indiği
  için çukur becerileri zayıfladı; dar paylı kaygan yokuşlarda ilerleme yok.
  Model alınmadı (`egitim_kayitlari/v17_deneme2`).

## Her yöne yürüyüş — `ppo_omni_250k` (2026-09-26 öğlen, yeni PC)

Politika artık ileri/geri (vx ±0.15), yana (vy ±0.08) ve dönüş (wz ±0.5)
komutlarının hepsini görüyor (`--omni`). Artık eylem modu, ödül v6,
rastgeleleştirme açık; 16 ortam. En iyi ara kayıt otomatik seçildi
(`best_model.zip`: her ara kayıtta yedi komutluk deterministik ölçüm).
Robotta: `policy.npz` sözleşmesinde her yön aralıkları ve ölü bölge (1/6)
var; düğüm sıfıra yakın komutta ayakta bekler, geri/yana/dönüşte yürür
(`tests/test_policy_sim.py`, robottaki denetleyiciyle Gazebo'da).

Ölçüm koşulları: düz zemin, 10 s, deterministik, hiçbiri devrilmedi. Her yön
seti: (0.1,0,0), (−0.1,0,0), (0,±0.06,0), (0,0,±0.4), (0.1,0.04,0.25).

| Model | Her yön seti ortalama ödül | İleri 0.05/0.10/0.15 ödül | Yön (ileri) | Güç 0.10 | Rastgeleleştirmede (0.10, 3 tohum) | Eklem gürültüsü 0.05 rad | Gövde ×1.6 |
|---|---|---|---|---|---|---|---|
| tripod (Samet) | 2.848 | 2.83 / 3.22 / 3.57 | ~0° | 1.9 W | 3.08–3.21 | 1.46 (0.067 m/s) | 3.22 |
| PhaseTripod (düzeltme 0) | 2.846 | 2.80 / 3.20 / 3.58 | ≤2° | 1.9 W | 3.04–3.23 | 1.83 (0.106 m/s) | 3.22 |
| ppo_res_250k (yalnız ileri) | 2.800 | 2.82 / 3.24 / 3.61 | +5..+12° | 2.3 W | 3.06–3.18 | 1.78 | 3.24 |
| **ppo_omni_250k** (v10) | **2.868** | 2.82 / **3.25 / 3.64** | −2..−4° | 2.0 W | 3.07–3.24 | **1.84** (0.114 m/s) | **3.25** |
| v11_omni 750k (std 0.05) | 2.850 | 2.83 / 3.23 / 3.62 | ≤1° | 2.0 W | 3.07–3.24 | 1.84 | 3.25 |

- Her yön politikası bütün komutları tripod kadar izliyor (gövde hızı
  komutun %95–100'ü); ppo_res_250k ileri eğitildiği için geri/yana/dönüşte
  geride (2.800).
- **Düz zeminde tripod'u anlamlı geçmiyor**: ortalama +%0.7. Rastgeleleştirme
  açıkken (3 tohum × 7 komut, deterministik) v11'in bütün ara kayıtları
  2.566–2.581, PhaseTripod 2.580, Samet'in tripod'u 2.609. Düz zeminde
  iyi bir tripod'un üstüne öğrenilecek çok şey yok; beklenen.
- Eklem gürültüsünde tripod'dan açık ara iyi (1.84'e 1.46): açık döngü tripod
  titreşimde hızının üçte birini kaybediyor.
- **ROS'lu simde gerçek politika düğümüyle** (`tools/wsl/politika_ros_olcum.sh`):
  ileri %101, geri %100, yana %101–104, dönüş %99–100, karışık
  (0.1, 0.04, 0.25) %100/%104/%96; sıfır komutta ayakta, hareket 0.0 mm.

**Eğitim:** taklit `bc_omni` (128 bölüm, 16 işçi, gürültü 0.25, std 0.15) →
PPO `v10_omni` (lr 1e-4, target_kl 0.02, std 0.15), 2.1M'de durduruldu; en
iyi ara kayıt 250k. Deterministik skor 250k'dan sonra düştü (1M'de 2.779,
tripod'un altı) ama eğitim ödülü %6 arttı: politika std 0.15'lik keşif
gürültüsüne uyuyor (ders 25). İkinci deneme `v11_omni` (std 0.05, kütle
rastgeleleştirmeli taklit `bc_omni_m`, 3M, 30 dk): düşüş yok ama tripod
düzeyinden de çıkmadı (2.829–2.850). Kayıtlar `egitim_kayitlari/`.

### Deneme zeminleri (`python -m hexapod_rl.terrain_probe`)

S5'in yerine geçmez; beklenti için. Komut 0.10 m/s ileri, 10 s; hız / ödül.
Hiçbiri devrilmedi.

| Zemin | tripod | PhaseTripod | ppo_res_250k | ppo_omni_250k | v11 750k |
|---|---|---|---|---|---|
| düz | 0.098 / 3.22 | 0.096 / 3.20 | 0.105 / 3.24 | 0.100 / 3.25 | 0.099 / 3.23 |
| yokuş yukarı 10° | 0.092 / 3.09 | 0.090 / 3.06 | 0.099 / 3.15 | 0.094 / 3.13 | 0.093 / 3.10 |
| yokuş aşağı 10° | 0.100 / 3.18 | 0.100 / 3.19 | 0.108 / 3.16 | 0.102 / 3.20 | 0.102 / 3.20 |
| yan eğim 10° | 0.098 / 3.15 | 0.097 / 3.14 | 0.105 / 3.17 | 0.100 / 3.17 | 0.099 / 3.16 |
| yokuş yukarı 20° | 0.077 / 2.70 | 0.079 / 2.75 | **0.089 / 2.90** | 0.084 / 2.83 | 0.082 / 2.81 |
| yokuş aşağı 20° | 0.099 / 3.02 | 0.100 / 3.03 | 0.109 / 2.94 | 0.102 / 3.02 | 0.103 / 3.02 |
| basamak 15 mm | 0.091 / 3.05 | 0.088 / 2.99 | 0.099 / 3.15 | 0.093 / 3.05 | 0.091 / 3.02 |
| basamak 30 mm | 0.071 / 2.46 | 0.076 / 2.62 | **0.086 / 2.79** | 0.081 / 2.71 | 0.077 / 2.63 |
| basamak 45 mm | **0.009 / 1.45** | 0.008 / 1.40 | 0.008 / 1.38 | 0.008 / 1.39 | 0.008 / 1.40 |

- Politikalar zorlaştıkça tripod'u geçiyor: 20° yokuşta ve 30 mm basamakta
  %6–21 daha hızlı. Hiçbiri zemin görmedi; fark düzeltmelerin genel
  sağlamlığından (ppo_res_250k'nın önü kısmen hedef hızı aşmasından).
- **45 mm basamağı hiçbiri çıkamıyor**: tripod ayağı 25 mm kaldırıyor, ön
  ayaklar basamağın yüzüne takılıyor. Zeminle eğitimin (S5) çözmesi
  gereken ilk somut örnek.

**Gerçek robotta DENENMEDİ.**

## Artık eylem (tripod + düzeltme) — en iyi sonuç (2026-09-26 sabah)

`ppo_res_250k`: politika eklem açısını değil, adım saatinin tripod'una
(`hexapod_policy.tripod.PhaseTripod`) eklenen düzeltmeyi üretir (`--residual`).
Ödül v6, düz zemin, 10 s, deterministik. Eylem gürültüsü **eklemde 0.05 rad**
(mutlak modda `--noise 0.1`, artık eylemde `--noise 0.25`; ölçekler 0.5 ve
0.2 rad). Hiçbiri devrilmedi.

| Model | Gürültüsüz ödül (0.05 / 0.10 / 0.15) | Gerçek hız | Yön (10 s) | Güç | Rastgeleleştirmede ödül (0.10, 3 tohum) | Eklem gürültüsünde ödül |
|---|---|---|---|---|---|---|
| tripod | 2.83 / 3.22 / 3.57 | 0.049 / 0.098 / 0.146 | ~0° | 1.5 / 1.9 / 2.6 W | 3.08–3.22 | 1.46 |
| artık, düzeltme 0 (bc_res) | 2.80 / 3.20 / 3.58 | 0.047 / 0.096 / 0.145 | <2.1° | 1.6 / 1.9 / 2.5 W | 3.04–3.23 | 1.89 |
| **ppo_res_250k** | 2.82 / **3.24 / 3.62** | 0.058 / 0.105 / 0.153 | 0 / +5 / +12° | 1.8 / 2.3 / 2.9 W | 3.06–3.18 | 1.78 |
| v9_res 1M | 2.76 / 3.16 / 3.54 | 0.064 / 0.115 / 0.167 | +1 / +4 / +8° | 2.3 / 3.0 / 3.8 W | 2.98–3.07 | 1.84 |
| v9_res 3M | 2.73 / 3.15 / 3.54 | 0.066 / 0.116 / 0.167 | 0 / +1 / +6° | 2.4 / 3.0 / 3.9 W | 2.97–3.09 | 1.82 |
| ppo_v7_8M (mutlak) | 2.55 / 2.94 / 3.31 | 0.053 / 0.098 / 0.146 | +6 / −3 / −11° | 5.6 / 6.4 / 8.0 W | 2.82–3.00 | 2.23 |

- **Enerji sorunu çözüldü:** artık eylem politikası tripod'dan ~%20 fazla
  harcıyor, mutlak moddaki PPO 3 kat harcıyordu.
- ppo_res_250k gürültüsüz ödülde tripod'u 0.10 ve 0.15'te geçiyor,
  rastgeleleştirmede neredeyse eşit, eklem gürültüsünde önde.
- Eğitim uzadıkça (1M, 3M) yine hedef hızı aşmaya kayıyor (gürültüye uyum,
  ders 25); en iyi ara kayıt 250k. Keşif std'si (0.15 → 0.11, artık eylem
  biriminde) düşürülerek ya da eğitim erken kesilerek denenebilir.
- Hızlıda sola hafif yön kayması (+12°/10 s, 0.15'te) kalan kusur.

## Gece karşılaştırması (2026-09-26 sabah, ödül v6)

Ödül v6 (`hexapod_rl.task`), düz zemin, 10 s, deterministik. Rastgeleleştirme:
servo gücü/sertliği, 0–18 ms gecikme, itme, IMU gürültüsü (`--randomize`).
Hiçbiri devrilmedi.

| Model | Gürültüsüz ödül (0.05 / 0.10 / 0.15) | Gerçek hız | Yön (10 s) | Güç | Rastgeleleştirmede ödül (vx 0.10, 3 tohum) | Eylem gürültüsünde ödül (0.10) |
|---|---|---|---|---|---|---|
| tripod (Samet) | **2.83 / 3.22 / 3.57** | 0.049 / 0.098 / 0.146 | ~0° | 1.5 / 1.9 / 2.6 W | 3.08–3.22 | 1.46 (0.067 m/s) |
| taklit (bc_v5) | 2.83 / 3.19 / 3.55 | 0.049 / 0.096 / 0.143 | <2.3° | 1.5 / 1.9 / 2.5 W | 3.03–3.22 | 1.73 |
| PPO v5_dr 5M | 2.23 / 2.47 / 2.84 | 0.060 / 0.113 / 0.158 | −24 / −42 / −41° | 7.0 / 7.6 / 9.3 W | 2.40–2.51 | 2.20 |
| **ppo_v7_8M** | 2.55 / 2.94 / 3.31 | **0.053 / 0.098 / 0.146** | +6 / −3 / −11° | 5.6 / 6.4 / 8.0 W | 2.82–3.00 | **2.23** (0.102 m/s) |
| v8 (v7 + güç −0.10/W, +2M; depoda değil) | 2.61 / 3.01 / 3.24 | 0.052 / 0.103 / 0.145 | −2 / −7 / −9° | 5.5 / 6.4 / 7.3 W | 2.86–2.98 | 2.16 |

Özet:
- **ppo_v7_8M en iyi PPO:** hız komutunu doğru izliyor, yön sapması v5_dr'nin
  dörtte biri. Eylem gürültüsü altında açık ara en iyisi (hızını koruyor).
- **Düz zeminde tripod ve taklit hâlâ önde:** PPO 3 kat enerji harcıyor ve
  hızlıda yön kaydırıyor. Mevcut rastgeleleştirme aralıkları tripod'u hiç
  zorlamıyor; RL'nin asıl sınavı zeminli dünyalar (S5).
- **Fark neredeyse tamamen enerji:** 0.10 m/s'de terim terim (v8'e karşı
  tripod) hız izleme 0.967'ye 0.962, ilerleme 0.971'e 0.928 (PPO önde),
  güç −0.316'ya −0.096 (6.3 W'a 1.9 W). Deterministik eylem titremiyor
  (eylem değişimi cezası −0.002); fazla güç yürüyüş biçiminin kendisinden.
  Güç cezasını iki katına çıkarmak (v8) 2M adımda bunu değiştirmedi.
- **gSDE (düzgün keşif) bu kurulumda işe yaramadı:** iki deneme (lr 3e-4 ve
  1e-4 + target_kl) eğitim ödülünü düşürdü ve dönmeye kaydı; durduruldu,
  depoya alınmadı.

## Karşılaştırma (2026-09-26, ödül v4)

Ödül v4, düz zemin, 10 s, `evaluate` (deterministik). "Gürültülü": eyleme
N(0, 0.1) eklendi (0.05 rad; servo titremesi ve PPO'nun eğitimde kendi keşif
gürültüsüyle koştuğu koşul). Tripod: Samet'in `hexapod_gait.TripodGait`'i
(`hexapod_rl.baseline`).

| Model | Gürültü | Komut vx | Gerçek hız | Yön sapması | Adım başı ödül | Güç |
|---|---|---|---|---|---|---|
| tripod | yok | 0.05 / 0.10 / 0.15 | 0.049 / 0.098 / 0.146 | +0.1° / −0.3° / +0.0° | **2.83 / 3.20 / 3.60** | 1.5 / 1.9 / 2.6 W |
| taklit_bc_v4 | yok | 0.05 / 0.10 / 0.15 | 0.050 / 0.095 / 0.145 | −0.1° / −0.0° / +2.7° | 2.83 / 3.15 / 3.55 | 1.5 / 1.9 / 2.5 W |
| ppo_v4_4M | yok | 0.05 / 0.10 / 0.15 | 0.056 / 0.112 / 0.156 | −0.9° / −3.3° / −33° | 2.64 / 3.11 / 3.46 | 6.0 / 6.9 / 8.3 W |
| tripod | 0.1 | 0.05 / 0.10 / 0.15 | 0.044 / 0.067 / 0.098 | −4° / −2° / +15° | 1.88 / 1.99 / 2.11 | ~12 W |
| taklit_bc_v4 | 0.1 | 0.05 / 0.10 / 0.15 | 0.057 / 0.100 / 0.156 | −10° / −18° / −15° | 1.89 / 2.16 / 2.64 | ~12 W |
| ppo_v4_4M | 0.1 | 0.05 / 0.10 / 0.15 | 0.066 / 0.108 / 0.151 | −16° / −9° / −13° | **2.12 / 2.64 / 3.00** | ~13 W |

Hiçbiri devrilmedi. Özet: gürültü altında PPO hızını koruyor ve en yüksek
ödülü alıyor; tripod açık döngü olduğu için titreşimde hızının üçte birini
kaybediyor. Gürültüsüz düz zeminde tripod hâlâ biraz önde, PPO 3–4 kat
enerji harcıyor. Asıl karşılaştırma zeminli dünyalarda yapılacak (S5, S6).

## tork_v2_10M — ilk yürüyen politika (2026-09-25)

- **Eğitim:** 10M adım, 8 paralel ortam, 8.1 saat (341 adım/s). Tork tabanlı
  servo modeli (`hexapod_rl.sim`), ödül v2 (`hexapod_rl.task`, ilerleme +
  tripod ritmi + mekanik güç cezası). Ödül ~5M adımda ~1800'e ulaşıp yatay kaldı.
- **Sonuç:** kararlı yürüyor, hiç devrilmiyor. Ama:
  - hız komutunu yok sayıyor: 0.05, 0.10 ve 0.15 m/s komutlarında aynı hızda
    (~0.087 m/s, 10 s'de ~0.86 m) yürüyor;
  - yön tutmuyor: saniyede ~12° sağa dönüyor (10 s'de −120°).

| Komut vx | Kendi yönünde yol (10 s) | İleri (dünya x) | Yana (dünya y) | Yön sapması | Devrilme |
|---|---|---|---|---|---|
| 0.05 m/s | 0.868 m | 0.448 m | −0.744 m | −119° | yok |
| 0.10 m/s | 0.865 m | 0.472 m | −0.725 m | −120° | yok |
| 0.15 m/s | 0.851 m | 0.431 m | −0.734 m | −125° | yok |

Sebep ödül ayarı: dönüş hızını izleme terimi zayıf (ağırlık 0.2, tolerans
0.5 rad/s) ve hız toleransı (0.1 m/s) farklı hızları ayırt ettirmeyecek
kadar geniş. Sıradaki eğitim (ödül v3) bunları hedefliyor.

**Gerçek robotta DENENMEDİ.** Politika geçici eklem limitleri (±90°), CAD'den
kütle tahmini (2.13 kg) ve tahmini servo sertliğiyle eğitildi; gerçek
değerlerle yeniden eğitim donanım vardiyasında (GOREVLER.md D10).

## taklit_bc_v4 — taklit ile başlatma (2026-09-25)

PPO'dan önce politikanın simetrik gösterim tripod'unu (`hexapod_rl.demo`)
taklit etmesi (`python -m hexapod_rl.pretrain`): 64 bölüm (64 bin adım, 8
süreçte 93 s), eyleme 0.1 gürültü, etiket gürültüsüz. Aktör RMSE 0.003,
kritik açıklanan varyans 0.75, std 0.15. Ayrıntı: `ozet.txt`. Dümdüz
yürüyor ve hız komutunu izliyor; ppo_v4_4M'nin başlangıç noktası.

Neden gerekti: ödül v3 ile tork_v2_10M'den 2.5M adım devam eğitimi dönmeyi
düzeltmedi (10 s'de −119..−124°). v3'ün anlık hız izleme terimleri PPO'nun
keşif gürültüsünde dönen yürüyüşü düz yürüyüşten çok ödüllendiriyordu;
ödül v4 bu terimlerde 0.5 s'lik ortalama hıza bakıyor (`hexapod_rl.task`).

## ppo_v4_4M — ödül v4, taklitten PPO (2026-09-26)

- **Eğitim:** taklit_bc_v4'ten başlayarak 4M adım, 8 paralel ortam, ~2.3 saat
  (~500 adım/s). 5M planlanmıştı; bilgisayar kapatılacağı için 4.17M'de
  durduruldu, bu 4M ara kaydı. Eğitim ödülü (gürültülü) 2009 → 2794, std
  0.149 → 0.088, hiç devrilme yok.
- **Sonuç:** yukarıdaki tablo. Gürültüye dayanıklı; gürültüsüz düz zeminde
  enerji verimsiz ve hızlıda yön tutmuyor.
- Bilinen ödül kusurları (ödül v5 için): progress anlık hızı komutla kırpıyor,
  bu da ortalamada hedef hızı biraz aşmayı ödüllendiriyor; güç cezası
  (−0.02/W) enerjiyi neredeyse bedava bırakıyor (7 W adım başı 0.14).

**Gerçek robotta DENENMEDİ** (tork_v2_10M'deki not geçerli).

## ppo_v7_8M — düşük gürültüyle devam, ödül v6 (2026-09-26)

- **Eğitim:** iki aşama, ikisi de rastgeleleştirme açık.
  1. v5_dr: taklitten (bc_v5), ödül v5, lr 3e-4, std 0.15 → 0.094,
     5M adım (10M planlanmıştı, 5M'de durduruldu).
  2. v7_lowstd: v5_dr 5M'den, ödül v6 (dönüş toleransı 0.1 rad/s),
     std 0.05, lr 1e-4, target_kl 0.02, 3M adım, 77 dk.
  Eğitim ödülü (v6) 2034 → 2522; ayarlar `ayarlar.txt`'de.
- **Neden düşük gürültü:** bağımsız adım gürültüsüyle eğitilen PPO, ortalama
  eylemi "gürültüyle uygulanacak" diye ayarlıyor: gürültüsüz koşunca hedef
  hızı aşıyor ve fazla enerji harcıyordu. Gürültüyü 0.05'e indirip küçük
  adımlarla devam edince hız izleme düzeldi (0.10 komutta 0.113 → 0.098).
- **Sonuç:** yukarıdaki tablo. Kalan sorunlar: enerji (tripod'un ~3 katı),
  hızlıda yön kayması (0.15'te −11°).

**Gerçek robotta DENENMEDİ.**

## ppo_res_250k — artık eylem, en iyi (2026-09-26)

- **Eğitim:** taklit (bc_res: düzeltme 0, kritik tripod + gürültünün
  getirileriyle; 64 bölüm, 63 s) sonra PPO v9_res: ödül v6, artık eylem
  (`--residual`, ölçek 0.2 rad, düzeltme cezası −0.5), rastgeleleştirme,
  std 0.15, lr 1e-4, target_kl 0.02. 3M adım koştu (75 dk, 668 adım/s),
  en iyi ara kayıt 250k. Ayarlar `ayarlar.txt`'de.
- **Robotta çalıştırmak:** `policy.npz` artık eylem sözleşmesini taşır;
  `hexapod_policy` düğümü tripod'u aynı kodla (`PhaseTripod`) kendisi
  hesaplar. Aktarma: `python -m hexapod_rl.export <zip> --residual`.
- **Sonuç:** yukarıdaki tablo.

**Gerçek robotta DENENMEDİ.**

