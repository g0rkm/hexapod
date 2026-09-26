# Eğitim kayıtları

Bütün PPO/taklit eğitimlerinin hafif kayıtları (2026-09-24 – 26). Eğitimi yapan
makinede `~/hexapod_runs/<ad>/` altındaydılar; bilgisayar değişince kaybolmasın
diye buraya kopyalandı.

Her klasörde (varsa):

- `progress.csv`: SB3 günlüğü. Adım, ödül, std, KL, açıklanan varyans...
- `ayarlar.txt`: öğrenme hızı, target_kl, std, rastgeleleştirme, eylem modu.
- `baslangic.txt`: hangi modelden devam edildi.
- `degerlendirme.txt`: eğitim sonundaki kısa ölçüm.
- `ozet.txt`: taklit (pretrain) özeti.

Depoda olmayanlar: ara kayıtlar (`checkpoints/`, toplam ~100 MB) ve ham
stdout günlükleri. Kayda değer modeller [models/](../models/README.md) altında.

**Dikkat:** `progress.csv`'deki ödül, o eğitimin kendi ödül sürümüyle
hesaplandı (v1…v6, artık eylem cezası, güç ağırlığı). Farklı eğitimlerin ödülleri
birbiriyle karşılaştırılamaz. Karşılaştırma için aynı koşullarda ölçülmüş
tablolar: `models/README.md`.

| Kayıt | Ne | Sonuç |
|---|---|---|
| `ilk_1M` | ilk PPO (ödül v1, hız komutlu servo modeli), 1M | robot yerinde durdu |
| `odul_v2_10M` | ödül v2, hız komutlu servo modeli; 5.75M'de durduruldu | ritmi öğrendi ama yerinde saydı; servo modeli geçersizdi |
| `tork_v2_10M` | tork servo modeli + ödül v2, 10M | ilk yürüyen politika → `models/tork_v2_10M` (hızı yok sayıyor, sağa dönüyor) |
| `v3_duman`, `v3_5M` | ödül v3 ile tork_v2_10M'den devam; 2.5M'de durduruldu | dönme düzelmedi (ders 20) |
| `bc_v4` | taklit (gösterim tripod'u), ödül v4 | → `models/taklit_bc_v4` |
| `v4_bc` | PPO ödül v4, bc_v4'ten; 4.17M'de durduruldu | → `models/ppo_v4_4M` (gürültüde iyi, 3–4 kat enerji) |
| `bc_v5` | taklit, rastgeleleştirme açık | v5_dr'nin başlangıcı |
| `v5_dr` | PPO ödül v5 + rastgeleleştirme; 5M'de durduruldu | devrilmiyor, yön kayıyor; v7'nin başlangıcı |
| `bc_v6`, `v6_sde`, `v6b_sde` | gSDE denemeleri | ikisi de bozuldu (ders 24) |
| `v7_lowstd` | v5_dr 5M'den, ödül v6, std 0.05, lr 1e-4, 3M | → `models/ppo_v7_8M` |
| `v8_power` | v7'den, güç cezası −0.10/W, 2M | enerji değişmedi |
| `bc_res`, `v9_res` | artık eylem (tripod + düzeltme), 3M | en iyi ara kayıt 250k → `models/ppo_res_250k` |
| `gece_zinciri_2026-09-26.log` | gece deneylerinin zaman çizelgesi | |
| `hiz_testi_env8/12/16` | yeni PC hız testi: 8/12/16 ortam, 40 güncelleme turu | 1342 / 1588 / 1818 adım/s (PROJE_DEVIR §0.5) |
| `bc_omni`, `v10_omni` | her yöne komut (`--omni`), std 0.15, 16 ortam; 2.1M'de durduruldu | en iyi ara kayıt 250k → `models/ppo_omni_250k`; sonra deterministik skor düştü (ders 25) |
| `bc_omni_m`, `v11_omni` | her yön, std 0.05, kütle rastgeleleştirmeli, 3M | düşüş yok, tripod düzeyinde (en iyi 750k, 2.850) |
| `v12_zemin` | `ppo_omni_250k`'dan devam, deneme zeminleri (`--terrains deneme`), taban 25 mm, std 0.1; 2.25M'de durduruldu | hiçbir engelde iyileşme yok (`zemin_olcumu.md`) |
| `bc_lift50`, `v13_lift50` | taban ayak kaldırma 50 mm, taklit ve PPO deneme zeminlerinde, 3M | 2.25M ara kaydı → `models/ppo_lift50_2250k`; bütün ara kayıtların zemin ölçümü `zemin_olcumu.md` (rastgeleleştirme açık, 3 tohum) |
| `v14_lift50_std05` | `ppo_lift50_2250k`'dan std 0.05 ile devam, 2M; ara kayıt seçimi zemini de ölçüyor | 1.5M ara kaydı → `models/ppo_lift50_3750k` (60 mm 3/3); düzde aşma azalmadı (`zemin_olcumu.md`, `duz_olcumu.txt`) |
| `v15_odul_v7`, `v16_odul_v7_k3` | `ppo_lift50_3750k`'dan ödül v7 (aşma cezası, katsayı 1 ve 3) ile devam, 2M'şer | aşma biraz azaldı (0.114 → 0.110/0.107), enerji azalmadı, katsayı 3'te zemin bozuldu; depoya model alınmadı (ders 33) |
| `omni_olcum_2026-09-26.txt` | v10/v11 ve tabanların ham ölçüm çıktısı (her yön, ileri, rastgeleleştirme, gürültü, kütle, deneme zeminleri) | tablolar `models/README.md`'de |

Yeni PC'den (2026-09-26 öğlen) itibaren her eğitimde ayrıca:
`ara_degerlendirme.csv` (her ara kayıtta deterministik ölçüm; en iyisi
`best_model.zip` olarak saklanır) ve elle koşulan
`ara_degerlendirme_dr*.csv` (ara kayıtların rastgeleleştirme açıkken ölçümü).
