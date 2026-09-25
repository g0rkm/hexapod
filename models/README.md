# Eğitilmiş politikalar

Her klasör bir PPO eğitiminin son modeli (`model.zip`, Stable-Baselines3) ve
değerlendirmesi. `policy.npz`: aynı politikanın torch'suz hâli (robotta
`hexapod_policy` düğümü okur; `python -m hexapod_rl.export <model.zip>` üretir). Eğitimlerin tam çıktıları (ara kayıtlar, `progress.csv`)
eğitimi yapan makinede `~/hexapod_runs/<ad>/` altında; depoda yalnızca
kayda değer modeller tutulur.

Değerlendirmek için (WSL, ortam: `tools/wsl/rl_kurulum.sh`):

```bash
python -m hexapod_rl.evaluate models/ppo_v4_4M/model.zip --vx 0.1
python -m hexapod_rl.evaluate models/ppo_v4_4M/model.zip --vx 0.1 --noise 0.1
python -m hexapod_rl.evaluate tripod --vx 0.1          # karşılaştırma: Samet'in tripod'u
```

## Karşılaştırma (2026-09-26)

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
