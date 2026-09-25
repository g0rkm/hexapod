# Eğitilmiş politikalar

Her klasör bir PPO eğitiminin son modeli (`model.zip`, Stable-Baselines3) ve
değerlendirmesi. Eğitimlerin tam çıktıları (ara kayıtlar, `progress.csv`)
eğitimi yapan makinede `~/hexapod_runs/<ad>/` altında; depoda yalnızca
kayda değer modeller tutulur.

Değerlendirmek için (WSL, ortam: `tools/wsl/rl_kurulum.sh`):

```bash
python -m hexapod_rl.evaluate models/tork_v2_10M/model.zip --vx 0.1
```

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
