# Yürüyüş ölçümleri (S6)

Denetleyicilerin S5 zeminlerindeki karşılaştırması. Üreten araç:

```bash
python -m hexapod_rl.olcum tripod tripod:50 models/<ad>/model.zip \
    --tohum 3 --saniye 10 --temiz \
    --csv docs/olcumler/tripod_vs_politika.csv -o docs/olcumler/tablo.md
```

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

## Sonuçlar (2026-09-28, ilk tam tablo)

Komut ileri 0.1 m/s, 10 s, 3 tohum, rastgeleleştirme açık. Tablo:
[tablo.md](tablo.md).

**1. Düz zeminde herkes başa baş, fark zorlu zeminde açılıyor.** Düzde hepsi
0.098-0.114 m/s. 45 mm basamakta `ppo_lift50_3750k` 0.107 m/s, düz tripod
0.019 m/s — **5.6 kat**. 40 mm çukurda 0.107'ye karşı 0.014 (7.6 kat). Bu,
"düz zeminde tripod tavanı" dersinin (§12.28) ölçülmüş hâli.

**2. Adil karşılaştırma 50 mm adımlı tripod.** Düz tripod (25 mm adım) engelde
sürünüyor, ama adımı 50 mm'ye çıkarınca 45 mm basamakta 0.052 m/s'ye çıkıyor.
Politikanın gerçek üstünlüğü buna karşı: 45 mm'de 2.1 kat, 60 mm'de 2.7 kat.

**3. Zemin becerisinin düzde bedeli var.** `ppo_lift50_3750k` düzde en hızlı
(0.114 m/s) ama en pahalı (45.5 J/m); `tripod:50` 27.2 J/m ile en verimli.
Kör politikanın düz verim ↔ zemin sağlamlığı ödünleşimi (§12.33), enerji
tarafından da görünüyor. `ppo_kaldirma35_250k` beklendiği gibi ortada.

**4. Düz zeminde eğitilmiş politika engelde tripod'dan iyi değil.**
`ppo_omni_250k` 45 mm basamakta 0.018 m/s — düz tripod'la aynı. Zemin
becerisi zeminde eğitimle geliyor, kendiliğinden genellenmiyor.

**5. Kaygan eğimde politikalar tırmanamıyor ama tutunuyor.** 15° μ0.3'te
kimse çıkamıyor (sürtünme payı yok, §LEVELS["kaygan"]). Fark düşme biçiminde:
tripod saniyede 0.77 m geri kayıp 3/3 devriliyor, politikalar 0.02-0.04 m/s
ile neredeyse yerinde duruyor ve hiç devrilmiyor. Yürünebilir olan 10° μ0.3'te
politikalar açıkça önde (0.071-0.077'ye karşı tripod 0.046).

**6. Beklenmeyen: yüksek adım düzde daha ucuz.** `tripod:50` düz zeminde hem
biraz daha hızlı hem belirgin daha verimli (27.2 J/m) — `tripod`'un 25 mm'lik
varsayılanından (39.2 J/m) **%31 az enerji**. Ayak sürtmesinin azalması
olabilir. `hexapod_gait`'in varsayılan adım yüksekliği (25 mm, S2) bu yüzden
gözden geçirilmeli; ölçüm bunu tek başına kanıtlamaz, ayrı bir tarama gerekir.

**G7 için:** "politika tripod'u geçiyor" şartı hızda karşılanıyor (eğim,
engebe, kaygan ayrı ayrı ölçüldü); düz zeminde enerjide karşılanmıyor.

## Dosyalar

| Dosya | Ne |
|---|---|
| `tablo.md` | karşılaştırma tablosu (zemin satır, denetleyici sütun) |
| `tripod_vs_politika.csv` | ham veri, her koşu bir satır (grafik çizmek için) |

## Ölçümün sınırları

- Yol, gidilen **düz mesafe**; robot daire çizerse gerçek patikadan kısa
  görünür ve J/m yüksek çıkar. Komutlar düz olduğu için pratikte sorun değil.
- Güç, simülasyonun servo modelinin uyguladığı tork üzerinden (`Σ|τ·ω|`);
  elektriksel tüketim değil, mekanik güç. Gerçek akü tüketimi donanım
  vardiyasında ölçülür.
- Zeminler CAD geometrisiyle ve tahmini kütleyle koşuyor; gerçek robotta
  farklar çıkabilir (D9, D10).
