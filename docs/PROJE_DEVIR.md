# Hexapod — Proje Devir Belgesi

> **Bu belgeyi okuyan Claude için:** Projeye yeni bir bilgisayardan, hesaptan
> ya da oturumdan devam ediyorsun. Önceki oturumların bütün bağlamı, kararları,
> bulunan hataları ve kullanıcının çalışma tercihleri burada. Herhangi bir iş
> yapmadan önce bu belgenin tamamını oku. `CLAUDE.md` bunun kısa özetidir;
> çelişki görürsen bu belge + koddaki güncel durum esastır.
>
> Son güncelleme: **2026-09-27** (4. oturum sonu). Şu anki durum ve depodaki
> modeller **§3.1**; bu oturumun işleri §3.3–3.7 (her yöne politika, zeminli
> eğitim, öğrenilmiş kaldırma, müfredat, mesafe sensörlü kaldırma refleksi,
> robota geçiş dayanıklılığı), dersler 26–41, açık işler §13.2, plan §14.
> · Testler: **Linux 286/286**, Windows 235 geçti + 9 atlandı (Gazebo/ROS/SB3
> testleri Windows'ta atlanır)
>
> Bu belgeyi güncel tut: önemli bir karar, bulunan bir hata ya da biten bir
> aşama olduğunda ilgili bölümü güncelle ve "Son güncelleme"yi değiştir.

---

## İçindekiler

0. [Yeni bilgisayara geçiş (RTX 5070'li PC)](#0-yeni-bilgisayara-geçiş-rtx-5070li-pc)
1. [Kullanıcı ve çalışma tarzı](#1-kullanıcı-ve-çalışma-tarzı)
2. [Proje özeti ve hedef](#2-proje-özeti-ve-hedef)
3. [Mevcut durum](#3-mevcut-durum)
4. [Donanım](#4-donanım)
5. [Geometri ve koordinat çerçeveleri](#5-geometri-ve-koordinat-çerçeveleri)
6. [IK ↔ kalibrasyon sözleşmesi](#6-ik--kalibrasyon-sözleşmesi)
7. [Yazılım mimarisi ve depo](#7-yazılım-mimarisi-ve-depo)
8. [Araçlar](#8-araçlar)
9. [Çalışma ortamı](#9-çalışma-ortamı)
10. [Git, güvenlik ve kişisel veri](#10-git-güvenlik-ve-kişisel-veri)
11. [Karar günlüğü](#11-karar-günlüğü)
12. [Bilinen tuzaklar ve çıkarılan dersler](#12-bilinen-tuzaklar-ve-çıkarılan-dersler)
13. [Açık kalan işler](#13-açık-kalan-işler)
14. [Sıradaki iş için hazır plan](#14-sıradaki-iş-için-hazır-plan)
15. [Depoda olmayan kaynakların özeti](#15-depoda-olmayan-kaynakların-özeti)
16. [Yeni oturumda ilk adımlar](#16-yeni-oturumda-ilk-adımlar)

---

## 0. Yeni bilgisayara geçiş (RTX 5070'li PC)

2026-09-26'da kullanıcı çalışmayı RTX 5070 ekran kartlı başka bir bilgisayara
taşıdı. "Proje klasöründeki her şey diğer bilgisayarda da olmalı" dedi.

### 0.1 Ne nerede

- **Depo (GitHub, gizli): https://github.com/g0rkm/hexapod**, dal `main`.
  Proje klasörünün tamamı depoda: kod, belgeler, CAD (~345 MB), eğitilmiş
  modeller (`models/`), bütün eğitimlerin hafif kayıtları
  (`egitim_kayitlari/`). Klasörde git dışında kalan yalnız Python önbellekleri
  (`__pycache__`, `.pytest_cache`). Taşıma öncesi kontrol edildi.
- **Faturalar ve TÜBİTAK ekran görüntüleri bu klasörde değil.** Kişisel veri
  içerdikleri için hiç depoya girmedi (§10); içerikleri bu belgede özetli
  (§4.1, §15).
- **Eski PC'de kalanlar:** hepsi yeniden üretilebilir.
  - WSL içindeki `~/hexapod_ws` (colcon derlemesi) ve `~/hexapod_venv` (RL
    paketleri): kurulum betikleri yeniden kurar.
  - `~/hexapod_runs`: eğitim ara kayıtları, toplam ~100 MB. Hafif kısmı
    depoda (`egitim_kayitlari/`), önemli modeller `models/`'da.
  - Claude'un eski PC'deki hafıza notları: içerikleri bu belgeye işlendi (§1).

### 0.2 Kurulum (yeni PC, sırayla)

Kullanıcı terminalde deneyimsiz. Adımları tek tek ver; sudo şifresini asla sen
girme (§1).

1. **Windows'ta NVIDIA sürücüsü güncel olsun.** WSL içinde CUDA için Linux'a
   ayrıca sürücü KURULMAZ; Windows sürücüsü yeter.
2. **WSL2 + Ubuntu 26.04.** Yeni PC'de WSL2 kurulu ama Ubuntu yok (kullanıcı
   2026-09-26'da söyledi). Sürüm önemli: ROS 2 Lyrical paketleri yalnız
   Ubuntu 26.04 (resolute) için var.
   - PowerShell'de `wsl --list --online` ile adı bak.
   - `wsl --install -d Ubuntu-26.04` ile kur. O ad yoksa `Ubuntu`'yu kur ve
     içinde `lsb_release -ds` ile 26.04 olduğunu doğrula.
   - İlk açılışta Ubuntu bir kullanıcı adı ve şifre ister; bunu **kullanıcı
     girer**. Eski PC'de kullanıcı adı `gorkem`, dağıtım adı `Ubuntu`
     (26.04.1 LTS) idi.
   - Birden fazla dağıtım varsa (ör. docker-desktop) Ubuntu varsayılan
     olmalı: `wsl --set-default <ad>`. Betikler ve `wsl -e bash`
     varsayılan dağıtımı kullanır.
3. **Depoyu al.** `git clone https://github.com/g0rkm/hexapod.git`.
   Eski PC'de depo OneDrive altındaydı (`C:\Users\gorke\OneDrive\Masaüstü\hexapod`).
   Yeni PC aynı OneDrive hesabıyla açılırsa klasör kendiliğinden de gelir.
   Ama aynı depoyu iki bilgisayarda OneDrive üzerinden aynı anda kullanma,
   `.git` bozulabilir; tek yerden çalış, değişiklikleri git ile taşı.
   Yeni kurulumda depoyu OneDrive dışına klonlamak daha güvenli.
4. **WSL'de, depo klasöründe** (ör. `cd /mnt/c/Users/<ad>/.../hexapod`):
   - `bash tools/wsl/ros_kurulum.sh`: ROS 2 Lyrical + Gazebo + ros2_control.
     sudo ister, kullanıcı girer. Sonunda "KURULUM TAMAM" der.
   - `bash tools/wsl/derle.sh`: paketleri `~/hexapod_ws`'te derler (kaynaklar
     depoya sembolik bağlı).
   - `bash tools/wsl/rl_kurulum.sh`: `~/hexapod_venv` (torch CPU, SB3 2.9,
     Gymnasium 1.3).
5. **Doğrula:**
   - Windows: `python -m pip install pytest numpy pyyaml matplotlib`, sonra
     depo kökünde `python -m pytest -q`. Beklenen: 235 geçti, 9 atlandı
     (kurulum günü 178 idi; sonra test eklendi, bkz. başlıktaki sayılar).
     Eski PC'de Python 3.11.
   - WSL: `source /opt/ros/lyrical/setup.bash; source ~/hexapod_ws/install/setup.bash;
     source ~/hexapod_venv/bin/activate`, sonra `python -m pytest -q`.
     Beklenen: 286 geçti, ~40 s (kurulum günü 208).
   - Model: `python -m hexapod_rl.evaluate models/ppo_omni_250k/model.zip --vx 0.1`
     (görev ayarı modelin `gorev.json`'ından). Beklenen: 0.100 m/s, yön
     −3.5°, güç 2.00 W, devrildi False (yeni PC'de ölçüldü, 2026-09-27).
   - ROS'lu sim: `ros2 launch hexapod_gazebo sim.launch.py`, ayrı terminalde
     `ros2 run hexapod_gazebo stand`. Robot 100 mm'ye kalkar.

### 0.3 RTX 5070 hakkında dürüst not

Bu projede eğitimin darboğazı **GPU değil, CPU**.

- Her paralel ortam kendi Gazebo fizik simülasyonunu CPU'da koşuyor.
- Politika ağı çok küçük (29 → 128 → 128 → 18; öğrenilmiş kaldırmada 19).
- SB3 de MlpPolicy'li PPO için CPU'yu öneriyor.
- Eski PC'de 8 ortamla saniyede ~500–690 adım; 3M adım ~75 dk sürüyordu.

Yeni PC'de eğitimi hızlandıran şey **çekirdek sayısı**. Ryzen 7 7700X:
8 çekirdek / 16 iş parçacığı. Eski i5-10300H 4 / 8'di, çekirdek başına da
~1.7–2 kat yavaştı.
- **Öneri:** `--envs 16`. `n_steps 256 × 16 = 4096`, batch 512'ye bölünür.
- **Ölçüldü (2026-09-26, §0.5):** 16 ortamda **1818 adım/s**, eski PC'nin
  (8 ortam, 668) 2.7 katı; 3M adım ~28 dk. Tahmin ~3 kat idi.
- Ortam sayısı değişince PPO'nun güncelleme başına verisi de değişir
  (2048 → 4096 adım). Eski eğitimlerle karşılaştırırken bunu belirt.
- Her ortam ayrı bir Gazebo süreci. Bellek sorun değil: 16 ortam toplam
  ~2.0 GB kullandı, WSL'e 7.3 GB düşüyor (§0.5).

GPU ancak GPU'da paralel çalışan bir simülatöre (Isaac Lab, MuJoCo MJX gibi)
geçilirse işe yarar. Bu, TÜBİTAK başvurusundaki Gazebo'dan sapma olur;
kullanıcıya sormadan yapma.

İstenirse torch'un CUDA sürümü: RTX 50 serisi (Blackwell) CUDA 12.8+ ile
derlenmiş PyTorch ister. Örnek:
`pip install torch --index-url https://download.pytorch.org/whl/cu128`
**Denenmedi**; `rl_kurulum.sh` CPU sürümünü kurar.

WSL varsayılan olarak Windows RAM'inin yarısını ve bütün çekirdekleri
kullanır; gerekirse `%UserProfile%\.wslconfig` ile değiştirilir
(`[wsl2]`, `memory=`, `processors=`).

### 0.4 Yeni PC'de değişebilecek yollar

Kodda sabit bir kullanıcı yolu yok. Betikler depoyu kendi konumundan bulur,
`robot.yaml` yukarı doğru aranır. Ama bu belgede ve eski betik örneklerinde
geçen `C:\Users\gorke\...`, `/mnt/c/Users/gorke/...`, `~/hexapod_runs` gibi
yollar eski PC'ye ait; yeni PC'de kullanıcı adı farklı olabilir.

Yeni PC'deki yollar (2026-09-26):
- Depo: `C:\Users\user\Desktop\hexapod` (OneDrive dışında), WSL'den
  `/mnt/c/Users/user/Desktop/hexapod`.
- WSL dağıtımı `Ubuntu-26.04` (varsayılan), Linux kullanıcısı `user`:
  `~/hexapod_ws` = `/home/user/hexapod_ws`, `~/hexapod_venv`, `~/hexapod_runs`.

### 0.5 Yeni PC kurulum sonucu (2026-09-26)

Kurulum §0.2'ye göre yapıldı; her şey ilk denemede çalıştı.

- **WSL:** WSL 2.7.3. Makinede zaten `Ubuntu-24.04` (içinde başka veriler,
  ~20 GB) ve `docker-desktop` vardı. 24.04'e dokunulmadı; ROS 2 Lyrical onda
  yok. `wsl --install -d Ubuntu-26.04 --no-launch` ile yanına **Ubuntu 26.04.1
  LTS (resolute)** kuruldu ve `wsl --set-default Ubuntu-26.04` ile varsayılan
  yapıldı. Kullanıcı hesabını kullanıcı açtı.
- **Kurulum betikleri:** `ros_kurulum.sh` (kullanıcı çalıştırdı, sudo) →
  ROS 2 Lyrical, Gazebo 10.5.0, Python 3.14.4. `derle.sh` → 9 paket, 9 s.
  `rl_kurulum.sh` → torch 2.14.0+cpu, SB3 2.9.0, Gymnasium 1.3.0.
- **Testler:** Windows (Python 3.11.8) 178 geçti + 8 atlandı, 4 s. WSL 208
  geçti, 31 s (eski PC ~1 dk).
- **Model ölçümü** (`ppo_res_250k`, artık eylem, düz zemin, 10 s; model
  2026-09-27'de depodan kaldırıldı, git geçmişinde): eski PC'yle **birebir
  aynı** (simülasyon makineden bağımsız, belirlenimci).

  | Komut vx | Gerçek hız | Yön | Adım başı ödül | Güç |
  |---|---|---|---|---|
  | 0.05 | 0.058 | +0.1° | 2.821 | 1.82 W |
  | 0.10 | 0.105 | +5.4° | 3.238 | 2.26 W |
  | 0.15 | 0.153 | +12.2° | 3.615 | 2.94 W |
  | tripod 0.10 | 0.098 | −0.3° | 3.216 | 1.91 W |

- **Bellek:** Windows'ta 16 GB (15.2 GB görünür). WSL varsayılanı: 7.3 GB
  bellek + 2 GB takas, 16 iş parçacığı (`free -g`: toplam 7). `.wslconfig`
  gerekmedi.
- **Eğitim hızı:** aynı tarifle (artık eylem + rastgeleleştirme,
  `ppo_res_250k`'dan devam, lr 1e-4, target_kl 0.02), her biri 40 güncelleme
  turu. Süreye Gazebo'ların açılışı da dahil, uzun eğitimde biraz daha hızlı
  olur.

  | Ortam | Adım/s | Duvar süresi | Bellek (tepe, toplam kullanılan) |
  |---|---|---|---|
  | 8 | 1342 | 82k adım, 72 s | 1.4 GB |
  | 12 | 1588 | 123k adım, 88 s | 1.7 GB |
  | 16 | **1818** | 164k adım, 100 s | 2.0 GB |

  Eski PC 8 ortamda 500–690 adım/s idi. Bundan sonra **`--envs 16`**; 3M adım
  ~28 dk, 10M ~1.5 sa. Kayıtlar `egitim_kayitlari/hiz_testi_env{8,12,16}`.
- GPU (RTX 5070, sürücü 616.64) kullanılmıyor; gerekçe §0.3.

---

## 1. Kullanıcı ve çalışma tarzı

**Kim:** Görkem Mutlu. Depo sahibi, GitHub kullanıcı adı `g0rkm`, commit
e-postası `gorkemmutlu227@gmail.com`. Proje bir üniversite kulübü takım
projesi (TÜBİTAK Milli Teknoloji Kulüpler Birliği, Kulüp Geliştirme
Desteği başvurusu). Görkem yazılım tarafını Claude ile yürütüyor.
Takım arkadaşı **Samet** (GitHub: Oruc74, `sametoruc74@gmail.com`) de
yazılım tarafında: yazılım görevleri ikisi arasında bölüşüldü. Donanım
işleri **durduruldu**, ayrı bir donanım vardiyasına taşındı. Görev sahipleri
ve bağımlılıklar [GOREVLER.md](../GOREVLER.md)'de.

**Önemli:** Görkem robotu **kendisi monte etmedi**. Donanım, lehim, kablo
ve terminal işlerinde deneyimli değil; bu tür işler istenince bunalıyor.
Bir keresinde açıkça "ben nasıl yapacam bilmiyorum, ben birleştirmedim
aleti, yazılımı elimizdeki bilgilerle yapamaz mıyız" dedi. Donanım işi
robotu kuran takım arkadaşına kalıyor.

**İletişim tercihleri:**
- Türkçe konuş. Samimi ve doğrudan olabilirsin.
- **Kısa, madde madde** cevap ver. Uzun, tablolu, jargonlu talimatları
  anlamadığını söyledi.
- Bir şey anlatman gerekiyorsa **çok basit** anlat: her adım tek eylem,
  komutlar ayrı kod bloklarında, terim kullanırsan tek cümleyle açıkla.
- Ondan bir şey istemeden önce sor: bu işi bir araçla otomatikleştirebilir
  miyim? Otomatikleştirilebiliyorsa talimat yazmak yerine aracı yaz (örnek:
  kabloları elle takip ettirmek yerine `map_channels.py`).
- Gereksiz soru sorma. Makul varsayımı yap, söyle, devam et.
- Donanım ya da ölçüm işi isteme. Geometri CAD'den geliyor (§5.3).

**Özerklik (3. oturumda yerleşti):**
- Kullanıcı uzun işleri bana bırakıyor. Kendi sözleriyle: "eğitim bittikten
  sonra kontrol edip pushlarsın", "devam ettir işlemleri, gece boyu
  çalışmaya devam edebilirsin".
- Uzun eğitimleri başlatmak, izlemek, bitince değerlendirmek, belgeleri
  güncellemek ve commit + push etmek benim işim.
- Sık sık "bitti mi", "ne zaman bitecek" diye soruyor: kısa bir durum (yüzde,
  kalan süre, ara sonuç) ver.
- Uzun eğitim sürerken bilgisayar uyumamalı. Eğitimi Windows'ta
  `SetThreadExecutionState` tutan bir PowerShell sarmalayıcısıyla başlat.
  Claude Code'un keep_awake'i oturum boşta kalınca bırakıyor.
- Bir karşılaştırma deneyi başlamadan önce RL ortam kodunu değiştirme: süreç
  başlarken diskteki kodu yükler.
- Yanlış bir sonucu push'ladıysan sonraki commit'te açıkça düzelt ve
  kullanıcıya söyle (3. oturumda bir hız ölçümü için oldu, §12.23).
- Kullanıcı bazen bilgisayarı acil kapatması gerektiğini söyler: ara
  kayıttan devam edilebildiğini söyle, kodu push'la, durumu belgeye yaz.

**Git tercihleri (kesin):**
- Commit mesajları **Türkçe ve detaylı**: ne değiştiğini değil, **neden**
  öyle yapıldığını da anlatsın. Ölçüm sonuçlarını sayılarıyla yaz.
- Commit'lerde **Claude imzası YOK**: `Co-Authored-By`, "Generated with
  Claude Code" ve benzeri satırlar yasak. Yazar yalnızca Görkem.
- Değişiklikleri **mantıklı, ayrı commit'lere** böl (düzeltme, özellik,
  belge ayrı).
- **Push:** kullanıcı "commit atıp pushla" diye istiyor ve bir iş bitince
  push'lamama izin verdi. Push öncesi kişisel veri kontrolü (§10).
- Samet de aynı depoya push'luyor. Çalışmaya başlamadan `git pull`
  (`--rebase`) yap. Push edilmemiş kendi commit'lerini onunkilerin üstüne al.
- Ağ hatasında (HTTP2) `git -c http.version=HTTP/1.1 push`.

---

## 2. Proje özeti ve hedef

Altı bacaklı (hexapod) bir robot. Her bacakta 3 eklem (coxa, femur,
tibia), toplam **18 servo**.

**Hedef (kullanıcının 2026-09-24'te doğruladığı hâliyle):**
> Otonom bir sistem. ROS 2 üzerinden Gazebo simülasyonu ile geliştirilip
> Raspberry Pi 4'e aktarılacak; her koşulda ve her zorlukta çözüm üreten
> bir sistem.

TÜBİTAK başvurusundaki yöntem (bkz. §15):
- Gazebo'da sanal IMU ve temas sensörleriyle donatılmış bir hexapod modeli.
- Python tabanlı bir RL ajanı (**Stable-Baselines3, PPO**).
- Ödül: robotun devrilmeden, en az enerjiyle ve en hızlı ilerlemesi.
- Sabit yürüyüş desenleri (CPG) yerine değişken zemine uyum sağlayan,
  öğrenilmiş bir yürüyüş.
- Eğitim simülasyonda yapılıp gerçek robota aktarılacak (sim-to-real).

**Gerçekçi beklenti (kullanıcıya söylendi):**
- RL politikası eğitimde gördüğü zorluk türlerine karşı sağlam olur, "her
  koşula" değil.
- Eğitim senaryolarının kapsamı (eğim, engebe, kaygan zemin, itme,
  kütle/sürtünme değişimi) sistemin neyi çözebileceğini belirler.
- Eğitim PC'de yapılır; Pi 4 yalnızca eğitilmiş politikayı çalıştırır.

**Strateji: önce yazılım, sonra donanım.** Her şey CAD geometrisiyle
simülasyonda geliştiriliyor. Kablolama ve kalibrasyon yazılımın önünde
engel değil; robotu kuran kişi tarafından yapılıp config'e girilecek.

**Asıl brif:** `docs/hexapod-proje-brifi.md`. Başlangıç belgesi. Şu kısımları
artık geçersiz ya da güncellendi:
- Mimari: brifteki düz, katmanlı Python yığını yerine **ROS 2 + Gazebo +
  RL** (TÜBİTAK başvurusu esas alındı). Katman sırası yine geçerli.
- "Segment uzunluklarını kumpasla ölç" notu **iptal**; CAD kullanılıyor.
- Brifte "kesinleşmemiş" olan DOF ve servo arayüzü artık kesin: 3 DOF,
  PCA9685 üzerinden PWM.

---

## 3. Mevcut durum

| # | Aşama | Durum |
|---|---|---|
| 1 | Servo sürücü katmanı (`hexapod_driver`) + kalibrasyon, kanal haritası, donanım kontrolü, CAD araçları | ✅ testli (G1) |
| 2 | Ters/düz kinematik + gövde pozu (`hexapod_kinematics`) | ✅ testli (G2) |
| 3 | URDF modeli (`hexapod_description`) | ✅ RViz ve check_urdf'ten geçti (G3) |
| 4 | Gazebo dünyası + ROS 2 eklem arayüzü (`hexapod_gazebo`, docs/ARAYUZ.md) | ✅ (G5). ROS'lu sim 09-26'dan beri **tork servo modelinde** |
| 5 | Klasik yürüyüş (tripod) | ✅ Samet: `hexapod_gait` (S2), `hexapod_teleop` (S3) |
| 6 | Gerçek robot sürücü düğümü (`hexapod_hardware`) | ✅ Samet (S4); yalnız dry-run, donanımda denenmedi |
| 7 | RL ortamı (`hexapod_rl`) | ✅ (G6); tripod aynı ortamda ölçüldü |
| 8 | PPO eğitimi | 🔄 (G7). Depodaki modeller ve ne için oldukları §3.1'de. Asıl zeminler S5'i, "bitti" ölçümü S6'yı bekliyor |
| 9 | Politika düğümü (`hexapod_policy`, torch'suz) | ✅ (G8). ROS'lu simde her yöne ve deneme zeminlerinde yürüdü; mesafe sensörlü refleks denetleyicide hazır, düğüme bağlanması S7/D8'i bekliyor |
| 10 | Pi 4'e aktarma | ⛔ donanım vardiyası (D11) |

**Görev dağılımı:** [GOREVLER.md](../GOREVLER.md).
- Yazılım aşaması: G = Görkem, S = Samet.
- Durdurulmuş donanım vardiyası: D1–D12.
- Her görev hangi görevi beklediğini ve hangisini açtığını söyler.
- Bitenler: G1–G6, G8; S1–S4.
- Sürenler: G7 (Görkem), S5 (zemin üreteci), S6 (ölçüm aracı), S7 (sensör
  sürücüleri). Son üçü Samet'in.

### 3.1 Şu anki durum (2026-09-27, 4. oturum sonu)

Depodaki modeller (`models/`; tablolar [models/README.md](../models/README.md)):

| Model | Ne için | Öne çıkan |
|---|---|---|
| `ppo_omni_250k` | düz zemin, her yöne, en verimli | taban 25 mm; düzde 0.10 m/s'de 2.00 W, tripod düzeyinde; 45 mm engel yok (§3.3) |
| `ppo_lift50_3750k` | zemin, sensörsüz | taban 50 mm; engel skoru 0.763, 60 mm basamak 3/3; düzde 4.0 W, hızı %14 aşıyor (§3.4) |
| `ppo_kaldirma35_250k` | orta yol; refleksle düzde en verimli | öğrenilmiş kaldırma (19. çıkış), kör ~35 mm: düzde 2.39 W, 45 mm engeller. Mesafe sensörlü refleksle düzde 25 mm'nin enerjisi, engelde sabit 50 mm'den iyi (§3.5, §3.6) |
| `ppo_refleks_1500k` | refleksle engelde en iyi | refleks açıkken eğitildi: 60 mm basamak, engebe 60'ta en iyi; düzde %8–17 daha pahalı; kör ~34 mm (§3.6) |
| `taklit_bc_v4` | yalnız testler | mutlak mod, taklit; uçtan uca düğüm testleri kullanıyor |

Eski modeller (`tork_v2_10M`, `ppo_v4_4M`, `ppo_v7_8M`, `ppo_res_250k`,
`ppo_lift50_2250k`) 2026-09-27'de depodan kaldırıldı: yerlerini yukarıdakiler
aldı, hiçbir kod ya da test kullanmıyordu. Ölçüm tabloları models/README'de
duruyor, dosyalar git geçmişinde.

Ana bulgular:
- **Düz zeminde tripod tavanı** (ders 28): RL düzde tripod'la başa baş;
  kazanç zeminde ve gürültüde.
- **Kör politikada düz verim ↔ zemin ödünleşimi** (ders 33): ödülle (v7),
  öğrenilmiş kaldırmayla (ders 35) ve müfredatla (ders 40) çözülmedi;
  **mesafe sensörlü kaldırma refleksiyle çözülüyor** (ders 41, §3.6).
- **Robota geçişte asıl risk kalibrasyon ve servo torku;** IMU eğikliği ve
  gecikme değil (ders 37, §3.7).
- **Bekleyenler:** S5 (zeminler), S6 (ölçüm), S7 (sensör sürücüleri →
  refleksi düğüme bağlama), D8 (sensör yerleşimi), donanım vardiyası.

### 3.2 Artık eylem modu (2026-09-26 sabah)

Ayrıntılı tablolar: [models/README.md](../models/README.md). Koşullar: düz
zemin, ödül v6, 10 s, deterministik. Hiçbir model devrilmedi. (`ppo_v7_8M`
ve `ppo_res_250k` 2026-09-27'de depodan kaldırıldı; git geçmişinde.)

| 0.10 m/s komut | Tripod (Samet) | `ppo_v7_8M` (mutlak) | **`ppo_res_250k` (tripod + düzeltme)** |
|---|---|---|---|
| Gerçek hız | 0.098 | 0.098 | 0.105 |
| Adım başı ödül | 3.22 | 2.94 | **3.24** |
| Mekanik güç | 1.9 W | 6.4 W | 2.3 W |
| Rastgeleleştirmede ödül | 3.08–3.22 | 2.82–3.00 | 3.06–3.18 |
| 0.05 rad eklem gürültüsünde ödül | 1.46 | 2.23 | 1.78 |

- **Artık eylem modu** (politika tripod'a düzeltme verir) enerji sorununu
  çözdü. Düz zeminde tripod'la başa baş ya da biraz önde.
- Kalan kusurlar:
  - Uzun eğitimde yine hedef hızı aşmaya kayıyor; en iyi ara kayıt 250k.
  - Hızlıda hafif sola yön kayması: 0.15 m/s'de 10 s'de +12°.
  - Politika yalnız **ileri 0.05–0.15 m/s** komutunu gördü. Yana, geri ve
    dönüş eğitilmedi; politika düğümü bu komutlarda ayakta bekler.
    → §3.3'te çözüldü.
- RL'nin tripod'u açıkça geçmesi beklenen yer zorlu zemin. Bunun için
  gerekenler: S5 (Samet) ve S6. Zemine göre ayak teması (G7) hazır
  (2026-09-26, §7.8).

### 3.3 Her yöne yürüyen politika (2026-09-26 öğlen, yeni PC)

`models/ppo_omni_250k`: ileri/geri, yana ve dönüş komutlarının hepsini
görmüş artık eylem politikası. Tablolar [models/README.md](../models/README.md).

- Yedi komutluk her yön setinde ortalama ödül **2.868**; tripod 2.848,
  PhaseTripod 2.846, yalnız ileri eğitilmiş `ppo_res_250k` 2.800. Bütün
  komutları %95–100 izliyor. Hiçbiri devrilmedi.
- Robottaki denetleyiciyle (düğüm çekirdeği) Gazebo'da geri, yana ve
  dönüşte yürüyor; sıfır komutta ayakta bekliyor (`tests/test_policy_sim.py`).
- **Düz zeminde tripod'u anlamlı geçmiyor** (+%0.7). Rastgeleleştirme
  açıkken ölçünce de eşit ya da ~%1 geride. Düz zeminde iyi bir tripod'un
  üstüne öğrenilecek çok şey yok.
- Eklem gürültüsünde açık ara önde: 1.84'e 1.46.
- **Deneme zeminleri** (`terrain_probe`, zemin görmeden): 20° yokuşta ve
  30 mm basamakta politikalar tripod'dan %6–21 hızlı. **45 mm basamağı
  hiçbiri çıkamıyor**: ayak 25 mm kalkıyor. Zeminli eğitimin çözmesi
  gereken ilk somut örnek. → §3.4.

### 3.4 Zeminli eğitim (2026-09-26 öğleden sonra–akşam)

Ayrıntı ve tablolar: [models/README.md](../models/README.md) ("Zeminli eğitim",
"Bütün zemin türleri").

- **Altyapı hazır:** ortam başına zemin (`train.py --terrains`), taklit de
  zeminlerde (`pretrain.py --terrains`), görev ayarı modelin yanında
  (`gorev.json`), taban ayak kaldırma ayarlı (`--lift-mm`). S5 gelince
  yalnız zemin listesi değişir.
- **Taban 25 mm'yken RL engelleri öğrenemedi** (`v12_zemin`, 2.25M adım):
  keşif gürültüsü ayağı yükseğe kaldırmayı bulamıyor (ders 29).
- **Taban 50 mm'de** (`ppo_lift50_2250k`; 2026-09-27'de depodan kaldırıldı,
  yerini 3750k aldı): 45 mm basamak, 45 mm çukurdan
  geri/yana çıkış, 50 mm yayladan iniş, 60 mm çukurdan geri çıkış hepsi
  3/3 (rastgeleleştirme açık). Samet'in tripod'u ve 25 mm'li politikalar
  bunların hiçbirini geçemiyor. Aynı 50 mm tabana göre RL engellerde
  **+%15** (zemin skoru 0.791 → 0.879). 60 mm basamağı ileri çıkmayı
  güvenilir öğrenmedi.
- **Bedeli düz zemin:** hedef hızı %10 aşıyor, güç 3.8 W (25 mm'li
  politika 2.0 W). Düz zeminde `ppo_omni_250k` daha iyi; hangisinin
  robota gideceği D11'de zemine göre seçilir.
- **Devam (v14, std 0.05):** düzde aşmayı azaltmadı (%14, 4.0 W) ama
  zeminde ilerledi: `ppo_lift50_3750k` 60 mm basamağı da 3/3 geçiyor,
  zemin skoru 0.957 (tablolar models/README). Sensörsüz zeminde en iyisi
  bu (mesafe sensörlü refleksle §3.6).
- **Aşmayı ödülle düzeltme (ödül v7, v15/v16):** komutu aşan hız
  `progress`'ten düşülünce (katsayı 1 ve 3) hız 0.114 → 0.110/0.107'ye
  indi ama güç 3.9–4.0 W'ta kaldı; katsayı 3'te zemin becerisi bozuldu
  (çukurdan yana çıkış 0/3). **Kör politikada düz verim ile zemin
  sağlamlığı arasında ödünleşim var** (ders 33).
- **Bütün zemin türleri** (eğim, engebe, kaygan, basamak; deneme
  zeminleri, rastgeleleştirme açık, 3 tohum): `ppo_lift50_3750k` engel
  skoru **0.763**; Samet'in tripod'u 0.281, 50 mm adımla 0.443, düzeltmesiz
  50 mm taban 0.628. Engebe ve kaygan eğim eğitimde yoktu, politika
  genelleşiyor. Samet'in tripod'u 15° kaygan yokuşta (μ 0.3) geriye kayıp
  3/3 devriliyor; politika devrilmeden tutunuyor ama çıkamıyor (sürtünme
  payı %10, fiziksel sınıra yakın). ROS'lu simde gerçek düğümle de
  45/60 mm basamağı ve çukuru çıkıyor (`ppo_omni_250k` takılıyor; §7.9).
  Payı olan kaygan yokuşu (10° μ 0.25)
  politika 3/3 çıkıyor, tripod 0/3. Bunları da eğitime katma denemesi
  (v17) iyileştirmedi (ders 34). Tablo: models/README.
- **Adil karşılaştırma:** Samet'in `TripodGait`'i de 50 mm adımla
  ölçüldü: zemin skoru 0.548 (25 mm'de 0.404), RL 0.879. Aynı adım
  yüksekliğinde de RL açık ara önde; 45 mm çukurdan yana çıkış ve 60 mm
  çukurdan geri çıkış tripod'da 0/3, RL'de 3/3.

### 3.5 Öğrenilmiş ayak kaldırma ve müfredat (2026-09-27)

Tablolar: models/README ("Öğrenilmiş ayak kaldırma", "Müfredat").

- **Öğrenilmiş ayak kaldırma (2026-09-27, v18–v21):** politika taban
  tripod'un kaldırmasını 19. çıkışla (20–60 mm) kendisi seçiyor; her
  salınımın başında seçilir, salınım boyunca sabit (§7.8). Amaç düzde az,
  engelde çok kaldırmaktı. **Olmadı:** kör politika kaldırmayı zemine göre
  değiştirmiyor, eğitim zemin karışımı için tek bir değere ayarlıyor
  (ders 35). 25 mm'den başlayınca ~28 mm'de takılıyor (engeller 28–35 mm
  arasındaki bir eşikte geçilmeye başlıyor); 35 mm'den başlayınca 4M'de
  53 mm'ye çıkıyor ve son model `ppo_lift50_3750k`'dan iki yönden de kötü
  (engel skoru 0.652 < 0.763, düz güç 4.65 > 3.99 W). Her adım seçilen
  kaldırma hiç kıpırdamadı (ders 36).
- **Yan ürün, orta yol modeli `ppo_kaldirma35_250k`** (v21'in 250k'sı,
  kaldırma ~35 mm, zeminden bağımsız): düzde 2.39 W (`ppo_omni_250k` 2.00,
  `ppo_lift50_3750k` 3.99), engel skoru 0.496 (0.338 / 0.763). 45 mm
  basamak, 60 mm engebe, 10° kaygan yokuş 3/3, 45 mm çukurdan geri
  çıkış; 60 mm basamak, 60 mm çukur ve 45 mm çukurdan yana çıkış yok. ROS'lu
  simde gerçek düğümle düzde her komutta %100–107; 45 mm basamak ve 45 mm
  çukurdan geri çıkıyor; yana ve 60 mm basamakta takılıyor (iki sim aynı;
  ilk yazılan "yana çıkıyor" ölçüm hatasıydı, ders 38). Tablolar:
  models/README ("Öğrenilmiş ayak kaldırma").
- **Müfredat (2026-09-27, v22–v23; `--curriculum`, §7.8):** sabit 25 mm
  tabanda müfredat küçük bir kazanç getirdi (engel skoru 0.338 → 0.371, v12
  0.316) ama 45 mm'lik engel yok. Öğrenilmiş kaldırmayla (25 mm'den) v20'nin
  28 mm'deki yerel en iyisinden çıktı (4M'de 33.6 mm, 45 mm basamak 3/3,
  engel skoru 0.499), ama 35 mm'den müfredatsız başlayan
  `ppo_kaldirma35_250k` (0.496, 2.39 W) ile aynı yere, daha çok güçle (3.15
  W) vardı; model alınmadı (ders 40). Tablolar: models/README ("Müfredat").

### 3.6 Mesafe sensörlü kaldırma refleksi (2026-09-27)

Tablolar: models/README ("Mesafe sensörlü kaldırma refleksi"). Kod: §7.8
(sim, eğitim), §7.9 (denetleyici).

- **Mesafe sensörlü kaldırma refleksi (2026-09-27, ders 41):** eğitimsiz
  bir kural (`hexapod_policy.lift_reflex`), önüne bakan sensörler engel
  görünce kaldırmayı yükseltiyor. Simde DENEYSEL bir yerleşimle (robot.yaml'da
  null) `ppo_kaldirma35_250k`'nın kaldırma çıkışını ezerek: düzde sabit 25
  mm'nin gücünde (3.95 W, sabit 50 mm 4.45 W; rastgeleleştirme açık), 45
  mm basamak 0.98 (sabit 50: 0.88), 60 mm basamak 0.86 3/3 (sabit 50: 0.41
  2/3), %5 gürültü + %10 okuma düşmesi + yarı hızda da aynı. Sensör 20–25°
  aşağı bakmalı; 45° hiç çalışmıyor. Kör politikanın ödünleşimi (ders 33)
  sensörle çözülüyor. Her yöne yürüyüşte yerleşim: bir sensör bakışının
  ~±30–45°'sini görüyor; biri ileri, ikisi ±90° ile ileri/yana/çapraz
  görülüyor, geride refleks devre dışı kalıp politikanın kör kaldırmasına
  dönüyor (`covers`); dört yönde de 45 mm basamak geçiliyor. Robot
  düğümüne bağlanması S7 ve D8'i bekliyor.
  Tablolar: models/README ("Mesafe sensörlü kaldırma refleksi").
- **Refleks açıkken eğitim (v24, 2026-09-27): `models/ppo_refleks_1500k`.**
  `train.py --reflex 20 --range-noise 5 --range-drop 10`,
  `ppo_kaldirma35_250k`'dan 3M; kaldırmayı refleks seçiyor, eklemler uyum
  sağlıyor. Refleksle 60 mm basamak 0.86 → 0.89 (2M'de 0.99), engebe 60
  0.78 → 0.90; kapalı döngüde 60 mm basamağın tamamen üstüne çıkıyor (0.63
  m, z 158 mm; öteki 0.48 m yarı yolda). Bedeli: aynı 25 mm kaldırmada düz
  güç %8–17 fazla (3.95 → 4.37 W). Refleks olmadan da çalışıyor (kör ~34
  mm). Engebeli arazide bu, düz ağırlıklı kullanımda `ppo_kaldirma35_250k`.

### 3.7 Robota geçiş: dayanıklılık (2026-09-27)

Tablolar: models/README ("Dayanıklılık taraması"). Ayrıntı ders 37.

- **Hassas olunanlar kalibrasyon ve servo torku** (`hexapod_rl.robustness`):
  eklem başına σ2°'lik kalibrasyon ofseti düzde en çok %4, σ4° %7–12
  kaybettiriyor (tripod da aynı kadar); servo torku katalog değerinin
  yarısının altına inince zemin modelleri engelde, 50 mm'lik ×0.4'te düzde
  de devriliyor. IMU'nun 20°'ye kadar eğik takılması ve 240 ms'ye kadar
  gecikme etkisiz.
- **Refleks kalibrasyon hatasına karşı da engelde payı açıyor:** σ2–4°'te
  engelde sabit 50 mm'den iyi (35 mm'lik modelin kırılganlığı kalkıyor).
- **Donanım vardiyasına notlar** GOREVLER'de: D6 (kalibrasyonda eklem başına
  ~2° doğruluk), D8 (sensör yerleşimi: ileri + ±90°, 20–25° aşağı), D9
  (yük altında servo gerilimi; robotta ilk politika denemesi 25 mm'lik
  modelle).

---

## 4. Donanım

### 4.1 Gerçekte alınanlar (faturalarla doğrulandı)

Faturalar **tek güvenilir kaynak**. Depoda değiller (kişisel veri
içeriyorlar, bkz. §10); içerikleri burada özetlendi. Alımlar Temmuz sonu –
Ağustos 2026.

| Parça | Adet | Not |
|---|---|---|
| Raspberry Pi 4, 4 GB | 1 | Ana kontrolcü |
| Raspberry Pi Camera Module V2 | 1 | CSI |
| MG996R servo, **180 derece** sürüm | 20 | 18 kullanım + 2 yedek |
| PCA9685 16 kanal I2C PWM sürücü | **2** | 18 servo tek karta sığmaz |
| Gravity BNO055 + BMP280 10DOF AHRS (IMU) | 1 | I2C |
| VL53L0X uçuş süresi mesafe sensörü | **3** | I2C, üçü de 0x29'da doğar |
| 7.4V 2S LiPo, **2800 mAh**, 40C | 2 | |
| 300W 20A DC-DC buck (voltaj düşürücü) | 3 | |
| Çift USB çıkışlı 5V 3A regülatör | 1 | Muhtemelen Pi beslemesi |
| JST konnektör takımı, 12 AWG silikon kablo | — | |
| M3 ve M5 pirinç insert somun | — | Montaj |
| PETG filament (siyah ×2, beyaz ×2, 1 kg) | 4 | Parçalar **PETG** basıldı, PLA değil |

MG996R katalog değerleri (simülasyon için kullanılabilir, `measured:
false` olarak işaretle): kütle ~55 g; tork ~9.4 kg·cm @4.8 V, ~11 kg·cm
@6 V; hız ~0.17 s/60° @4.8 V, ~0.14 s/60° @6 V; çalışma gerilimi 4.8–7.2 V.
Mil dişlisi **25 diş** (diş başına 14.4°).

### 4.2 Malzeme listeleri güvenilmez

Depoda (`docs/malzeme/`) iki malzeme listesi var ve **ikisi de hatalı**:
- `gömülü malzemeler listesi.txt`: faturalara daha yakın (VL53L0X, 300W
  buck) ama PCA9685'i **1**, VL53L0X'i **2**, bataryayı **2200 mAh**
  gösteriyor. Doğrusu 2, 3 ve 2800 mAh.
- `gömülü malzemeler listesi (alternatif).txt`: TF-Luna LiDAR, LM2596,
  3 batarya. **Bunlar hiç alınmadı.**
- `Malzeme Listesi.txt`: mekanik montaj listesi (vida, insert, rulman).
  "Total: 18x MG996R Servo Horn" satırı 3 DOF'nin ilk kanıtıydı.

Çelişki olursa sıra: **fatura > CAD > listeler.**

### 4.3 CAD donör bir tasarım

`cad/Hexapod/` ve `cad/Baskı Dosyaları/` altındaki her şey **Sir Kuhnhero, "3D
Printed Hexapod", Printables model 606030, CC BY-SA 4.0** tasarımından.
Lisans ve atıf belgesi: `cad/Hexapod/606030-*.pdf` (depodaki tek PDF).

- **Mekanik geometri geçerli**, robot bu parçalarla basıldı.
- **Elektronik geçerli DEĞİL.** Donör tasarım STM32 BluePill + 4×18650
  hücre + XL4016 8A buck kullanıyor (`body-v35.step` içinde görünür). Bu
  robot Pi 4 + 2S LiPo kullanıyor. CAD'deki kart yerleşimine bakarak güç
  ya da kontrolcü varsayımı yapma. Donör tasarımdaki 2× PCA9685 ise
  bizimkiyle örtüşüyor.
- Donör tasarımcının kendi belirttiği sorunlar (PDF'te): **akım
  sıçramalarında servo buck'ının yetmemesi** ve **STM32 ile servo
  sürücüler arasındaki I2C hattının çökmesi** (ekranlı kabloyla
  çözülmüş). İkisi de bu robot için gerçek risk.

### 4.4 Elektrik ve güvenlik notları

- **Servo gerilimi:** dolu 2S LiPo 8.4 V verir, MG996R en fazla 7.2 V
  kaldırır. Servolar **voltaj düşürücü üzerinden ~6 V** ile beslenmeli.
  Bağlamadan önce multimetreyle ayarlanmalı.
- **Brownout riski (brifin 1 numaralı riski):** 18 servo yük altındayken
  gerilim çökebilir ve Pi resetlenir; bu yazılım hatası gibi görünür ve
  günlerce kovalanır. Servo ve Pi beslemesi ayrı olmalı, toprak ortak
  olmalı.
- **PCA9685 VCC Pi'nin 3.3 V'una (pin 1) bağlanmalı, 5 V'a DEĞİL.** Kart
  I2C hatlarını VCC'ye çeker; 5 V verilirse Pi'nin GPIO'ları zarar görür.
  Bağlantı: VCC→pin 1, SDA→pin 3, SCL→pin 5, GND→pin 6.
- **PCA9685 adresleri:** iki kart da fabrikadan **0x40** gelir. Birinin
  **A0** lehim noktası birleştirilince **0x41** olur. Her PCA9685 ayrıca
  **0x70**'te (ALLCALL, ortak çağrı adresi) cevap verir; bu kartın kendi
  adresi değildir.
- **VL53L0X:** üçü de 0x29'da doğar; ayrı XSHUT pinleriyle sırayla
  uyandırılıp yeniden adreslenmeleri gerekir. BNO055 de ADR yüksekken
  0x29'dadır, çakışmaya dikkat.
- **MG996R kablo renkleri:** kahverengi = GND, kırmızı = V+, turuncu =
  sinyal (PWM).

---

## 5. Geometri ve koordinat çerçeveleri

### 5.1 Değerler (`config/robot.yaml`)

Hepsi CAD'den türetildi, hepsi `measured: false`. Yeniden üretmek için
`python tools/cad_extract.py`.

| Parametre | Değer | Kaynak |
|---|---|---|
| coxa | **50.0 mm** | STEP: J1 ekseninden J2 eksenine yatay (tam 50.000) |
| femur | **80.0 mm** | STEP: J2→J3 (79.997, tasarım değeri 80) |
| tibia | **126.6 mm** | STL + STEP montaj dönüşümü: J3 → ayak kapağının en uç noktası (126.635) |
| femur_joint_z_offset | **−10.05 mm** | Femur eklemi, bacak montaj referans düzleminin 10.05 mm altında |
| coxa_axis_radius | **100.0 mm** | Coxa eksenleri gövde merkezinden 100 mm'de (montaj orijinleri 120'de, eksen 20 mm içeride) |
| Bacak azimutları (CAD) | 0, ±60, ±120, 180° | Radyal altıgen yerleşim |
| Ayak ucu yuvarlaklık yarıçapı | ~5.1 mm | STL'e daire uydurma (bilgi amaçlı, config'de alan yok) |

Diğer CAD bulguları: tibia gövdesi (`pla_tibia_main`) J3'ten 118.62 mm'de
bitiyor, ayak kapağı (`pla_tibia_tip`) onun ucuna geçip 8 mm taşıyor ve
yere değen parça o. Gövdenin kendi bacak montaj vida çemberi 80 mm
yarıçapta (`body-v35.step`).

### 5.2 Türetme yöntemi

- STEP assembly ağacı çözüldü (`NEXT_ASSEMBLY_USAGE_OCCURRENCE` +
  `ITEM_DEFINED_TRANSFORMATION`); her parçanın montaj çerçevesindeki
  konumu hesaplandı (`tools/cadlib/assembly.py`).
- Her eklemin dönme ekseni = o eklemin **servo horn'u ile karşısındaki
  bushing'i birleştiren doğru**. Bacak yerel çerçevesinde (`leg-v2-v20.step`;
  +Y yukarı, bacak −Z yönünde dışarı uzanıyor):
  - J1 coxa (yaw): yerel Y boyunca, (x=0, z=+20)
  - J2 femur (pitch): yerel X boyunca, (y=−10.05, z=−30)
  - J3 tibia (pitch): yerel X boyunca, (y=+10.66, z=−107.27)
- Tibia: **basılan STL'ler** (`pla_tibia_tip.stl`) kullanıldı. STL'ler
  parçanın kendi çerçevesinde; STEP'teki montaj dönüşümü uygulanınca
  bacak çerçevesine oturuyorlar (STEP yüzey köşeleriyle 0.004 mm örtüşme
  doğrulandı).
- Tam montaj (`hexapod-v8.step`): bacak orijinleri 120 mm yarıçapta; coxa
  eksenleri 100 mm'de. Altı bacağın da tam 100.0 çıkması araçta
  tutarlılık kontrolü olarak duruyor.

### 5.3 Neden kumpasla ölçülmüyor

Brif ölçmeyi öneriyordu; kullanıcı "çizimler varken fiziksel ölçüm şart
mı" diye sordu ve vazgeçildi. Hata bütçesi:

| Hata kaynağı | Ayak ucunda etkisi | Nasıl giderilir |
|---|---|---|
| Baskı toleransı | < 0.5 mm | gerek yok |
| Yuvarlak ayak ucunun kayması (~r·sin açı) | 1–3 mm | gerek yok |
| Servo horn'unun mile oturduğu diş (25 diş, 14.4°/diş) | **~10 mm'ye kadar** | **kalibrasyon** |

Doğruluk kalibrasyondan gelir. Robot yürürken ayakta tutarlı bir sapma
görülürse ancak o zaman ölçüme dönülür. **Kullanıcıdan kumpas ölçümü
isteme.**

### 5.4 Koordinat çerçeveleri

- **CAD çerçevesi** (`hexapod-v8.step`): +Y yukarı, X ve Z yatay. Bacak
  azimutu α = atan2(z, x). Bu açı **yukarıdan bakınca saat yönünde** artar.
- **Gövde çerçevesi** (ROS REP-103): +x ileri, +y sol, +z yukarı. Orijin,
  coxa eksenlerinin geçtiği çemberin merkezi, bacak montaj referans
  düzleminin yüksekliğinde. Açılar yukarıdan bakınca **saat yönünün
  tersine** artar.
- **Dönüşüm:** `gövde_azimut = forward_offset_deg − cad_azimut`.
  (İşaret ters, çünkü iki çerçevede açılar zıt yönlerde artıyor. İlk
  yazılan formül `cad − offset` idi ve **yanlıştı**; hiçbir kod
  kullanmadan düzeltildi. Yanlış hâli sol ve sağ bacakları yer
  değiştirirdi.)
- **`forward_offset_deg = 90`**: CAD +Z ileri kabul edildi. **Keyfi**,
  çünkü robot simetrik: altı bacağın geometrisi aynı ve 60° aralıklı.
  Kullanıcı da kamera yerinin fark etmediğini söyledi.
- **Bacak çerçevesi** (IK): orijin coxa'nın dikey ekseni üzerinde, femur
  ekleminin yüksekliğinde; +x bacak dümdüz dışarı (coxa=0 iken), +z yukarı.

### 5.5 Bacak tablosu

| id | CAD az. | Gövde az. | Konum | Fiziksel bant no.* | Aynalı |
|---|---|---|---|---|---|
| 0 | 0° | +90° | sol orta | 6 | hayır |
| 1 | 60° | +30° | sol ön | 1 | hayır |
| 2 | 120° | −30° | sağ ön | 2 | evet |
| 3 | 180° | −90° | sağ orta | 3 | evet |
| 4 | −120° | −150° | sağ arka | 4 | evet |
| 5 | −60° | +150° | sol arka | 5 | hayır |

\* Fiziksel bant no.: robotu kuran kişi iki bacak arasına "ÖN" bandı
yapıştırıp yukarıdan bakarak saat yönünde 1–6 numaralandıracak (1 sol ön,
2 sağ ön, 3 sağ orta, 4 sağ arka, 5 sol arka, 6 sol orta).
`tools/map_channels.py` bu numaraları soruyor.

**Aynalı bacaklar:** kinematik olarak normal bacakla aynıdır (eklem
eksenleri bacak düzleminde aynı yerde). Fark yalnızca servo yön
işaretlerinde olabilir; o da kalibrasyonda eklem eklem belirlenir.
Fiziksel robot başka bir "ön" ile etiketlenirse "aynalı" bilgisi yanlış
tarafı gösterebilir. IK ve kalibrasyon buna dayanmaz; yalnızca
simülasyon modeli dayanır (aynalı bacaklarda kütle dağılımı y'de çevrilir,
bkz. §7.6).

---

## 6. IK ↔ kalibrasyon sözleşmesi

Kalibrasyonda kaydedilen **merkez** (`center_us`) ile IK'daki **0 açısı
aynı duruştur**. Biri değişirse öteki de değişmeli, yoksa her ayak
sistematik olarak kayık basar.

| Eklem | 0 açısı | + yönü |
|---|---|---|
| coxa | bacak gövdeden dümdüz dışarı | yukarıdan bakınca saat yönünün tersi |
| femur | femur yere paralel | bacak yukarı kalkar |
| tibia | tibia femura dik (femur yataysa dümdüz aşağı) | diz açılır, ayak dışarı gider |

Kaynaklar: `src/hexapod_kinematics/hexapod_kinematics/leg.py` modül
açıklaması ve `tools/calibrate.py` yardım metni (ikisi aynı tabloyu
gösterir).

Sıfır duruşunda ayak bacak çerçevesinde **(130, 0, −126.6) mm**'de.
Servo sinyali: `pulse_us = center_us + direction × açı × us_per_deg`.

---

## 7. Yazılım mimarisi ve depo

### 7.1 Temel ilkeler

1. **Eksik değer uydurulmaz.** `robot.yaml`'da her sayı `{value, source,
   measured}` üçlüsü. `value: null` bilinmiyor demektir. Okunmaya çalışılırsa
   `MissingValue` fırlar ve değerin nereden geleceğini söyler. Hiçbir katman
   null'a varsayılan koyamaz.
2. **`measured: false`** = CAD'den ya da veri sayfasından türetildi, robottan
   doğrulanmadı. Çalışır ama "doğru" sayılmaz. Simülasyon tahminleri
   `robot.yaml` → `simulation` altında, "TAHMİN" işaretli.
3. **Donanıma dokunan çekirdek saf Python, ROS'a bağımlı değil.** Paketler
   ROS 2 `ament_python` olarak derlenir ama tezgâh üstü araçlar ve testler ROS
   kurulu olmadan çalışır. Her ROS düğümü aynı desende yazılır:
   - `controller.py`: ROS'suz çekirdek, testli.
   - `node.py`: ince rclpy kabuğu.
4. **Aynı anda tek servo** (kalibrasyon ve kanal haritasında). Brownout
   riskine karşı.
5. **Mutlak darbe sınırı:** `servo.pulse_us_hard_limits` (500–2500 µs)
   dışına hiçbir koşulda darbe gitmez.
6. **Kırpma yok:** IK erişilemeyen hedefi en yakın noktaya kırpmaz,
   `ReachError` fırlatır.
7. **Donanımsız geliştirme:** `DryRunBackend` I2C yazmalarını kaydeder.
   Bütün testler ve `--dry-run` bayrakları bunu kullanır.
8. **Simülasyonla robot aynı arayüzü konuşur** (docs/ARAYUZ.md):
   - `/leg_controller/commands`: 18 değer, radyan.
   - `/joint_states`: gerçek robotta ölçüm değil, son komut.
   - `/imu`.
   Yayınlayan kod (tripod, politika) sim → robot geçişinde değişmez.
9. **Politikanın gözlemi yalnız gerçek robotta da olanlar:**
   - IMU: yerçekimi yönü ve açısal hız,
   - son eklem komutları,
   - hız komutu,
   - adım saati.
   Ayak teması ve ölçülen eklem açısı yalnız ödülde ve değerlendirmede.

### 7.2 Katmanlar

```
Politika düğümü (hexapod_policy)  ← RL politikası (hexapod_rl ile eğitilir)     ✅
Tripod düğümü (hexapod_teleop)    ← tripod çekirdeği (hexapod_gait)             ✅
        │ /leg_controller/commands (docs/ARAYUZ.md)
        ▼
Simülasyon: Gazebo + ros2_control (hexapod_gazebo + hexapod_description)       ✅
Gerçek robot: sürücü düğümü (hexapod_hardware) → ServoBus (hexapod_driver)      ✅ (dry-run)
Ters kinematik + gövde pozu (hexapod_kinematics)                                ✅
```

Her katman altındakine bağımlı, üstündekinden habersiz.

### 7.3 Depo yapısı

```
hexapod/
├── CLAUDE.md               kısa özet + kurallar (Claude Code otomatik okur)
├── README.md               insanlar için genel bakış + çalıştırma adımları
├── GOREVLER.md             görev dağılımı (G/S/D), bağımlılıklar, durum
├── docs/
│   ├── PROJE_DEVIR.md      bu belge
│   ├── ARAYUZ.md           eklem komut arayüzü (konular, sıra, birim, sim-robot farkları)
│   ├── hexapod-proje-brifi.md  ilk brif (bir kısmı geçersiz, bkz. §2)
│   └── malzeme/            malzeme listeleri (güvenilmez, §4.2)
├── config/
│   ├── robot.yaml          robotun fiziksel tanımı + simulation (TAHMİN) bölümü
│   └── calibration.yaml    (henüz yok) calibrate.py üretecek; GİT'TE TUTULMALI
├── src/                    ROS 2 ament_python paketleri
│   ├── hexapod_driver/     servo sürücü katmanı (G1): config, calibration, pca9685, backends, servo_bus
│   ├── hexapod_kinematics/ IK/FK + gövde pozu (G2): leg.py, body.py
│   ├── hexapod_description/ simülasyon modeli (G3, G5): model.py (RobotModel), urdf.py,
│   │                        interface.py (arayüz sabitleri), control.py (ros2_control ayarı),
│   │                        launch/display.launch.py, meshes/
│   ├── hexapod_gazebo/     worlds/flat.sdf, launch/sim.launch.py, pose.py, stand.py (G5)
│   ├── hexapod_gait/       tripod çekirdeği, TripodGait (S2, Samet)
│   ├── hexapod_teleop/     /cmd_vel -> TripodGait -> komut (S3, Samet)
│   ├── hexapod_hardware/   gerçek sürücü düğümü, komut -> ServoBus (S4, Samet)
│   ├── hexapod_rl/         RL (G6, G7): sim, state, task, env, demo, pretrain, train,
│   │                        evaluate, export, baseline, math3d, terrain_probe, widen,
│   │                        robustness, rangefinder, reflex_probe
│   └── hexapod_policy/     politika düğümü (G8): mlp, controller, tripod, lift_reflex, node
├── models/                 kayda değer modeller: model.zip (SB3) + policy.npz (torch'suz) + README (tablolar)
├── egitim_kayitlari/       bütün eğitimlerin progress.csv/ayarlar/değerlendirmeleri (hafif)
├── tools/
│   ├── map_channels.py, calibrate.py, hwcheck.py     donanım araçları (§8)
│   ├── cad_extract.py, cad_sim_model.py, cadlib/     CAD'den geometri ve simülasyon verisi
│   ├── make_urdf.py, preview_urdf.py                 ROS'suz URDF üretimi/önizleme
│   ├── politika_ros_olcum.py                        ROS'lu simde yürüyüş ölçümü (wsl/ betiğiyle)
│   └── wsl/ ros_kurulum.sh, derle.sh, rl_kurulum.sh, politika_ros_olcum.sh
├── tests/                  pytest; ROS/Gazebo/SB3 gerekenler Windows'ta atlanır
├── conftest.py             src/ paketlerini sys.path'e ekler (ROS'suz test için)
├── pytest.ini, .gitignore, .gitattributes
└── cad/                    CAD ve baskı dosyaları (depoda, ~345 MB)
```

### 7.4 `hexapod_driver`

- `RobotConfig.load(path=None)`: `config/robot.yaml`'ı bulur
  (`HEXAPOD_CONFIG_DIR` ortam değişkeni ya da yukarı doğru arama).
  - Alanlar: `segments`, `coxa_axis_radius`, `femur_joint_z_offset`,
    `forward_offset_deg`, `standing_height`, `total_mass_kg` (hepsi `Value`);
    `legs`, `drivers`, `joints`.
  - Yardımcılar: `joint(leg, name)`, `unknowns()`, `unverified()`,
    `wiring_is_complete()`, `wiring_gaps()`.
- `Calibration`: `config/calibration.yaml`.
  - Anahtar `leg{N}_{eklem}`.
  - Alanlar: `center_us`, `direction`, `us_per_deg`, `limit_min_us`,
    `limit_max_us`.
  - `limits_deg()` eksik bilgiyle None döner; `save()` atomik yazar.
- `ServoBus(config, calibration, backend=None, dry_run=False)`:
  - `set_pulse_us`: ham darbe, yalnız kalibrasyon araçları kullanır.
  - `set_angle`: kalibrasyon ya da limit eksikse `MissingValue`/`LimitError`.
  - `set_angles`: hep ya da hiç; Samet ekledi (S4).
  - `release_all`.

### 7.5 `hexapod_kinematics`

- `LegGeometry`, `JointAngles` (derece). `forward`/`inverse` bacak
  çerçevesinde; diz-yukarıda çözüm dalı. Erişim dışında `ReachError`; femur
  açısı (−180, 180]'e sarılır.
- `HexapodKinematics.from_config(config)`: `mounts` (her bacak için
  `LegMount(x, y, z, yaw)`; z = −10.05 mm). `inverse(ayaklar, pose=BodyPose())`
  hata verirse bacak numarasını söyler.
- `BodyPose(x, y, z, roll, pitch, yaw)`: mm ve derece, ZYX (REP-103).

### 7.6 `hexapod_description` (G3, G5)

- `RobotModel.from_config`: SI birimleri. Geometri `HexapodKinematics`'ten,
  kütle/atalet/çarpışma `robot.yaml → simulation.links`'ten (TAHMİN).
  - `limits`: kalibrasyon limitleri varsa onlar, yoksa geçici ±90°.
  - Servo sabitleri: `effort` 1.08 N·m, `velocity` 7.48 rad/s,
    `servo_stiffness` 20 N·m/rad, `servo_damping` 0.05.
- `build_urdf(model, meshes=None, gazebo=None)`:
  - `base_link` ataletsiz; gövde sabit eklemle bağlı.
  - Eklemler `leg{i}_{coxa,femur,tibia}_joint`; coxa ekseni +z,
    femur/tibia ekseni −y.
  - Ayak küresi tibia'nın ilk çarpışması (`leg{i}_tibia_foot`).
  - `GazeboOptions(controllers_yaml, servo="torque"|"velocity")`:
    ros2_control bloğu, IMU ve ayak temas sensörleri.
- `interface.py`: arayüz sabitleri ve dönüşümler. Bkz. docs/ARAYUZ.md.
- `control.py`: kontrolcü ayarı.
  - `servo="torque"` (varsayılan): `leg_controller` →
    `servo_controller` (pid_controller, P = sertlik, çıkış ±durma torku,
    1 kHz) → eklem eforu. Sönüm URDF'te eklem sönümü.
  - `servo="velocity"`: eski konum/hız modeli.

### 7.7 `hexapod_gazebo` (G5)

- `ros2 launch hexapod_gazebo sim.launch.py [gui:=false] [servo:=velocity]
  [world:=...]`.
  - Fizik adımı 1 ms.
  - Köprüler: `/clock`, `/imu`, `/leg{i}/foot_contact`.
- `ros2 run hexapod_gazebo stand`: 100 mm'ye kalkış.
- Ölçülen (tork modeli, düz zemin): tripod 0.08 m/s komutta 0.079 m/s;
  politika 0.10'da 0.096–0.106 m/s.

### 7.8 `hexapod_rl` (G6, G7) — pekiştirmeli öğrenme

**Simülasyon (`sim.py`, `HexapodSim`):** ROS'suz, süreç içi Gazebo (gz.sim
Python bağları, TestFixture).
- Fizik adımı 2 ms, eylem 50 Hz (kontrol adımı başına 10 fizik adımı).
- 8 paralel süreçte eski PC'de saniyede ~500–690 adım; yeni PC'de 16 süreçte
  ~1800 (§0.5).
- **Servo modeli, her fizik adımında Python'da:**
  `tork = Kp·(hedef−konum) − Kd·hız`. Hareket yönünde
  `durma torku·(1−|hız|/yüksüz hız)`, frenlerken durma torkuyla sınırlı.
- Rastgeleleştirme düğmeleri: `set_servo(strength, stiffness)`,
  `latency_steps`, `push(force, seconds)`.
- **Zemin (S5 için):** `terrain_sdf` (düz zemin yerine statik bir `<model>`
  SDF parçası) + `terrain_height(x, y)` (aynı zeminin üst yüzeyinin z'si, m).
  İkisi birlikte verilmek zorunda; yalnız biri verilirse `ValueError`.
  - Robot orijinde, sıfır duruşundaki ayakların altındaki zemine göre doğar
    (`spawn_height`); orijinin z=0'da olması gerekmez.
  - **Ayak teması zemine göre:** ayak küresinin alt ucu altındaki zeminin
    2 mm yakınında mı (dikey). Fizik motorunun temas sensörüyle 0–20° eğimde
    %97.8–98.8 uyuşuyor (§12.26). ~40°'ye kadar geçerli; basamak kenarında
    birkaç mm yanılabilir.
  - `SimState.ground_z`: gövde merkezinin altındaki zemin. Ödülün `height`
    terimi ve devrilme (`< 45 mm`) buna göre.
  - Düz zeminde her şey eskisiyle birebir aynı.
- Her süreç kendi gz-transport bölümünde (`GZ_PARTITION=hexapod_rl_<pid>`,
  §12.22) ve kendi keşif portlarında (`sim.discovery_ports`, ders 39).

**Görev (`task.py`):**

| | |
|---|---|
| Gözlem (29) | yerçekimi yönü (3), açısal hız (3), son eklem hedefleri (18, (hedef−duruş)/0.5), hız komutu (3), adım saati sin/cos (2) |
| Eylem (18) | [-1, 1]. `action_mode="absolute"`: ayakta duruş + 0.5 rad × eylem. `"residual"`: `PhaseTripod(saat, komut)` + 0.2 rad × eylem |
| Duruş | ayak erişimi 130 mm, gövde yüksekliği 100 mm |
| Adım saati | 1.5 Hz; ilk yarıda tripod grubu 0 havada |
| Bölüm | 20 s (+1 s yerleşme); devrilme: gövde < 45 mm ya da > 45° yatık |
| Komut aralığı | Varsayılan: vx 0.05–0.15 m/s, vy = wz = 0 (yalnız ileri). `--omni` (`OMNI_COMMANDS`, 2026-09-26): vx ±0.15, vy ±0.08 m/s, wz ±0.5 rad/s; her bileşen %50 sıfırlanır, aralığın 1/3'ünden küçük komut çekilmez (`sample_command`) |

Ödül (şimdiki: v6, `TaskConfig.w`):
- lin_vel 1.0 × exp(−(hız hatası/0.05)²)
- progress 10 × komut yönündeki hız (komutla kırpılı)
- gait 0.5 × tripod ritmine uyum
- yaw_rate 1.0 × exp(−(dönüş hatası/0.1)²)
- orientation −2, height −20
- power −0.05 × Σ|τ·ω| (W)
- action_rate −0.01, fall −10
- residual −0.5 × ortalama eylem karesi (yalnız artık eylem modunda)

İsteğe bağlı ödül v7 (`--overshoot k`, `TaskConfig.progress_overshoot`;
varsayılan 0 = v6): komutu aşan hız `progress`'ten k katsayısıyla düşülür.
Raporlar bütün modellerde ortak ödülle (`task.standard_reward`, v6)
verilir; hız ve güç ayrıca yazılır (ders 33).

İzleme terimleri gövde hızının 0.5 s'lik üstel ortalamasına bakar
(`VelocityFilter`). Ödülün v1'den v6'ya nasıl ve neden değiştiği `task.py`
modül açıklamasında ve karar günlüğünde (§11).

**Alan rastgeleleştirme (`Randomization`, `--randomize`):** hepsi TAHMİN
aralıklar.
- Durma torku ×0.8–1.1, sertlik ×0.7–1.3.
- Komut gecikmesi 0–18 ms.
- 2–5 s'de bir 0–4 N yatay itme.
- IMU gürültüsü.
- **Gövde kütlesi ×0.9–1.6** (2026-09-26). Bölüm başına değil **ortam
  başına**: kütle dünya kurulurken URDF'e yazılıyor, gz.sim Python'dan
  sonradan değiştirilemiyor. `task.body_mass_scales` aralığı ortamlara eşit
  dağıtır (16 ortamda 0.92…1.58). Gerekçe: gövde CAD tahmini 0.70 kg (tek
  batarya, tek buck varsayımı); faturadaki ikinci batarya, iki buck daha ve
  kapak da üstündeyse +0.38 kg. Robot toplamı 2.06–2.55 kg. Değerlendirme:
  `evaluate --mass 1.4`.
- Sürtünme yok: düz zeminde etkisiz ölçüldü, eğimle (S5) anlamlı.

**Ortam (`env.py`, `HexapodEnv`):** Gymnasium. `make_env(rank, task,
terrain_sdf=..., terrain_height=...)`, `SubprocVecEnv` ile. İsteğe bağlı:
`terrain_levels` (müfredat), `perturbation` (sabit bozulmalar),
`range_sensors`, `lift_reflex`, `range_noise`, `range_drop` (aşağıda).

**Taklit (`demo.py`, `pretrain.py`):** politika PPO'dan önce bir gösterimi
taklit eder.
- Gösterim `PhaseTripod`'dur; artık eylem modunda etiket 0.
- Veri toplanırken uygulanan eyleme gürültü eklenir, etiket gürültüsüzdür.
- Aktör MSE ile, kritik indirimli getirilerle eğitilir; std ayarlanır.
- `--sde`: gSDE; bu kurulumda bozuldu (§12.24).

**Eğitim (`train.py`):**
- `--steps --envs --name --init-from --randomize --residual --omni
  --terrains --curriculum --lift-mm --lift-range --lift-std --reflex
  --range-noise --range-drop --lr --target-kl --std --power-weight
  --overshoot`. Görev ayarı
  bayraklardan `task.task_from_flags` ile kurulur ve çıktı klasörüne
  **`gorev.json`** olarak yazılır; `evaluate`, `export` ve `terrain_probe`
  modelin yanındaki (ara kayıtsa bir üstündeki) bu dosyayı okur, bayrak
  gerekmez. `models/<ad>/gorev.json` de depoda.
- `--terrains AD`: ortam başına zemin (`terrain_probe.TRAIN_SETS`; liste
  ortamlara sırayla dağıtılır, kütle çarpanları karıştırılır). Taklit de
  aynı zeminlerde toplanabilir (`pretrain --terrains`).
- `--curriculum AD` (2026-09-27): kolaydan zora müfredat
  (`terrain_probe.CURRICULA`; `HexapodEnv(terrain_levels=[üreteç, ...])`).
  Ortam başına bir zemin türü ve seviyeleri (ör. çukur 10 → 60 mm); her
  ortam en kolayından başlar. Bölüm sonunda robot doğduğu yerden 0.5 m
  (`CURRICULUM_PROMOTE_M`) uzaklaştıysa bir zorlaşır; devrildiyse ya da
  komutun istediği yolun yarısını gidemediyse bir kolaylaşır (yerinde
  dönüşte karar yok); en zoru geçince rastgele bir seviyeye döner. Zemin
  değişince Gazebo dünyası yeniden kurulur: reset 0.14 s, kurma + reset
  0.22 s. Her süreç kendi gz-transport keşif portlarında
  (`sim.discovery_ports`); ortak portlarla aynı makinedeki öteki Gazebo
  süreçlerine her kurmada ~32 soket + ~2 MB sızıyordu (ders 39). Yalıtılınca
  60 kurmada soket ve bellek sabit. Ortalama seviye `progress.csv`'de
  `mufredat/<tür>`. `--terrains` ile birlikte verilmez.
- `--lift-mm`: artık eylemde taban tripod'un ayak kaldırması (varsayılan 25;
  zeminli eğitimde 50, §3.4).
- `--lift-range EN_AZ EN_COK` (+ `--lift-std`): **öğrenilmiş ayak kaldırma**
  (2026-09-27; `TaskConfig.lift_action`). Eylem 19 boyutlu; son eylem
  [-1, 1] → bu aralıkta kaldırma (`task.lift_from_action`). Yalnız her
  salınımın ilk adımında (`PhaseTripod.swing_group` değişince) seçilir,
  salınım boyunca sabit; reset'te sıfırlanır (ders 36). Gözlem değişmedi:
  politika tutulan kaldırmayı kendi eklem hedeflerinden görüyor. Düzeltme
  cezası yalnız eklem eylemlerine; kaldırmanın bedelini güç cezası öder.
  `--lift-std` yalnız bu boyutun keşif std'si.
- `python -m hexapod_rl.widen MODEL --lift-range 20 60 --lift-std 0.5
  [--lift-start 35] --out W/model.zip`: eğitilmiş bir artık eylem modelinin
  ağına kaldırma çıkışı ekler (sıcak başlangıç; yeni satır 0, sapma
  başlangıç kaldırmasına karşılık gelen eylem; eklemler birebir aynı,
  test). Sonra `train --lift-range 20 60 --init-from W/model.zip`.
- Çıktılar `~/hexapod_runs/<ad>/`: model.zip (son), **best_model.zip**,
  checkpoints/ (250k'da bir), progress.csv, ara_degerlendirme.csv,
  ayarlar.txt, degerlendirme.txt.
- **En iyi ara kayıt otomatik** (2026-09-26): her ara kayıtta politika
  deterministik ölçülür (düz zemin, rastgeleleştirmesiz, 10 s;
  `evaluate.eval_commands`: ileri modelde 0.05/0.10/0.15, her yön modelinde
  yedi komut). Adım başı ödül ortalaması en yüksek olan `best_model.zip`.
  Ölçüm tek bir ortamı yeniden kullanır; sonucu yeni ortamdakiyle aynı
  (test).

**Değerlendirme (`evaluate.py`):**
- `python -m hexapod_rl.evaluate <zip|tripod> [--vx] [--vy] [--wz] [--noise]
  [--randomize] [--residual] [--seed]`.
- Ölçtükleri: hız, yön sapması, adım başı ödül, mekanik güç, devrilme,
  ritim; gövde çerçevesinde ortalama vx/vy ve açılmış dönüş hızı (yana ve
  dönüş komutlarında komutla karşılaştırmak için).
- `evaluate_set(model, komutlar, env=...)`: aynı ortamda birkaç komut.
- `tripod`, Samet'in `TripodGait`'ini ölçer (`baseline.py`).
- Gürültü eylem biriminde: artık eylemde aynı eklem gürültüsü için ×2.5
  (0.1 mutlak = 0.25 artık = 0.05 rad).

**Mesafe sensörü (DENEYSEL, 2026-09-27):** `HexapodEnv(range_sensors=[...])`
her adımda `info["ranges_m"]` verir (gözlem değişmez): `rangefinder.read`,
gövdeye bağlı ideal ışın zemin yüksekliği fonksiyonuna çarpana kadar
ilerler (5 mm adım + ikiye bölme). `RangeSensor` (yer, bakış yönü, menzil)
ve `obstacle_height` robotta da çalışan `hexapod_policy.lift_reflex`'te;
yerleşim her kullanımda açıkça verilir, robot.yaml'dan okunmaz ve
varsayılanı yok (robot.yaml'da null, D8). Deneme: `python -m
hexapod_rl.reflex_probe [--pitch ...] [--noise %] [--drop %] [--every N]`.
`HexapodEnv(lift_reflex=LiftReflex(), range_noise=, range_drop=)` kaldırmayı
robottaki denetleyiciyle aynı kuralla refleksle seçer; eğitimde `train.py
--reflex AÇI [--range-noise %] [--range-drop %]` (yerleşim ileri + ±90°,
ara kayıt seçimi de refleksle).

**Sabit bozulmalar ve dayanıklılık taraması (2026-09-27):**
- `task.Perturbation` (`HexapodEnv(perturbation=...)`, `evaluate(...,
  perturbation=...)`): eğitimde rastgeleleştirilmeyen, robota geçişte
  beklenen hatalar. `joint_offset_deg` (18 eklem): servo sıfırının
  kalibrasyon hatası, eklem komut + ofset'e gider (`HexapodSim.joint_offset`),
  gözlem komutu görür. `imu_tilt_deg` (roll, pitch): gözlemdeki yerçekimi ve
  jiroskop döner. `delay_ms`: kontrol adımından uzun olabilen gecikme (tam
  adımlar env kuyruğunda, kalanı `sim.latency_steps`); gözlem son komutu
  görür (denetleyici gibi). `servo_strength`: durma torku çarpanı.
  Varsayılan `Perturbation()` ortamı birebir aynı bırakır (eski koda göre
  aynı yörünge özeti doğrulandı).
- `python -m hexapod_rl.robustness tripod models/<ad>/model.zip ...`: her
  model × (düz, basamak 45, engebe 40) × bozulma, 0.1 m/s, deterministik,
  rastgeleleştirme kapalı; ofset ve IMU yönü tohumlu. Değerler ölçüm değil,
  taranan büyüklükler. Sonuç tabloları models/README ("Dayanıklılık
  taraması"), özet ders 37.

**Aktarma (`export.py`):** `python -m hexapod_rl.export <zip> [--residual]
[--omni]` → `policy.npz` (aktör ağı + sözleşme). Aktarım SB3 çıktısıyla
karşılaştırılarak doğrulanır. `--omni`'de sözleşmeye ölü bölge
(`command_deadband` = 1/6) yazılır: komut bunun altındaysa düğüm ayakta
bekler.

**Depodaki modellerin tarifleri** (yeni PC, 16 ortam; çıktılar
`~/hexapod_runs/<ad>/`, hafif kayıtları `egitim_kayitlari/<ad>/`):

`ppo_omni_250k` — taklit + her yöne PPO, en iyi ara kayıt 250k (2.1M'de
durduruldu; bu eğitimde kütle rastgeleleştirmesi henüz yoktu):

```bash
python -m hexapod_rl.pretrain --name bc_omni --episodes 128 --workers 16 --noise 0.25 --std 0.15 --randomize --residual --omni
```

```bash
python -m hexapod_rl.train --steps 5000000 --envs 16 --name v10_omni --randomize --residual --omni --lr 1e-4 --target-kl 0.02 --init-from ~/hexapod_runs/bc_omni/model.zip
```

`ppo_lift50_3750k` — taban 50 mm, deneme zeminleri: taklit (`bc_lift50`) +
3M PPO (`v13_lift50`, 2.25M ara kaydı eski `ppo_lift50_2250k`), ondan std
0.05 ile 2M (`v14_lift50_std05`), 1.5M ara kaydı (toplam 3.75M):

```bash
python -m hexapod_rl.pretrain --name bc_lift50 --episodes 128 --workers 16 --noise 0.25 --std 0.1 --randomize --residual --omni --lift-mm 50 --terrains deneme
```

```bash
python -m hexapod_rl.train --steps 3000000 --envs 16 --name v13_lift50 --randomize --residual --omni --lift-mm 50 --terrains deneme --lr 1e-4 --target-kl 0.02 --std 0.1 --init-from ~/hexapod_runs/bc_lift50/model.zip
```

```bash
python -m hexapod_rl.train --steps 2000000 --envs 16 --name v14_lift50_std05 --randomize --residual --omni --lift-mm 50 --terrains deneme --lr 1e-4 --target-kl 0.02 --std 0.05 --init-from ~/hexapod_runs/v13_lift50/checkpoints/ppo_2250000_steps.zip
```

`ppo_kaldirma35_250k` — `ppo_omni_250k`'ya kaldırma çıkışı eklenir (35
mm'den başlar), deneme zeminlerinde 4M (`v21_kaldirma35`), 250k ara kaydı:

```bash
python -m hexapod_rl.widen models/ppo_omni_250k/model.zip --lift-range 20 60 --lift-std 0.5 --lift-start 35 --out ~/hexapod_runs/w_kaldirma35/model.zip
```

```bash
python -m hexapod_rl.train --steps 4000000 --envs 16 --name v21_kaldirma35 --randomize --residual --omni --lift-range 20 60 --terrains deneme --lr 1e-4 --target-kl 0.03 --std 0.1 --lift-std 0.5 --init-from ~/hexapod_runs/w_kaldirma35/model.zip
```

`ppo_refleks_1500k` — `ppo_kaldirma35_250k`'dan refleks açıkken 3M
(`v24_refleks`), en iyi ara kayıt 1.5M (refleksli ölçüm):

```bash
python -m hexapod_rl.train --steps 3000000 --envs 16 --name v24_refleks --randomize --residual --omni --lift-range 20 60 --terrains deneme --reflex 20 --range-noise 5 --range-drop 10 --lr 1e-4 --target-kl 0.03 --std 0.1 --lift-std 0.3 --init-from models/ppo_kaldirma35_250k/model.zip
```

Aktarma: `python -m hexapod_rl.export models/<ad>/model.zip` (görev ayarı
`gorev.json`'dan).

### 7.9 `hexapod_policy` (G8) — politika düğümü

- `ros2 run hexapod_policy policy --ros-args -p policy:=models/ppo_omni_250k/policy.npz [-p use_sim_time:=true]`
  (hangi model ne için: §3.1).
- Dinlediği ve yayınladığı: `/imu` + `/cmd_vel` → `/leg_controller/commands`,
  50 Hz.
- **Torch'suz:** `mlp.py` numpy ile MLP çalıştırır. `.npz` politikanın
  eğitildiği sözleşmeyi taşır: eylem ölçeği, varsayılan duruş, adım saati,
  komut hızı, komut aralıkları, eylem modu, artık eylem tabanı. Eski
  dosyalar mutlak modda yüklenir.
- `controller.py`: gözlemi eğitimdekiyle birebir kurar (testle
  karşılaştırılıyor).
  - Güvenlik: şu durumlarda politika koşmaz, ayakta duruş yayınlanır ve adım
    saati sıfırlanır:
    - komut yok ya da 0.5 s zaman aşımı,
    - dur komutu: yalnız ileri eğitilmiş dosyada vx eğitim aralığının
      alt ucunun yarısının altında; her yöne eğitilmiş dosyada komut
      sözleşmenin ölü bölgesinde (`command_deadband`),
    - IMU yok ya da 0.2 s bayat,
    - gövde 45°'den fazla yatık.
  - Aralık dışı komut kırpılır.
- `tripod.py`, `PhaseTripod`: adım saatinin fonksiyonu olan tripod. Eğitim ve
  robot aynı kodu kullanır.
- **Öğrenilmiş ayak kaldırma** (`PolicyContract.lift_range`, 2026-09-27):
  alan varsa ağın 19. çıkışı eğitimdeki eşlemeyle kaldırmaya çevrilir;
  eğitimdeki gibi yalnız salınımın ilk adımında, beklemeden sonra ilk
  adımda yeniden seçilir (`ctl.lift_mm`). Eski dosyalarda alan yok,
  davranış aynı. ROS'lu simde `ppo_kaldirma35_250k` ile doğrulandı (§3.5).
- **Mesafe sensörlü kaldırma refleksi** (2026-09-27): `PolicyController(...,
  range_sensors=[RangeSensor...], reflex=LiftReflex())` + `on_ranges(mesafeler,
  now)`. Her salınımın başında kaldırmayı refleks seçer (yalnız yeni ölçümle
  güncellenir; sensör yavaşsa aynı ölçüm art arda sayılmaz). Mesafe yok ya
  da `range_timeout_s` (0.2 s) bayatsa kör davranışa döner: kaldırma çıkışı
  varsa onun, yoksa tabanın kaldırması (`reflex_status`). Yalnız artık eylem
  modunda. Yürüyüş yönüne hiçbir sensör bakmıyorsa (±45°, `covers`) de
  refleks devre dışı ("yön görülmüyor"). Kapalı döngü (numpy politika + denetleyici + süreç içi Gazebo,
  ROS'suz, `ppo_kaldirma35_250k`, 10 s): düzde refleks 25 mm (politika 35),
  ikisi ~1.0 m; 45 mm basamak politika 0.70 m, refleks 0.94 m; 60 mm
  basamak politika 0.07 m takılı, refleks 0.48 m (ön kısmı üstte).
  **Düğüme bağlanmadı:** mesafe konuları S7'nin, yerleşim D8'in
  (robot.yaml'da alan yok: `direction_deg` dışında yer ve aşağı açı
  gerekecek); o zaman `node.py` konulara abone olup `on_ranges`'i çağırır.
- Çıkarım: PC'de tick başına 44 µs; Pi 4 için <0.5 ms tahmini.
- **ROS'lu simde her yön politikası** (`ppo_omni_250k`, 2026-09-26; tork
  servo modeli, hız gz poz yayınından sim zamanıyla, hareketin orta %80'i,
  gövde çerçevesinde): ileri 0.10 → %101, geri %100, yana ±0.06 → %101–104,
  dönüş ±0.4 rad/s → %99–100, karışık (0.1, 0.04, 0.25) → %100/%104/%96;
  gövde 98 mm; sıfır komutta "dur (komut ölü bölgede)", hareket 0.0 mm.
  Ölçüm aracı: `bash tools/wsl/politika_ros_olcum.sh [policy.npz]` (sim +
  düğüm + ölçüm + kapatma, kendi ROS_DOMAIN_ID/GZ_PARTITION'ı ile, ~2 dk).
- **ROS'lu simde zemin** (2026-09-26): deneme zemini dünya dosyasına gömülür
  (`terrain_probe.world_sdf`; ROS'lu sim düz zemine göre doğurduğu için
  orijini z=0'da olanlar: basamak, çukur). `ppo_lift50_3750k` 12 s'de 45 mm
  basamakta 1.14 m (üstte), 60 mm basamakta 0.96 m, 45 mm çukurdan geri
  1.17 m; `ppo_omni_250k` üçünde de takılıyor. Kullanım:
  `WORLD=/yol/dunya.sdf WORLD_NAME=zemin KOMUTLAR="0.1,0,0" SURE=12 bash
  tools/wsl/politika_ros_olcum.sh models/ppo_lift50_3750k/policy.npz`.

### 7.10 Testler

```bash
python -m pytest -q          # depo kökünden
```

- Windows: 235 geçti, 9 atlandı, ~3 s.
- WSL (ROS + venv kaynaklı): 286 geçti, ~40 s (yeni PC).

Öne çıkanlar:
- Eksik değerde `MissingValue`.
- IK gidiş-dönüş.
- URDF ↔ IK ayak konumları.
- Gazebo'da ayakta duruş ve sıfırlama.
- `test_simulasyon_yurumeye_izin_veriyor`: elle tripod beklenen hızın
  %70'inden hızlı olmalı; servo modeli bozulursa yakalar.
- Ödül sürümlerinin istenen davranışı.
- Rastgeleleştirme düğmeleri.
- Gözlem sözleşmesi (düğüm = eğitim).
- Uçtan uca: export → düğüm çekirdeği → Gazebo'da yürüyüş (mutlak ve artık
  eylem); refleksli denetleyici 45 mm basamağı çıkıyor.
- Sabit bozulmalar, müfredat seviyesi, dünya yeniden kurulurken soket
  sızıntısı (öteki süreç gerçekten başlatılıyor), mesafe sensörü ve refleks.
- ROS düğümleri gerçek süreç olarak: SIGTERM'de çıkış 0, eksik dosyada çıkış 2.

---

## 8. Araçlar

Hepsi `python tools/<araç>.py` ile çalışır. Hepsinde `--dry-run` var
(donanıma yazmaz). Windows konsolunda Türkçe karakter bozulmasın diye
stdout'u UTF-8'e zorlarlar.

### `map_channels.py` — kanal haritası
Servo kartlarını I2C'de kendisi bulur (0x70 hariç), her kanaldaki servoyu
sırayla ±100 µs kıpırdatır; kullanıcı kıpırdayan eklemi yazar:
`1c` = bacak 1 coxa, `3f` = femur, `6t` = tibia, Enter = boş kanal,
`t` = tekrar, `q` = bitir. Sonunda haritayı basar. Kabloların hangi
sırayla takıldığı önemli değil. Robot kutu üstünde, bacaklar havada
olmalı (servo ilk sinyalde orta konuma zıplar). Tek kart bulursa
çalışmayı reddeder (iki kart aynı adresteyse harita yanlış çıkar).
**Çıktısı `robot.yaml`'a henüz otomatik yazılmıyor**; çıktı geldiğinde
bacak numaraları §5.5 tablosuyla id'lere çevrilip elle girilecek.

### `calibrate.py` — kalibrasyon
Kablolama (`drivers[*].address`, `joints[*].driver/channel`) dolu
değilse başlamaz ve eksikleri listeler. Aynı anda tek servo beslenir.
Başlangıç darbesi 1500 µs (RC nötrü, robot parametresi değil).
Komutlar: `+`/`-`, `+N`/`-N`, `=N`, `step N`, `c` (merkez = sıfır duruşu,
kaydedip sıradakine geçer), `dir +`/`dir -`, `off`, `span D` (şu anki
konum merkezden D derece → us/derece), `limit min`/`limit max`,
`n`/`p`/`go L J`, `list`, `gaps`, `limits` (robot.yaml formatında
basar), `save`, `q`, `?`.
Limit bulma sırası: `c` → `dir` → `span` → `limit min`/`limit max` →
`limits` çıktısını robot.yaml'a yapıştır.

### `hwcheck.py` — I2C kontrolü
Veri yolunu tarar, bilinen adresleri tahmin eder (BNO055 0x28/0x29,
VL53L0X 0x29, BMP280 0x76/0x77, PCA9685 0x40–0x7F; 0x70 ALLCALL olarak
etiketlenir), robot.yaml ile karşılaştırır, adres önerir. İki kart aynı
adresteyse A0'ın lehimlenmesini söyler.

### `cad_extract.py` — geometri türetme
`--leg-only`, `--body-only`, `--verbose`. CAD'i `cad/Hexapod/` (ya da eski
`Hexapod/`, `Kerem Baltacı/Hexapod/`) altında arar. Çıktı: coxa 50.000, femur
80.000, tibia 126.635 mm, altı bacakta coxa yarıçapı 100.0; iki
tutarlılık kontrolü. Tam montaj çözümlemesi (85 MB STEP) ~5 sn.

### `cad_sim_model.py` — simülasyon verisi
Basılan parçaların STL'lerinden link başına kütle, ağırlık merkezi,
atalet ve çarpışma kutularını hesaplar; `robot.yaml`'a yapıştırılacak
`simulation.links` bloğunu basar, `meshes.yaml`'ı yazar. `--copy-meshes`
STL'leri pakete kopyalar. Girdiler `simulation.mass_inputs`'ta (PETG
yoğunluğu, doluluk oranı, servo kütlesi/boyutu, elektronik kütlesi).
Kontroller: segment uzunlukları cad_extract ile aynı çıkmalı; CAD coxa
eksenleri robot.yaml montajlarıyla 0.000 mm örtüşmeli. Parça→link
ataması ve gerekçesi aracın başındaki açıklamada (coxa servosunun
gövdesi bacakla döner; Coxa_top gövdeye bağlı). ~8 sn.

Her iki CAD aracı da ortak kodu `tools/cadlib/`'den alır (STEP
ayrıştırma, montaj ağacı, dönüşümler, STL/kütle, çerçeveler).

### `make_urdf.py` ve `preview_urdf.py` — URDF
`make_urdf.py` robot.yaml'dan URDF yazar (ROS'suz). `preview_urdf.py`
URDF'i okuyup STL'leri link konumlarına yerleştirerek PNG çizer (numpy +
matplotlib; tek görünüş ~1 dk, `--collision` ~1 sn). RViz'in yerini
tutmaz; mesh yerleşimini gözle kontrol etmek için.

### `wsl/ros_kurulum.sh` — ROS 2 kurulumu (G4)
WSL Ubuntu 26.04'e ROS 2 Lyrical + Gazebo + ros2_control + RViz eklem
kaydırıcılarını kurar. Adımlar resmi belgeden (ros2_documentation,
`lyrical` dalı, Ubuntu-Install-Debs.rst; docs.ros.org bot korumasıyla
erişimi engelliyor). Paketlerin resolute deposunda var olduğu doğrulandı.
sudo'yu bir kez sorar, tekrar çalıştırmak güvenli, sonunda kendini test
eder (talker → /chatter, `gz sim --version`). Günlük: /tmp/ros_kurulum.log.

### `wsl/derle.sh` — paketleri derle
Depodaki her ROS paketini `~/hexapod_ws/src`'ye sembolik bağlar, kırık
bağlantıları temizler, `colcon build --symlink-install`. Ek argümanlar
colcon'a gider (ör. `--packages-select hexapod_policy`). Yeni paket eklenince
tekrar çalıştır. Python dosyası değişikliği yeniden derleme istemez.

### `wsl/rl_kurulum.sh` — RL ortamı
`~/hexapod_venv` (sistem paketlerini görür, ROS'un Python'u ve gz.sim
bağları için): torch 2.14.0 CPU, stable-baselines3 2.9.0, gymnasium 1.3.0.
`python3-venv` yoksa get-pip ile sudo'suz kurar. GPU sürümü için §0.3.

### `wsl/politika_ros_olcum.sh` + `politika_ros_olcum.py` — ROS'lu simde ölçüm
`bash tools/wsl/politika_ros_olcum.sh [policy.npz]` (varsayılan
`models/ppo_omni_250k`). `sim.launch.py gui:=false` + politika düğümü açar,
kontrolcüleri bekler, ileri/geri/iki yana/iki dönüş/karışık/sıfır komutu
8'er sim saniyesi verir, gövde hızını gz poz yayınından sim zamanıyla ölçer
(ders 23; dönerken de gövde çerçevesinde doğru), Markdown tablosu basar ve
her şeyi kapatır. Her koşu kendi (rastgele) `ROS_DOMAIN_ID`'sinde ve
`GZ_PARTITION`'ında; sim ve düğüm kendi süreç gruplarında (`setsid`),
kapatırken bütün grup öldürülür. (İlk sürüm sabit domain 57 kullanıyordu:
arka arkaya koşularda kapanmakta olan eski `controller_manager` "aktif"
cevabı verip yeni sim hazır olmadan ölçüm başlıyordu.) Ortam değişkenleri:
`WORLD`, `WORLD_NAME`, `KOMUTLAR` ("vx,vy,wz;..."), `SURE`. Python
tarafı çıkışta `os._exit` kullanır: gz.transport + rclpy birlikte kapanırken
segfault veriyor (ölçüm bittikten sonra).

---

## 9. Çalışma ortamı

| | Nerede | Sistem |
|---|---|---|
| Geliştirme + simülasyon + RL eğitimi | Kullanıcının PC'si, **WSL2** | **Ubuntu 26.04** |
| Robot | Raspberry Pi 4 | **Ubuntu Server 26.04 arm64** (planlanan) |

**Bilgisayarlar:**
- **Eski PC** (1–3. oturum): Windows 11 Pro, i5-10300H (8 iş parçacığı),
  16 GB RAM (WSL'e 7 GB), GTX 1650.
- **Yeni PC** (2026-09-26'dan sonra): Windows 11 Pro, **AMD Ryzen 7 7700X**
  (8 çekirdek / 16 iş parçacığı, masaüstü, Zen 4), 16 GB RAM (WSL'e 7.3 GB),
  RTX 5070 12 GB. Kurulum §0.2'de, sonucu ve hız ölçümü §0.5'te.

**Yazılım:**
- **ROS 2: Lyrical Luth** (LTS, Mayıs 2031'e kadar), Ubuntu 26.04'ün birincil
  sürümü. **Gazebo Jetty (10.5)** `ros-lyrical-desktop` ile geliyor. Python 3.14.
- Kurulum betikleri: `tools/wsl/ros_kurulum.sh` (sudo),
  `tools/wsl/derle.sh`, `tools/wsl/rl_kurulum.sh`.
- **RL sanal ortamı** `~/hexapod_venv`: torch 2.14 CPU, SB3 2.9.0,
  Gymnasium 1.3.0. `python3-venv` yoksa betik get-pip ile sudo'suz kurar.
- Her yeni terminalde:
  ```bash
  source /opt/ros/lyrical/setup.bash; source ~/hexapod_ws/install/setup.bash; source ~/hexapod_venv/bin/activate
  ```
- Eğitim çıktıları `~/hexapod_runs/<ad>/` (depoda değil; hafif kısmı
  `egitim_kayitlari/`'na kopyalanır).
- Paketler `bash tools/wsl/derle.sh` ile `~/hexapod_ws`'te derlenir.
  Kaynaklar depoya sembolik bağlı (`--symlink-install`); yeni paket
  eklenince betiği tekrar çalıştır.
- **colcon derlemesini OneDrive ya da depo klasöründe yapma:** `build/`,
  `install/`, `log/` senkronlanır ve /mnt/c yavaştır.

**Claude'un WSL'de çalışma notları:**
- sudo gerektirmeyen her şeyi çalıştırabilirsin: `wsl -e bash <betik>`.
  PowerShell'den ver (Git Bash'ten `wsl -e bash /mnt/...` yolu Windows yoluna
  çevrilip bozuluyor). Git Bash'ten çalışan biçim:
  `wsl -d Ubuntu-26.04 -e bash -c "bash /mnt/c/.../betik.sh"` (yol tırnak
  içinde çevrilmiyor).
- Tırnaklı uzun komutlar PowerShell → wsl geçişinde bozuluyor. Betiği
  scratchpad'e **Write aracıyla** yaz, sonra çalıştır. Uzun heredoc'lar da
  Bash aracında bozuluyor (§12.12). Python ile dosya yamasında
  `write_bytes(s.encode())` kullan; `write_text` Windows'ta CRLF'e çevirir.
- Aynı anda birden fazla ROS'lu simülasyon ya da test koşacaksa:
  - farklı `ROS_DOMAIN_ID`,
  - ROS'lu sim eğitimle aynı anda koşacaksa ayrı bir `GZ_PARTITION`
    (§12.22).
- `pkill -f <desen>` kendi kabuğunu öldürebilir. Desenler bir betik
  dosyasında olsun ya da `pgrep` sonucundan kendi PID'ini çıkar.
- WSL, içinde süreç kalmayınca bir dakika sonra kendini kapatır; eğitim
  sürerken açık kalır.

**Uzun eğitim:** bilgisayar uyumasın diye eğitimi, Windows'ta
`SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED)` tutan bir
PowerShell sarmalayıcısıyla başlat. Arka plan görevi bitince bildirim gelir.
Ara kontrol için `progress.csv`'yi sütun **adıyla** oku; sütun sırası
eğitimden eğitime değişiyor.

```powershell
param([string]$Script)
Add-Type -Namespace W -Name P -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint f);'
[void][W.P]::SetThreadExecutionState([uint32]"0x80000001")
try { wsl -e bash $Script } finally { [void][W.P]::SetThreadExecutionState([uint32]"0x80000000") }
```

**Diğer:**
- **WSL'de `sudo` şifre ister.** Kurulum komutlarını kullanıcıya ver,
  şifreyi asla sen girme.
- **Ubuntu 24.04+ sistem Python'una `pip install` engelli** (PEP 668).
  `apt` ya da venv kullan.
- **Pi'de I2C:** Ubuntu'da genelde açık gelir (`ls /dev/i2c-1`); kullanıcı
  `i2c` grubuna eklenmeli.
- **Satır sonları:** `.gitattributes` `eol=lf` zorluyor. Windows'ta CRLF ile
  commit'lenen bir betiğin shebang'i Linux'ta `python3\r` olur ve "bad
  interpreter" verir.
- **Kodlama:** Windows konsolu cp1254/cp857. Türkçe çıktı basan betiklerde
  `sys.stdout.reconfigure(encoding="utf-8")` kullanılıyor.

---

## 10. Git, güvenlik ve kişisel veri

- **Faturalar kesinlikle depoya girmez.** İçlerinde TCKN, ev adresi,
  telefon ve e-posta var (üçüncü kişilerin). GitHub'a giden kişisel veri
  fork/cache/indeks yüzünden geri alınamaz.
- `.gitignore` bunu **klasör adından bağımsız** koruyor: bütün `*.pdf`,
  `Ekran görüntüsü*`, `Screenshot*`, `yavuz selim/` dışarıda. Tek istisna
  `!cad/Hexapod/606030-3d-printed-hexapod-*.pdf`. **Bu kuralı gevşetme.**
  Paylaşılabilir bir PDF eklenecekse açık bir `!` istisnası yaz ve önce
  içinde kişisel veri olmadığından emin ol.
- **CAD binary'leri** (`*.step`, `*.f3z`, `*.3mf`, `*.stl`, toplam 345 MB)
  **2026-09-24'ten beri depoda** (`cad/`). Depo gizli olduğu ve iki kişi
  aynı dosyalarla çalışacağı için kullanıcı istedi. En büyük iki STEP 85 ve
  67 MB: GitHub 50 MB'ın üstünü uyarıyla kabul ediyor, 100 MB'ı reddediyor.
  Depo herkese açılacaksa (TÜBİTAK: açık kaynak) lisans (CC BY-SA 4.0)
  uygun ama Git LFS düşünülmeli. Faturalar İSE HÂLÂ depoya girmez.
- **`config/calibration.yaml` git'te TUTULMALI** (brifteki risk:
  kalibrasyon kayıt altına alınmazsa her seferinde sıfırdan başlanır).
  Yalnızca `.tmp` hâli yoksayılıyor.
- Push öncesi alışkanlık: `git diff --cached` ile eklenecek dosyaları ve
  boyutlarını gözden geçir; TCKN/telefon deseni tara; commit'lerde imza
  satırı olmadığını kontrol et.
- Commit stili: `tür(kapsam): özet` + Türkçe gövde. Türler: `feat`,
  `fix`, `config`, `docs`, `test`, `chore`.

---

## 11. Karar günlüğü

| Tarih | Karar | Gerekçe |
|---|---|---|
| 09-23 | 3 DOF, coxa–femur–tibia | 18 servo horn (liste), bacak CAD'inde 3 servo, faturada 20 MG996R |
| 09-23 | Servo arayüzü: PCA9685 ×2, PWM | Faturalar; 18 servo > 16 kanal |
| 09-23 | Mimari: ROS 2 + Gazebo + RL (TÜBİTAK başvurusu), brifteki düz Python değil | Kullanıcı "TÜBİTAK başvurusunda ne diyorsa onu yap" dedi |
| 09-23 | robot.yaml CAD değerleriyle şimdi yazılsın, `measured: false` | Kullanıcı seçti; şema sabit kalır, sayılar sonra değişir |
| 09-23 | Donanım çekirdeği ROS'tan bağımsız saf Python | Kalibrasyon ROS ayağa kaldırmadan yapılabilsin |
| 09-23 | Faturalar ve CAD binary'leri depoya alınmaz | Kişisel veri; 344 MB |
| 09-23 | Satır sonları LF | Pi'de shebang kırılması |
| 09-24 | **Kumpas ölçümü yok, geometri CAD'den** | Hata bütçesi (§5.3); kullanıcı sordu |
| 09-24 | tibia = 126.6 mm (121 değil) | STL'den doğru parça ölçüldü (§12) |
| 09-24 | **Ön keyfi**, forward_offset = 90 | Robot simetrik; kullanıcı "kamera yeri fark etmez" dedi |
| 09-24 | Kanal haritası elle değil, araçla | Kullanıcı donanımda deneyimsiz |
| 09-24 | **Önce yazılım/simülasyon, donanım işi kullanıcıdan istenmez** | Kullanıcı robotu kurmadı; "yazılımı elimizdeki bilgilerle yapamaz mıyız" |
| 09-24 | IK'ya başlandı ve bitti | Kullanıcının yukarıdaki cümlesi onay sayıldı |
| 09-24 | ROS 2 Lyrical + Gazebo Jetty, Ubuntu 26.04 | PC'deki WSL Ubuntu 26.04; LTS 2031'e kadar; Pi'de de aynı sistem |
| 09-24 | Klasörler düzleştirildi (`Kerem Baltacı/`, `yavuz selim/` kalktı) | Kullanıcı yaptı; ezilen liste geri kondu, alternatif ayrı adla saklandı |
| 09-24 | .gitignore desen tabanlı (tüm PDF'ler) | Klasör kalkınca eski koruma boşa düşmüştü |
| 09-24 | Simülasyon kütle/atalet CAD'den, `robot.yaml` → `simulation` altında, hepsi TAHMİN işaretli | Robot tartılmadı; uydurmak yerine STL hacmi × PETG × doluluk (0.6) + katalog; araçla tekrar üretilebilir |
| 09-24 | Geçici eklem limitleri ±90° (yalnız simülasyon) | Servo aralığının yarısı; kalibrasyon limitleri gelince otomatik onlar kullanılır |
| 09-24 | Gövde kapağı kütleye dahil değil (~43 g) | STL baskı tablası konumunda, CAD'deki yeri bilinmiyor; yer tahmin edilmedi |
| 09-24 | Tibia çarpışması iki kutu + ayak küresi (r = 5.1 mm) | Tek kutu, tibia ~25°'den fazla eğilince ayaktan önce yere değiyordu |
| 09-24 | Aynalı bacaklarda simülasyon kütlesi y'de çevrilir; görsel mesh çevrilmez | URDF mesh yansıtamaz; fark yalnız görünüşte |
| 09-24 | Klasör düzeni: `docs/` (devir, brif, `malzeme/`), `cad/` (yerel CAD), `tools/cadlib/` | Kullanıcı istedi: kök dağınıktı, kütüphane kodu script'lerin içindeydi |
| 09-24 | Görev dağılımı GOREVLER.md'de, bağımlılıklarla (ilk hâli: Görkem yazılım, Samet donanım) | Kullanıcı istedi |
| 09-24 | **Donanım durduruldu, ayrı vardiyaya (D1–D12) taşındı; yazılım Görkem (G) ve Samet (S) arasında bölüşüldü** | Kullanıcı istedi. Vardiya için önerilen başlama şartı: S3 (tripod sim) + S4 (sürücü düğümü) |
| 09-24 | CAD (~345 MB) ve mesh'ler depoya alındı; faturalar hâlâ dışarıda | Depo gizli, iki kişi aynı dosyalarla çalışacak; kullanıcı istedi |
| 09-24 | **Eklem komut arayüzü:** `/leg_controller/commands`, Float64MultiArray, 18 değer, radyan, bacak bacak coxa-femur-tibia (docs/ARAYUZ.md) | ros2_control'ün standart ForwardCommandController'ı; gerçek sürücü (S4) aynı konuyu dinlerse sim → robot geçişinde yayınlayan kod değişmez |
| 09-24 | Ayak temas sensörü yalnız simülasyonda; politika gözlemine girmez | Gerçek robotta yok; sim-to-real'de olmayan bilgiye dayanmasın |
| 09-24 | `/joint_states` gerçek robotta ölçüm değil son komut | MG996R geri bildirim vermiyor; politika da simde komut edilen açıyı görmeli |
| 09-24 | Simde servo = birinci derece sistem, T = 0.05 s (TAHMİN) | gz_ros2_control konum komutunu hız kontrolüyle uyguluyor; kazanç = 1/(T x 100 Hz) = 0.2 |
| 09-24 | **RL simülasyonu ROS'suz, süreç içi Gazebo (`hexapod_rl.sim`)** | ROS'lu sim ~1.3x; süreç içi 2 ms adım 4.5x/süreç, 8 süreç ~21x. Aynı URDF, fizik ve servo modeli |
| 09-24 | RL fizik adımı 2 ms (ROS simi 1 ms) | 1 ms 1.8x, 2 ms 4.5x, 4 ms 7.7x; üçünde de robot 100.0 mm'de duruyor. Yürüyüşte temas doğruluğu için 2 ms; yürüyüş gelince tekrar bakılacak |
| 09-24 | PyTorch CPU sürümü (venv'de) | Politika küçük MLP, SB3 PPO için CPU öneriyor; CUDA sürümü GB'larca, CPU 196 MB |
| 09-24 | RL gözlemi yalnız gerçek robotta da olanlar: IMU (yerçekimi yönü, açısal hız), son eklem komutları, hız komutu, adım saati | Sim-to-real: ölçülen açı ve ayak teması gerçekte yok; ödülde kullanılabilir, gözlemde değil |
| 09-24 | RL eylemi: ayakta duruş + 0.5 rad x [-1,1], limitlere kırpılır | Politika sıfırdan değil, dengeli bir duruştan başlasın |
| 09-24 | İlk PPO (1M adım, 34 dk): robot yürümedi (10 s'de -1.5 cm), devrilmedi; ödül 550 → 810 | Ödül yerinde durmayı fazla ödüllendiriyor. Kullanıcı: ödülü düzelt + uzun eğitim, S3 gelince tripod üstüne öğrenme |
| 09-25 | Ödül v2 ile 10M eğitim 5.75M'de DURDURULDU | Robot ritmi öğrendi (%93) ama yerinde saydı (10 s'de 0.7 cm). Sebep servo modeli çıktı (alttaki satır); o fizikle eğitmek boşa |
| 09-25 | **RL simi tork tabanlı servo: tork = Kp·hata − Kd·hız, DC motor tork-hız doğrusuyla sınırlı (Kp 20, Kd 0.05, TAHMİN)** | Hız komutlu model (gz_ros2_control'ünki) ile elle yazılmış tripod bile beklenenin %12'siyle yürüyordu (ayaklar kayıyor); tork modeliyle aynı 1.08 N·m'de %94–97. Hareketsiz tripod iki modelde de 0.6 N·m'de sağlam: sorun dinamik |
| 09-25 | Enerji cezası artık gerçek mekanik güç Σ\|τ·ω\| (W) | Tork modeliyle tork biliniyor; TÜBİTAK'taki "en az enerji" tanımına uygun |
| 09-25 | **İlk yürüyen politika** (tork modeli + ödül v2, 10M adım, 8.1 sa): ~0.087 m/s, hiç devrilmiyor; hız komutunu yok sayıyor, ~12°/s sağa dönüyor | Model depoda (`models/tork_v2_10M/`). Sıradaki ödül v3: dönüş izleme ağırlığı/toleransı ve hız toleransı |
| 09-25 | Kayda değer eğitilmiş modeller depoda `models/<ad>/` (zip ~0.5 MB); tam eğitim çıktıları depoda değil | Samet ve politika düğümü (G8) aynı modeli kullanabilsin |
| 09-25 | Ödül v3 (dönüş izleme 0.2 → 1.0, toleranslar daraldı) ile tork_v2_10M'den devam; 2.5M'de DURDURULDU | Dönme hiç düzelmedi (−119..−124°). Teşhis: elle simetrik tripod dümdüz gidiyor (model simetrik) ve v3'te 2.77 alıyor, politika 1.90; ama keşif gürültüsünde (std 0.35) anlık izleme terimleri çöküyor ve dönen politika 0.98, düz tripod 0.63 alıyor |
| 09-25 | **Ödül v4: hız ve dönüş izleme gövde hızının 0.5 s'lik üstel ortalamasına bakar** (`VelocityFilter`); progress anlık | Önemli olan ortalama yön/hız, adım içi salınım değil; 0.1 gürültüde düz tripod 2.23, dönen politika 1.90 |
| 09-25 | **Politika taklitle başlatılır** (`hexapod_rl.pretrain`): simetrik gösterim tripod'u (`demo.py`), gürültülü koşulur, etiket gürültüsüz; aktör + kritik; std 0.15 | Dönerek yürüme yerel tepesinden PPO kendi çıkamıyordu. Taklit başlangıcı hız komutunu %95–100 izliyor, yön <3° |
| 09-25 | Taklit için Samet'in `TripodGait`'i değil kendi `demo.py` gösterimimiz | Taklit edilen eylem gözlemden çıkarılabilmeli: demo yalnız saat + komuta bağlı; TripodGait dünya çapası tutuyor. İkisi RL ortamında neredeyse aynı ölçülüyor |
| 09-26 | Tripod RL ortamında ölçüldü (`hexapod_rl.baseline`, `evaluate tripod`), G6 bitti | G6'nın bitti şartı; G7'nin şartı politikanın bunu geçmesi |
| 09-26 | Değerlendirme gürültüsüz ve eylem gürültüsüyle (`--noise 0.1`) ayrı ayrı | PPO stokastik politikayı eniyiler: v4'te gürültü altında tripod'u geçiyor (0.10 m/s'de ödül 2.64'e 1.99), gürültüsüzde biraz geride (3.11'e 3.20) ve 3–4 kat enerji harcıyor. İkisi de raporlanmalı |
| 09-26 | PPO v4 5M yerine 4M'de bırakıldı (`models/ppo_v4_4M`) | Kullanıcı bilgisayarı acil kapattı; ara kayıt 250k'da bir, kayıp ~170k adım. Sıradaki eğitim ödül v5 ile yapılacağı için tamamlanmadı |
| 09-26 | Ödül v5 (progress süzülmüş hıza, güç −0.05/W) + alan rastgeleleştirme (servo gücü/sertliği, 0–18 ms gecikme, itme, IMU gürültüsü; TAHMİN aralıklar), taklit yeniden (bc_v5), 10M eğitim (v5_dr) | v4 hedef hızı aşıyor ve 3–4 kat enerji harcıyordu; rastgeleleştirme açıkken v4 yön tutamıyordu (−21..−52°), tripod ve taklit tutuyordu |
| 09-26 | **Politika düğümü torch'suz: aktör ağı + eğitim sözleşmesi .npz'de, numpy ile çıkarım** (`hexapod_policy`, `hexapod_rl.export`) | Pi'ye torch kurmak gereksiz ağırlık; ağ küçük (tick 44 µs PC'de). Sözleşme dosyada olunca eğitim ayarı değişse de eski politika doğru çalışır |
| 09-26 | Politika düğümü eğitilmemiş komutta yürümez, ayakta bekler (komut yok/zaman aşımı, vx eğitim aralığının yarısının altında, IMU yok/bayat, >45° yatık) | Politika yalnız ileri 0.05–0.15 m/s gördü; geri/yana/dönüş komutunda ne yapacağı bilinmiyor |
| 09-26 | **ROS'lu simde tork servo modeli (varsayılan `servo:=torque`):** `leg_controller` → `pid_controller` (`servo_controller`, P = sertlik, çıkış ±durma torku, 1 kHz) → eklem eforu; sönüm URDF'te eklem sönümü. Eski model `servo:=velocity` | Hız komutlu modelde ayaklar kayıyordu (tripod %84, politika %83); tork modelinde tripod %98, politika %96 (RL simiyle aynı düzey). Komut arayüzü değişmedi. C++ eklenti ya da Python sistem eklentisi yerine: köprü `Float64MultiArray` taşımıyor, zincir standart ros2_control |
| 09-26 | v5_dr 10M yerine 5M'de durduruldu; gSDE denendi (v6_sde lr 3e-4, v6b lr 1e-4 + target_kl 0.02), ikisi de bozuldu, bırakıldı | gSDE'de eğitim ödülü sistematik düştü (2300 → 1630) ve politika dönmeye kaydı; KL küçükken bile. Sebep çözülmedi (§12.24) |
| 09-26 | **Ödül v6: dönüş toleransı 0.2 → 0.1 rad/s; devam eğitiminde keşif std 0.05, lr 1e-4, target_kl 0.02** (`--std`, `--lr`, `--target-kl`) | Bütün PPO'lar yavaşça sağa dönüyordu; 2°/s sapma v5'te terimin %3'ü. Düşük gürültü, eğitimde koşan davranışı gürültüsüz davranışa yaklaştırır. Sonuç `models/ppo_v7_8M`: hız izleme doğru, yön 4 kat iyi; enerji hâlâ tripod'un 3 katı |
| 09-26 | v8: v7'den güç cezası −0.10/W ile 2M (`--power-weight`); depoya alınmadı | Enerji değişmedi (0.10 m/s'de 6.4 W; tripod 1.9 W). Terim terim fark neredeyse tamamen güç; eylem titremiyor, fazla güç yürüyüş biçiminden. Öneri: tripod üstüne artık eylem |
| 09-26 | **Artık eylem modu (`--residual`): hedef = adım saatinin tripod'u (`hexapod_policy.tripod.PhaseTripod`, eğitim ve robot aynı kod) + 0.2 rad x eylem; düzeltme cezası −0.5 x ortalama eylem karesi.** En iyi model `models/ppo_res_250k` | Mutlak modda PPO 3 kat enerji harcıyordu, güç cezası düzeltmedi. Artık eylemde eylem 0 = tripod: enerji tripod'dan ~%20 fazla; gürültüsüz ödülde tripod'u 0.10/0.15'te geçiyor, rastgeleleştirmede eşit, gürültüde önde. G7 için önerilen yol |
| 09-26 | **Süreç içi Gazebo her süreçte ayrı gz-transport bölümünde** (`GZ_PARTITION=hexapod_rl_<pid>`) | Eğitim sürerken açılan ROS'lu simin `ros_gz_sim create` isteği eğitimin "rl" dünyasına gitti, ROS'lu simde robot doğmadı (§12.22) |
| 09-26 | Yeni PC: Ubuntu-26.04 mevcut Ubuntu-24.04'ün yanına kuruldu, varsayılan yapıldı; eğitim `--envs 16` | 24.04'te başka veriler var ve Lyrical yok. 16 ortam 1818 adım/s, eski PC'nin 2.7 katı (§0.5) |
| 09-26 | **Her yöne komutla eğitim** (`--omni`: vx ±0.15, vy ±0.08, wz ±0.5; bileşenler %50 sıfırlanır; aralığın 1/3'ünden küçük komut çekilmez) ve **düğümde ölü bölge** (`command_deadband` 1/6) | Robotta geri/yana/dönüş gerekiyor; sınırlar Samet'in teleop'unun, tripod bunlarda test edildi. "Dur"u politika değil düğüm karşılar |
| 09-26 | **En iyi ara kayıt eğitim içinde otomatik** (`best_model.zip`, her ara kayıtta deterministik ölçüm) | Uzun eğitimde deterministik davranış kötüleşiyor, en iyi ara kayıt elle aranıyordu |
| 09-26 | v10_omni (std 0.15) 2.1M'de durduruldu; `ppo_omni_250k` depoya | Deterministik skor 250k'dan sonra düştü (1M'de tripod'un altı), rastgeleleştirme açıkken de; eğitim ödülü artıyordu (ders 25). Kalan 3M boşa giderdi |
| 09-26 | Gövde kütlesi rastgeleleştirmesi ×0.9–1.6, **ortam başına** | Faturadaki fazla batarya/buck ve kapak tahmine girmemişti; gz.sim kütleyi sonradan değiştiremediği için bölüm başına olamıyor |
| 09-26 | Deneme zeminleri (`terrain_probe`) S5'ten ayrı, yalnız beklenti için | Plan §14-5. Zeminli eğitim ve "bitti" ölçümü S5 + S6 ile |
| 09-26 | **Görev ayarı modelin yanında** (`gorev.json`); evaluate/export/terrain_probe bayraksız okur | Ayak kaldırması farklı bir modeli yanlış tabanla ölçmek ya da robota aktarmak sessiz bir hata olurdu |
| 09-26 | Zeminli eğitim ortam başına (`--terrains`), ilk deneme kendi deneme zeminlerimle | S5 henüz yok; altyapı hazır olsun, S5 gelince yalnız liste değişsin |
| 09-26 | v12_zemin (taban 25 mm) 2.25M'de durduruldu; **taban ayak kaldırma 50 mm** ile yeniden (`--lift-mm 50`) | 2M adımda hiçbir engelde iyileşme yoktu; düzeltmesiz tripod 40–60 mm kaldırmayla 45–60 mm engelleri geçiyor ve düzde ödül değişmiyor |
| 09-26 | `ppo_lift50_2250k` depoya, ara kayıt elle (zemin skoruyla) seçildi | Eğitim içi seçim düz zemine bakıp 250k'yı seçti; zemin skorunda en iyisi 2.25M |
| 09-26 | Deneme zeminlerine kaygan (μ) ve engebe (`rough`) eklendi; v17 (bunlarla eğitim) depoya alınmadı | G7 "bitti" şartının üç türü ölçülebilsin. v17 engel skorunu 0.763'ten 0.68–0.72'ye düşürdü (ders 34) |
| 09-26 | Ödül v7 (`progress_overshoot`) eklendi ama varsayılan 0 (v6) kaldı; v15/v16 modelleri depoya alınmadı | Aşma biraz azaldı, enerji azalmadı, katsayı 3'te zemin bozuldu (ders 33). Raporlar ortak ödülle (`standard_reward`, v6) |
| 09-26 | v14 (std 0.05, 2M) → `ppo_lift50_3750k` depoya; ara kayıt rastgeleleştirmeli 3 tohumla elle seçildi | Düzde aşma azalmadı ama 60 mm basamak 3/3, zemin skoru 0.957; eğitim içi deterministik seçim (500k) ikili durumlarda gürültülü (ders 32) |
| 09-26 | **Ayak teması ve gövde yüksekliği zemin yüksekliği fonksiyonuna göre** (`terrain_height(x, y)`, `terrain_sdf` ile zorunlu çift); fizik motorunun temas sensörü değil | Sensör yolu (Contact sistemi + gz.transport) tek süreci 381 → 182 adım/s yavaşlattı ve mesajlar eşzamansız (tekrarlanabilirlik bozulur). Yükseklik yolu bedava, sensörle 0–20° eğimde %97.8–98.8 uyumlu (§12.26). Yüksekliksiz zemin reddedilir: z=0 varsaymak değer uydurmak olur |
| 09-27 | **Öğrenilmiş ayak kaldırma** (19. çıkış, `lift_action`/`lift_range`) eklendi; seçim salınım başına | Sabit kaldırmada 25 mm düzde verimli ama engelde takılıyor, 50 mm tersi (ders 33); 18 eklemin keşfiyle bulunamayan hareket tek düğmeyle denenebilirdi. Her adım seçim keşfedilemedi (ders 36) |
| 09-27 | v21 (35 mm'den öğrenilmiş kaldırma) son modeli alınmadı; 250k ara kaydı `ppo_kaldirma35_250k` olarak depoya | Kaldırma zemine göre değişmedi, 53 mm'ye kaydı ve `ppo_lift50_3750k`'dan kötü (ders 35). 250k düz verim ile engel arasında boş kalan bir noktayı dolduruyor (2.39 W, 45 mm engeller); D11'de seçenek |
| 09-27 | Dayanıklılık taraması (`Perturbation`, `robustness`); IMU eğikliği ve uzun gecikme eğitime **eklenmedi** | 20°'lik IMU eğikliği ve 240 ms gecikme hiçbir modeli bozmadı. Hassas olunanlar kalibrasyon ofseti (σ ≥ 3°) ve servo gücü (×0.5 altı); ofseti politika göremiyor ve tripod da aynı kaybediyor, önce donanımda ölçülmeli (D6, D9), sonra aralığı eğitime girer (D10) |
| 09-27 | **Müfredat** (`--curriculum`, `CURRICULA`) eklendi, varsayılan değil; v22/v23 modelleri alınmadı | Sabit 25 mm tabanda 45 mm engel yok; öğrenilmiş kaldırmada yerel en iyiden çıkardı ama iyi bir başlangıçtan (35 mm) daha iyi bir yere varmadı (ders 40). S5'in zeminleri için altyapı olarak kalıyor |
| 09-27 | **Mesafe sensörlü kaldırma refleksi** (`LiftReflex`) eğitimsiz bir kural olarak; mesafe sensörü simde yalnız `info`'da, gözleme girmedi | Kural düzde 25 mm'nin enerjisinde, engelde 50 mm'den iyi (ders 41); politikaya sensör gözlemi verip yeniden eğitmek gerekmeden ödünleşimi çözüyor. Sensör gözlemli RL denenmedi: yerleşim ve sensör davranışı (D8, S7) bilinmeden gözlemi sabitlemek erken |
| 09-27 | v24'ün 1.5M'i `ppo_refleks_1500k` olarak depoya; `ppo_kaldirma35_250k` yerinde kalıyor | Refleksle engelde en iyi (60 mm basamak, engebe 60) ama düzde %8–17 pahalı; ikisi farklı kullanıma. En iyi ara kayıt refleksli ölçümle seçildi; 2M 60 mm basamakta daha iyi (0.99) ama çaprazda ve geride (kör) daha kötü, düzde daha pahalı |
| 09-27 | Eski modeller (`tork_v2_10M`, `ppo_v4_4M`, `ppo_v7_8M`, `ppo_res_250k`, `ppo_lift50_2250k`) depodan kaldırıldı; `taklit_bc_v4` testler için kalıyor | Yerlerini `ppo_omni_250k`, `ppo_lift50_3750k`, `ppo_kaldirma35_250k`, `ppo_refleks_1500k` aldı; hiçbir kod ya da test kullanmıyordu. Tablolar models/README'de, dosyalar git geçmişinde |

---

## 12. Bilinen tuzaklar ve çıkarılan dersler

1. **STEP'te parça adı ↔ geometri eşlemesine güvenme.** `SHAPE_DEFINITION_
   REPRESENTATION` üzerinden yapılan eşleme `PLA_Tibia_tip` ile
   `PLA_Tibia_main` için **ters** çıktı. İlk türetmede tibia bu yüzden
   121 yazıldı. Montaj konumları (occurrence/NAUO adları) güvenilir; brep
   → ürün adı eşlemesi değil.
2. **B-spline kontrol noktaları yüzeyin dışında durur.** Bir parçanın en
   uç noktası STEP `CARTESIAN_POINT`'lerinden bulunamaz; mesh köşelerini
   (STL/3MF) kullan. Ayak ucu STEP'te analitik küre değil, serbest form.
3. **Aynalı bacaklar:** Fusion bunları ayrı gövde olarak dışa aktarmış;
   dönüşüm matrisi düzgün bir dönme (det +1), yansıma geometride. Yerel Z
   ters yönde uzanıyor. Bunu hesaba katmayınca aynalı bacaklar 140 mm,
   normaller 100 mm yarıçapta çıkıyordu (simetrik robotta imkânsız).
   Eklem ofseti aynalılarda ters işaretle alınmalı.
4. **CAD azimut yönü:** CAD'de +Y yukarıyken atan2(z, x) yukarıdan saat
   yönünde artar. Gövde çerçevesine geçişte işaret ters:
   `gövde = offset − cad`.
5. **IK açı sarması:** ayak femur ekleminin gerisine düştüğünde
   `atan2 + α` 180°'yi aşıyor ve femur −60 yerine 300° çıkıyordu. Servoya
   giderse ters döner. (−180, 180]'e sarılıyor; gidiş-dönüş testi yakalar.
6. **PCA9685 0x70 (ALLCALL):** her kart burada da cevap verir; kart adresi
   sanma. `hwcheck.py` bir ara bunu yanlışlıkla ikinci kart önerebiliyordu.
7. **Aynı adlı dosya çakışması:** klasörler düzleştirilirken iki "gömülü
   malzemeler listesi.txt" çakıştı, biri ötekini ezdi. Klasör taşımaları
   sonrası `git status`'u dikkatle oku; beklenmeyen `M`/`D` varsa
   dokunmadan önce sor ya da incele.
8. **Dosyalar dışarıdan değişebilir.** Kullanıcı çalışırken klasör
   düzenleyebiliyor. Commit'lerde `git add -A` yerine dosyaları açıkça
   seç; kullanıcının değişikliğini kendi commit'ine katma, önce söyle.
9. **Tahmin etme, uydurma.** İlk oturumda kullanıcı açıkça "tahmin etme,
   varsayılan değer uydurma" dedi; bu, `MissingValue` tasarımının kökeni.
   Simülasyon için tahmini değer gerekirse (kütle, atalet), kaynağıyla ve
   `measured: false` / "simülasyon tahmini" notuyla gir.
10. **Yansıtılan mesh'in hacmi negatif çıkar.** Bir STL'i yansıtınca
    (ör. aynalı bacak için y → −y) üçgenlerin dönüş yönü tersine döner;
    dörtyüzlü toplamıyla hesaplanan hacim ve kütle negatif olur, parça
    kütle eklemek yerine çıkarır. İlk hesapta gövde ağırlık merkezi bu
    yüzden 9 mm kaymıştı. Yansıtınca üçgen sırasını da çevir
    (`cadlib.mesh.flip_winding`).
11. **Her STL, STEP parça çerçevesinde değil.** `pla_body-lid.stl`
    (yalnız `Baskı Dosyaları/`'nda) baskı tablası konumunda dışa
    aktarılmış; montaj dönüşümü uygulanınca gövdenin 100 mm dışına
    düşüyordu. Yeni bir STL kullanmadan önce dönüştürülmüş sınır
    kutusunun beklenen yerde olduğunu kontrol et.
12. **Uzun heredoc'lar Bash aracında bozuluyor** ("unexpected EOF").
    Uzun dosyaları Write aracıyla yaz; kısa betikler için heredoc olur.
13. **ROS paket adlarını ezbere yazma, Lyrical'da doğrula.** `position_controllers`
    Lyrical'da YOK (ForwardCommandController + interface_name: position kullan).
    Doğrulama yolu: ROS apt deposunun paket listesi
    (packages.ros.org/ros2/ubuntu/dists/resolute/main/binary-amd64/Packages.gz)
    ve ilgili GitHub deposunun `lyrical` dalı. docs.ros.org bot korumalı.
14. **sdformat çarpışma adları:** URDF → SDF'te çarpışma `<ad>_collision` (sırası 0
    değilse `_<sıra>` eki) olur. Temas sensörü bu adı ister; ayak küresi bu yüzden
    tibia'nın ilk çarpışması. gz-sim temas sensörünün konusu `<contact><topic>`
    içinde (sensör düzeyinde değil).
15. **RViz WSLg'de Wayland'de çöker** ("Invalid parentWindowHandle ... GLXWindow",
    100 denemeden sonra abort). `QT_QPA_PLATFORM=xcb` ile XWayland'de çalışıyor;
    display.launch.py bunu Wayland varsa kendisi veriyor. Gazebo penceresi etkilenmiyor.
16. **ament_python'da `setup.cfg` şart:** yoksa console_scripts `bin/`'e kurulur ve
    `ros2 run paket komut` "No executable found" der. `[develop] script_dir` ve
    `[install] install_scripts` `$base/lib/<paket>` olmalı.
17. **gz.sim Python bağları:** `import gz.math` yapılmadan `Link.world_pose` çağrılırsa
    pybind11 Pose3d'yi çeviremez ve süreç ÇÖKER (yakalanamaz). `reset_all()`
    isteği bir adım gecikmeyle işlenir ve sıfırlama `run(n)`'in adımlarından birini
    yer; `HexapodSim.reset()` bunu tek adımlık çağrılarla çözüyor. Hız komutu
    (`Joint.set_velocity`) adımlar arasında kalıcı: her fizik adımında değil, servo
    döngüsünde vermek yetiyor ve ~%30 hızlandırıyor. Kamera sensörü, konusunu
    dinleyen yoksa kare üretmiyor (`<save>` olsa bile).
18. **Servo modeli yürümeyi belirliyor; önce fiziği doğrula.** Hız komutlu servo
    modelinde robot yürüyemiyordu ama bu RL eğitiminden anlaşılmadı: iki eğitim
    (6+ saat) "ödül yanlış" sanılarak harcandı. Teşhis sırası şuydu ve işe yaradı:
    (1) RL'siz, elle yazılmış açık döngü yörüngeyle yürüyor mu? (2) değilse
    parametreleri tek tek değiştir (tork, sürtünme, tepki süresi) (3) hareketsiz
    duruşla statik/dinamik ayrımı. Artık `test_simulasyon_yurumeye_izin_veriyor`
    bunu her test koşusunda denetliyor. Ayrıca: gz.sim'de `Joint.transmitted_wrench`
    Python'da bozuk (gz::msgs::Wrench çevrilemiyor); tork gerekiyorsa servo
    modelinin uyguladığı tork kullanılır (`SimState.joint_effort`).
19. **ROS düğümünü birim testleriyle yetinmeden canlı çalıştır.** `rclpy`
    Lyrical'da `logger.warn` yok, adı `warning`. `hexapod_hardware` düğümü
    bu yüzden ilk reddedilen komutta çöküyordu; ROS'suz çekirdek testleri
    (9 test) hepsi geçiyordu çünkü düğüm kabuğuna dokunmuyorlar. Yeni bir
    düğüm yazınca hata yollarını da (bozuk komut, eksik config) gerçek
    `ros2 run` ile tetikle. Artık `tests/test_ros_nodes.py` düğümleri gerçek
    süreç olarak başlatıp bunu otomatik yapıyor (rclpy'li ortamda).
    İkinci hata: SIGTERM'de (servis durdurma) ve arka plan Ctrl+C'de
    `rclpy.spin` KeyboardInterrupt değil `ExternalShutdownException` ya da
    `RCLError` fırlatır; yakalanmazsa hata izi + çıkış kodu 1. Doğrudan
    başlatılan süreçte Ctrl+C ise KeyboardInterrupt yolundan geçtiği için
    düzeltmesiz de temiz görünür: kapanışı **SIGTERM ile** test et. Ayrıca:
    aynı betikte `pkill -f <desen>` kendi WSL kabuğunu da öldürebilir
    (komut satırında desen geçtiği için).
20. **Ödülü eğitimdeki gürültüyle de ölç.** PPO stokastik politikayı
    eniyiler; eylemlere keşif gürültüsü eklenmiş hâldeyken ödülün neyi
    ödüllendirdiği, deterministik bakıştakiyle zıt olabilir. Ödül v3'te
    anlık gövde hızına bakan izleme terimleri gürültüde çöktüğü için dönen
    politika düz yürüyüşten çok puan alıyordu; 2.5M adımlık eğitim boşa
    gitti. Teşhis sırası: (1) elle yazılmış iyi bir davranış (simetrik
    tripod) ile politikayı AYNI ödülde karşılaştır, (2) bunu eylem
    gürültüsüyle tekrarla (std, eğitimdeki `train/std` kadar), (3) terim
    terim bak. Çıkarım: izleme terimleri süzülmüş hıza baksın; ölçümlerde
    gürültüsüz ve gürültülü ikisini de raporla (`evaluate --noise`).
21. **PPO bir yerel tepeye yerleşince ödül değişikliği yetmeyebilir.** v2'nin
    dönerek yürüyen politikası v3 ödülüyle devam eğitiminde hiç değişmedi
    (ödül ~1740'ta yatay). İyi bir gösterimi taklit ederek başlatmak
    (`pretrain.py`) daha hızlı ve güvenilir: taklit 93 s veri + birkaç
    dakika eğitim; başlangıç hemen dümdüz yürüyor. Taklitte gözlemdeki
    "son komutlar" gösterimi kopyalamaya izin verir; uygulanan eyleme
    gürültü ekleyip etiketi gürültüsüz tutmak bunu önler.
22. **gz-transport ROS_DOMAIN_ID'yi bilmez; süreç içi Gazebo'lar da ağda.**
    `hexapod_rl.sim`'in TestFixture dünyaları ("rl") gz-transport'ta tam bir
    sunucu gibi görünür (`/gazebo/worlds`, `/world/rl/create`...). Eğitim
    sürerken ROS'lu sim açılınca `ros_gz_sim create` dünya listesini sordu,
    henüz yüklenmemiş "flat" yerine eğitimin "rl"sini buldu ve robotu oraya
    göndermeye çalıştı ("Entity creation successful"); ROS'lu simde robot
    hiç doğmadı, spawner sonsuza kadar controller_manager bekledi. Aynı adlı
    robot zaten olduğu için eğitime zarar vermedi. Çözüm: süreç içi dünya
    kendi bölümünde (`GZ_PARTITION`), ayrıca aynı anda iki Gazebo çalışacaksa
    birine elle bölüm ver. Belirti: ROS'lu simde "Entity creation successful"
    dünya yüklenmeden önce görünüyor ve `ros2 control list_controllers` takılıyor.
    Ek not: `ros2 control` komutları servis yoksa sonsuza kadar bekler;
    betiklerde `timeout` ile çağır.
23. **Hızı, hareketin olduğu pencerede ve sim zamanıyla ölç.** Politika
    düğümünün ROS'lu simdeki hızı önce "%71" diye yazıldı ve push'landı:
    başlangıç/son pozu `gz model -p` ile alıp duvar saatiyle bölmüştüm;
    sorgu ~1 s sürüyor ve pencereye boşta geçen süre giriyor, RTF de 1'in
    altında. Doğrusu (gz poz yayınına abone olup sim zamanıyla, hareketin
    orta %80'inde): %83 (eski model), %96 (tork modeli). Aynı hata Samet'in
    teleop ölçümünde de olabilir; karşılaştırmalar aynı yöntemle yapılmalı.
24. **gSDE bu kurulumda PPO'yu bozdu.** Taklitten başlayıp gSDE ile (SB3
    use_sde, full_std, sde_sample_freq 16) eğitince, lr 1e-4 ve target_kl
    0.02 ile güncellemeler küçükken bile eğitim ödülü 750k adımda 2300'den
    1630'a düştü, politika dönmeye kaydı, entropi arttı (gürültü gizli
    katmanın büyüklüğüyle büyüyor). Aynı ayarlarda bağımsız gürültüyle
    ödül artıyordu. Sebep bulunmadı; bağımsız gürültü + düşük std (0.05)
    işe yaradı. Tekrar denenirse önce stokastik değerlendirmeyle eğitim
    davranışına bak.
25. **PPO'nun keşif gürültüsü davranışın parçası olur.** std 0.1–0.15'lik
    bağımsız gürültüyle eğitilen politika, ortalama eylemi gürültüyü telafi
    edecek şekilde ayarlıyor: gürültüsüz koşunca hedef hızı %10–30 aşıyor
    ve fazla enerji harcıyor (stokastik koşunca hız doğru). Taklitten
    başlayan bir politikayı iyileştirirken gürültüyü küçük tut (0.05) ya da
    std'yi zamanla düşür; değerlendirmeyi hem deterministik hem stokastik
    yap.
26. **gz.sim Python'da fizik temas verisi pahalı ve eşzamansız.** Python
    bağlarında ECM'den temas bileşeni (`ContactSensorData`) okunamıyor;
    `gz.sim` yalnız Link/Joint/Model/World sarmalayıcılarını veriyor. Tek yol:
    dünyaya `gz-sim-contact-system`, tibia'ya `<sensor type="contact">`,
    süreç içinden `gz.transport.Node().subscribe(Contacts, konu, cb)`.
    Ölçülenler (tek süreç, 2026-09-26):
    - Sensörsüz 381, sensör + Contact sistemi (abone yok) 316, Python
      aboneliğiyle 182 adım/s. Maliyetin çoğu Python geri çağrısı.
    - Contact sistemi `update_rate`'i dinlemiyor, her fizik adımında
      (2 ms) ve yalnız temas varsa yayınlıyor. Mesaj damgası sim zamanı.
    - Geri çağrı ana iş parçacığında değil. Denemede bütün mesajlar
      `server.run()` dönmeden gelmişti ama bunun garantisi yok; yük
      altında adım sonundaki temas eksik görülebilir.
    - Aynı `GZ_PARTITION` içinde olduğu için abone, `Node` süreç içi dünya
      kurulduktan (ortam değişkeni ayarlandıktan) sonra yaratılmalı.
    Karar: zemin yüksekliği fonksiyonu. Sensör yolu yalnız doğrulama
    aracı olarak işe yaradı: yükseklik temasıyla 0°, 10° (aşağı, yukarı,
    yana), 20° (aşağı, yukarı) eğimde %97.8–98.8 uyum. Farkların hepsi
    "ayak 2 mm içinde ama değmiyor" (iniş/kalkış); fiziğin gördüğü temas hiç
    kaçmadı; temasta dikey boşluk en çok 0.73 mm. Deneme betiği depoda
    değil; yöntemi bu madde anlatıyor.
27. **Süreç içi Gazebo SIGTERM'i yakalıyor.** Eğitimi durdurmak için ana
    sürece `kill -TERM` gönderildi; 30 s sonra 17 süreç (ana + 16 ortam)
    hâlâ çalışıyordu: gz-sim sunucusu SIGINT/SIGTERM için kendi işleyicisini
    kuruyor. `kill -KILL` ile durdu; ara kayıtlar (250k'da bir) ve
    progress.csv sağlam kaldı. Pratik: uzun eğitimi durdurmak gerekirse
    `pgrep -f "name <ad>"` ile bütün süreçleri bul, `kill -KILL` gönder
    (betik dosyasından; ders 19'daki pkill uyarısı). Sarmalayıcı betik bunu
    hata olarak kaydeder ("EĞİTİM HATASI"), beklenen.
28. **Düz zeminde tripod tavanı.** Her yön eğitiminde de ders 25 tekrarlandı:
    std 0.15'le eğitim ödülü %6 artarken deterministik skor tripod'un
    altına indi; std 0.05'le düşüş olmadı ama tripod düzeyinden de
    çıkılmadı (±%1). Rastgeleleştirme açıkken ölçmek aynı sonucu verdi.
    Çıkarım: düz zeminde tripod'u geçmek için daha çok eğitim anlamsız;
    ölçülebilir kazanç eğim/basamakta (§3.3) ve eklem gürültüsünde. Ara
    kayıt seçimi için deterministik düz zemin ölçümü şimdilik yeterli:
    rastgeleleştirmeli ölçüm v10'da aynı sırayı verdi (250k > 2M > 1M),
    v11'de ara kayıtlar ikisinde de gürültü düzeyinde farklı.
29. **Kör politika büyük bir hareket değişikliğini keşif gürültüsüyle
    bulamaz; tabanı doğru kur.** Taban tripod ayağı 25 mm kaldırırken 45 mm
    basamağı geçmek ~25 mm daha yüksek salınım istiyordu; artık eylem
    ölçeği (0.2 rad) buna yetiyordu ama std 0.1'lik bağımsız gürültü
    bütün bacaklarda tutarlı bir yükseltmeyi hiç denemedi: 2.25M adımda
    sıfır ilerleme (v12). Tabanı elle 50 mm yapınca aynı engeller hemen
    geçildi ve RL üstüne %15 kattı (v13). Önce düzeltmesiz tabanın
    parametrelerini zeminde tara (`terrain_probe phase:<mm>`), RL'yi
    tabanın yetmediği farkı öğrenmeye bırak.
30. **Zeminli eğitimde ara kayıt seçimi zemini görmeli.** Eğitim içi seçim
    düz zemine bakıyordu; zeminde en iyi ara kayıt (2.25M) düzde hedef hızı
    %10 aştığı için düşük puan aldı ve 250k seçildi (zemin skoru 0.755'e
    0.879). Zemin ölçümü şimdilik elle, paralel betikle yapıldı
    (`egitim_kayitlari/v13_lift50/zemin_olcumu.md`). Ölçüt olarak hedef
    hızı aşmayı ödüllendirmeyen "en fazla beklenen yol" kullanıldı.
    Sonra eğitim içine alındı: `--terrains` verilince her ara kayıtta
    `terrain_probe.EVAL_CASES` da ölçülür, skor düz + zemin durumlarının adım
    başı ödül ortalaması (ödül aşmayı ve enerjiyi zaten cezalandırıyor).
    v13 ara kayıtlarında 2.25M'yi seçiyor (2.694; 250k 2.656).
31. **`HexapodEnv.close()` Gazebo'yu bırakmıyordu.** Geri çağrılar (bound
    method) TestFixture'ın C++ tarafında tutulduğu için Python'un çöp
    toplayıcısı döngüyü göremiyordu; aynı süreçte her kur/kapat ~31 MB ve 2
    iş parçacığı bırakıyordu (`gc.collect()` bile temizlemedi) ve gz "Another
    world of the same name is running" diyordu. `HexapodSim.close()`
    fixture/sunucu referanslarını keser: bellek ve iş parçacığı sabit, uyarı
    yok. Eğitim içi zemin ölçümü süreçte onlarca dünya kurduğu için şarttı.
    Test: `test_kapatinca_gazebo_birakilir`.
32. **Geç/geçeme durumlarında tek deterministik ölçüm gürültülü.** Eğitim
    içi seçim her zemin durumunu bir kez, rastgeleleştirmesiz ölçüyor; 60 mm
    gibi sınırdaki bir engel küçük bir farkla geçiliyor ya da geçilemiyor
    ve skor sıçrıyor (v14 250k'da 60 mm çukur 1.17, 500k'da 2.77). Seçim
    500k'yı seçti; rastgeleleştirme açık 3 tohumlu ölçüm 1.5M'yi açıkça öne
    koydu (60 mm basamak 1/3'e 3/3). Zeminli bir modeli depoya almadan önce
    ara kayıtları çok tohumla ölç; eğitim içi seçim ilk eleme.
    Ayrıca: std 0.05 (ders 25) 50 mm tabanlı politikada hız aşmasını
    azaltmadı (%10 → %14); aşma burada gürültü uyumundan değil, engel
    geçmenin getirdiği agresif yürüyüşten olabilir.
33. **Kör politikada düz verim ↔ zemin sağlamlığı ödünleşimi.** 50 mm
    tabanlı zemin politikası düzde hedef hızı %14 aşıyor ve 4.0 W harcıyor
    (aşmayan taban 3.1 W). Ödül v7 ile aşma cezalandırıldı (önce ders 20:
    v7 davranışları doğru sıralıyordu). Katsayı 1'de hız 0.114 → 0.110,
    katsayı 3'te 0.107; **güç ikisinde de 3.9–4.0 W**, katsayı 3'te zemin
    becerisi bozuldu. Ara kayıt adım başı terim dökümü (`reward_terms`):
    fazla güç hızdan değil, engel geçiren düzeltmelerden. Politika önünde
    engel olup olmadığını bilmiyor (gözlem yalnız IMU + komut + saat); bu
    yürüyüşü her yerde kullanıyor. Çözüm ödülde değil gözlemde: ileri bakan
    mesafe sensörleri (3 VL53L0X alındı, S7/D8) ya da iki model arasında
    düğümde seçim (düzde `ppo_omni_250k`, engelde `ppo_lift50_3750k`).
34. **Eğitim setini çeşitlendirmek var olan beceriyi sulandırabilir.** v17'de
    16 ortamlık sete kaygan eğim ve engebe eklenince çukur ortamı 8'den
    3'e indi; 3M adım sonra çukur becerileri zayıfladı (60 mm çukurdan geri
    3/3 → 1/3–2/3), yeni türlerde anlamlı kazanç olmadı (engebeye politika
    zaten genelleşiyordu; dar paylı kaygan yokuşlar fiziksel sınırda).
    Ders: yeni bir zemin türü eklemeden önce mevcut politikanın o türde
    zaten ne yaptığını ölç (engebe: iyi); fiziksel olarak çözülebilir mi
    bak (kaygan yokuşta μ > tan θ payı); set oranlarını koru ya da ortam
    sayısını artır. S5 gelince müfredat bunu gözetmeli.
35. **Kör politika verilen düğmeyi zemine göre çeviremez.** Öğrenilmiş
    ayak kaldırmada (v20, v21) kaldırma düz, basamak ve engebede aynı
    (±1 mm; basamağa takılınca yükselmiyor). Politika düğmeyi eğitim
    zemin karışımına göre tek bir değere ayarlıyor, ayar da başlangıca
    bağlı: 25 mm'den ~28 mm'de yerel en iyide kalıyor, çünkü engeller
    28–35 mm arasında bir eşikte geçilmeye başlıyor ve arada ödül farkı
    yok (sabit kaldırma taraması: models/README); 35 mm'den başlayınca
    53 mm'ye kadar çıkıyor. Engele takılmayı IMU'dan sezmek yetmedi.
    Zemine göre seçim için engeli görmek gerekir (ileri bakan mesafe
    sensörü, S7/D8); kaldırma çıkışı ve düğüm tarafı buna hazır. Mesafe
    sensörünün yönü/yeri robot.yaml'da `null`, simülasyona o değerler
    olmadan eklenmez.
36. **Keşif gürültüsünün zaman ölçeği eylemin etki ölçeğine uymalı.**
    Kaldırma her kontrol adımında (50 Hz) bağımsız gürültüyle seçilince
    (v18, v19) bir salınım (~17 adım) içinde gürültü ortalanıyor, servo
    da süzüyor: ayak hiçbir zaman belirgin yüksek bir salınım yapmıyor,
    engeli geçmek gibi eşikli bir ödül hiç görülmüyor. v19'da 750k adımda
    ortalama 25.7 → 26.1 mm, kaldırma std'si 0.40 → 0.41. Seçim salınım
    başına alınınca (v20) aynı gürültü bütün salınıma yayıldı ve kaldırma
    öğrenilmeye başladı. Yan kazanç: yörünge salınım içinde titremiyor.
    gSDE'nin (§12.24) aradığı şey de buydu; burada tek boyut için elle
    yapıldı. v18'de ayrıca target_kl 0.02 her güncellemede düşük std'li
    (0.05) eklem boyutlarında tükendi.
37. **Robota geçişte asıl risk kalibrasyon ve servo torku; IMU ve gecikme
    değil** (dayanıklılık taraması, models/README). Ofset σ2°'ye kadar düzde
    ≤%4 kayıp, σ4°'de %7–12, σ8°'de %16–47; tripod da aynı kadar
    kaybediyor (ayak yanlış yere basıyor, politika ofseti göremez). Engel
    becerisinin payı kaldırmaya bağlı: 35 mm'lik model basamakta σ1°'de
    0.80 → 0.62 m, σ3°'te 0.35 m (eşiğe yakın); 50 mm'lik σ4°'te 0.98 m.
    Servo gücü ×0.5'te iki zemin modeli basamakta yarıya iniyor; 50 mm'lik
    ×0.4'te düzde de devriliyor, 25 mm'likler ×0.4'te yürüyor (yüksek
    kaldırmanın tork payı en dar). IMU 20°'ye kadar eğik takılı olsa da,
    komut 240 ms gecikse de sonuç neredeyse aynı: açık döngü taban tripod
    yürüyüşü taşıyor. Sonuç: donanım vardiyasında kalibrasyon doğruluğu
    (~2°) ve yük altındaki servo gerilimi/torku öncelikli ölçülmeli;
    ilk denemeler 25 mm'lik modelle.
38. **İki sim çelişirse önce ölçüm koşullarını karşılaştır.**
    `ppo_kaldirma35_250k` için "45 mm çukurdan yana: RL simde 0/3, ROS'lu
    simde 0.75 m çıkıyor" yazılmıştı. RL simde rastgeleleştirme, fizik
    adımı (1/2 ms) ve süre (10/12 s) değiştirilince hep 0.11–0.21 m'de
    kaldı. Sebep ROS ölçüm aracıydı: `KOMUTLAR="-0.1,0,0;0,0.06,0"` aynı
    dünyada robot sıfırlanmadan art arda koşuyor; yana komutu, robot geri
    komutuyla çukurdan çıktıktan sonra başlamıştı (ortalama yükseklik 144
    mm = baştan dışarıda). Tek başına koşunca ROS'lu simde de 0.20 m,
    çukurda (RL simle aynı); `ppo_lift50_3750k` iki simde de çıkıyor.
    Araç artık yükseklik sütununda başlangıç → bitiş z'sini yazıyor;
    zeminde her komut ayrı koşulur. Sim-sim farkı sanılan şey ölçüm
    hatasıydı; gerçek bir fark bulunsaydı sim-to-real için uyarı olurdu.
39. **Kaynak sızıntısını tek başına değil, gerçek koşulda ölç.** Müfredatta
    dünya yeniden kurulurken bellek 5 kurmada sabit ölçüldü ve yeterli
    sanıldı; v23 2.5M adımda (16 ortam) "Too many open files" ile çöktü,
    ana süreç de askıda kaldı. Tek süreçte sızıntı yok; aynı makinede başka
    süreç içi Gazebo varken her kurmada ~32 soket + ~2 MB (öteki süreçler
    kapanınca soketler de kapanıyor). Sebep gz-transport keşfi: GZ_PARTITION
    ayrı olsa da keşif portları ortak (10317/10318), süreçler birbirini
    buluyor ve her yeni dünyanın düğümleri için bağlantı açılıyor. Düzeltme:
    her süreç kendi keşif portlarında (`GZ_DISCOVERY_MSG_PORT/SRV_PORT`,
    `sim.discovery_ports`). Test öteki süreci gerçekten başlatıp soket
    sayısına bakıyor; düzeltmesiz kodda 17 → 29 ile düşüyor.
40. **Müfredat yerel en iyiden çıkarır, iyi bir başlangıcın yerini tutmaz;
    kör politikada yeni beceri açmaz.** Öğrenilmiş kaldırma sabit sette 25
    mm'den 28 mm'de takılıyordu (v20); müfredatla (v23) 4M'de 33.6 mm'ye
    çıktı, 45 mm basamak geçildi. Ama sonuç 35 mm'den müfredatsız başlayan
    modelle aynı engel skorunda (0.499 / 0.496) ve düzde daha pahalı (3.15
    / 2.39 W). Sabit 25 mm tabanda (v22) çukur seviyesi 500k'dan beri ~35
    mm'de kaldı: eklem düzeltmesinin taşıyabildiği yükseklik bu. Bir de:
    seviye, eğitimdeki gürültülü politikanın başarısına göre değişiyor;
    kaldırma gürültüsü (salınım boyunca sabit, std 0.5) yüksek salınımlarla
    engeli geçirdiği için eğitimde 45–50 mm görünen seviye deterministik
    politikanın yeteneği değil (v23 1M: gürültülüyle 45 mm çukurdan 0.47 m,
    deterministikle 0.12 m).
41. **Ödünleşimi ödül değil algı çözer; önce basit bir kuralla dene.** Kör
    politikada düz verim ↔ zemin ödünleşimini ödülle (v7, ders 33),
    öğrenilmiş kaldırmayla (ders 35) ve müfredatla (ders 40) çözemedik.
    Önüne bakan mesafe sensörüne bakan eğitimsiz bir refleks
    (`LiftReflex`) ilk denemede çözdü: düzde 25 mm'nin gücü, engellerde
    50 mm'den iyi geçiş (models/README). Sensör gözlemiyle RL eğitmeden
    önce kuralı denemek hem ucuz (1.5 dk) hem de sensörün nereye bakması
    gerektiğini gösterdi: 45° aşağı bakan ışın engeli çok geç görüyor,
    20–25° doğru. Gürültüde ilk eşik (12 mm, tek okuma) yanlış alarm
    verdi; eşik 20 mm + art arda 2 okuma yeterli. Kalibrasyon ofsetinde
    (σ2–4°, ders 37) de engelde sabit 50 mm'den iyi: engelde payı açtığı
    için 35 mm'lik modelin kırılganlığı kalkıyor. Değerler robot.yaml'a
    yazılmadı: yerleşim D8'in kararı, sim yalnız öneri.
42. **Zemin karmaşıklığı doğrudan eğitim hızı demek.** Engebe, dik yanlı
    kutulardan kuruluyor (SDF ile yükseklik fonksiyonunun birebir tutması
    için); kutu sayısı süreç içi Gazebo'yu yavaşlatıyor: 716 kutu 3.05x,
    393 kutu 2.03x, 184 kutu 1.59x (düz zemine göre, ölçüldü 2026-09-27).
    Kazanç ~300-400 kutudan sonra azalıyor. Önemlisi: paralel eğitimde
    ortamlar adım başına birbirini beklediği için **en yavaş ortam hızı
    belirler** — tek engebeli ortam bütün eğitimi yavaşlatır. Zemin
    koridorunu bölüm süresinin gerektirdiği kadar tut (20 s'de robot ~2 m
    gider). Yükseklikleri kademelendirip komşu kutuları birleştirmek denendi:
    8 kademede bile kazanç %23, zemin gözle kabalaşıyor; bırakıldı.
43. **Üretilen zemini bir kez de gözle gör.** `hexapod_terrain` engebesinde
    robot düz bir karede doğuyordu ama karenin hemen kenarında yarım genlikte
    (50-70 mm) duvarlar oluşuyordu: robot "engebeye yürüyerek giren" değil,
    "duvarla çevrili çukurda doğan" bir şeydi. Testler bunu yakalamadı
    (SDF ile yükseklik tutarlıydı, genlik doğruydu); önizleme resmi
    (yükseklik haritası + yan kesit) ilk bakışta gösterdi. Düzlükten tam
    engebeye 0.5 m rampa eklendi.

---

## 13. Açık kalan işler

### 13.1 `robot.yaml`'da boş alanlar (donanım vardiyası doldurur)

- `drivers[0].address`, `drivers[1].address`: kart adresleri (muhtemelen
  0x40 ve 0x41, A0 lehimlenince). `map_channels.py`/`hwcheck.py` bulur.
- `joints[*].driver`, `joints[*].channel` (36): kanal haritası;
  `map_channels.py` çıkarır.
- `joints[*].limits_deg.min/max` (36): `calibrate.py` içinde `span` +
  `limit` ile bulunur. Simülasyon şimdilik geçici ±90° kullanıyor.
  **Gerçek limitlerle yeniden eğitim şart** (D10); coxa ±90'da komşu bacağa
  girer.
- `body.standing_height`: hedef değer, ölçüm değil. Simülasyon ve RL 100 mm
  kullanıyor.
- `body.total_mass_kg`: terazi (D7). Simülasyon CAD tahmini kullanıyor:
  2.13 kg.
- `sensors.imu.*`: adres ve montaj yönü (D8).
- `sensors.range_finders.devices[*]`: XSHUT GPIO, adres, bakış yönü. Kaldırma
  refleksi için ayrıca her sensörün gövdedeki yeri (x, y, z) ve aşağı bakış
  açısı gerekecek; bu alanlar henüz yok, D8 ekleyecek (öneri GOREVLER D8).

### 13.2 Yazılım (Görkem, G7)

Bu oturumda bitenler §14'te. Açık kalanlar:

1. **Refleksi düğüme bağla** (S7 + D8 gelince): `node.py` mesafe konularına
   (`sensor_msgs/Range`) abone olup `PolicyController.on_ranges`'i çağırır;
   yerleşim robot.yaml'dan (§13.1). Sonra `reflex_probe` ve yerleşim
   karşılaştırması gerçek yerleşimle yeniden koşulur; ROS'lu simde menzil
   sensörüyle uçtan uca denenir.
2. **Asıl zeminlerle eğitim** (S5): S5'in üreteci `TRAIN_SETS`/`CURRICULA`'ya.
3. **Eğimde `orientation` cezası** gövdeyi **dünyaya** göre düz istiyor (10°
   eğimde ayakta duran robot adım başı ~0.06 kaybeder). v13 20° yokuşta
   yürüdü, zorladığına dair işaret yok; S5'in eğimlerinde bakılmalı,
   gerekirse zemin normaline göre ölçülür.
4. **Uzun eğitimde deterministik davranışın kötüleşmesi** (ders 25, 28): en
   iyi ara kayıt otomatik seçiliyor; std'yi zamanla düşürmek denenmedi.
5. **Hızlıda yön kayması:** `ppo_omni_250k` 0.15 m/s'de 10 s'de −4°.
6. ROS'lu simde tork-hız doğrusu yok (gz_ros2_control hız sınırında torku
   kesiyor, RL simi doğrusal azaltıyor); ölçülen hızlar iki simde aynı
   düzeyde, şimdilik yeterli.
7. **Geri yürüyüşte mesafe sensörü yok** (önerilen yerleşimde): refleks
   orada kör kaldırmaya (~35 mm) dönüyor. D8 yerleşimine bağlı.
8. **Sensör gözlemli RL** (mesafe politikanın gözlemine girer): yerleşim ve
   gerçek sensör davranışı belli olunca.
9. **Kalibrasyon ofseti rastgeleleştirmesiyle eğitim:** ölçülen aralıkla D10'da.
10. **Robot düğümünde model seçimi** (düz / zemin; operatör seçer): küçük iş,
    değeri düşük.

### 13.3 Yazılım (Samet)

- **S5:** zemin üreteci. Entegrasyon notu GOREVLER.md'de: her zemin için
  `terrain_sdf` (statik `<model>` parçası) **ve** `terrain_height(x, y)`.
  Orijinin z=0'da olması artık gerekmiyor.
- **S6:** ölçüm aracı. Çekirdeği `hexapod_rl.evaluate` + `baseline.TripodPolicy`
  hazır; eksik olanlar zemin seçimi, N tekrar ve tablo.
- **S7:** sensör sürücüleri (saf Python, dry-run testli).

### 13.4 Donanım tarafı (⏸ durduruldu; GOREVLER.md D1–D12)

1. Bacaklara "ÖN" + 1–6 bandı (§5.5).
2. Kartlardan birinin A0'ını lehimle.
3. Kartları Pi'ye bağla (VCC 3.3 V!), servoları kartlara tak (sıra fark
   etmez), servo gücü ~6 V buck'tan.
4. Pi'ye Ubuntu Server 26.04 kur; `git clone`; `python3 tools/map_channels.py`
   → çıktıyı robot.yaml'a işle.
5. `python3 tools/calibrate.py` ile 18 eklemin merkez/yön/span/limitleri.
6. Güç bağlantısını kontrol et: servo hattı Pi'den ayrı mı, topraklar ortak
   mı, sigorta nerede.
7. **S4'ten devredilenler:** sürücü düğümü donanımda hiç denenmedi. Ayrıntılı
   liste: GOREVLER.md, "S4'ten devredilen, robotta doğrulanacaklar".

---

## 14. Sıradaki iş için hazır plan

**Görkem'in işi G7 (PPO + alan rastgeleleştirme).** Bitti şartı: politika
S6'nın ölçümünde tripod'u geçiyor; eğim, engebe ve kaygan zeminde ayrı ayrı
ölçüldü. Düz zeminde tripod'la başa baş; bu tavan (§12.28). Eksik olan asıl
zeminler (S5) ve ölçüm aracı (S6).

**S5 gelmeden yapılanlar (2026-09-26/27, 4. oturum; hepsi bitti):**

1. Gerçek ayak teması: zemin yüksekliği fonksiyonu (§7.8, ders 26).
2. Her yöne komut (`--omni`, `ppo_omni_250k`, §3.3); G8 her yön
   politikasıyla tekrarlandı (ROS'lu simde %96–104, §7.9).
3. En iyi ara kayıt otomatik; zemin durumlarını da ölçüyor (ders 30–32).
4. Kütle rastgeleleştirmesi (gövde ×0.9–1.6, ortam başına).
5. Deneme zeminleri (`terrain_probe`: basamak, çukur, yayla, eğim, engebe,
   kaygan) ve ROS'lu sim dünyaları.
6. Zeminli eğitim (`--terrains`, `--lift-mm`, `gorev.json`):
   `ppo_lift50_3750k`; tripod'la adil karşılaştırma (§3.4).
7. Düzde aşma ve enerji: ödülle çözülmedi (ders 33); sensörle çözüldü (13).
8. Öğrenilmiş ayak kaldırma (`widen`, `--lift-range`): kör politika zemine
   göre seçmedi (ders 35–36); `ppo_kaldirma35_250k` (§3.5).
9. Dayanıklılık taraması (`robustness`, ders 37, §3.7).
10. RL sim ↔ ROS'lu sim farkı: ölçüm hatasıydı, araç düzeltildi (ders 38).
11. Kolaydan zora müfredat (`--curriculum`, ders 40, §3.5); bu sırada
    bulunan soket sızıntısı giderildi (ders 39).
12. Mesafe sensörü simi (`rangefinder`) ve deneysel yerleşim karşılaştırması;
    D8'e öneri: ileri + ±90°, 20–25° aşağı (§3.6).
13. Mesafe sensörlü kaldırma refleksi (`lift_reflex`, `reflex_probe`,
    denetleyicide; ders 41, §3.6) ve refleks açıkken eğitim
    (`ppo_refleks_1500k`).

**Açık, S5/S6/S7 gelmeden yapılabilecek küçükler:** §13.2'deki 3–5, 9, 10.

**S7 + D8 gelince:** refleksi düğüme bağla (§13.2-1); `reflex_probe` ve
yerleşim karşılaştırmasını gerçek yerleşimle tekrarla; ROS'lu simde menzil
sensörüyle uçtan uca dene.

**S5 gelince:** S5'in üretecini `TRAIN_SETS`/`CURRICULA`'ya yaz;
`ppo_kaldirma35_250k` ve `ppo_refleks_1500k`'dan (refleksle) zeminli eğitim;
S6 tablosu (tripod ve politika her zeminde) → G7 bitti → G8'i son
politikayla tekrarla.

**Yöntem notları (bu projede işe yarayanlar):**
- Yeni bir ödül ya da ayar denemeden önce elle yazılmış iyi bir davranışı
  (tripod) aynı ödülde ölç. Hem gürültüsüz hem eğitimdeki gürültüyle ölç
  (ders 20).
- PPO'yu sıfırdan değil, taklitten başlat (ders 21). Artık eylem modu
  varsayılan tercih.
- Devam eğitimlerinde lr 1e-4, target_kl 0.02, düşük std (ders 25).
- Her ölçümü gürültüsüz, eklem gürültülü ve rastgeleleştirmeli ayrı raporla.
  Tabloyu `models/README.md`'ye yaz.
- Uzun eğitimde ara kayıtları (250k'da bir) değerlendir; kötüye gidiyorsa
  erken durdur. gSDE'de ve v10_omni'de böyle oldu. `ara_degerlendirme.csv`
  bunu eğitim sürerken gösteriyor; durdurma §12.27.
- Keşif gürültüsü, eylemin etkisiyle aynı zaman ölçeğinde olmalı (ders 36).
- Bir ödünleşimi ödülle zorlamadan önce eksik algıyı ve basit bir kuralı
  dene (ders 41).
- İki sim çelişirse önce ölçüm koşullarını karşılaştır (ders 38); kaynak
  sızıntısını tek süreçte değil gerçek koşulda (16 ortam) ölç (ders 39).
- Yeni PC'de: `--envs 16`; 3M adım ~30 dk (ara ölçümler dahil).

---

## 15. Depoda olmayan kaynakların özeti

**TÜBİTAK başvurusu** (Milli Teknoloji Kulüpler Birliği, Kulüp Geliştirme
Desteği Başvuru Formu; ekran görüntüsü olarak vardı, depoda değil):
- Konu: altı bacaklı robotların karmaşık ve değişken zeminlerde (engebeli,
  kumlu, eğimli) stabil ve verimli hareketi.
- Sorun: geleneksel kontrol önceden programlanmış sabit desenlere (CPG)
  dayanır; kuma saplanma, gevşek taşa basma, eğim gibi durumlarda enerji
  verimsizliği ve devrilme olur. IMU ve temas verisindeki anlık
  değişimlere dinamik tepki vermez.
- Öneri: kural tabanlı değil, **Pekiştirmeli Öğrenme (RL)** tabanlı, veri
  güdümlü ve modelden bağımsız (model-free) bir kontrol politikası. Ajan
  sanal ortamı (değişken araziler) ve iç durumunu (sensörler) gözleyip
  deneme-yanılmayla en yüksek ödülü (ör. devrilmeden ileri gitme)
  öğrenir; kaygan yüzeyde küçük adım atmak ya da engel tırmanırken farklı
  bacak sırası kullanmak gibi stratejileri kendisi keşfeder.
- Yöntem: **ROS + Gazebo**; sanal IMU ve temas sensörleriyle hexapod
  modeli; Python, **Stable-Baselines3, PPO**; ödül: devrilmeden, en az
  enerjiyle, en hızlı ilerleme.
- Plan: **12 ay**, dört ana başlık: simülasyon ortamının kurulması, RL
  algoritmasının kodlanması, robotun montajı, simülasyondaki en iyi
  sonuçlarla gerçek arazide deneme.
- Yaygın etki: adaptif kontrol algoritması; ulusal/uluslararası konferans
  sunumu; **kodların açık kaynak paylaşılması** (bu yüzden depoda kişisel
  veri olmaması ayrıca önemli).
- İlgili öncelikli alanlar: "Robotik-mekatronik", "Modelleme ve
  simülasyon teknolojileri".

**Diğer ekran görüntüleri** (referans amaçlı YouTube kareleri): başka bir
hexapod tasarımı ("I Redesigned My Hexapod In Fusion 360") ve DS3225
servo karşılaştırması (25 kg·cm). **DS3225 alınmadı**, robot MG996R ile.

**Faturalar:** içerik §4.1'de. Kişisel veri nedeniyle depoda değil.

**Eğitim ara kayıtları:** WSL içinde `~/hexapod_runs/` (yeni PC, ~100 MB,
her eğitimin `checkpoints/` klasörü; eski PC'dekiler taşınmadı). Hafif kısmı
(progress.csv, ayarlar, değerlendirmeler) depoda `egitim_kayitlari/`, önemli
modeller `models/`'da.

---

## 16. Yeni oturumda ilk adımlar

1. Bu belgeyi (`docs/PROJE_DEVIR.md`), `CLAUDE.md`'yi, `GOREVLER.md`'yi ve
   `models/README.md`'yi oku.
2. Yeni bilgisayardaysan ve kurulum yapılmadıysa §0.2'yi uygula. sudo'lu
   adımları kullanıcıya ver; her adım tek eylem, komutlar ayrı kod
   bloklarında.
3. Depoyu güncelle ve durumu doğrula:
   ```bash
   git pull --rebase
   git log --oneline | head -5
   python -m pytest -q
   ```
   - Windows: 235 geçti, 9 atlandı. WSL: 286 geçti.
   - Samet yeni test eklediyse sayı artmış olabilir; düşmüşse incele.
4. `git status`'ta beklenmeyen değişiklik varsa kullanıcının ya da Samet'in
   olabilir; dokunmadan incele (§12, madde 7–8).
5. GOREVLER.md'de Samet'in ilerlemesine bak (S5, S6, S7). S5 geldiyse zeminli
   eğitime, S7 geldiyse refleksin düğüme bağlanmasına geç (§14); gelmediyse
   §13.2'deki açıklardan devam et.
6. Kullanıcıdan donanım ya da ölçüm işi isteme.
7. Önemli bir karar ya da biten aşama olduğunda bu belgeyi, GOREVLER.md'yi ve
   gerekiyorsa `models/README.md`'yi güncelle. Commit + push'la.
