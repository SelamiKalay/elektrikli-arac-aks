# Çalışma Özeti — 6-7 Temmuz 2026

STM32F407VET6 araç beyni: kontaktör/röle açılış sorunlarının çözümü, polarite
düzeltmesi ve Emerson şarj entegrasyonu üzerine yapılan tüm çalışmanın kaydı.

> **ÖNEMLİ:** `CALISAN_SISTEM.md` artık ESKİ düzeni (aktif-HIGH) anlatıyor,
> röle/kontaktör bölümleri GÜNCEL DEĞİL. Güncel gerçek bu dosyadadır.

---

## 1. Başlangıçtaki Sorun

Batarya açıldığında:
- Kontaktör rölesi arka arkaya çekip bırakıyordu (şak-şak-şak, 3-4 kez)
- Bazı denemelerde tüm röleler bir anda çekiyordu
- Sonuç: Kelly kondansatör inrush → BMS aşırı akım koruması → **batarya arızaya düşüyordu**

## 2. Kök Neden (kesinleşen teşhis)

**Röle kartı AKTİF-LOW** (IN → GND = röle çeker; ölçümle doğrulandı).
Koddaki `ROLE_AKTIF_LOW 0` (aktif-HIGH) varsayımı YANLIŞTI. Bu yüzden:

- Kod "röleleri KAPALI tut" derken pinleri **LOW** basıyordu → aktif-LOW kartta bu **ÇEK** demek
- Açılış yumuşatması (BOOT_GRACE) tüm röleleri "OFF"=LOW tutuyordu → **tüm röleler açılışta çekiyordu**
- Toplu bobin akımı besleme çökertip STM32'yi brown-out reset'e sokuyordu → reset döngüsü = şak-şak-şak
- Kontaktör de bu kaosta çekip bırakınca inrush → BMS arıza

Ek bulgu: kodun boşta-çek mantıkları da (seyir lambaları sabit yanık, far/silecek
ters okuma) aktif-LOW'da rölelerin boşta çekik kalmasına yol açıyordu.

## 3. Yapılan Değişiklikler (ÇALIŞAN GÜNCEL DÜZEN)

### 3.1 Pin değişimi
| Görev | Eski pin | Yeni pin |
|---|---|---|
| KONTAKTOR_ROLE | PD3 | **PD5** |
| UYARI_ROLE | PD5 | **PD3** |

`main.h` + `.ioc` güncellendi (CubeMX içinden label değişimiyle). Donanımda
kontaktör sinyali PD5'e, uyarı sinyali PD3'e bağlandı.

### 3.2 Polarite (main.c)
- `ROLE_AKTIF_LOW = 1` → ROLE_ON = LOW (çeker), ROLE_OFF = HIGH (bırakır)
- UYARI makroları artık diğerleriyle aynı: `UYARI_ON = ROLE_ON`
- USER CODE 1 (HAL_Init'ten önce): tüm röle pinleri BSRR ile **HIGH + iç PULL-UP**
  → açılışta hiçbir röle çekmez
- USER CODE 2: kalıcı PULL-UP (belt-and-suspenders)

### 3.3 CubeMX (7 röle pini: PC0, PC1, PC3, PD2, PD3, PD4, PD5)
- GPIO output level = **High**
- Mode = **Output Push Pull**
- Pull = **Pull-up**

(Open-drain denendi, bu kartta işe yaramadı → push-pull'a dönüldü.)

### 3.4 Boşta hiçbir röle çekmez (mantık ayarları)
- **Sinyal:** boşta İKİSİ DE KAPALI (eski "seyir=ikisi sabit yanık" kaldırıldı).
  Kol çekince sadece o taraf 0.7 s blink; dörtlüde ikisi blink.
- **Far / Silecek:** TERS okuma kaldırıldı → `== OPTO_AKTIF` (buton/switch aktif
  = opto LOW = röle çeker; boşta çekmez).
- **Stop:** değişmedi (fren basılı = çeker).
- **Uyarı:** alarm varsa çeker, yoksa bırakık.

### 3.5 Kontaktör (PD5) — KENAR TETİKLİ, tek çekiş/tek bırakış
```
Emerson'dan veri gelmeye başladı (3 ardışık gerçek frame, obc_link 0→1)
    → BİR KEZ ÇEKER (ışık yanar, motor/sistem kesilir)
Emerson fişten çekildi (2 sn sessizlik, obc_link 1→0)
    → BİR KEZ BIRAKIR (açılıştaki hal: HIGH, bırakık, ışık sönük)
Aradaki döngü turlarında pine HİÇ dokunulmaz.
```
Açılış/normal/sürüşte kontaktör rölesi hiç oynamaz. Eski "NORMAL modda çek"
durum makinesi kaldırıldı (KONTAKTOR_SARJ_BEKLE/ZORLA define'ları kullanılmıyor,
duruyorlar).

### 3.6 Test sonucu
**7 Temmuz: sistem bu düzenle ÇALIŞTI** — açılışta sessiz, batarya arıza yok.

## 4. Yedekler (masaüstünde)

| Dosya | İçerik |
|---|---|
| `CALISAN_main_2026-07-07.txt` | Çalışan main.c |
| `CALISAN_main_h_2026-07-07.txt` | Çalışan main.h (pin swap dahil) |
| `CALISAN_ioc_2026-07-07.txt` | Çalışan .ioc |
| `main_yedek_2026-07-06.txt` | ⚠️ ESKİ bozuk düzen — geri yükleme! |

## 5. AÇIK SORUNLAR (çözüm bekliyor)

### 5.1 Raspberry Pi kadranı "veri bekleniyor"da kalıyor
- Yedek .txt ile güncel kod **tam diff** ile karşılaştırıldı:
  `send_telemetri()`, RAW/OBC dökümleri, boot mesajı, USART1 init, periyotlar
  **birebir aynı** → sorun firmware gönderiminde DEĞİL.
- Port `/dev/ttyAMA0` idi; "Bağlandı" yazıp veri gelmemesi = port açılıyor ama
  hatta byte yok → en olası: **PA9(TX) kablosu / ortak GND** (PD3↔PD5 kablo
  değişimi sırasında oynamış olabilir) veya STM32 o an yayın yapmıyor.
- Teşhis aracı hazır: `araba_projesi/RaspberryPi/tani_thonny.py` (Thonny'de
  çalışır; tüm portları tarar, veri akan portu bulur, ham satırları ve
  mcp/obc/orx/tumF özetini basar).

### 5.2 Emerson takıldı ama kontaktör çekmedi
- Emerson veri yolladı (arıza yok) AMA: MCP2515 frame'leri STM32 okumasa bile
  CAN'de kendiliğinden ACK'ler → "Emerson mutlu" ≠ "STM32 okudu".
- `obc_link` 1 olmadıysa kontaktör çekmez. Teşhis telemetrideki alanlarla:
  - `mcp:0` → MCP2515 init başarısız (SPI hattı)
  - `mcp:1, orxall:0` → MCP hazır ama frame okunamıyor
  - `orxall>0, orx:0` → frame var ama ID'ler `0x0707F803/0x060F8003` değil →
    ekrandaki `OBC XXXXXXXX ...` ham satırlarından gerçek ID'ler alınıp koda eklenecek
- Önce 5.1 çözülmeli (seri hat), çünkü teşhis verisi oradan okunacak.

## 6. Sonraki Adımlar

1. `tani_thonny.py`'yi Pi'de çalıştır → hangi portta veri var / yok gör
2. Veri yoksa: PA9→Pi RX kablosu + GND + STATUS_LED (PA6 kalp atışı) kontrol
3. Veri gelince: `mcp / orxall / orx / obc` değerlerini oku → kontaktör sorununun
   hangi katmanda olduğu netleşir → gereken düzeltme (muhtemelen ID listesi) yapılır
4. Kadran scripti (`telemetry_uart.py`) doğru porta sabitlenir
5. İstenirse `CALISAN_SISTEM.md` bu güncel düzene göre yeniden yazılır
