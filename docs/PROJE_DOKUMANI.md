# Elektrikli Araç Kontrol Sistemi (AKS) — Tam Proje Dokümanı

**Donanım:** STM32F407VET6 ("F4VE / JL-32F4XX" geliştirme kartı)
**Tarih:** 23 Ağustos 2026
**Kod durumu:** commit `8812f5a`

Bu dosya projenin **tamamını** anlatır: mimari, her pinin görevi ve ayarı, tüm
haberleşme protokolleri, çalışan mantık, **şu an takıldığımız nokta** ve
**Emerson yerleşik şarj cihazıyla ne yapmak istediğimiz**.

---

## 1. Sistem Mimarisi

```
┌──────────────────┐  CAN1 250k (PB8/PB9)   ┌────────────────────┐
│ Kelly KLS7245HC  │───────────────────────►│                    │
│ (motor sürücü)   │        29-bit ext.     │                    │
└──────────────────┘                        │                    │
                                            │                    │
┌──────────────────┐  CAN2 500k (PB12/PB13) │   STM32F407VET6    │  USART2 (PA2)
│ TommaTech / JK   │───────────────────────►│      168 MHz       │─────────────►  Raspberry Pi 5
│ BMS (Pylontech)  │        11-bit std.     │                    │  115200 8N1    (Python kadran)
└──────────────────┘                        │                    │
                                            │                    │
┌──────────────────┐  CAN 125k    ┌───────┐ │                    │
│ Emerson R48-1800 │─────────────►│MCP2515│─┤ SPI3 (PC10/11/12)  │
│ (yerleşik şarj)  │  29-bit ext. │8 MHz  │ │                    │
└──────────────────┘              └───────┘ └────────┬───────────┘
                                                     │
                   Optokuplör girişleri ─────────────┤
                   (sinyal, far, kapı, fren, vb.)    │
                                                     │
                   Röle kartı (AKTİF-LOW) ◄──────────┘
                   (sinyal, far, silecek, stop, kontaktör, uyarı)
```

### Neden MCP2515?
STM32'nin dahili iki CAN çevre birimi (CAN1, CAN2) zaten Kelly ve BMS tarafından
kullanılıyor. Emerson için üçüncü bir CAN gerektiğinden, SPI üzerinden çalışan
harici bir CAN denetleyicisi (MCP2515 + TJA1050) eklendi.

---

## 2. Saat (Clock) Yapılandırması

| Parametre | Değer |
|---|---|
| Osilatör | HSE (harici kristal, 8 MHz) |
| PLL | M=8, N=336, P=2 |
| SYSCLK / HCLK | **168 MHz** |
| APB1 | 42 MHz (`RCC_HCLK_DIV4`) — CAN1, CAN2, SPI3, USART2, I2C2 |
| APB2 | 84 MHz (`RCC_HCLK_DIV2`) — USART1, ADC1 |
| Flash gecikmesi | `FLASH_LATENCY_5` |
| LSE | 32.768 kHz (RTC için ayrılı, kullanılmıyor) |

---

## 3. TAM PİN HARİTASI

### 3.1 Optokuplör Girişleri (hepsi `GPIO_Input` + `GPIO_PULLUP`)

Sistem **aktif-LOW** çalışır: optokuplör iletime geçince pin GND'ye çekilir (0 okunur).

| Pin | Label | Görev | Okuma yönü |
|---|---|---|---|
| **PE2** | `DORTLU_BUTON` | Dörtlü flaşör (kalıcı switch) | `== OPTO_AKTIF` |
| **PC13** | `SAG_SIGNAL_KOL` | Sağ sinyal kolu | `== OPTO_AKTIF` |
| **PE6** | `SOL_SIGNAL_KOL` | Sol sinyal kolu | `== OPTO_AKTIF` |
| **PE7** | `SAG_KAPI` | Sağ kapı switch'i | `== OPTO_AKTIF` |
| **PE9** | `SOL_KAPI` | Sol kapı switch'i | `== OPTO_AKTIF` |
| **PE8** | `FAR_BUTON` | Far anahtarı | `== OPTO_AKTIF` |
| **PE10** | `FREN_GIRIS` | Fren pedalı | `== OPTO_AKTIF` |
| **PA8** | `SILECEK_BILGISI` | Silecek switch'i (kalıcı) | `== OPTO_AKTIF` |

> **Not:** Geçmişte far, silecek ve fren girişleri ters kablolandığı için kodda
> `!= OPTO_AKTIF` (ters okuma) kullanılmıştı. Mevcut `8812f5a` sürümünde tümü
> normal (`==`) okunuyor. Bir giriş ters davranırsa çözüm: o satırdaki
> `==` ↔ `!=` değiştirilir.

### 3.2 Röle Çıkışları (hepsi `GPIO_Output` + `GPIO_PULLUP` + başlangıç `HIGH`)

Röle kartı **AKTİF-LOW**: pin **LOW** = röle **çeker**, pin **HIGH** = röle **bırakır**.

| Pin | Label | Görev |
|---|---|---|
| **PC0** | `SAG_ROLE` | Sağ sinyal lambası |
| **PC1** | `SOL_ROLE` | Sol sinyal lambası |
| **PC3** | `SILECEK_ROLE` | Silecek motoru |
| **PD2** | `FAR_ROLE` | Far |
| **PD3** | `UYARI_ROLE` | Buzzer + flaşör (acil durum uyarısı) |
| **PD4** | `STOP_ROLE` | Stop lambası |
| **PD5** | `KONTAKTOR_ROLE` | **Ana kontaktör** (şarj/sürüş geçişi) |

**Başlangıç güvenliği:** Tüm röle pinleri `HIGH` (bırakık) durumda başlar.
`USER CODE 1` bloğunda, `HAL_Init()`'ten bile **önce**, BSRR register'ı ile
doğrudan HIGH yapılır ve iç pull-up eklenir — böylece açılışta hiçbir röle çekmez.

### 3.3 MCP2515 (Emerson CAN köprüsü) — SPI3

| MCP2515 pini | STM32 pini | Label | Not |
|---|---|---|---|
| VCC | 5V | — | TJA1050 transceiver 5V ister |
| GND | GND | — | Ortak toprak şart |
| SCK | **PC10** | `SPI3_SCK` | |
| SO (MISO) | **PC11** | `SPI3_MISO` | 5V toleranslı |
| SI (MOSI) | **PC12** | `SPI3_MOSI` | |
| CS | **PD6** | `MCP_CS` | Yazılım kontrollü, başlangıç HIGH |
| INT | **PD0** | `MCP_INT` | EXTI0 falling + pull-up. **Kesme kodda kapatıldı** (fırtına riski); RX her 10 ms yoklamayla okunuyor |
| RESET | **PD1** | `MCP_RESET` | Aktif-LOW, başlangıç HIGH |
| CANH/CANL | Emerson CAN | — | 125 kbps |

**SPI3 ayarları:** Full-Duplex Master, 8-bit, CPOL=Low, CPHA=1Edge (Mode 0),
NSS Soft, Prescaler 32 (≈1.3 MHz), MSB First.

### 3.4 Haberleşme ve Diğer

| Pin | Görev | Ayar |
|---|---|---|
| **PB8 / PB9** | CAN1_RX / CAN1_TX — **Kelly** | 250 kbps: Prescaler 12, BS1 11TQ, BS2 2TQ, ABOM açık |
| **PB12 / PB13** | CAN2_RX / CAN2_TX — **BMS** | 500 kbps: Prescaler 6, BS1 11TQ, BS2 2TQ, ABOM açık |
| **PA2 / PA3** | USART2_TX / RX — **Raspberry Pi telemetri** | 115200 8N1 (**aktif hat**) |
| **PA9 / PA10** | USART1_TX / RX | Kurulu ama **kullanılmıyor** (PA9 hattı arızalı çıktı, PA2'ye taşındı) |
| **PA1** | ADC1_IN1 — gaz pedalı | 12-bit, 3 cycle örnekleme |
| **PC6** | TIM3_CH1 — gaz PWM çıkışı | Presc 83, Period 9999 → 100 Hz, %0'da tutuluyor |
| **PB10 / PB11** | I2C2_SCL / SDA | Kurulu, kullanılmıyor |
| **PA6** | `STATUS_LED` | Kalp atışı — 100 ms'de bir toggle (aktif-LOW) |
| **PA7** | `CAN_LED` | Kelly bağlıysa yanık (aktif-LOW) |
| **PA13 / PA14** | SWDIO / SWCLK | Programlama/debug |
| **PC14 / PC15** | LSE kristal | |
| **PH0 / PH1** | HSE kristal (8 MHz) | |

### 3.5 Raspberry Pi Bağlantısı

| STM32 | Yön | Raspberry Pi 5 |
|---|---|---|
| **PA2** (TX) | ───► | **Pin 10** (GPIO15 / RXD) |
| PA3 (RX) | ◄─── | Pin 8 (GPIO14 / TXD) — *kullanılmıyor, boş bırakılabilir* |
| **GND** | ─── | **Pin 6** (veya herhangi GND) |

Pi tarafında `raspi-config` → Serial Port: **login shell = NO**, **hardware = YES**.

### 3.6 Aktif Kesmeler (NVIC)

| Kesme | Durum |
|---|---|
| `CAN1_RX0_IRQn` | **Açık** — Kelly mesajları |
| `CAN2_RX0_IRQn` | **Açık** — BMS mesajları |
| `EXTI0_IRQn` (MCP_INT) | CubeMX'te açık ama **kodda kapatılıyor** (`HAL_NVIC_DisableIRQ`) |
| SysTick | Açık (HAL zaman tabanı) |

---

## 4. Polarite Makroları (tek satırdan çevrilebilir)

```c
#define ROLE_AKTIF_LOW   1     // 1 = röle kartı aktif-LOW (LOW=çeker) [DOĞRULANDI]
#define OPTO_AKTIF_LOW   1     // 1 = optolar aktif-LOW + pull-up      [DOĞRULANDI]
```

Bir röle veya giriş ters çalışırsa ilk bakılacak yer buradır.

---

## 5. Haberleşme Protokolleri

### 5.1 Kelly KLS7245HC (CAN1, 250 kbps, 29-bit)

| CAN ID | İçerik |
|---|---|
| `0x0CF11E05` | RPM (d0-1), **akım işaretli** (d2-3), gerilim ×10 (d4-5), hata bitleri (d6-7) |
| `0x0CF11F05` | Gaz (d0), kontrolcü sıc. d1−40, motor sıc. d2−30, yön (d4), switch (d5) |

Tümü little-endian. 300 ms veri gelmezse link "KOPUK".

**Hız hesabı:**
```
km/h = rpm × (dişli 18/31) × (teker çevresi π×0.55 m = 1.728 m) × 60/1000
     → kodda: hiz_x10 = rpm × 933 / 1550
```
Kalibrasyon tek sayı: `933`. (Geçmişte çap/çevre karışıklığı ve dişli 12/31→18/31
değişimi düzeltildi.)

### 5.2 BMS (CAN2, 500 kbps, 11-bit, Pylontech)

| CAN ID | İçerik |
|---|---|
| `0x351` | CVL / CCL / DCL / DVL limitleri |
| `0x355` | SOC / SOH (%) |
| `0x356` | Gerilim 0.01V, akım 0.1A (işaretli), sıcaklık 0.1°C |
| `0x359` | Koruma + alarm bayrakları |
| `0x35C` | Şarj/deşarj izin bitleri |
| `0x200` | Max/min hücre gerilimi (mV) |
| `0x150` | Kalan / toplam kapasite (0.1 Ah) |
| `0x364` | Paralel pack sayısı |

Batarya: **16S LFP** (CVL 58.4V = 16×3.65V ile doğrulandı). 3 s sessizlik = KOPUK.
**BMS/CAN2 bu projede hiç sorun çıkarmadı — en güvenilir veri kaynağı.**

### 5.3 Emerson R48-1800 (MCP2515 üzerinden, 125 kbps, 29-bit)

| CAN ID | Amaç |
|---|---|
| `0x0707F803` | Telemetri (multiplex: d0=seçici, değer d4-7 float32 **big-endian**) |
| `0x060F8003` | İkinci telemetri ID'si |
| `0x06000783` | Sorgu (master → doğrultucu) |
| `0x0607FF83` | Kontrol (voltaj/akım komutu) |

**MCP2515 bit-timing (8 MHz kristal, 125k):** `CNF1=0x01, CNF2=0xB1, CNF3=0x85`

**Emerson voltaj ayarı:** Cihaz fabrika çıkışı 53.4V idi; batarya için gereken
**57.4V**'a ayrı bir STM32 ile CAN üzerinden ayarlandı. Bu ayar **kalıcı** —
prizden çekilip takılınca doğrudan 57.4V'a çıkıyor, tazeleme gerekmiyor.

---

## 6. AKS (Araç Kontrol) Mantığı

### 6.1 Açılış
İlk **1 saniye** (`BOOT_GRACE_MS`) tüm girişler yok sayılır, tüm röleler bırakık
tutulur. Sebep: optokuplör beslemesi otururken girişler yanlış okunup röleleri
çektiriyordu.

### 6.2 Sinyal Sistemi
- Boşta: **iki lamba da sönük**
- Sol/sağ kol: o taraf **0.7 s** periyotla yanıp söner
- İki kol birden: **son seçilen kazanır**
- Dörtlü (kalıcı switch, doğrudan okunur): ikisi birlikte yanıp söner, önceliklidir

### 6.3 Diğer İşlevler
| İşlev | Davranış |
|---|---|
| **Far** | Anahtarın aynası (açık → far yanar) |
| **Silecek** | Kalıcı switch aynası |
| **Stop lambası** | Fren pedalının aynası |
| **Kapı switch'leri** | Sadece okunur, telemetriye gider (röle yok) |
| **Korna** | Sistemden çıkarıldı (manuel bağlandı) |

### 6.4 Uyarı Rölesi (PD3 — buzzer + flaşör)
Aşağıdakilerden **biri** olursa çeker (her biri `#define` ile aç/kapa):

| Şart | Sabit |
|---|---|
| Kelly **5 sn** veri göndermezse | `UYARI_KELLY_KOPUK_MS 5000` |
| Kelly hata bildirirse (`kel_fault≠0`) | `UYARI_KELLY_HATA` |
| BMS koruma/alarm bayrağı aktifse | `UYARI_BMS_KORUMA` |
| Motor ≥100°C veya kontrolcü ≥80°C | `UYARI_MOT_SIC_MAX/KTRL_SIC_MAX` |
| SOC ≤ %10 | `UYARI_SOC_MIN` |
| BMS kopuk | `UYARI_BMS_KOPUK` (varsayılan **kapalı**) |

Emerson takılıyken alarm **bastırılır** (şarjda Kelly bilerek kopuk gösterildiği için).

---

## 7. Emerson Şarj Sistemi — NE YAPMAK İSTİYORUZ

### 7.1 Amaç (basit hali)

```
Emerson prize TAKILDI  →  KONTAKTÖR açılıştaki halinin TERSİNE geçer
Emerson prizden ÇEKİLDİ →  KONTAKTÖR açılıştaki haline döner
```

Amaç güvenlik: **şarj sırasında motor sürücüye giden elektrik kesilsin**, araç
şarjdayken hareket edemesin. Fiş çekilince sistem normale dönsün, araç kullanılabilsin.

### 7.2 "Emerson takılı mı?" sorusunu nasıl cevaplıyoruz

Bu, projenin en zor kısmı oldu. Sırayla denenen yöntemler:

| Yöntem | Sonuç |
|---|---|
| Geçerli CAN çerçevesi say | ❌ Emerson kendiliğinden yayın yapmıyor, `tumFrame=0` |
| Belirli ID filtresi | ❌ Hiç eşleşme olmadı |
| REC (alım hata sayacı) hareketi | ❌ REC hep 0 — hatta alım aktivitesi yok |
| **TEC (gönderim hata sayacı) davranışı** | ✅ **ÇALIŞIYOR** |

**Neden TEC işe yarıyor:** Emerson kendiliğinden konuşmuyor ama bizim gönderdiğimiz
çerçeveleri **ACK'liyor**. CAN kuralı gereği TEC yalnızca **başarılı** gönderimde
azalır (−1), başarısızda +8 artar. Yani:

```
Emerson TAKILI  → ACK geliyor → TEC düşüyor / düşük kalıyor
Emerson ÇEKİLİ  → ACK yok     → TEC 128'e tırmanıp ÇAKILIYOR
```

### 7.3 Mevcut algoritma (commit `8812f5a`)

**Adım 1 — Her 500 ms'de "Emerson var mı?" ölçümü.**
Şunlardan biri olursa "var" sayılır ve zaman damgası (`obc_last_tick`) tazelenir:
1. **TEC azaldı** → en az bir ACK alındı (birincil, anında tetiklenir)
2. **TEC < 64** → link sağlıklı
3. **REC hareket etti** → gerçek yayın (yedek, hiç olmuyor)

Açılış/reset sonrası **4 saniye** (`EMERSON_TEC_GUARD_MS`) karar verilmez —
TEC her zaman 0'dan başladığı için sahte "takılı" olmasın diye.

**Adım 2 — Çekme/bırakma kararı.**
```c
DEBOUNCE_MS = 2000   // sinyal KESİNTİSİZ 2 sn varsa → ÇEK
HOLDOFF_MS  = 6000   // sinyal 6 sn boyunca YOKSA   → BIRAK
```

**Adım 3 — Kontaktör sürüşü (kenar tetikli, her geçiş bir kez yazılır).**
```c
obc_link 0→1  →  PD5 = ROLE_ON  (çeker)
obc_link 1→0  →  PD5 = ROLE_OFF (bırakır)
```
Aradaki turlarda pine hiç dokunulmaz — titreme olmaz.

### 7.4 Telemetri Alanları (JSON, 100 ms'de bir)

**Kelly:** `spd, rpm, volt, amp, pwm, tmp, ctmp, dir, sw, flt, kelok, can`
**AKS:** `sig, lsol, lsag, far, brk, ksag, ksol, sil, uyari`
**BMS:** `bsoc, bsoh, bvolt, bcur, btmp, bcmax, bcmin, brem, bfull, bprot, balrm, bmsok`
**Emerson:** `sarj, obc, mcp, chgpct, ovout, oiout, otmp, orx, orxall, eflg, tec, rec`

Ayrıca 500 ms'de bir `RAW ...` (BMS ham) ve `OBC ...` (Emerson ham) satırları.

---

## 8. ŞU ANDA TAKILDIĞIMIZ NOKTA

### 8.1 Çalışanlar ✅
- Kelly, BMS okuma — sorunsuz
- Tüm AKS işlevleri (sinyal, far, silecek, stop, kapı, uyarı)
- Pi telemetrisi (PA2 üzerinden)
- **Emerson takılınca kontaktör doğru çekiyor** ve şarj boyunca çekili kalıyor
- Fiş çekilince kontaktör bırakıyor (Emerson öldükten ~6 sn sonra)

### 8.2 Açık Sorun ❌ — Bırakma anında BMS koruması

**Gözlemlenen zaman çizelgesi (log ile doğrulandı):**
```
t=0     Fiş çekildi
t=0-12  Emerson HÂLÂ ACK veriyor (can çekişiyor, TEC düşük)
t=12    Emerson tamamen ölüyor → TEC 128'e çakılıyor
t=18    6 sn hold-off doluyor → KONTAKTÖR BIRAKIYOR
t=18+   BMS korumaya giriyor → çıkışı kesiyor → STM32'nin de gücü gidiyor
```

Son log kanıtı: paket 10663'te `SARJ AKTIF`, paket 10664'te `SARJ yok`
(kontaktör bıraktı) — **ve veri tam orada bitiyor.**

### 8.3 Kritik Bulgu — Şarj hiç gerçekleşmiyor

Loglardaki SOC seyri:
```
71% → 55% → 48% → 47% → 46%      (sürekli DÜŞÜYOR)
53.20V → 52.50V → 52.40V         (gerilim de düşüyor)
Akım: hep 0.4-0.7A               (Emerson takılıyken de, takılı değilken de)
```

57.4V'luk bir kaynak 52.4V'luk bataryaya gerçekten bağlı olsaydı **onlarca amper**
akardı. 0.4A yalnızca STM32+Pi+rölelerin tüketimi. Yani **Emerson batarya ile
gerçek bir bağlantı kurmuyor** — muhtemelen kontaktör çekiliyken şarj hattı kesik.

**Bunun sonucu:** Kontaktör bıraktığında 57.4V'luk (veya kondansatörleri dolu)
Emerson çıkışı, 52.4V'luk bataryaya **aniden** bağlanıyor. 5V'luk fark düşük
empedans üzerinden büyük bir akım darbesi yaratıyor → BMS koruması → kesme.

**Neden bazen çalışıyor:** Emerson öldükten sonra çıkış kondansatörleri boşalıyor.
Bırakma yeterince geç olursa fark kapanmış olur, darbe küçük kalır, BMS atmaz.

### 8.4 Doğrulanması gereken tek şey

**Kontaktör çekiliyken Emerson gerçekten şarj ediyor mu?**
- Emerson'ın kendi göstergesinde akım var mı?
- Kontaktör hangi hattı kesiyor: motor sürücüyü mü, şarj hattını mı?

Bu cevap, çözümün yönünü belirler.

---

## 9. NASIL İLERLEYECEĞİZ

### Seçenek A — Bırakmayı geciktirmek (saf yazılım, en hızlı deneme)
`HOLDOFF_MS` 6000 → **25000**. Emerson'ın çıkış kondansatörleri tamamen boşalana
kadar bekleriz, gerilim farkı kapanır, darbe oluşmaz.
- ✅ Donanıma dokunmaz, tek satır
- ❌ Fiş çekildikten ~35 sn sonra sistem normale döner

### Seçenek B — Kontaktörün hangi hattı kestiğini doğrulayıp düzeltmek
Eğer kontaktör şarj hattını kesiyorsa mantık ters demektir; bu durumda şarj
sırasında bağlı kalır (gerçekten şarj olur) ve sonda ani bağlanma hiç olmaz.
- ✅ Kök nedeni çözer, şarj da gerçekten çalışır
- ❌ Kablolamanın doğrulanması gerekir

### Seçenek C — Ön-şarj (pre-charge) devresi
Kontaktöre paralel ~100 Ω / 10 W direnç + küçük röle. Sıra: önce direnç üzerinden
yavaşça doldur, sonra ana kontaktörü kapat.
- ✅ Elektrikli araçların standart çözümü, darbeyi tamamen bitirir
- ❌ Donanım eklemesi gerektirir

**Önerilen sıra:** Önce **8.4**'teki soruyu cevapla → sonra **A** (hızlı test) →
sonuç alınmazsa **B** veya **C**.

---

## 10. Ayar Noktaları (tek satır değişiklikler)

| Ne | Sabit | Değer |
|---|---|---|
| Açılış bekletme | `BOOT_GRACE_MS` | 1000 ms |
| Hız kalibrasyonu | `933UL / 1550UL` | dişli 18/31, teker 0.55 m |
| Kelly akım ölçeği | `KELLY_AKIM_BOL` | 1 |
| Kelly kopuk alarm süresi | `UYARI_KELLY_KOPUK_MS` | 5000 ms |
| Sıcaklık/SOC eşikleri | `UYARI_MOT_SIC_MAX / KTRL_SIC_MAX / SOC_MIN` | 100 / 80 / 10 |
| Emerson çekme süresi | `DEBOUNCE_MS` | 2000 ms |
| Emerson bırakma süresi | `HOLDOFF_MS` | 6000 ms |
| Emerson TEC eşiği | `EMERSON_TEC_ESIK` | 64 |
| TEC oturma süresi | `EMERSON_TEC_GUARD_MS` | 4000 ms |
| Röle/opto polaritesi | `ROLE_AKTIF_LOW` / `OPTO_AKTIF_LOW` | 1 / 1 |

---

## 11. Denenip Başarısız Olan Yaklaşımlar (tekrar denenmesin)

| Deneme | Sonuç |
|---|---|
| Emerson tespitinde ID filtresi | Hiç eşleşme yok, `tumFrame=0` |
| REC (alım hatası) tabanlı tespit | REC hep 0, işe yaramaz |
| "Çekiliyken TEC azalmasını sayma" (64/100/128 eşikleriyle) | **Üçü de titremeye yol açtı** — şarj sırasında kontaktörü ayakta tutan tek sinyal TEC azalması olduğu için |
| `HOLDOFF_MS` 2000'e düşürme | Titreme (şarjdaki TEC sıçramaları tolere edilemedi) |
| KESİN-YOK tespiti (TEC 120+ çakılı) | 6 sn korumayı deliyordu |
| BMS şarj akımı ile tespit (eşik 0.5A) | Boşta akım da ~0.5A, ayırt edemedi |
| SPI sessizlik penceresi | Faydası kanıtlanamadı |
| Hata yakalayıcılarda `NVIC_SystemReset` | Devreye hiç girmedi (CPU hata almıyor, gücü gidiyor) |
| Kademeli anahtarlama (300 ms) | Bir süre işe yaradı ama sonuç tutarsız |

---

## 12. Dosya Yapısı

| Dosya | İçerik |
|---|---|
| `Core/Src/main.c` | Tüm uygulama kodu (Kelly, BMS, MCP2515/Emerson, AKS, telemetri) |
| `Core/Inc/main.h` | Pin tanımları (CubeMX üretimi) |
| `Core/Src/stm32f4xx_it.c` | Kesme işleyicileri |
| `Core/Src/stm32f4xx_hal_msp.c` | Çevre birim düşük seviye init |
| `Stm32f407vet6_elektrikliaraba.ioc` | **CubeMX projesi** — pin/çevre birim yapılandırması |
| `Drivers/` | STM32 HAL + CMSIS kütüphaneleri |
| `PROJE_DOKUMANI.md` | **Bu dosya** |
| `CALISAN_SISTEM.md`, `CALISMA_OZETI_*.md` | Geçmiş çalışma kayıtları (kısmen eski) |

**Raspberry Pi tarafı** (bu depoda değil):
`araba_projesi/RaspberryPi/telemetry_uart.py` — otomatik port bulan kadran scripti.

---

## 13. Derleme ve Yükleme

1. STM32CubeIDE ile projeyi aç (`.ioc` dosyasına çift tıkla veya klasörü içe aktar)
2. **Project → Clean...** (eski nesne dosyaları kalmasın)
3. **Build** (Ctrl+B)
4. **Run/Debug** ile karta yükle (ST-Link)

> **Sık karşılaşılan tuzak:** Kod değiştirildikten sonra Clean+Build yapılmazsa
> karta eski binary yüklenebilir ve test yanıltıcı olur. Bu proje boyunca
> birkaç kez yaşandı.

**BOOT0 jumper'ı:** Normal çalışma için **GND** konumunda olmalı. HIGH okunursa
çip bootloader'a girer, kod hiç çalışmaz (LED yanmaz, telemetri gelmez).
