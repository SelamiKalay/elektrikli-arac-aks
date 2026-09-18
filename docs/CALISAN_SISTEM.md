# Elektrikli Araç Kontrol Sistemi (AKS) — Çalışan Sistem Dokümantasyonu

STM32F407VET6 tabanlı araç beyni: Kelly motor sürücü + TommaTech/JK BMS + Emerson R48-1800
yerleşik şarj + araç kontrol (sinyal/far/silecek/kapı/stop/uyarı) + Raspberry Pi 5 kadran.

Bu dosya, uzun geliştirme/deneme sürecinin sonunda **çalıştığı doğrulanan** sistemin
mantığını anlatır. Kod: `Core/Src/main.c` (tek dosya, USER CODE bölgelerinde).

---

## 1. Genel Mimari

```
Kelly KLS7245HC ──CAN1 250k (PB8/PB9, 29-bit)──┐
TommaTech BMS  ──CAN2 500k (PB12/PB13, 11-bit)─┤
Emerson R48    ──CAN 125k──► MCP2515 ──SPI3────┤──► STM32F407VET6 ──USART1 115200──► Raspberry Pi 5
Opto girişler  ──(sinyal kolları, far, kapı..)─┤                                      (Python kadran)
Röle kartı     ◄──(sinyal, far, silecek, stop, kontaktör, uyarı)──┘
```

- **CAN1** = Kelly (motor sürücü), 250 kbps, extended ID. Prescaler 12 / BS1 11 / BS2 2.
- **CAN2** = BMS, 500 kbps, standart ID (Pylontech protokolü). Prescaler 6 / BS1 11 / BS2 2.
- **SPI3 + MCP2515** = Emerson yerleşik şarj (dahili CAN'ler dolu olduğu için harici köprü).
  MCP2515 kristali **8 MHz**, CAN 125 kbps (CNF1=0x01 CNF2=0xB1 CNF3=0x85), SPI ~1.3 MHz.
- **USART1** = Pi'ye JSON telemetri, 115200 8N1, her 100 ms.
- Saat: HSE 8 MHz → PLL (M=8, N=336, P=2) → **168 MHz**, APB1=42 MHz, APB2=84 MHz.

## 2. Pin Haritası

### Girişler (optokuplör, hepsi GPIO_Input + PULL-UP)
| Pin | Görev | Okuma |
|---|---|---|
| PE2 | Dörtlü (kalıcı switch) | aktif-LOW (`== OPTO_AKTIF`) |
| PC13 | Sağ sinyal kolu | aktif-LOW |
| PE6 | Sol sinyal kolu | aktif-LOW |
| PE7 | Sağ kapı switch | aktif-LOW (elektrik var = kapı açık) |
| PE9 | Sol kapı switch | aktif-LOW |
| PE8 | Far butonu | **TERS** (`!= OPTO_AKTIF`) |
| PE10 | Fren | aktif-LOW |
| PA8 | Silecek (kalıcı switch) | **TERS** (`!= OPTO_AKTIF`) |
| PA1 | Gaz pedalı | ADC1_IN1 |

### Çıkışlar (röle kartı **AKTİF-HIGH**: HIGH = röle çeker)
| Pin | Görev |
|---|---|
| PC0 | SAG_ROLE (sağ sinyal lambası) |
| PC1 | SOL_ROLE (sol sinyal lambası) |
| PC3 | SILECEK_ROLE |
| PD2 | FAR_ROLE |
| PD3 | KONTAKTOR_ROLE (motor sürücü güç kontaktörü) |
| PD4 | STOP_ROLE (fren/stop lambası) |
| PD5 | UYARI_ROLE (buzzer+flaşör) — **TERS sürülür** (UYARI_ON=ROLE_OFF) |
| PA6 | STATUS_LED (kalp atışı, aktif-LOW) |
| PA7 | CAN_LED (Kelly link OK = yanık, aktif-LOW) |

### MCP2515 (SPI3)
| MCP2515 | STM32 | Not |
|---|---|---|
| VCC | 3.3V/5V | TJA1050'li mavi modül; SPI seviye sorunu yaşandıysa 3.3V |
| CS | PD6 | yazılım kontrollü |
| SO/MISO | PC11 | 5V toleranslı |
| SI/MOSI | PC12 | |
| SCK | PC10 | |
| INT | PD0 | EXTI0 falling + her 10 ms yoklama (INT'siz de çalışır) |
| RESET | PD1 | aktif-LOW donanım reset |

### Polarite makroları (main.c başında, tek satırla çevrilebilir)
```c
#define ROLE_AKTIF_LOW 0   // 0 = aktif-HIGH röle kartı (mevcut, doğrulanmış)
#define OPTO_AKTIF_LOW 1   // 1 = optolar aktif-LOW + pull-up (mevcut, doğrulanmış)
```

## 3. Açılış Sırası (doğrulanmış çalışan akış)

1. `USER CODE 1` (HAL_Init'ten önce): röle pinleri **register (BSRR) seviyesinde OFF**
   sürülür + output yapılır + iç pull (normal=PULL-DOWN, UYARI=PULL-UP). Amaç: açılış
   röle "tık"ını kod tarafında minimuma indirmek.
2. Normal CubeMX init'leri, ardından `USER CODE 2`: tüm röleler tekrar OFF, PWM %0,
   boot mesajı UART'a, CAN1/CAN2/MCP2515 başlatılır.
3. **AÇILIŞ YUMUŞATMA (`BOOT_GRACE_MS = 1000`)**: ilk 1 saniye tüm girişler yok sayılır,
   tüm röleler (kontaktör dahil) OFF tutulur. Sebep: optokuplör beslemesi otururken
   ters-okunan girişler (far/silecek) bir an "aktif" görünüp röleleri çektiriyordu.
4. 1 sn sonra normal çalışma başlar: seyir ışıkları yanar, kontaktör çeker (motor hazır).

> Not (donanım gerçeği): Güç/reset penceresinde (kod çalışmadan önce) pinler serbesttir;
> bu pencereyi hiçbir yazılım kapatamaz. Testlerde bootloader modunda (kod yokken)
> rölelerin ÇEKMEDİĞİ görüldü → serbest pin masum; asıl tetik kodun açılış okumasıydı,
> o da BOOT_GRACE ile çözüldü. Harici pull-down'a gerek kalmadı.

## 4. Ana Döngü Görevleri

- **0.7 s blink fazı**: sinyal yanıp sönmesi.
- **20 ms görevi**: girişleri oku → debounce → sinyal durum makinesi → röleleri sür → ADC.
- **CAN link sağlığı**: Kelly 300 ms, BMS 3 s, Emerson 2 s zaman aşımı.
- **500 ms**: ham CAN dökümleri (RAW/OBC satırları) + MCP tanı (EFLG/TEC/REC, bus-off kurtarma).
- **10 ms**: MCP2515 RX yoklaması (INT bayrağı + güvenlik yoklaması).
- **1 s**: Emerson'a telemetri isteği (`R48_RequestData`).
- **100 ms**: JSON telemetri + durum LED'leri. Kelly durumu değişince (koptu/geldi)
  100 ms beklemeden ANINDA paket gider ve rpm/akım/gaz sıfırlanır (kadran donmaz).

### Giriş debounce (EMI süzgeci)
Tüm opto girişleri tek bayta okunur; yeni durum ancak **4 ardışık örnekte (~80 ms)**
aynı okunursa kabul edilir. Milisaniyelik parazit süzülür, butonlar 80 ms gecikmeyle çalışır.

## 5. AKS Mantığı (araç kontrol)

- **Sinyal sistemi**: Sinyal LED'leri aynı zamanda **seyir lambası** — boşta İKİSİ DE SABİT YANIK.
  - Sol kol → sol 0.7 s blink, sağ sabit yanık. Sağ kol → tersi.
  - İki kol birden aktifse → **son seçilen kazanır** (`sig_son_sel`).
  - **Dörtlü** (kalıcı switch, doğrudan okunur): ikisi birlikte blink. Öncelik dörtlüde.
- **Far**: buton aynası — buton ON iken far ON (giriş ters okunur, `!=`).
- **Silecek**: kalıcı switch, doğrudan okunur (aç → çalışır, kapat → durur). Giriş ters.
- **Stop lambası**: fren girişi aynası.
- **Kapı switchleri**: sadece okunur, telemetriye gider (röle yok).
- **Korna**: sistemden ÇIKARILDI (manuel bağlandı).

### Uyarı / acil durum rölesi (PD5, buzzer+flaşör — TERS sürülür)
Şu şartlardan biri olursa çeker (her biri `#define` ile aç/kapa):
- Kelly'den **5 sn** veri gelmezse (`UYARI_KELLY_KOPUK_MS 5000`) — açılışta da doğal 5 sn tolerans
- Kelly hata bildiriyorsa (`kel_fault != 0`)
- BMS koruma/alarm bayrağı aktifse
- Motor ≥100°C veya kontrolcü ≥80°C
- SOC ≤ %10
- **Emerson takılıyken alarm BASTIRILIR** (şarjda Kelly bilerek kopuk gösterildiği için).

## 6. Kelly (motor sürücü, CAN1)

- MSG1 `0x0CF11E05`: rpm (d0-1), **akım İŞARETLİ** (d2-3, regen'de negatif), gerilim ×10 (d4-5), hata (d6-7).
- MSG2 `0x0CF11F05`: gaz (d0), kontrolcü sıc. d1−40, motor sıc. d2−30, yön (d4), switch (d5).
- Hepsi little-endian. Akım ölçeği: `KELLY_AKIM_BOL` (1 = ham amper).
- **Hız hesabı** (kalibre edilmiş):
  `km/h = rpm × (dişli 18/31) × (teker çevresi π×0.55 = 1.728 m) × 60/1000`
  → kodda `hiz_x10 = rpm * 933 / 1550`. Kalibrasyon tek sayı: 933 (büyüt = hız artar).
  (Tarihçe: önce çap/çevre karışıklığı π kat düşük gösteriyordu; sonra dişli 12/31→18/31 oldu.)

## 7. BMS (CAN2, Pylontech protokolü — ham veriyle doğrulandı)

| ID | İçerik |
|---|---|
| 0x351 | CVL/CCL/DCL/DVL limitleri (0.1V / 0.1A) |
| 0x355 | SOC / SOH (%) |
| 0x356 | Gerilim 0.01V / akım 0.1A işaretli (+şarj −deşarj) / sıcaklık 0.1°C |
| 0x359 | Koruma + alarm bayrakları |
| 0x35C | Şarj/deşarj izin bitleri |
| 0x200 | Max/min hücre gerilimi (mV) |
| 0x150 | Kalan / toplam kapasite (0.1 Ah) |
| 0x364 | Paralel pack sayısı |

Kadran SOC (`chgpct`) = BMS SOC. Batarya: 16S LFP (CVL 58.4V = 16×3.65V ile doğrulandı).

## 8. Emerson R48-1800 Yerleşik Şarj (MCP2515/SPI3)

- **Gerçek telemetri ID'leri (canlı capture ile bulundu):** `0x0707F803` (multiplex,
  d0=selector, değer d4-7 float32 big-endian) ve `0x060F8003`. Dokümandaki 0x06080783 YANLIŞTI.
- **Şarj algılama:** SADECE bu iki gerçek ID sayılır + **3 ardışık frame** şartı (debounce).
  Junk/gürültü ID'ler (0x00000000, 0x00FF0000, ...) yok sayılır — eskiden bunlar sahte
  şarj algılatıp kontaktörü aç-kapa yaptırıyordu (inrush → BMS arıza).
- 2 sn sessizlik = fiş çekildi (`EMERSON_TIMEOUT_MS 2000`).
- **Kadran V/I değerleri**: Emerson CAN'de gerçek Vout/Iout'u bu formatta vermediği için
  `ovout/oiout` **BMS'in gerçek şarj ölçümünden** doldurulur (bataryaya giren değer).
- Batarya %100 olursa `R48_ChargeOff()` (çıkış akımı 0 komutu) gönderilir.

### Şarj / Kontaktör durum makinesi (ÇALIŞAN HALİ — sade)
```
Emerson takılı + veri var + SOC < %100  →  ŞARJ MODU:  KONTAKTOR_ROLE = OFF (motor kesik)
Fiş çekildi (2 sn sessiz)  VEYA  SOC=%100 →  NORMAL MOD: KONTAKTOR_ROLE = ON  (motor hazır)
İlk 1 sn (açılış yumuşatma)              →  her zaman OFF
```
- Şarj takılıyken ayrıca: `kelly_ok` zorla 0 gösterilir (motor devre dışı kabul) ve
  uyarı buzzer'ı bastırılır.

## 9. Pi'ye Giden JSON (özet alanlar)

Kelly: `spd, rpm, volt, amp(işaretli), pwm, tmp, ctmp, dir, sw, flt, kelok` + CAN tanı.
AKS: `sig, lsol, lsag, far, brk, ksag, ksol, sil, uyari`.
BMS: `bsoc, bsoh, bvolt, bcur, btmp, bcvl/bccl/bdcl/bdvl, bcmax, bcmin, brem, bfull, bprot, balrm, bmsok`.
OBC: `sarj, obc, mcp, chgpct, ovout, oiout, otmp, orx, orxall, eflg, tec, rec`.
Ayrıca 500 ms'de `RAW ...` (BMS ham) ve `OBC ...` (Emerson ham) satırları (doğrulama için).

Pi tarafı: `araba_projesi/RaspberryPi/telemetry_uart.py` — tek ekran kadran; şarj
aktifken ŞARJ EKRANI (V/I/Güç kW/SOC/tahmini bitiş = (bfull−brem)/I).

## 10. Bilinen Sorunlar ve Çözümleri

| Sorun | Durum / Çözüm |
|---|---|
| **Fişten çekince BMS arızaya düşüyor** | AÇIK — fiziksel: şarjda kontaktör açık → Kelly kondansatörleri boşalıyor → geri bağlanınca ani akım (inrush) → BMS aşırı-akım koruması. Yazılım darbenin zamanını değiştirebilir, büyüklüğünü değiştiremez (denendi). **Kesin çözüm: PRE-CHARGE** — kontaktöre paralel ~100Ω/10W direnç + boş röle kanalı; sekans yazılıma eklenecek. |
| "Bazen hiç açılmıyor, reset düzeltiyor" | **BOOT0 jumper'ı** marjinal — çalışma konumunda (GND) SAĞLAM olmalı (gerekirse lehim). BOOT0 HIGH okunursa bootloader'a girer, kod hiç çalışmaz. |
| Açılışta tüm röleler bir kez çekiyordu | ÇÖZÜLDÜ — BOOT_GRACE (1 sn giriş yok sayma + röle OFF). Bootloader testi serbest pinin masum olduğunu kanıtladı; harici pull-down gerekmedi. |
| Sahte şarj algılama (junk CAN ID) | ÇÖZÜLDÜ — sadece gerçek Emerson ID + 3 frame şartı. |
| Kelly akımı saçma değer gösteriyordu | ÇÖZÜLDÜ — akım işaretli (int16), regen'de negatif. |
| Hız yanlıştı | ÇÖZÜLDÜ — çevre=π×çap düzeltmesi + dişli 18/31 → katsayı 933/1550. |
| Emerson sarı LED (CAN yok) | ÇÖZÜLDÜ — SPI hızı düşürüldü, alım yoklaması eklendi; TEC=REC=0, bus sağlıklı. |

## 11. Başarısız Denemeler (tekrar denenmesin)

- **Şarj dondurma + 300 ms debounce + kontaktörde 5 sn + BMS koruma kapısı**: açılışta
  sistemi bozdu (batarya direkt arızaya düştü) → tamamen geri alındı. Çalışan sade akış korundu.
- **Harici pull-down teorisi**: bootloader testi çürüttü (kod yokken röleler çekmiyor).
- Register-seviyesi erken GPIO init + iç pull: zararsız, duruyor; ama tek başına açılış
  sorununu çözmedi — çözen BOOT_GRACE oldu.

## 12. Ayar Noktaları (tek satır değişiklikler)

| Ne | Nerede | Varsayılan |
|---|---|---|
| Açılış bekletme | `BOOT_GRACE_MS` | 1000 ms |
| Hız kalibrasyonu | `933UL / 1550UL` | dişli 18/31, teker 0.55 m |
| Kelly akım ölçeği | `KELLY_AKIM_BOL` | 1 |
| Kelly kopuk alarm süresi | `UYARI_KELLY_KOPUK_MS` | 5000 ms |
| Alarm şartları | `UYARI_*` define'ları | hepsi 1, BMS_KOPUK 0 |
| Sıcaklık/SOC eşikleri | `UYARI_MOT_SIC_MAX/KTRL_SIC_MAX/SOC_MIN` | 100 / 80 / 10 |
| Emerson fiş-çekildi süresi | `EMERSON_TIMEOUT_MS` | 2000 ms |
| Batarya dolu eşiği | `BMS_FULL_SOC` | 100 |
| Röle/opto polaritesi | `ROLE_AKTIF_LOW` / `OPTO_AKTIF_LOW` | 0 / 1 |
| Debounce süresi | `db_say >= 3` (×20 ms) | ~80 ms |
