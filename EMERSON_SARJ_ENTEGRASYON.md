# Emerson Yerleşik Şarj Entegrasyonu — Değişiklik Özeti

STM32F407VET6 araç beynine **Emerson R48-1800** yerleşik şarj cihazının
**MCP2515 (SPI-CAN köprüsü)** üzerinden bağlanması, **kontaktör kontrolü** ve
**Raspberry Pi telemetrisi**. Yalnızca `Core/Src/main.c` düzenlendi; Kelly (CAN1),
BMS (CAN2), sinyal/far/silecek/fren rölesi mantığına **dokunulmadı**.

---

## 1. Sistem Mantığı

```
Emerson R48-1800  ──CAN 125k──►  MCP2515  ──SPI3──►  STM32F407  ──UART──►  Raspberry Pi
 (yerleşik şarj)   29-bit ext.  (8MHz krist.)                115200      (kadran / kayıt)
```

- Emerson verisi **CAN** ile gelir, MCP2515 bunu **SPI3**'e çevirir.
- STM32 veriyi çözer, **kontaktör** rölesini yönetir ve Pi'ye **JSON telemetri** basar.
- MCP2515 init/CAN başarısız olsa bile sistem **DURMAZ** (`mcp_hazir=0`, motor serbest).

---

## 2. Kontaktör / Motor Kontrol Kuralı

`Sarj_Kontrol()` fonksiyonu her döngü turunda çalışır:

| Durum | `sarj_aktif` | Kontaktör rölesi | Sonuç |
|-------|--------------|------------------|-------|
| Emerson takılı + veri var + batarya **%100 değil** | 1 (ŞARJ MODU) | **BIRAK** (`ROLE_OFF`) | — |
| Emerson fişten çekildi (2 sn sessizlik) **veya** batarya **%100** | 0 (NORMAL MOD) | **ÇEK** (`ROLE_ON`) | motor çalışabilir; Emerson'a "şarjı durdur" komutu |

> ⚠️ **Röle sürüşü TERS sisteme ayarlıdır:** ŞARJ MODU → `ROLE_OFF`, NORMAL MOD → `ROLE_ON`.
> Karar mantığı (`sarj_aktif` üretimi) değişmedi; yalnızca rölenin çekme yönü çevrildi.

- **Link kopukluğu:** `EMERSON_TIMEOUT_MS = 2000` (2 sn veri yoksa fişten çekildi sayılır).
- **Dolu eşiği:** `BMS_FULL_SOC = 100` (bu SOC'da batarya dolu kabul edilir).
- Açılışta kontaktör bir anlık `ROLE_OFF` başlar; ilk döngü turunda `Sarj_Kontrol()` doğru duruma getirir.

---

## 3. Pin Bağlantıları (MCP2515 — SPI3)

Header sırası: **VCC GND CS SO SI SCK INT**

| MCP2515 | STM32 | İşlev |
|---------|-------|-------|
| VCC | **5V** | Besleme (TJA1050 5V ister) |
| GND | **GND** | Ortak toprak (şart) |
| CS  | **PD6** | Chip Select |
| SO (MISO) | **PC11** | SPI3_MISO (5V toleranslı) |
| SI (MOSI) | **PC12** | SPI3_MOSI |
| SCK | **PC10** | SPI3_SCK |
| INT | **PD0** | EXTI0, falling edge, pull-up |
| CANH / CANL | Emerson CAN-H / CAN-L | 125 kbps |

- **RESET** pini modül header'ında yok → `PD1` boşta kalabilir (yazılım reset yeterli).
- 120 Ω sonlandırma: 2 uçlu hatta karttaki jumper açık bırakılır.
- Not: Dokümantasyon SPI1 (PA5/6/7) diyor; proje gerçekte **SPI3** kullanıyor.

---

## 4. Eklenen / Değişen Kod (`Core/Src/main.c`)

Tümü `USER CODE` bölgelerinde:

| Yer | Değişiklik |
|-----|-----------|
| Değişken tanımları | `obc_*`, `mcp_*`, `sarj_aktif` blokları `send_telemetri()`'den **önce** olacak şekilde dosya başına taşındı; `t_obc` zamanlayıcı eklendi |
| `#define`'lar | `R48_ID_CONTROL 0x0607FF83`, `BMS_FULL_SOC 100` eklendi |
| `send_telemetri()` | JSON'a OBC alanları eklendi, buffer 800 → 1024 |
| `HAL_GPIO_EXTI_Callback()` | **Yeni** — MCP_INT (PD0) düşünce `mcp_int_flag=1` |
| `R48_ChargeOff()` | **Yeni** — Emerson'a "çıkış akımı = 0" komutu (güvenli yön) |
| `Sarj_Kontrol()` | **Yeni** — kontaktör / motor durum makinesi |
| `main()` init | `mcp_hazir = MCP2515_Init();` + `t_obc` başlatma |
| `main()` döngü | `2c` bölümü: INT'te `MCP2515_Recv` boşaltma + 1 sn'de `R48_RequestData` + her tur `Sarj_Kontrol`; 500 ms'de `obc_raw_dump()` |

### Zaten var olan (aktifleştirilen) sürücü fonksiyonları
`MCP2515_Init / _Send / _Recv`, `MCP_ReadReg/WriteReg/BitModify`,
`R48_RequestData / R48_ParseFrame / R48_be_float`, `OBC_RawCapture / obc_raw_dump`
— bunlar önceden yazılıydı ama `main()`'e bağlı değildi; sadece bağlandı.

---

## 5. Raspberry Pi'ye Giden Yeni Telemetri Alanları (JSON)

Mevcut alanlara **dokunulmadan** eklendi:

| Alan | Anlam |
|------|-------|
| `sarj` | Şarj modu aktif mi (kontaktör kararı) 1/0 |
| `obc`  | Emerson takılı + yayında mı 1/0 |
| `mcp`  | MCP2515 init başarılı mı 1/0 |
| **`chgpct`** | **Şarj yüzdesi (SOC) — kadranda gösterilecek** |
| `ovout` | Emerson çıkış gerilimi (×10, 0.1V) |
| `oiout` | Emerson çıkış akımı (×10, 0.1A) |
| `ovin`  | Emerson giriş/şebeke gerilimi (×10, 0.1V) |
| `otmp`  | Emerson iç sıcaklık (°C, signed) |
| `orx`   | Emerson'dan alınan çerçeve sayacı |

Ayrıca her 500 ms **`OBC <sel> : b0..b7`** ham satırları basılır (byte eşlemesi doğrulama).

---

## 6. Doğrulanması Gerekenler (canlı test)

1. **`R48_ChargeOff()` payload'ı** — "şarjı durdur" byte düzeni R48 modeline göre
   değişebilir. Güvenli yönde yazıldı (çıkışı yalnızca azaltır); canlı CAN kaydıyla
   teyit edilip 4 bayt gerekirse güncellenir. Asıl güvenlik zaten kontaktördür.
2. **`R48_ParseFrame` byte offset'leri** — `OBC ...` ham satırlarıyla teyit edilip
   `switch (f->data[1])` selector'ları gerekirse düzeltilir.
3. **Röle polaritesi** — kontaktör kartının aktif-LOW/HIGH durumu; ters çalışırsa
   `ROLE_AKTIF_LOW` makrosu veya `Sarj_Kontrol` sürüş yönü ayarlanır.
