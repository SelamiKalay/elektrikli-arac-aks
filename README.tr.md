# Elektrikli Araç Kontrol Sistemi (AKS) — STM32F407

[English](README.md) | **Türkçe**

Kastamonu Üniversitesi Elektrikli Araç Takımı'nın Teknofest için geliştirdiği
elektrikli aracın araç kontrol sistemi (AKS) yazılımı. Tek bir STM32F407VET6; motor
sürücüyü, batarya yönetim sistemini ve yerleşik şarj cihazını üç ayrı CAN hattı
üzerinden okur, aracın aydınlatma ve güvenlik rölelerini yönetir ve Raspberry Pi
kadranına telemetri gönderir.

## Sistem Mimarisi

```
Kelly KLS7245HC  ──CAN1 250 kbps (29-bit)──┐
(motor sürücü)                              │
TommaTech / JK   ──CAN2 500 kbps (11-bit)──┤
BMS (Pylontech)                             ├──► STM32F407VET6 ──UART 115200──► Raspberry Pi 5
Emerson R48-1800 ──CAN 125 kbps──► MCP2515 ─┤      168 MHz        JSON / 100 ms  (Python kadran)
(yerleşik şarj)               (SPI3 köprü)  │
Optokuplör girişleri ───────────────────────┤  sinyal kolları, far, kapılar, fren …
Röle kartı (aktif-low) ◄────────────────────┘  sinyaller, far, silecek, stop lambası,
                                               ana kontaktör, uyarı (buzzer/flaşör)
```

Mikrodenetleyicinin iki dahili CAN birimi motor sürücü ve BMS tarafından
kullanıldığı için şarj cihazı, SPI üzerinden çalışan harici bir **MCP2515** CAN
denetleyicisiyle bağlanmıştır.

## Özellikler

- **Üç CAN hattı** — Kelly motor sürücü (CAN1), Pylontech protokollü BMS (CAN2),
  MCP2515 üzerinden Emerson şarj cihazı (SPI3)
- **Araç kontrol mantığı** — sinyaller (0.7 sn yanıp sönme, son seçilen kazanır),
  dörtlü flaşör, far, silecek, stop lambası, kapı switch'leri
- **Güvenli açılış** — ilk 1 saniye tüm girişler yok sayılır ve röleler bırakık
  tutulur; optokuplör beslemesi otururken rölelerin çekip bırakması engellenir
- **Yayın yapmayan şarj cihazının tespiti** — Emerson kendiliğinden veri göndermez
  ama gönderdiğimiz çerçeveleri *onaylar (ACK)*; fişin takılı olduğu MCP2515'in
  **gönderim hata sayacı (TEC)** davranışından anlaşılır, sahte tetiklemelere karşı
  debounce ve bekleme süreleri kullanılır
- **Kontaktör durum makinesi** (sürüş → şarj → çıkış bekleme → ön kapama) — şarj
  sırasında motor sürücünün enerjisi kesilir, araç hareket edemez
- **Uyarı rölesi** (buzzer + flaşör) — BMS haberleşmesi koptuğunda (açılıştan
  sonraki 15 sn beklemenin ardından 5 sn sessizlik), batarya sıcaklığı −15 °C … 55 °C
  aralığının dışına çıktığında, motor 100 °C'ye veya sürücü 80 °C'ye ulaştığında ya
  da NTC sensörü 50 °C'yi aştığında devreye girer; diğer koşullar (motor sürücü
  kopukluğu/hatası, BMS koruma bayrağı, düşük şarj seviyesi) kodda hazırdır ve
  `#define` ile açılabilir
- **Telemetri** — Raspberry Pi kadranına 100 ms'de bir JSON satırı (hız, akımlar,
  SOC, sıcaklıklar, röle/kontaktör durumu, CAN tanı bilgileri)

## Donanım

| Parça | Görev |
|---|---|
| STM32F407VET6 (F4VE kartı) | Araç kontrol birimi, 168 MHz |
| Kelly KLS7245HC | Motor sürücü |
| TommaTech / JK BMS | Batarya yönetimi (Pylontech CAN protokolü) |
| Emerson R48-1800 + MCP2515/TJA1050 | Yerleşik şarj cihazı ve SPI-CAN köprüsü |
| Röle kartı (aktif-low) + optokuplör girişleri | Aydınlatma, silecek, kontaktör, uyarı |
| Raspberry Pi 5 | Kadran (UART telemetrisini okur) |

## Derleme ve Yükleme

1. Projeyi **STM32CubeIDE** ile aç (`.ioc` dosyasına çift tıkla veya klasörü içe aktar)
2. **Project → Clean…**, ardından **Build** (Ctrl+B)
3. **Run/Debug** ile karta yükle (ST-Link)

> Normal çalışma için BOOT0 jumper'ı **GND** konumunda olmalıdır.

Uygulama kodu `Core/Src/main.c` içindeki `USER CODE` bölgelerindedir; geri kalanı
STM32CubeMX'in ürettiği kod ile ST HAL/CMSIS sürücüleridir.

## Dokümantasyon

Ayrıntılı tasarım notları [`docs/`](docs) klasöründedir:

| Dosya | İçerik |
|---|---|
| [`PROJE_DOKUMANI.md`](docs/PROJE_DOKUMANI.md) | Tam pin haritası, saat ayarı, protokoller, kontrol mantığı, şarj cihazı tespiti ve denenen yaklaşımlar |
| [`CALISAN_SISTEM.md`](docs/CALISAN_SISTEM.md) | Çalıştığı doğrulanan ilk sistemin anlatımı |
| [`CALISMA_OZETI_2026-07-07.md`](docs/CALISMA_OZETI_2026-07-07.md) | Çalışma kaydı: röle/kontaktör açılış sorunları ve polarite düzeltmeleri |
| [`EMERSON_SARJ_ENTEGRASYON.md`](docs/EMERSON_SARJ_ENTEGRASYON.md) | Emerson şarj entegrasyonu notları |

Bu notlar geliştirme sürecinde yazılmıştır; kodla farklı olduğu yerlerde güncel
başvuru kaynağı `Core/Src/main.c` dosyasıdır.

## Takım

**Kastamonu Üniversitesi Teknofest Elektrikli Araç Takımı** tarafından geliştirilmiştir.

- **Mustafa Enes Elbüz** — yazılım geliştirme, CAN entegrasyonu, dokümantasyon
- **Selami Kalay** — yazılım geliştirme, Emerson şarj tanı çalışmaları

Geliştirme geçmişinin tamamı commit kayıtlarında korunmuştur.

## Üçüncü Taraf Kod

`Drivers/` klasörü STMicroelectronics ve Arm'ın STM32F4 HAL ve CMSIS
kütüphanelerini içerir; bunlar kendi lisanslarıyla dağıtılır (ilgili klasörlerdeki
`LICENSE.txt` dosyalarına bakın).
