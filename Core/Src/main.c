/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body  (STM32F407VET6 - Kelly CAN telemetri)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== ROLE KARTI POLARITESI =====
 *   1 = AKTIF-LOW  : GPIO LOW  -> role CEKER (mavi opto-role kartlarinin cogu)
 *   0 = AKTIF-HIGH : GPIO HIGH -> role CEKER
 * KESIN: kart AKTIF-LOW (IN->GND = role ceker, olculdu). ROLE_ON=LOW.
 * ONEMLI: bosta HICBIR role cekik durmaz; role ancak fonksiyonu
 * aktiflesince ceker (asagida mantik buna gore duzenlendi). */
#define ROLE_AKTIF_LOW   1
#if ROLE_AKTIF_LOW
  #define ROLE_ON   GPIO_PIN_RESET
  #define ROLE_OFF  GPIO_PIN_SET
#else
  #define ROLE_ON   GPIO_PIN_SET
  #define ROLE_OFF  GPIO_PIN_RESET
#endif
/* ===== OPTOKUPLOR GIRIS POLARITESI =====
 *   1 = AKTIF-LOW  : aktif olunca pin GND'ye cekilir (0), giris PULL-UP  [gecmiste calisan]
 *   0 = AKTIF-HIGH : aktif olunca pin 3.3V (1), giris PULL-DOWN
 * Gecmiste CALISAN sistem AKTIF-LOW idi -> 1 yapildi.
 * Girisler ters gorunurse (bosta 'aktif' okunuyorsa) 0 yap. */
#define OPTO_AKTIF_LOW   1
#if OPTO_AKTIF_LOW
  #define OPTO_AKTIF  GPIO_PIN_RESET
  #define OPTO_PULL   GPIO_PULLUP
#else
  #define OPTO_AKTIF  GPIO_PIN_SET
  #define OPTO_PULL   GPIO_PULLDOWN
#endif
/* ===== TELEMETRI UART SECIMI =====
 * PA9/PA10 (USART1) hatti suphesi nedeniyle telemetri USART2'ye tasindi:
 *   YENI KABLO: PA2 = TX -> Pi pin10 (GPIO15 RX),  PA3 = RX (bos kalabilir), GND ortak.
 * USART1'e geri donmek icin asagidaki satiri 'huart1' yap (baska degisiklik gerekmez). */
#define TELEM_UART  huart2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===================================================================
 *  Kelly KLS7245HC -> CAN (250 kbps, 29-bit genisletilmis ID, J1939)
 *  Cihaz ID = 0x05 (fabrika varsayilani). Degisirse SON baytini duzelt.
 *    MSG1 0x0CF11E05 : rpm, motor akimi, batarya gerilimi, hata bitleri
 *    MSG2 0x0CF11F05 : gaz, kontrolcu/motor sicakligi, yon, switch'ler
 *  Tum coklu-bayt alanlar LITTLE-ENDIAN.
 * =================================================================== */
#define KELLY_CAN_ID_MSG1   0x0CF11E05UL
#define KELLY_CAN_ID_MSG2   0x0CF11F05UL
#define KELLY_TIMEOUT_MS    300U        /* 300ms veri yoksa link KOPUK */
/* Kelly akim olcegi:  gosterilen A = ham(d2-3) / KELLY_AKIM_BOL
 *   1  -> ham dogrudan amper (1A cozunurluk)   [varsayilan]
 *   10 -> ham 0.1A cozunurluk ise
 * Surucu ekraniyla eslesmiyorsa SADECE bu sayiyi degistir. */
#define KELLY_AKIM_BOL      1

/* Kelly'den CAN ile cozulen veriler */
static volatile uint16_t kel_rpm      = 0;   /* devir/dak            */
static volatile uint16_t kel_volt_x10 = 0;   /* gerilim x10 (V)      */
static volatile int16_t  kel_cur  = 0;   /* motor akimi (ham d2-3, Amper) - ISARETLI: regen/frende negatif */
static volatile uint16_t kel_fault    = 0;   /* hata bitleri         */
static volatile uint8_t  kel_throttle = 0;   /* gaz ham (0..255)     */
static volatile int16_t  kel_ctrl_tmp = 0;   /* kontrolcu sic. (C)   */
static volatile int16_t  kel_mot_tmp  = 0;   /* motor sic. (C)       */
static volatile uint8_t  kel_dir      = 0;   /* yon                  */
static volatile uint8_t  kel_sw       = 0;   /* switch baytti        */
static volatile uint32_t kel_msg1_cnt = 0;
static volatile uint32_t kel_msg2_cnt = 0;
static volatile uint32_t kel_last_tick = 0;
static uint8_t  kelly_ok = 0;

/* CAN tani (Pi'de gorunur) */
static volatile uint8_t  can_hazir = 0;
static volatile uint8_t  can_step  = 0;   /* 0=ok 1=filtre 2=start 3=notify */
static volatile uint8_t  can_state = 0;
static volatile uint32_t can_err   = 0;
static volatile uint16_t mcu_devid = 0;   /* F407 = 0x413 */
static volatile uint8_t  can_clk   = 0;

/* ===================================================================
 *  TommaTech PB / JK-BMS -> CAN2 (500 kbps, 11-bit STANDART ID, Pylontech)
 *  Little-endian, int16 olcekli. Sadece 0x351..0x35F dinlenir.
 *  Kelly (CAN1) tamamen ayri, bu blok ona DOKUNMAZ.
 * =================================================================== */
#define BMS_TIMEOUT_MS   3000U   /* BMS ~1s yayinlar; 3s sessizlik = KOPUK */

static volatile uint16_t bms_soc       = 0;   /* %          (0x355)        */
static volatile uint16_t bms_soh       = 0;   /* %          (0x355)        */
static volatile uint16_t bms_volt_x100 = 0;   /* 0.01 V     (0x356)        */
static volatile int16_t  bms_cur_x10   = 0;   /* 0.1 A  +sarj -desarj (0x356) */
static volatile int16_t  bms_temp_x10  = 0;   /* 0.1 C  signed (0x356)     */
static volatile uint16_t bms_cvl_x10   = 0;   /* sarj  V limit 0.1V (0x351) */
static volatile int16_t  bms_ccl_x10   = 0;   /* sarj  A limit 0.1A (0x351) */
static volatile int16_t  bms_dcl_x10   = 0;   /* desarj A limit 0.1A (0x351) */
static volatile uint16_t bms_dvl_x10   = 0;   /* desarj V limit 0.1V (0x351) */
static volatile uint8_t  bms_prot0     = 0;   /* koruma bayrak 1  (0x359)  */
static volatile uint8_t  bms_prot1     = 0;   /* koruma bayrak 2  (0x359)  */
static volatile uint8_t  bms_alarm0    = 0;   /* alarm  bayrak 1  (0x359)  */
static volatile uint8_t  bms_alarm1    = 0;   /* alarm  bayrak 2  (0x359)  */
static volatile uint8_t  bms_packs     = 0;   /* paralel batarya sayisi (0x364) */
static volatile uint16_t bms_cmax_mv   = 0;   /* max hucre gerilimi mV (0x200) */
static volatile uint16_t bms_cmin_mv   = 0;   /* min hucre gerilimi mV (0x200) */
static volatile uint8_t  bms_chg_flags = 0;   /* sarj/desarj izin bayrak (0x35C d0) */
static volatile uint16_t bms_rem_ah_x10 = 0;  /* kalan kapasite 0.1Ah (0x150)  */
static volatile uint16_t bms_full_ah_x10 = 0; /* toplam kapasite 0.1Ah (0x150) */
static volatile uint32_t bms_rx_cnt    = 0;
static volatile uint32_t bms_last_tick = 0;
static volatile uint8_t  bms_can_hazir = 0;   /* CAN2 basladi mi (1/0)     */
static uint8_t  bms_ok = 0;                    /* BMS link sagligi          */

/* HAM CAN2 TARAMA: gelen her ID'yi islemeden Pi'ye dokmek icin tablo.
 * Amac: hangi ID hangi ham baytlarla geliyor GORMEK, sonra decode'u dogrulamak. */
#define RAW_MAX 24
static volatile uint32_t raw_id[RAW_MAX];      /* gorulen distinct ID'ler   */
static volatile uint8_t  raw_d[RAW_MAX][8];    /* her ID'nin son 8 bayti    */
static volatile uint8_t  raw_dlc[RAW_MAX];     /* veri uzunlugu             */
static volatile uint32_t raw_cnt[RAW_MAX];     /* kac kez geldi             */
static volatile uint8_t  raw_n = 0;            /* tabloda kac ID var        */

/* Arac kontrol degiskenleri */
static uint32_t pkt_id      = 0;
static uint8_t  sig_state   = 0;   /* 0=SEYIR(ikisi yanik) 1=SOL 2=SAG 3=DORTLU */
static uint8_t  dortlu_aktif = 0;
static uint8_t  fren_aktif   = 0;
static uint16_t gaz_adc      = 0;

/* AKS (arac kontrol sistemi) - sinyal/far/silecek/kapi/stop (korna MANUEL, cikarildi) */
static uint8_t  far_aktif    = 0;   /* far butonu aktif mi          */
static uint8_t  sag_kapi     = 0;   /* sag kapi acik mi (switch)    */
static uint8_t  sol_kapi     = 0;   /* sol kapi acik mi (switch)    */
static uint8_t  sig_son_sel  = 0;   /* son secilen sinyal: 1=SOL 2=SAG */
static uint8_t  led_sol      = 0;   /* sol sinyal LED anlik (telemetri) */
static uint8_t  led_sag      = 0;   /* sag sinyal LED anlik (telemetri) */
static uint32_t t_blink      = 0;   /* 0.7s blink zamanlayici       */
static uint8_t  blink_on     = 0;   /* blink fazi (0/1)             */
static uint8_t  silecek_aktif = 0;  /* silecek toggle durumu (bas-ac/bas-kapa) */
static uint8_t  uyari_aktif   = 0;  /* UYARI rolesi (buzzer+flasor) durumu */

/* ===== UYARI / ACIL DURUM ROLESI (tek pin: UYARI_ROLE, far gibi surulur) =====
 * Asagidaki sartlardan HERHANGI biri olusursa role cekilir (buzzer+flasor
 * donanimi kendi calisir). Her sarti 1/0 ile ac-kapa, esikleri ayarla. */
#define UYARI_KELLY_KOPUK   1   /* motor surucu CAN kopuk */
#define UYARI_KELLY_HATA    1   /* Kelly kontrolcu hata bildiriyor (kel_fault) */
#define UYARI_BMS_KORUMA    1   /* BMS koruma/alarm bayragi aktif */
#define UYARI_SICAKLIK      1   /* motor/kontrolcu asiri sicak */
#define UYARI_DUSUK_SOC     1   /* kritik dusuk batarya */
#define UYARI_BMS_KOPUK     0   /* BMS CAN kopuk (BMS her zaman takili degilse 0 birak) */
#define UYARI_MOT_SIC_MAX   100 /* C - motor sicaklik esigi */
#define UYARI_KTRL_SIC_MAX  80  /* C - kontrolcu sicaklik esigi */
#define UYARI_SOC_MIN       10  /* % - dusuk batarya esigi */
#define UYARI_KELLY_KOPUK_MS 5000U /* Kelly'den bu kadar sure veri gelmezse alarm (5 sn) */
/* ACILIS YUMUSATMA: ilk N ms girisler YOK SAYILIR, TUM roleler OFF tutulur.
 * Sebep: opto beslemesi otururken ters-okunan girisler (far/silecek) bir an
 * "aktif" gorunup TUM roleleri cektiriyordu; kontaktor de aninda cekince
 * Kelly kondansator inrush'i BMS'i arizaya dusurebiliyordu. */
#define BOOT_GRACE_MS 1000U
/* SARJDAN CIKIS BEKLETMESI: fisten cekildigi haber alininca (obc_link 1->0)
 * ACILISTAKI GIBI bu kadar sure tum roleler OFF bekletilir (optolar/sistem
 * otursun), sonra normal calismaya gecilir. */
#define SARJ_CIKIS_GRACE_MS 1000U
/* KONTAKTOR SARJ-SONRASI BEKLEME:
 * Gozlem: fisten cekilince kontaktor + isiklar cekiyor, ~0.5-1 sn sonra BMS
 * koruma atiyor. Sebep: sarj yeni bittigi icin BMS DESARJ iznini henuz
 * acmamis oluyor; Kelly baglaninca "izinsiz desarj" koruma atiyor.
 * Cozum: kontaktor fis cekildikten sonra EN AZ su kadar bekler VE BMS'in
 * desarj izni bayragini (0x35C bit6) bekler; izin gelmezse emniyet suresinde
 * yine de ceker (arac kilitli kalmasin). */
#define KONTAKTOR_SARJ_BEKLE_MS  5000U   /* fis cekildikten sonra minimum bekleme  */
#define KONTAKTOR_ZORLA_MS      15000U   /* izin gelmese de en gec bu surede cek   */
/* SARJ SONRASI ALARM SUSTURMA (log kaniti 04:53:28): fis cekilince 1 sn'de
 * 'Kelly kopuk' alarmi patliyordu — oysa kontaktor 5 sn bekliyor, Kelly ondan
 * sonra acilip CAN'e basliyor. Buzzer+flasorun ani yuku girisleri (far/silecek/
 * kapi) surekli yanlis okutup o roleleri de cektiriyordu -> BMS arizasi.
 * Cozum: fisten cekildikten sonra bu sure boyunca Kelly-kopuk alarmi CALMAZ
 * (Kelly'ye toparlanma suresi). Kelly gercekten yoksa sure sonunda yine calar. */
#define SARJ_SONRASI_ALARM_SUSKUN_MS 20000U
/* KONTAKTOR ACILIS GECIKMESI: diger roleler 1 sn'de doner, kontaktor ise
 * beslemenin/BMS'in iyice oturmasi icin daha gec, TEK seferde ceker.
 * (Acilista 4 kez sak-sak cekme = buyuk ihtimal bobin akiminin beslemeyi
 * cokertip STM32'yi resetlemesi; gecikme ilk cekisi saglam zemine alir.) */
#define KONTAKTOR_ACILIS_MS 3000U
/* UYARI rolesi: digerleriyle AYNI surulur (kart aktif-LOW, ROLE_ON=LOW=ceker).
 * Alarm var -> ceker (buzzer+flasor calisir), alarm yok -> birakik. */
#define UYARI_ON   ROLE_ON
#define UYARI_OFF  ROLE_OFF

/* Zamanlayicilar */
static uint32_t t_tel   = 0;
static uint32_t t_buton = 0;
static uint32_t t_raw   = 0;   /* ham CAN2 dokum periyodu */
static uint32_t t_obc   = 0;   /* Emerson'dan veri isteme periyodu */
static uint32_t t_mcp   = 0;   /* MCP RX yoklama periyodu (INT gelmese de) */

/* ===================================================================
 *  Emerson yerlesik sarj (OBC) durum degiskenleri.
 *  send_telemetri() bunlari kullandigi icin burada, dosyanin ustunde
 *  tanimli olmalari GEREKIR. (Sadece bu blok yukari alindi; alt bloktaki
 *  MCP/R48 fonksiyonlari ayni degiskenleri kullanmaya devam eder.)
 * =================================================================== */
static volatile uint16_t obc_vout_x10  = 0;   /* cikis gerilimi 0.1V   */
static volatile uint16_t obc_iout_x10  = 0;   /* cikis akimi    0.1A   */
static volatile uint16_t obc_vin_x10   = 0;   /* giris (sebeke) 0.1V   */
static volatile int16_t  obc_temp      = 0;   /* ic sicaklik C         */
static volatile uint32_t obc_rx_cnt    = 0;   /* R48 telemetri (eslesen ID) sayaci */
static volatile uint32_t obc_rx_all    = 0;   /* MCP'den alinan TUM cerceve sayaci (ID farketmez) */
static volatile uint16_t obc_streak    = 0;   /* ardisik GERCEK Emerson frame sayaci (debounce) */
static volatile uint32_t obc_last_tick = 0;
static uint8_t  obc_link   = 0;   /* sarj cihazi takili + yayinda mi (1/0) */
static uint8_t  mcp_hazir  = 0;   /* MCP2515 init basarili mi (1/0)        */
static uint8_t  mcp_step   = 0;   /* 0=ok 1=config 2=normal giremedi       */
static uint8_t  mcp_eflg   = 0;   /* MCP hata bayraklari (bus tani)        */
static uint8_t  mcp_tec    = 0;   /* MCP TX hata sayaci (bus tani)         */
static uint8_t  mcp_rec    = 0;   /* MCP RX hata sayaci (bus tani)         */
static volatile uint8_t mcp_int_flag = 0;      /* EXTI (MCP_INT) set eder   */
static uint8_t  sarj_aktif = 0;   /* SARJ MODU: kontaktor cekili, motor gucu kesik */
static uint32_t t_sarj_cikis = 0; /* fisten cekilme ani (acilis-benzeri bekletme icin) */

/* CAN filtre + start + RX0 kesmesi. Hata olsa da sistem DURMAZ:
 * Pi'ye USART1 telemetrisi her kosulda akar. */
static void Kelly_CAN_Baslat(void)
{
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;   /* mask 0 -> tum ID'ler gecer */
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;

    can_state = (uint8_t)hcan1.State;
    mcu_devid = (uint16_t)(DBGMCU->IDCODE & 0x0FFFU);
    can_clk   = (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_CAN1EN) != 0U) ? 1U : 0U;

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) { can_step = 1; can_err = hcan1.ErrorCode; return; }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)            { can_step = 2; can_err = hcan1.ErrorCode; return; }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
                                                    { can_step = 3; can_err = hcan1.ErrorCode; return; }
    can_step = 0;
    can_hazir = 1;
}

/* Telemetri JSON'u USART1'den Pi'ye. Format F103 ile AYNI -> Python degismez.
 * HIZ HESABI:  km/h = rpm * (disli 18/31) * (teker CEVRESI) * 60/1000
 *   Teker cevresi = PI * cap = 3.1416 * 0.55m = 1.728 m
 *   DISLI ORANI 12/31 -> 18/31 GUNCELLENDI (mekanik degisiklik).
 *   km/h x10 katsayisi = (18/31)*1.728*60/1000*10 = 0.6020  ->  rpm * 933/1550
 *   KALIBRASYON: gercek hizla fark olursa 933'u ayarla (buyut = hiz artar). */
static void send_telemetri(void)
{
    uint32_t hiz_x10 = (uint32_t)kel_rpm * 933UL / 1550UL;
    unsigned int hiz_tam = (unsigned int)(hiz_x10 / 10U);
    unsigned int hiz_ond = (unsigned int)(hiz_x10 % 10U);
    unsigned int volt_tam = (unsigned int)(kel_volt_x10 / 10U);
    unsigned int volt_ond = (unsigned int)(kel_volt_x10 % 10U);
    int amp = (int)kel_cur / KELLY_AKIM_BOL;   /* ISARETLI Amper (regen'de negatif) */
    unsigned int pwm = (unsigned int)((kel_throttle * 100U) / 255U);

    char buf[1200];
    int  len = snprintf(buf, sizeof(buf),
        "{\"id\":%u,\"spd\":%u.%u,\"rpm\":%u,"
        "\"volt\":%u.%u,\"amp\":%d,\"pwm\":%u,\"bamp\":0,"
        "\"tmp\":%d,\"ctmp\":%d,"
        "\"brk\":%u,\"sig\":%u,\"gadc\":%u,"
        "\"dir\":%u,\"sw\":%u,\"flt\":%u,"
        "\"kelok\":%u,\"can\":%u,\"cstep\":%u,\"cstate\":%u,\"cerr\":%u,"
        "\"devid\":%u,\"canclk\":%u,"
        "\"ktx\":%u,\"krx\":%u,\"kfe\":%u,"
        /* ----- AKS (arac kontrol) alanlari ----- */
        "\"far\":%u,\"ksag\":%u,\"ksol\":%u,\"lsol\":%u,\"lsag\":%u,\"sil\":%u,\"uyari\":%u,"
        /* ----- BMS (CAN2) alanlari ----- */
        "\"bsoc\":%u,\"bsoh\":%u,\"bvolt\":%u,\"bcur\":%d,\"btmp\":%d,"
        "\"bcvl\":%u,\"bccl\":%d,\"bdcl\":%d,\"bdvl\":%u,"
        "\"bcmax\":%u,\"bcmin\":%u,\"bchg\":%u,"
        "\"brem\":%u,\"bfull\":%u,"
        "\"bprot\":%u,\"balrm\":%u,\"bpacks\":%u,"
        "\"bmsok\":%u,\"bmscan\":%u,\"brx\":%u,"
        /* ----- EMERSON YERLESIK SARJ (OBC) alanlari ----- */
        "\"sarj\":%u,\"obc\":%u,\"mcp\":%u,\"chgpct\":%u,"
        "\"ovout\":%u,\"oiout\":%u,\"ovin\":%u,\"otmp\":%d,\"orx\":%u,"
        "\"orxall\":%u,\"eflg\":%u,\"tec\":%u,\"rec\":%u}\r\n",
        (unsigned int)pkt_id++,
        hiz_tam, hiz_ond, (unsigned int)kel_rpm,
        volt_tam, volt_ond, amp, pwm,
        (int)kel_mot_tmp, (int)kel_ctrl_tmp,
        (unsigned int)fren_aktif, (unsigned int)sig_state, (unsigned int)gaz_adc,
        (unsigned int)kel_dir, (unsigned int)kel_sw, (unsigned int)kel_fault,
        (unsigned int)kelly_ok, (unsigned int)can_hazir,
        (unsigned int)can_step, (unsigned int)can_state, (unsigned int)can_err,
        (unsigned int)mcu_devid, (unsigned int)can_clk,
        (unsigned int)kel_msg1_cnt, (unsigned int)kel_msg2_cnt, (unsigned int)kel_fault,
        (unsigned int)far_aktif, (unsigned int)sag_kapi,
        (unsigned int)sol_kapi, (unsigned int)led_sol, (unsigned int)led_sag,
        (unsigned int)silecek_aktif, (unsigned int)uyari_aktif,
        /* ----- BMS degerleri ----- */
        (unsigned int)bms_soc, (unsigned int)bms_soh, (unsigned int)bms_volt_x100,
        (int)bms_cur_x10, (int)bms_temp_x10,
        (unsigned int)bms_cvl_x10, (int)bms_ccl_x10, (int)bms_dcl_x10, (unsigned int)bms_dvl_x10,
        (unsigned int)bms_cmax_mv, (unsigned int)bms_cmin_mv, (unsigned int)bms_chg_flags,
        (unsigned int)bms_rem_ah_x10, (unsigned int)bms_full_ah_x10,
        (unsigned int)((bms_prot0 << 8) | bms_prot1),
        (unsigned int)((bms_alarm0 << 8) | bms_alarm1),
        (unsigned int)bms_packs,
        (unsigned int)bms_ok, (unsigned int)bms_can_hazir, (unsigned int)bms_rx_cnt,
        /* ----- OBC degerleri ----- */
        (unsigned int)sarj_aktif, (unsigned int)obc_link, (unsigned int)mcp_hazir,
        (unsigned int)bms_soc,                       /* chgpct = sarj yuzdesi */
        (unsigned int)obc_vout_x10, (unsigned int)obc_iout_x10, (unsigned int)obc_vin_x10,
        (int)obc_temp, (unsigned int)obc_rx_cnt,
        (unsigned int)obc_rx_all, (unsigned int)mcp_eflg, (unsigned int)mcp_tec, (unsigned int)mcp_rec);

    HAL_UART_Transmit(&TELEM_UART, (uint8_t*)buf, (uint16_t)len, 100);
}

/* BMS (Pylontech, CAN2) standart-ID paketlerini LITTLE-ENDIAN coz */
static void BMS_Decode(uint32_t id, const uint8_t *d)
{
    switch (id)
    {
    case 0x351:  /* sarj/desarj limitleri */
        bms_cvl_x10 = (uint16_t)(d[0] | (d[1] << 8));   /* CVL 0.1V */
        bms_ccl_x10 = (int16_t )(d[2] | (d[3] << 8));   /* CCL 0.1A */
        bms_dcl_x10 = (int16_t )(d[4] | (d[5] << 8));   /* DCL 0.1A */
        bms_dvl_x10 = (uint16_t)(d[6] | (d[7] << 8));   /* DVL 0.1V */
        break;
    case 0x355:  /* SOC / SOH */
        bms_soc = (uint16_t)(d[0] | (d[1] << 8));
        bms_soh = (uint16_t)(d[2] | (d[3] << 8));
        break;
    case 0x356:  /* anlik V / I / sicaklik */
        bms_volt_x100 = (uint16_t)(d[0] | (d[1] << 8)); /* 0.01V */
        bms_cur_x10   = (int16_t )(d[2] | (d[3] << 8)); /* 0.1A +sarj -desarj */
        bms_temp_x10  = (int16_t )(d[4] | (d[5] << 8)); /* 0.1C */
        break;
    case 0x359:  /* koruma/alarm bayraklari (pack burada 0; 0x364'ten aliniyor) */
        bms_prot0  = d[0];
        bms_prot1  = d[1];
        bms_alarm0 = d[2];
        bms_alarm1 = d[3];
        break;
    case 0x35C:  /* sarj/desarj izin bayraklari (d0: bit7 sarj, bit6 desarj) */
        bms_chg_flags = d[0];
        break;
    case 0x200:  /* max/min HUCRE gerilimi mV (dogrulandi: 3290/3286 ~ 52.6/16) */
        bms_cmax_mv = (uint16_t)(d[0] | (d[1] << 8));
        bms_cmin_mv = (uint16_t)(d[2] | (d[3] << 8));
        break;
    case 0x150:  /* ozet: V, I, KALAN Ah, TOPLAM Ah (dogrulama: 38.2/100 ~ SOC 39) */
        bms_rem_ah_x10  = (uint16_t)(d[4] | (d[5] << 8));
        bms_full_ah_x10 = (uint16_t)(d[6] | (d[7] << 8));
        break;
    case 0x364:  /* paralel pack sayisi (d0) */
        bms_packs = d[0];
        break;
    default:     /* diger ID'ler HAM TARAMA tablosunda gorunur */
        break;
    }
    bms_rx_cnt++;
    bms_last_tick = HAL_GetTick();
    bms_ok = 1;
}

/* CAN2 (BMS) filtre + start + RX0 kesmesi. Hata olsa da sistem DURMAZ. */
static void BMS_CAN2_Baslat(void)
{
    CAN_FilterTypeDef f = {0};
    /* CAN2 filtreleri bank 14'ten baslar (SlaveStartFilterBank=14).
     * HAM TARAMA icin mask=0 -> TUM ID'ler gecer (hicbir BMS paketi kacmaz).
     * (Hat sadece BMS'e bagli oldugu icin flood olmaz.) */
    f.FilterBank           = 14;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &f) != HAL_OK) return;
    if (HAL_CAN_Start(&hcan2) != HAL_OK)            return;
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return;
    bms_can_hazir = 1;
}

/* Gelen ham frame'i tabloya isle (ISR icinde, blocking YOK) */
static void RAW_Capture(uint32_t id, uint8_t dlc, const uint8_t *d)
{
    uint8_t i;
    for (i = 0; i < raw_n; i++)
        if (raw_id[i] == id) break;
    if (i == raw_n)
    {
        if (raw_n >= RAW_MAX) return;   /* tablo dolu */
        raw_id[raw_n] = id;
        raw_n++;
    }
    raw_dlc[i] = (dlc > 8) ? 8 : dlc;
    for (uint8_t k = 0; k < 8; k++)
        raw_d[i][k] = (k < raw_dlc[i]) ? d[k] : 0;
    raw_cnt[i]++;
}

/* Ham tabloyu USART1'den Pi'ye dok (ana donguden cagrilir, ISR'den DEGIL).
 * Satir formati:  RAW <id> <dlc> <b0..b7 hex> <adet>  */
static void raw_dump(void)
{
    char rb[64];
    for (uint8_t i = 0; i < raw_n; i++)
    {
        int n = snprintf(rb, sizeof(rb),
            "RAW %03lX %u %02X %02X %02X %02X %02X %02X %02X %02X %lu\r\n",
            (unsigned long)raw_id[i], (unsigned)raw_dlc[i],
            raw_d[i][0], raw_d[i][1], raw_d[i][2], raw_d[i][3],
            raw_d[i][4], raw_d[i][5], raw_d[i][6], raw_d[i][7],
            (unsigned long)raw_cnt[i]);
        if (n > 0)
            HAL_UART_Transmit(&TELEM_UART, (uint8_t *)rb, (uint16_t)n, 50);
    }
}

/* CAN RX FIFO0 callback - CAN1=Kelly (extended), CAN2=BMS (standart) */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanRx)
{
    CAN_RxHeaderTypeDef rxh;
    uint8_t d[8];
    if (HAL_CAN_GetRxMessage(hcanRx, CAN_RX_FIFO0, &rxh, d) != HAL_OK) return;

    /* --- CAN2: BMS, 11-bit standart ID --- */
    if (hcanRx->Instance == CAN2)
    {
        if (rxh.IDE != CAN_ID_STD) return;
        RAW_Capture(rxh.StdId, rxh.DLC, d);   /* ham tabloya isle (tarama) */
        BMS_Decode(rxh.StdId, d);             /* ayrica cozumle             */
        return;
    }

    /* --- CAN1: Kelly, 29-bit genisletilmis ID --- */
    if (rxh.IDE != CAN_ID_EXT) return;

    if (rxh.ExtId == KELLY_CAN_ID_MSG1)
    {
        kel_rpm      = (uint16_t)(d[0] | (d[1] << 8));
        kel_cur  = (int16_t )(d[2] | (d[3] << 8));   /* ISARETLI: regen/frenlemede negatif olabilir */
        kel_volt_x10 = (uint16_t)(d[4] | (d[5] << 8));
        kel_fault    = (uint16_t)(d[6] | (d[7] << 8));
        kel_msg1_cnt++;
        kel_last_tick = HAL_GetTick();
        kelly_ok = 1;
    }
    else if (rxh.ExtId == KELLY_CAN_ID_MSG2)
    {
        kel_throttle = d[0];
        kel_ctrl_tmp = (int16_t)d[1] - 40;
        kel_mot_tmp  = (int16_t)d[2] - 30;
        kel_dir      = d[4];
        kel_sw       = d[5];
        kel_msg2_cnt++;
        kel_last_tick = HAL_GetTick();
        kelly_ok = 1;
    }
}

/* ===================================================================
 *  YERLESIK SARJ: Emerson R48-1800  ->  MCP2515 (SPI3)  ->  STM32
 *  MCP2515 <-> Emerson : CAN 125 kbps, 29-bit extended ID
 *  STM32   <-> MCP2515 : SPI3 (PC10 SCK, PC11 MISO, PC12 MOSI), CS=PD6
 *                        MCP_RESET=PD1 (aktif-LOW), MCP_INT=PD0 (EXTI0, falling)
 *  Bit-timing (8 MHz kristal, 125k): CNF1=0x01 CNF2=0xB1 CNF3=0x85
 * =================================================================== */
/* --- MCP2515 SPI komutlari --- */
#define MCP_CMD_RESET   0xC0
#define MCP_CMD_READ    0x03
#define MCP_CMD_WRITE   0x02
#define MCP_CMD_RTS_TX0 0x81
#define MCP_CMD_BITMOD  0x05
/* --- MCP2515 registerlari --- */
#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F
#define MCP_TEC         0x1C   /* TX hata sayaci (bus tani)  */
#define MCP_REC         0x1D   /* RX hata sayaci (bus tani)  */
#define MCP_EFLG        0x2D   /* hata bayraklari (bus tani) */
#define MCP_CNF3        0x28
#define MCP_CNF2        0x29
#define MCP_CNF1        0x2A
#define MCP_CANINTE     0x2B
#define MCP_CANINTF     0x2C
#define MCP_TXB0CTRL    0x30
#define MCP_TXB0SIDH    0x31
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71

/* --- Emerson R48 CAN ID'leri (29-bit extended) --- */
#define R48_ID_TELEMETRY   0x06080783UL   /* Dogrultucu -> Master (olcum)  */
#define R48_ID_QUERY       0x06000783UL   /* Master -> Dogrultucu (veri iste) */
#define R48_ID_CONTROL     0x0607FF83UL   /* Master -> Dogrultucu (voltaj/akim komut) */
/* ===== KONTAKTOR CEK-BIRAK FILTRESI (Kural 1 debounce + Kural 2 hold-off) =====
 *   DEBOUNCE_MS: Emerson verisi KESINTISIZ bu kadar sure gelirse kontaktor CEKER
 *                (fis takarken ark/titremeyi filtreler; ilk frame'de hemen cekmez).
 *   HOLDOFF_MS : kontaktor cekiliyken veri ANLIK koparsa hemen birakmaz; veri
 *                bu kadar sure geri gelmezse "fis cekildi" deyip BIRAKIR. */
#define DEBOUNCE_MS   2000U   /* Kural 1: cekmeden once kesintisiz veri suresi   */
#define HOLDOFF_MS    6000U   /* Kural 2: birakmadan once veri-yok bekleme suresi.
                              * Emerson ~2-3 sn'de bir yayinlar; pencere bunu ASMALI,
                              * yoksa DEBOUNCE_MS hicbir zaman KESINTISIZ dolamaz. */
#define BMS_FULL_SOC       100U           /* bu SOC'da batarya DOLU sayilir */
/* ===== EMERSON VARLIK TESPITI: TX-ACK (TEC) =====
 * CANLI OLCUM (dogrulandi):
 *   TAKILI  -> yolladigimiz sorgular ACK'leniyor -> TEC DUSUK kalir (0..16 oynar)
 *   CEKILI  -> hic ACK yok -> TEC 128'e tirmanip SABITLENIR
 * TEC yalnizca gonderim BASARILI olunca azalir; dusuk TEC = hatta ACK veren
 * bir dugum VAR demektir. (REC kullanilmaz: Emerson kendiliginden yayin
 * yapmiyor, REC hep 0 kaliyor.) */
#define EMERSON_TEC_ESIK      64U     /* TEC bunun ALTINDA = ACK aliniyor = takili */
#define EMERSON_TEC_GUARD_MS  4000U   /* acilis/reset sonrasi TEC'in oturma suresi */

/* CAN cerceve yapisi */
typedef struct {
    uint32_t id;
    uint8_t  extended;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

/* NOT: obc_* / mcp_* / sarj_aktif durum degiskenleri send_telemetri()'den
 * ONCE, dosyanin ustunde tanimlandi (yukari alindi). Buradaki fonksiyonlar
 * ayni degiskenleri kullanir. */

/* Emerson HAM tarama: (CAN ID + selector byte0) bazinda yakala.
 * Emerson 0x0707F803'u MULTIPLEX yolluyor (byte0=01,02,... her biri farkli olcum),
 * bu yuzden ID bazli degil (ID+selector) bazli tutariz -> hepsini goruruz. */
#define OBC_RAW_MAX 24
static volatile uint32_t obc_raw_id[OBC_RAW_MAX];    /* CAN ID   */
static volatile uint8_t  obc_raw_b0[OBC_RAW_MAX];    /* selector (byte0) */
static volatile uint8_t  obc_raw_d[OBC_RAW_MAX][8];
static volatile uint8_t  obc_raw_dlc[OBC_RAW_MAX];
static volatile uint32_t obc_raw_cnt[OBC_RAW_MAX];
static volatile uint8_t  obc_raw_n = 0;

/* --- MCP2515 dusuk seviye SPI --- */
static void MCP_CS_L(void){ HAL_GPIO_WritePin(MCP_CS_GPIO_Port, MCP_CS_Pin, GPIO_PIN_RESET); }
static void MCP_CS_H(void){ HAL_GPIO_WritePin(MCP_CS_GPIO_Port, MCP_CS_Pin, GPIO_PIN_SET);   }

static uint8_t MCP_ReadReg(uint8_t addr)
{
    uint8_t tx[2] = { MCP_CMD_READ, addr };
    uint8_t rx = 0;
    MCP_CS_L();
    HAL_SPI_Transmit(&hspi3, tx, 2, 10);
    HAL_SPI_Receive(&hspi3, &rx, 1, 10);
    MCP_CS_H();
    return rx;
}
static void MCP_WriteReg(uint8_t addr, uint8_t val)
{
    uint8_t tx[3] = { MCP_CMD_WRITE, addr, val };
    MCP_CS_L();
    HAL_SPI_Transmit(&hspi3, tx, 3, 10);
    MCP_CS_H();
}
static void MCP_BitModify(uint8_t addr, uint8_t mask, uint8_t val)
{
    uint8_t tx[4] = { MCP_CMD_BITMOD, addr, mask, val };
    MCP_CS_L();
    HAL_SPI_Transmit(&hspi3, tx, 4, 10);
    MCP_CS_H();
}
static void MCP_ResetCmd(void)
{
    uint8_t c = MCP_CMD_RESET;
    MCP_CS_L();
    HAL_SPI_Transmit(&hspi3, &c, 1, 10);
    MCP_CS_H();
}

/* MCP2515'i baslat: reset -> CONFIG -> bit-timing -> RX filtre kapa -> NORMAL */
static uint8_t MCP2515_Init(void)
{
    /* Donanim reset (PD1 aktif-LOW) */
    HAL_GPIO_WritePin(MCP_RESET_GPIO_Port, MCP_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(MCP_RESET_GPIO_Port, MCP_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    MCP_ResetCmd();          /* yazilim reset -> CONFIG moduna girer */
    HAL_Delay(10);

    if ((MCP_ReadReg(MCP_CANSTAT) & 0xE0) != 0x80) { mcp_step = 1; return 0; }  /* CONFIG degil */

    MCP_WriteReg(MCP_CNF1, 0x01);   /* 125 kbps @ 8MHz */
    MCP_WriteReg(MCP_CNF2, 0xB1);
    MCP_WriteReg(MCP_CNF3, 0x85);

    MCP_WriteReg(MCP_RXB0CTRL, 0x64);  /* RXM=11: filtre kapali + BUKT: RXB0 tasarsa RXB1'e devret */
    MCP_WriteReg(MCP_RXB1CTRL, 0x60);
    MCP_WriteReg(MCP_CANINTE,  0x01);  /* RX0IE */
    MCP_WriteReg(MCP_CANINTF,  0x00);  /* bayraklari temizle */

    MCP_WriteReg(MCP_CANCTRL,  0x00);  /* NORMAL mod */
    HAL_Delay(10);
    if ((MCP_ReadReg(MCP_CANSTAT) & 0xE0) != 0x00) { mcp_step = 2; return 0; }  /* NORMAL degil */

    mcp_step = 0;
    return 1;
}

/* Bir CAN cercevesi gonder (TXB0, 29-bit extended) */
static void MCP2515_Send(const can_frame_t *f)
{
    uint8_t sidh, sidl, eid8, eid0;
    if (f->extended) {
        sidh = (uint8_t)(f->id >> 21);
        sidl = (uint8_t)(((f->id >> 13) & 0xE0) | 0x08 | ((f->id >> 16) & 0x03)); /* 0x08=EXIDE */
        eid8 = (uint8_t)(f->id >> 8);
        eid0 = (uint8_t)(f->id);
    } else {
        sidh = (uint8_t)(f->id >> 3);
        sidl = (uint8_t)((f->id << 5) & 0xE0);
        eid8 = 0; eid0 = 0;
    }
    MCP_WriteReg(MCP_TXB0SIDH + 0, sidh);
    MCP_WriteReg(MCP_TXB0SIDH + 1, sidl);
    MCP_WriteReg(MCP_TXB0SIDH + 2, eid8);
    MCP_WriteReg(MCP_TXB0SIDH + 3, eid0);
    MCP_WriteReg(MCP_TXB0SIDH + 4, (uint8_t)(f->dlc & 0x0F));
    for (uint8_t i = 0; i < (f->dlc & 0x0F); i++)
        MCP_WriteReg(MCP_TXB0SIDH + 5 + i, f->data[i]);
    /* Request-To-Send TXB0 */
    { uint8_t c = MCP_CMD_RTS_TX0; MCP_CS_L(); HAL_SPI_Transmit(&hspi3, &c, 1, 10); MCP_CS_H(); }
}

/* Biriken bir cerceveyi oku (RXB0 veya RXB1). Yoksa 0 doner. */
static uint8_t MCP2515_Recv(can_frame_t *f)
{
    uint8_t intf = MCP_ReadReg(MCP_CANINTF);
    uint8_t base, clr;
    if      (intf & 0x01) { base = MCP_RXB0SIDH; clr = 0x01; }
    else if (intf & 0x02) { base = MCP_RXB1SIDH; clr = 0x02; }
    else return 0;

    uint8_t sidh = MCP_ReadReg(base + 0);
    uint8_t sidl = MCP_ReadReg(base + 1);
    uint8_t eid8 = MCP_ReadReg(base + 2);
    uint8_t eid0 = MCP_ReadReg(base + 3);
    uint8_t dlc  = MCP_ReadReg(base + 4) & 0x0F;

    if (sidl & 0x08) {   /* extended */
        f->extended = 1;
        f->id = ((uint32_t)sidh << 21) | ((uint32_t)(sidl & 0xE0) << 13)
              | ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)eid8 << 8) | eid0;
    } else {             /* standart */
        f->extended = 0;
        f->id = ((uint32_t)sidh << 3) | (sidl >> 5);
    }
    if (dlc > 8) dlc = 8;
    f->dlc = dlc;
    for (uint8_t i = 0; i < dlc; i++)
        f->data[i] = MCP_ReadReg(base + 5 + i);

    MCP_BitModify(MCP_CANINTF, clr, 0x00);   /* RX bayragini temizle */
    return 1;
}

/* float32 big-endian (byte[4..7]) -> float */
static float R48_be_float(const uint8_t *p)
{
    uint32_t u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    float v; memcpy(&v, &u, 4); return v;
}

/* Dogrultucuya "tum telemetriyi gonder" komutu */
static void R48_RequestData(void)
{
    can_frame_t f;
    f.id = R48_ID_QUERY; f.extended = 1; f.dlc = 8;
    f.data[0]=0x01; f.data[1]=0xF0; f.data[2]=0x00; f.data[3]=0x00;
    f.data[4]=0x00; f.data[5]=0x00; f.data[6]=0x00; f.data[7]=0x00;
    MCP2515_Send(&f);
}

/* Emerson HAM tarama: gelen HER cerceveyi CAN ID bazinda tabloya isle.
 * (R48_ParseFrame'den ONCE, TUM ID'ler icin cagrilir -> Emerson'in gercek
 *  ID'lerini gorup R48_ID_TELEMETRY dogru mu teyit ederiz.) */
static void OBC_RawCapture(const can_frame_t *f)
{
    uint8_t i;
    for (i = 0; i < obc_raw_n; i++)
        if (obc_raw_id[i] == f->id && obc_raw_b0[i] == f->data[0]) break;
    if (i == obc_raw_n)
    {
        if (obc_raw_n >= OBC_RAW_MAX) return;
        obc_raw_id[obc_raw_n] = f->id;
        obc_raw_b0[obc_raw_n] = f->data[0];
        obc_raw_n++;
    }
    obc_raw_dlc[i] = (f->dlc > 8) ? 8 : f->dlc;
    for (uint8_t k = 0; k < 8; k++) obc_raw_d[i][k] = f->data[k];
    obc_raw_cnt[i]++;
}

/* MCP2515 bus tani sayaclarini oku (EFLG/TEC/REC) -> Pi'de gorunur.
 *   TEC yuksek/255 -> TX ACK alamiyor (karsi dugum yok / baud / kablo)
 *   EFLG RX bit'leri -> RX hatasi (baud/kablo)
 *   Hepsi 0 + obc_rx_all=0 -> hatta hic aktivite yok (CANH/L, termination, Emerson) */
static void MCP_ReadDiag(void)
{
    mcp_eflg = MCP_ReadReg(MCP_EFLG);
    mcp_tec  = MCP_ReadReg(MCP_TEC);
    mcp_rec  = MCP_ReadReg(MCP_REC);
    if (mcp_eflg & 0xC0)                        /* RX0OVR/RX1OVR tasma -> temizle */
        MCP_BitModify(MCP_EFLG, 0xC0, 0x00);
}

/* Emerson GERCEK telemetri ID'si (capture ile bulundu): 0x0707F803.
 * MULTIPLEX: byte[0]=selector (01,02,...), deger byte[4..7] float32 BIG-ENDIAN. */
#define R48_ID_TELEM   0x0707F803UL

/* Iout (cikis akimi) selector'u: Pi decode listesinden (BMS akimiyla eslesen)
 * tespit edilip buraya yazilacak. 0xFF = henuz bilinmiyor. */
#define R48_IOUT_SEL   0xFF

static void R48_ParseFrame(const can_frame_t *f)
{
    if (f->id != R48_ID_TELEM) return;
    obc_rx_cnt++;

    float val = R48_be_float(&f->data[4]);   /* byte[4..7] float32 big-endian */
    uint8_t sel = f->data[0];                /* multiplex selector */

    /* Cikis gerilimi (Vout): batarya araligindaki (45-65V) deger cikis gerilimidir. */
    if (val >= 45.0f && val <= 65.0f)
        obc_vout_x10 = (uint16_t)(val * 10.0f + 0.5f);

    /* Cikis akimi (Iout): selector tespit edilince otomatik (R48_IOUT_SEL ayarla). */
    if (sel == R48_IOUT_SEL)
        obc_iout_x10 = (uint16_t)((val < 0.0f ? 0.0f : val) * 10.0f + 0.5f);
}

/* Emerson ham tablosunu Pi'ye dok:  OBC <id> <dlc> <b0..b7> <adet> */
static void obc_raw_dump(void)
{
    char rb[72];
    for (uint8_t i = 0; i < obc_raw_n; i++)
    {
        int n = snprintf(rb, sizeof(rb),
            "OBC %08lX %u %02X %02X %02X %02X %02X %02X %02X %02X %lu\r\n",
            (unsigned long)obc_raw_id[i], (unsigned)obc_raw_dlc[i],
            obc_raw_d[i][0], obc_raw_d[i][1], obc_raw_d[i][2], obc_raw_d[i][3],
            obc_raw_d[i][4], obc_raw_d[i][5], obc_raw_d[i][6], obc_raw_d[i][7],
            (unsigned long)obc_raw_cnt[i]);
        if (n > 0) HAL_UART_Transmit(&TELEM_UART, (uint8_t *)rb, (uint16_t)n, 50);
    }
}

/* MCP_INT (PD0/EXTI0) dustugunde: yeni CAN cercevesi biriktigini isaretle.
 * ISR icinde blocking YOK; asil okuma ana dongude yapilir. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MCP_INT_Pin)
        mcp_int_flag = 1;
}

/* Emerson'a "sarji durdur" komutu (cikis akimini 0'a cek).
 * GUVENLI YON: bu cerceve cikisi yalnizca AZALTIR, asla artirmaz; bu yuzden
 * payload tam dogrulanmamis olsa bile tehlike olusturmaz (cihaz ya uyar ya
 * yok sayar). Byte duzeni R48 ailesine gore degisebilir -> canli capture ile
 * teyit edilmeli (dokumantasyondaki uyarinin aynisi).
 *   byte0=0x00 : gecici (EEPROM'a yazma)
 *   byte1=0x03 : "cikis akim limiti" seciciligi
 *   byte4..7   : deger = 0  -> 0 A */
static void R48_ChargeOff(void)
{
    can_frame_t f;
    f.id = R48_ID_CONTROL; f.extended = 1; f.dlc = 8;
    f.data[0]=0x00; f.data[1]=0x03; f.data[2]=0x00; f.data[3]=0x00;
    f.data[4]=0x00; f.data[5]=0x00; f.data[6]=0x00; f.data[7]=0x00;
    MCP2515_Send(&f);
}

/* ===================================================================
 *  SARJ / MOTOR KONTAKTOR DURUM MAKINESI  (ana donguden cagrilir)
 *  Kural:
 *   - Emerson fise takili + veri geliyor + batarya DOLU degil
 *          -> SARJ MODU: KONTAKTOR CEK -> motor surucu elektrigi KESILIR.
 *   - Emerson fisten cekildi  VEYA  batarya %100
 *          -> NORMAL MOD: KONTAKTOR birak -> motor surucu calisabilir,
 *             ayrica Emerson'a "sarji durdur" komutu gonderilir.
 * =================================================================== */
static void Sarj_Kontrol(uint32_t now)
{
    /* ===== KURAL 1 (DEBOUNCE) + KURAL 2 (HOLD-OFF) =====
     * Ham sinyal "VAR": son HOLDOFF_MS icinde frame geldi (Kural 2 -> anlik kopma
     *   HOLDOFF_MS'ten kisaysa sinyal HALA VAR sayilir, kontaktor birakmaz).
     * obc_link=1: sinyal KESINTISIZ DEBOUNCE_MS boyunca VAR olunca (Kural 1).
     * obc_link=0: sinyal HOLDOFF_MS boyunca YOK olunca (fis cekildi). */
    {
        static uint32_t t_var = 0;   /* sinyal surekli VAR olmaya basladigi an */
        /* obc_last_tick==0 = "hic veri gelmedi" (sentinel). Bu kontrol olmadan
         * "now-0=now < HOLDOFF_MS" acilistan itibaren HOLDOFF_MS boyunca YANLISLIKLA
         * "sinyal var" sayilip, hic Emerson baglanmadan kontaktoru sahte cektiriyordu. */
        uint8_t sinyal_var = ((obc_last_tick != 0u) &&
                              ((now - obc_last_tick) < HOLDOFF_MS)) ? 1u : 0u;

        if (sinyal_var)
        {
            if (t_var == 0) t_var = now;                     /* sinyal yeni basladi */
            if (!obc_link && (now - t_var) >= DEBOUNCE_MS)
                obc_link = 1;                                /* Kural 1: 2sn kesintisiz -> CEK */
        }
        else
        {
            t_var = 0;
            if (obc_link) { obc_link = 0; t_sarj_cikis = now; } /* Kural 2 doldu -> BIRAK */
        }
        obc_streak = 0;   /* (kullanilmiyor; ID/frame filtresi kaldirildi) */
    }

    uint8_t sarj_takili  = obc_link;                                   /* takili + yayinda */
    uint8_t batarya_dolu = (bms_ok && (bms_soc >= BMS_FULL_SOC));      /* %100 */

    if (sarj_takili && !batarya_dolu)
        sarj_aktif = 1;    /* SARJ MODU */
    else
        sarj_aktif = 0;    /* NORMAL MOD (motor serbest) */

    /* KADRAN icin V_out / I_out: Emerson bu degerleri CAN'de bu formatta vermiyor
     * (decode ~0 cikiyor). Sarj gerilim/akimi = bataryaya giren gercek deger,
     * onu BMS'ten yansitiriz -> JSON ovout/oiout dogru gelir. */
    if (sarj_takili && bms_ok)
    {
        int16_t ci = bms_cur_x10;  if (ci < 0) ci = (int16_t)(-ci);
        obc_vout_x10 = (uint16_t)(bms_volt_x100 / 10U);   /* 0.01V -> 0.1V */
        obc_iout_x10 = (uint16_t)ci;                       /* 0.1A (mutlak) */
    }

    /* Kontaktor rolesi (PD5) - KENAR TETIKLI, her gecis BIR KEZ yazilir:
     *   Emerson verisi KESINTISIZ DEBOUNCE_MS gelince (obc_link 0->1) -> CEKER
     *       (isik yanar, sistem/motor kesilir).
     *   Emerson verisi HOLDOFF_MS boyunca gelmeyince (obc_link 1->0) -> BIRAKIR
     *       (acilistaki haline doner: HIGH, birakik, isik sonuk).
     * Aradaki turlarda pine HIC dokunulmaz -> titreme/tekrar yazma olmaz.
     * Kart aktif-LOW: ROLE_ON=LOW=ceker. obc_link'e Sarj_Kontrol'deki debounce
     * (Kural 1) + hold-off (Kural 2) karar verir -> anlik kopmalar filtrelenir. */
    {
        static uint8_t kont_onceki = 0;   /* acilis hali = birakik (0) */
        if (obc_link != kont_onceki)
        {
            HAL_GPIO_WritePin(KONTAKTOR_ROLE_GPIO_Port, KONTAKTOR_ROLE_Pin,
                              obc_link ? ROLE_ON : ROLE_OFF);
            kont_onceki = obc_link;
        }
    }

    /* Batarya doldu ama hala fise takiliysa Emerson'a sarji birak komutu */
    if (mcp_hazir && sarj_takili && batarya_dolu)
        R48_ChargeOff();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* ACILIS ROLE GUVENLIGI (kart AKTIF-LOW: LOW=ceker, HIGH=birakir):
   *  Reset sonrasi EN ERKEN anda (HAL_Init'ten ONCE) TUM role pinleri HIGH
   *  (=BIRAKIK) yapilir + ic PULL-UP eklenir. Boylece acilista hicbir role
   *  cekmez; guc/reset penceresinde de pull-up pini HIGH'a ceker. */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  {
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    /* 1) Seviyeyi register'dan garanti et: BSRR alt-16 = set(HIGH) = BIRAKIK */
    GPIOC->BSRR = (uint32_t)(SAG_ROLE_Pin | SOL_ROLE_Pin | SILECEK_ROLE_Pin);              /* HIGH */
    GPIOD->BSRR = (uint32_t)(FAR_ROLE_Pin | KONTAKTOR_ROLE_Pin | STOP_ROLE_Pin)            /* HIGH */
#if defined(UYARI_ROLE_Pin)
                | (uint32_t)UYARI_ROLE_Pin                                                  /* HIGH */
#endif
                ;

    /* 2) Output + ic PULL-UP (tum role pinleri ayni: HIGH = birakik) */
    g.Pull = GPIO_PULLUP;
    g.Pin  = SAG_ROLE_Pin | SOL_ROLE_Pin | SILECEK_ROLE_Pin;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin  = FAR_ROLE_Pin | KONTAKTOR_ROLE_Pin | STOP_ROLE_Pin
#if defined(UYARI_ROLE_Pin)
           | UYARI_ROLE_Pin
#endif
           ;
    HAL_GPIO_Init(GPIOD, &g);
  }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /* Girisler CubeMX'te zaten PULL-UP ayarli (opto aktif-LOW) -> ek reinit YOK.
   * Roleleri baslangicta KAPALI tut (aktif-HIGH -> LOW=kapali). Kontaktor guvenli. */
  HAL_GPIO_WritePin(SOL_ROLE_GPIO_Port,       SOL_ROLE_Pin,       ROLE_OFF);
  HAL_GPIO_WritePin(SAG_ROLE_GPIO_Port,       SAG_ROLE_Pin,       ROLE_OFF);
  HAL_GPIO_WritePin(STOP_ROLE_GPIO_Port,      STOP_ROLE_Pin,      ROLE_OFF);
  HAL_GPIO_WritePin(FAR_ROLE_GPIO_Port,       FAR_ROLE_Pin,       ROLE_OFF);
  HAL_GPIO_WritePin(SILECEK_ROLE_GPIO_Port,   SILECEK_ROLE_Pin,   ROLE_OFF);
  HAL_GPIO_WritePin(KONTAKTOR_ROLE_GPIO_Port, KONTAKTOR_ROLE_Pin, ROLE_OFF);
#if defined(UYARI_ROLE_Pin)
  HAL_GPIO_WritePin(UYARI_ROLE_GPIO_Port, UYARI_ROLE_Pin, UYARI_OFF);  /* TERS: boste kapali */
#endif

  /* Ic pull'u KALICI yap: kart AKTIF-LOW -> TUM role pinleri PULL-UP
   * (HIGH = birakik). Belt-and-suspenders. */
  {
    GPIO_InitTypeDef gp = {0};
    gp.Mode = GPIO_MODE_OUTPUT_PP; gp.Speed = GPIO_SPEED_FREQ_LOW;
    gp.Pull = GPIO_PULLUP;
    gp.Pin  = SAG_ROLE_Pin | SOL_ROLE_Pin | SILECEK_ROLE_Pin;   HAL_GPIO_Init(GPIOC, &gp);
    gp.Pin  = FAR_ROLE_Pin | KONTAKTOR_ROLE_Pin | STOP_ROLE_Pin
#if defined(UYARI_ROLE_Pin)
            | UYARI_ROLE_Pin
#endif
            ;                                                    HAL_GPIO_Init(GPIOD, &gp);
  }

  /* Gaz PWM cikisini baslat, %0'da tut (guvenli). */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  /* Acilis mesaji: UART hattinin calistigini kanitlar (CAN'den bagimsiz) */
  {
    const char *boot = "{\"boot\":1,\"msg\":\"F407 acildi TELEM UART OK (PA2)\"}\r\n";
    for (int i = 0; i < 3; i++)
      HAL_UART_Transmit(&TELEM_UART, (uint8_t *)boot, (uint16_t)strlen(boot), 100);
  }

  /* CAN'lari baslat (hata olsa da sistem durmaz) */
  Kelly_CAN_Baslat();     /* CAN1 - Kelly 250k */
  BMS_CAN2_Baslat();      /* CAN2 - BMS   500k */

  /* Emerson yerlesik sarj koprusu: MCP2515 (SPI3) baslat.
   * Basarisiz olsa da sistem DURMAZ (mcp_hazir=0 kalir, motor serbest kalir,
   * telemetride "mcp":0 gorunur). */
  mcp_hazir = MCP2515_Init();
  t_obc     = HAL_GetTick();

  /* MCP_INT (PD0/EXTI0) KESMESI KAPATILDI: RX zaten ana dongude her 10 ms
   * yoklamayla okunuyor (asagidaki "2c" blogu) -> INT gereksiz. Kontaktor
   * darbesi/parazit PD0'i tetikleyip "kesme firtinasi" ile ana donguyu
   * BOGAMASIN diye devre disi. RX aynen calisir; sadece EXTI tetiklenmez.
   * GERI ALMAK: bu satiri sil (INT tekrar aktif olur). */
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);

  t_tel   = HAL_GetTick();
  t_buton = HAL_GetTick();
  t_raw   = HAL_GetTick();
  t_blink = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* ── Sinyal blink fazi: 0.7s ac / 0.7s kapa ─────────────── */
    if (now - t_blink >= 700)
    {
        t_blink = now;
        blink_on ^= 1u;
    }

    /* ── 1) Girisleri oku + roleleri sur (her 20ms) ─────────── */
    if (now - t_buton >= 20)
    {
        t_buton = now;

        /* --- ROLE BEKLETME KOSULLARI ---
         * 1) ACILIS (ilk 1 sn): optolar otururken girisler yok sayilir.
         * 2) SARJ TAKILIYKEN: buzzer+kontaktor HARIC her sey TAMAMEN KAPALI
         *    (sinyal/far/silecek/stop OFF; buzzer kendi ters mantigiyla sessiz
         *    bekler, kontaktor Sarj_Kontrol'de yonetilir).
         * 3) SARJDAN CIKINCA: fisten cekilme haberi gelince ACILISTAKI GIBI
         *    1 sn bekletme -> optolar yerine gelsin, sonra sistem aktif olsun. */
        uint8_t sarj_gecis = ((t_sarj_cikis != 0u) &&
                              ((now - t_sarj_cikis) < SARJ_CIKIS_GRACE_MS)) ? 1u : 0u;
        if (now < BOOT_GRACE_MS || obc_link || sarj_gecis)
        {
            HAL_GPIO_WritePin(SOL_ROLE_GPIO_Port,     SOL_ROLE_Pin,     ROLE_OFF);
            HAL_GPIO_WritePin(SAG_ROLE_GPIO_Port,     SAG_ROLE_Pin,     ROLE_OFF);
            HAL_GPIO_WritePin(FAR_ROLE_GPIO_Port,     FAR_ROLE_Pin,     ROLE_OFF);
            HAL_GPIO_WritePin(STOP_ROLE_GPIO_Port,    STOP_ROLE_Pin,    ROLE_OFF);
            HAL_GPIO_WritePin(SILECEK_ROLE_GPIO_Port, SILECEK_ROLE_Pin, ROLE_OFF);
#if defined(UYARI_ROLE_Pin)
            HAL_GPIO_WritePin(UYARI_ROLE_GPIO_Port,   UYARI_ROLE_Pin,   UYARI_OFF);
#endif
        }
        else
        {
        /* --- Optokuplor girislerini HAM oku (tek bayt; far/silecek TERS okunur) --- */
        uint8_t g_raw = 0;
        if (HAL_GPIO_ReadPin(SAG_SIGNAL_KOL_GPIO_Port, SAG_SIGNAL_KOL_Pin) == OPTO_AKTIF) g_raw |= 0x01;
        if (HAL_GPIO_ReadPin(SOL_SIGNAL_KOL_GPIO_Port, SOL_SIGNAL_KOL_Pin) == OPTO_AKTIF) g_raw |= 0x02;
        if (HAL_GPIO_ReadPin(FAR_BUTON_GPIO_Port,      FAR_BUTON_Pin)      == OPTO_AKTIF) g_raw |= 0x04; /* far: buton=opto LOW (ters okuma kaldirildi; bosta role cekmesin) */
        if (HAL_GPIO_ReadPin(SAG_KAPI_GPIO_Port,       SAG_KAPI_Pin)       == OPTO_AKTIF) g_raw |= 0x08;
        if (HAL_GPIO_ReadPin(SOL_KAPI_GPIO_Port,       SOL_KAPI_Pin)       == OPTO_AKTIF) g_raw |= 0x10;
        if (HAL_GPIO_ReadPin(FREN_GIRIS_GPIO_Port,     FREN_GIRIS_Pin)     != OPTO_AKTIF) g_raw |= 0x20; /* fren: opto TERS kablolu (bosta AKTIF okunuyor) -> ters okuma */
        if (HAL_GPIO_ReadPin(DORTLU_BUTON_GPIO_Port,   DORTLU_BUTON_Pin)   == OPTO_AKTIF) g_raw |= 0x40;
#if defined(SILECEK_BILGISI_Pin)
        if (HAL_GPIO_ReadPin(SILECEK_BILGISI_GPIO_Port, SILECEK_BILGISI_Pin) == OPTO_AKTIF) g_raw |= 0x80; /* silecek: switch=opto LOW (ters okuma kaldirildi; bosta role cekmesin) */
#endif

        /* --- DEBOUNCE / EMI SUZGECI: Emerson fisi TAK/CEK aninda sebeke paraziti
         * tum optolari bir an 'aktif' gosterip TUM roleleri cektiriyordu.
         * Yeni durum ancak 4 ARDISIK ornekte (~80ms) AYNI okunursa kabul edilir:
         * ms'lik parazit suzulur, butonlar 80ms gecikmeyle aynen calisir. */
        {
            static uint8_t db_prev = 0, db_say = 0, db_ok = 0;
            if (g_raw == db_prev) { if (db_say < 200) db_say++; }
            else                  { db_say = 0; }
            db_prev = g_raw;
            if (db_say >= 3) db_ok = g_raw;
            g_raw = db_ok;                 /* asagida SUZULMUS deger kullanilir */
        }

        uint8_t sag_kol = (g_raw & 0x01) ? 1u : 0u;
        uint8_t sol_kol = (g_raw & 0x02) ? 1u : 0u;
        far_aktif    = (g_raw & 0x04) ? 1u : 0u;
        sag_kapi     = (g_raw & 0x08) ? 1u : 0u;
        sol_kapi     = (g_raw & 0x10) ? 1u : 0u;
        fren_aktif   = (g_raw & 0x20) ? 1u : 0u;
        dortlu_aktif = (g_raw & 0x40) ? 1u : 0u;   /* kalici switch, dogrudan */

        /* --- Sinyal durum makinesi: SON SECILEN KAZANIR --- */
        if (dortlu_aktif)
            sig_state = 3;                              /* DORTLU */
        else if (sag_kol && sol_kol)
            sig_state = sig_son_sel;                    /* ikisi birden -> son secilen */
        else if (sag_kol) { sig_state = 2; sig_son_sel = 2; }   /* SAG */
        else if (sol_kol) { sig_state = 1; sig_son_sel = 1; }   /* SOL */
        else
            sig_state = 0;                              /* SEYIR: ikisi de sabit yanik */

        /* --- Sinyal LED durumu (seyir + blink) --- */
        switch (sig_state)
        {
        case 1:  led_sol = blink_on; led_sag = 0;        break;  /* SOL blink, SAG kapali */
        case 2:  led_sag = blink_on; led_sol = 0;        break;  /* SAG blink, SOL kapali */
        case 3:  led_sol = blink_on; led_sag = blink_on; break;  /* DORTLU birlikte */
        default: led_sol = 0;        led_sag = 0;        break;  /* BOSTA: ikisi de KAPALI (role cekmez) */
        }

        /* --- Roleleri sur (AKTIF-LOW: yanik/cek = ROLE_ON=LOW) --- */
        HAL_GPIO_WritePin(SOL_ROLE_GPIO_Port,   SOL_ROLE_Pin,   led_sol     ? ROLE_ON : ROLE_OFF);
        HAL_GPIO_WritePin(SAG_ROLE_GPIO_Port,   SAG_ROLE_Pin,   led_sag     ? ROLE_ON : ROLE_OFF);
        HAL_GPIO_WritePin(FAR_ROLE_GPIO_Port,   FAR_ROLE_Pin,   far_aktif   ? ROLE_ON : ROLE_OFF); /* far: mirror */
        HAL_GPIO_WritePin(STOP_ROLE_GPIO_Port,  STOP_ROLE_Pin,  fren_aktif  ? ROLE_ON : ROLE_OFF); /* stop: fren */
        /* Kapi switch'leri (sag_kapi/sol_kapi) sadece okundu -> telemetriye gider */

#if defined(SILECEK_ROLE_Pin) && defined(SILECEK_BILGISI_Pin)
        /* --- Silecek: KALICI switch (SUZULMUS okuma, ters okuma yukarida) --- */
        silecek_aktif = (g_raw & 0x80) ? 1u : 0u;
        HAL_GPIO_WritePin(SILECEK_ROLE_GPIO_Port, SILECEK_ROLE_Pin,
                          silecek_aktif ? ROLE_ON : ROLE_OFF);
#endif

        /* --- UYARI / ACIL DURUM: kritik sartlardan biri varsa roleyi cek --- */
        {
            uint8_t uyari = 0;
#if UYARI_KELLY_KOPUK
            /* Kelly 5 sn sustuysa alarm; ANCAK sarjdan yeni cikildiysa CALMAZ:
             * kontaktor 5 sn bekliyor + Kelly'nin acilip CAN'e baslamasi zaman
             * aliyor -> toparlanma penceresi taninir. */
            if (((now - kel_last_tick) > UYARI_KELLY_KOPUK_MS) &&
                !((t_sarj_cikis != 0u) && ((now - t_sarj_cikis) < SARJ_SONRASI_ALARM_SUSKUN_MS)))
                uyari = 1;
#endif
#if UYARI_KELLY_HATA
            if (kel_fault != 0) uyari = 1;                  /* kontrolcu hata bildiriyor */
#endif
#if UYARI_BMS_KORUMA
            if (bms_ok && (bms_prot0 | bms_prot1 | bms_alarm0 | bms_alarm1))
                uyari = 1;                                  /* BMS koruma/alarm aktif */
#endif
#if UYARI_SICAKLIK
            if (kelly_ok && (kel_mot_tmp >= UYARI_MOT_SIC_MAX ||
                             kel_ctrl_tmp >= UYARI_KTRL_SIC_MAX))
                uyari = 1;                                  /* asiri sicaklik */
#endif
#if UYARI_DUSUK_SOC
            if (bms_ok && bms_soc <= UYARI_SOC_MIN) uyari = 1;  /* kritik dusuk batarya */
#endif
#if UYARI_BMS_KOPUK
            if (!bms_ok) uyari = 1;                         /* BMS CAN kopuk */
#endif
            /* Emerson (sarj) TAKILIYKEN buzzer/flasor CALMASIN: sarjda AKS zaten
             * calisir, Kelly bilerek KOPUK gosterilir -> alarm bastirilir. */
            if (obc_link) uyari = 0;
            uyari_aktif = uyari;
#if defined(UYARI_ROLE_Pin)
            HAL_GPIO_WritePin(UYARI_ROLE_GPIO_Port, UYARI_ROLE_Pin,
                              uyari_aktif ? UYARI_ON : UYARI_OFF);   /* TERS surulur */
#endif
        }
        }  /* acilis yumusatma else sonu */

        /* --- Gaz pedali ADC (PA1) --- */
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
            gaz_adc = (uint16_t)HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    /* ── 2) CAN link sagligi ────────────────────────────────── */
    if ((now - kel_last_tick) > KELLY_TIMEOUT_MS)
        kelly_ok = 0;
    if ((now - bms_last_tick) > BMS_TIMEOUT_MS)
        bms_ok = 0;

    /* Emerson (sarj) TAKILIYKEN motor surucu KULLANILAMAZ -> Kelly KOPUK goster.
     * Kelly aç/kapat bataryaya bagli oldugu icin sarjda bile veri gelebilir;
     * sarj modunda motor devre disi kabul edilir. */
    if (obc_link)
        kelly_ok = 0;

    /* ── 2a) Motor surucu (Kelly) durumu DEGISTIGI an Pi'ye HEMEN yolla ──
     * Normalde telemetri 100ms'de bir gider; ama motor surucu KAPANDIGI an
     * (kelly_ok 1->0) 100ms slotunu BEKLEMEDEN aninda bir paket basariz.
     * Ayrica kapaninca donmus rpm/akim/gaz degerlerini SIFIRLARIZ ki kadran
     * son hizda donmus kalmasin, hemen 'durdu' gostersin. */
    {
        static uint8_t kelly_ok_prev = 1;
        if (kelly_ok != kelly_ok_prev)
        {
            if (!kelly_ok)   /* motor surucu kapandi/koptu */
            {
                kel_rpm      = 0;
                kel_cur  = 0;
                kel_throttle = 0;
            }
            send_telemetri();          /* durum degisimini ANINDA bildir */
            kelly_ok_prev = kelly_ok;
        }
    }

    /* ── 2b) HAM CAN2 tarama dokumu (her 500ms) ─────────────── */
    if (now - t_raw >= 500)
    {
        t_raw = now;
        raw_dump();
        obc_raw_dump();               /* Emerson ham cerceveleri (gercek ID'ler) */
        if (mcp_hazir)
        {
            static uint32_t t_mcp_reset = 0;        /* son MCP reset ani (TEC sifirlanir) */
            uint8_t rec_onceki = mcp_rec;
            MCP_ReadDiag();                         /* EFLG/TEC/REC oku */

            /* ===== EMERSON VARLIK TESPITI =====
             * Gecerli cerceve alinamiyor (Emerson kendiliginden yayin yapmiyor,
             * REC hep 0). ANCAK bizim sorgularimizi ACK'liyor -> bu ACK'i TEC
             * uzerinden olcuyoruz. Sonuc ayni obc_last_tick'i besler; debounce
             * (Kural 1) + hold-off (Kural 2) mantigi DEGISMEDEN calisir. */
            /* ASIL TESPIT: TEC dusuk = Emerson cercevelerimizi ACK'liyor = TAKILI.
             * Acilistan ve her MCP reset'inden sonra TEC 0'dan basladigi icin
             * EMERSON_TEC_GUARD_MS boyunca karar verilmez (sahte "takili" olmasin). */
            uint8_t tec_oturdu = (now >= EMERSON_TEC_GUARD_MS) &&
                                 ((now - t_mcp_reset) >= EMERSON_TEC_GUARD_MS);
            uint8_t emerson_var = (tec_oturdu && (mcp_tec < EMERSON_TEC_ESIK)) ? 1u : 0u;
            /* YEDEK: gercek yayin baslarsa (REC hareketi) o da varlik sayilir */
            if ((mcp_rec != rec_onceki) && (mcp_rec != 0u)) emerson_var = 1u;

            if (emerson_var && (now >= BOOT_GRACE_MS))
            {
                obc_last_tick = now;
                obc_rx_cnt++;   /* TANI: kadranda "R48f" olarak gorunur.
                                 * Gecerli cerceve alinamadigi icin bu sayac artik
                                 * "hat aktivitesi kac kez algilandi" demektir.
                                 * R48f ARTIYORSA tespit calisiyor; 0'da KALIYORSA
                                 * tespit hic tetiklenmiyor (veya eski firmware yuklu). */
            }

            if (mcp_eflg & 0x20)                    /* TXBO: bus-off -> kilitlenmesin */
            {
                mcp_hazir = MCP2515_Init();         /* yeniden baslat (kurtarma) */
                mcp_rec = 0; mcp_tec = 0;           /* reset sonrasi sayaclar sifir */
                t_mcp_reset = now;                  /* TEC yeniden oturana kadar bekle */
            }
        }
    }

    /* ── 2c) Emerson / MCP2515 (yerlesik sarj) ──────────────── */
    if (mcp_hazir)
    {
        /* RX bosaltma: INT bayragi VEYA her 10ms yoklama.
         * INT pini/EXTI calismasa bile frame'ler yine okunur (guvenli). */
        if (mcp_int_flag || (now - t_mcp >= 10))
        {
            t_mcp = now;
            mcp_int_flag = 0;
            can_frame_t rf;
            uint8_t guard = 0;
            while (MCP2515_Recv(&rf) && guard++ < 16)
            {
                obc_rx_all++;            /* tani: alinan TUM cerceveler (ID farketmez) */
                OBC_RawCapture(&rf);     /* ham sakla (tani/decode) - TUM ID'ler icin */
                R48_ParseFrame(&rf);     /* eslesen ID -> Vout vb. */
                /* ID FILTRESI: SADECE gercek Emerson telemetri ID'leri (canli
                 * yakalamayla dogrulanmis: 0x0707F803, 0x060F8003) obc_last_tick'i
                 * tazeler. obc_link'i BURADA set ETMIYORUZ; karari Sarj_Kontrol'deki
                 * debounce (Kural 1) + hold-off (Kural 2) veriyor. ACILIS KORUMASI: ilk
                 * BOOT_GRACE_MS icindeki frame'ler sayilmaz (acilis artigi). */
                if ((rf.id == R48_ID_TELEM || rf.id == 0x060F8003UL) &&
                    (now >= BOOT_GRACE_MS))
                    obc_last_tick = now;
            }
        }
        /* Emerson'dan periyodik telemetri iste (her 1s) */
        if (now - t_obc >= 1000)
        {
            t_obc = now;
            R48_RequestData();
        }
    }

    /* Sarj/motor kontaktor durum makinesi (her turda calisir) */
    Sarj_Kontrol(now);

    /* ── 3) Telemetri + LED (her 100ms) ─────────────────────── */
    if (now - t_tel >= 100)
    {
        t_tel = now;
        send_telemetri();

        /* STATUS_LED (PA6) kalp atisi - aktif dusuk */
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
        /* CAN_LED (PA7): Kelly link OK ise yanik (aktif dusuk) */
        HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin,
            kelly_ok ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SAG_ROLE_Pin|SOL_ROLE_Pin|SILECEK_ROLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STATUS_LED_Pin|CAN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MCP_RESET_Pin|FAR_ROLE_Pin|UYARI_ROLE_Pin|STOP_ROLE_Pin
                          |KONTAKTOR_ROLE_Pin|MCP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DORTLU_BUTON_Pin SOL_SIGNAL_KOL_Pin SAG_KAPI_Pin FAR_BUTON_Pin
                           SOL_KAPI_Pin FREN_GIRIS_Pin */
  GPIO_InitStruct.Pin = DORTLU_BUTON_Pin|SOL_SIGNAL_KOL_Pin|SAG_KAPI_Pin|FAR_BUTON_Pin
                          |SOL_KAPI_Pin|FREN_GIRIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SAG_SIGNAL_KOL_Pin */
  GPIO_InitStruct.Pin = SAG_SIGNAL_KOL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SAG_SIGNAL_KOL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SAG_ROLE_Pin SOL_ROLE_Pin SILECEK_ROLE_Pin */
  GPIO_InitStruct.Pin = SAG_ROLE_Pin|SOL_ROLE_Pin|SILECEK_ROLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_LED_Pin CAN_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin|CAN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SILECEK_BILGISI_Pin */
  GPIO_InitStruct.Pin = SILECEK_BILGISI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SILECEK_BILGISI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCP_INT_Pin */
  GPIO_InitStruct.Pin = MCP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCP_RESET_Pin MCP_CS_Pin */
  GPIO_InitStruct.Pin = MCP_RESET_Pin|MCP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : FAR_ROLE_Pin UYARI_ROLE_Pin STOP_ROLE_Pin KONTAKTOR_ROLE_Pin */
  GPIO_InitStruct.Pin = FAR_ROLE_Pin|UYARI_ROLE_Pin|STOP_ROLE_Pin|KONTAKTOR_ROLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
