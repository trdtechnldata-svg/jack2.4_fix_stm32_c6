#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// --- Cấu hình chân kết nối màn hình ST7735 ---
#define TFT_CS PA4
#define TFT_RST PB6
#define TFT_DC PB7 // chân RS trên board
#define BUTTON_PIN PA6

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
int maLCD = 1;
int yLCD = 5;
#define MAU_DO 0x001F
#define MAU_XANH_DUONG 0xF800
#define MAU_XANH_LA 0x07E0
#define MAU_VANG 0x07FF
#define MAU_DEN 0x0000
#define MAU_TRANG 0xFFFF
int currentPage = 0;

unsigned long thoigian, main_delay, thoigian_2, thoigian_3, thoigian1;
unsigned long lastTime, lastPageTime;

uint8_t sum_3, soc = 50, dem;

uint8_t var1_1, var1_2, var1_3, var1_4, var1_5, var1_6, var1_7, var1_8, var1_9, var1_10, var1_11, var1_12, var1_13, var1_14, var1_15, var1_16, var1_17, var1_18, var1_19, var1_20;
uint8_t var2_1, var2_2, var2_3, var2_4, var2_5, var2_6, var2_7, var2_8, var2_9, var2_10, var2_11, var2_12, var2_13, var2_14, var2_15, var2_16, var2_17, var2_18, var2_19, var2_20;
uint16_t var3_1, var3_2, var3_3, var3_4, var3_5, var3_6, var3_7, var3_8, var3_9, var3_10, var3_11, var3_12, var3_13, var3_14, var3_15, var3_16, var3_17, var3_18, var3_19, var3_20;
uint8_t var1_21, var1_22, var1_23, var1_24, var1_25, var1_26, var1_27, var1_28, var1_29, var1_30, var1_31, var1_32, var1_33, var1_34, var1_35, var1_36, var1_37, var1_38, var1_39, var1_40;
uint8_t var2_21, var2_22, var2_23, var2_24, var2_25, var2_26, var2_27, var2_28, var2_29, var2_30, var2_31, var2_32, var2_33, var2_34, var2_35, var2_36, var2_37, var2_38, var2_39, var2_40;
uint16_t var21, var22, var23, var24, var25, var26, var27, var28, var29, var30, var31, var32, var33, var34, var35, var36, var37, var38, var39, var40;
uint16_t var_100, var_101, var_102, var_103;

float dienap_nguyen = 0;
float dienapphantram = 0;
float volpin = 0;
float a4 = 0;
float a5 = 0, a_load;
float a6, a7, a_total;
float ampe_nguyen = 0;
float ampe_phantram = 0;
float ampe_charge = 0;
float v_max_lfp, v_min_lfp, rcap_lfp, soh_lfp;
uint8_t temp1, tempbms, phantram;
bool systemReady = false;

float dienAp20Cell[20];

#ifndef CAN1
#define CAN1 ((CAN_TypeDef *)0x40006400)
#endif

/**
 * Cấu hình xung nội HSI cho STM32F103 (Chạy ở 64MHz ổn định tuyệt đối không cần thạch anh ngoài)
 */
extern "C" void SystemClock_Config(void)
{
  // 1. Bật HSI và chờ ổn định
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY))
    ;

  // 2. Cấu hình Flash Latency = 2 Wait States cho dải tần số trên 48MHz
  FLASH->ACR |= FLASH_ACR_PRFTBE;
  FLASH->ACR &= ~FLASH_ACR_LATENCY;
  FLASH->ACR |= FLASH_ACR_LATENCY_2;

  // 3. Phân phối xung Bus: AHB = 64MHz, APB1 = 32MHz (Cho CAN), APB2 = 64MHz
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

  // 4. Cấu hình PLL: Nguồn từ HSI/2 (4MHz) nhân 16 = 64MHz
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
  RCC->CFGR |= RCC_CFGR_PLLMULL16; // 4MHz * 16 = 64MHz

  // 5. Bật PLL và chờ ổn định
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY))
    ;

  // 6. Chuyển nguồn hệ thống sang PLL
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;
}

// Khởi tạo CAN cho STM32F103 với APB1 = 32MHz (Tính toán BitTiming chuẩn 250Kbps)
void CAN_Hardware_Init(void)
{
  uint32_t timeout = 0;

  // 1. Bật xung Clock cho GPIOA, CAN1 và AFIO
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

  // 2. Cấu hình chân CAN: PA12 (TX): AF Push-Pull, PA11 (RX): Input Pull-up
  GPIOA->CRH &= ~(0xFF << 12);
  GPIOA->CRH |= (0x0B << 16);
  GPIOA->CRH |= (0x08 << 12);
  GPIOA->ODR |= (1 << 11);

  // 3. Vào chế độ khởi tạo
  CAN1->MCR |= CAN_MCR_INRQ;
  timeout = 0xFFFF;
  while (((CAN1->MSR & CAN_MSR_INAK) == 0) && (timeout > 0))
  {
    timeout--;
  }

  // 4. Thoát Sleep Mode
  CAN1->MCR &= ~CAN_MCR_SLEEP;
  timeout = 0xFFFF;
  while (((CAN1->MSR & CAN_MSR_SLAK) != 0) && (timeout > 0))
  {
    timeout--;
  }

  // 5. Bật tự phục hồi lỗi (ABOM) và Tự thức dậy (AWUM)
  CAN1->MCR |= (CAN_MCR_ABOM | CAN_MCR_AWUM);
  CAN1->MCR &= ~CAN_MCR_NART;

  // 6. Cấu hình Bit Timing 250Kbps với APB1 = 32MHz: Prescaler = 8 (BRP=7), TS1 = 13, TS2 = 2
  CAN1->BTR = (2UL << 24) | (13UL << 20) | (7UL << 0);

  // 7. Chuyển sang chạy thực tế
  CAN1->MCR &= ~CAN_MCR_INRQ;
  timeout = 0xFFFF;
  while (((CAN1->MSR & CAN_MSR_INAK) != 0) && (timeout > 0))
  {
    timeout--;
  }

  // 8. Cấu hình Filter cơ bản nhận mọi ID
  CAN1->FMR |= CAN_FMR_FINIT;
  CAN1->FA1R &= ~1UL;
  CAN1->FS1R |= 1UL;
  CAN1->FM1R &= ~1UL;

  CAN1->sFilterRegister[0].FR1 = 0x00000000;
  CAN1->sFilterRegister[0].FR2 = 0x00000000;

  CAN1->FA1R |= 1UL;
  CAN1->FMR &= ~CAN_FMR_FINIT;
}

bool CAN_Transmit_Data(uint32_t stdId, uint8_t *data, uint8_t len)
{
  uint8_t mailbox = 0xFF;
  if ((CAN1->TSR & CAN_TSR_TME0) != 0)
    mailbox = 0;
  else if ((CAN1->TSR & CAN_TSR_TME1) != 0)
    mailbox = 1;
  else if ((CAN1->TSR & CAN_TSR_TME2) != 0)
    mailbox = 2;

  if (mailbox == 0xFF)
    return false;

  CAN1->sTxMailBox[mailbox].TIR = (stdId << 21);
  CAN1->sTxMailBox[mailbox].TDTR = (len & 0x0F);

  uint32_t dataLow = 0, dataHigh = 0;
  if (len > 0)
    dataLow |= data[0];
  if (len > 1)
    dataLow |= ((uint32_t)data[1] << 8);
  if (len > 2)
    dataLow |= ((uint32_t)data[2] << 16);
  if (len > 3)
    dataLow |= ((uint32_t)data[3] << 24);

  if (len > 4)
    dataHigh |= data[4];
  if (len > 5)
    dataHigh |= ((uint32_t)data[5] << 8);
  if (len > 6)
    dataHigh |= ((uint32_t)data[6] << 16);
  if (len > 7)
    dataHigh |= ((uint32_t)data[7] << 24);

  CAN1->sTxMailBox[mailbox].TDLR = dataLow;
  CAN1->sTxMailBox[mailbox].TDHR = dataHigh;
  CAN1->sTxMailBox[mailbox].TIR |= 1UL;
  return true;
}

void setLCD()
{
  if (maLCD == 1)
  {
    pinMode(TFT_RST, OUTPUT);
    digitalWrite(TFT_RST, LOW);
    delay(150);
    digitalWrite(TFT_RST, HIGH);
    delay(200);

    tft.initR(INITR_GREENTAB);
    tft.invertDisplay(true);
    tft.setRotation(3);
    delay(50);
    tft.setRotation(1);
    delay(50);

    tft.fillScreen(MAU_DEN);
    yLCD = 25;
  }
  else
  {
    tft.initR(INITR_MINI160x80);
    tft.invertDisplay(false);
    tft.setRotation(1);
    tft.fillScreen(MAU_DEN);
    yLCD = 0;
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);

  setLCD();
  CAN_Hardware_Init();
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  currentPage = 0;
  tft.setTextColor(MAU_VANG);
  tft.setTextSize(1);
  tft.setCursor(15, 30 + yLCD);
  tft.print("DANG KHOI DONG...");

  lastTime = millis() - 200;
  thoigian = millis() - 200;
}

// Khai báo các mảng dữ liệu CAN
byte data100[] = {0x00, 0x00, 0x11, 0x0B, 0x00, 0x00, 0x00, 0x00};
byte data101[] = {0x11, 0x10, 0x01};
byte data201[] = {0x00, 0x00, 0x17, 0xF9, 0x00, 0x00, 0x07, 0xEA};
byte data033[] = {0x00, 0x00, 0x00, 0x00, 0xCC, 0xF1, 0xFF, 0xFF};
byte data300[] = {0x03, 0x52, 0xf9, 0xE9, 0x04, 0xF7, 0xF6, 0x48};
byte data301[] = {0x92, 0x18, 0x03, 0x52, 0x00, 0xFA, 0x64, 0x00};
byte data303[] = {0x12, 0x32, 0x01, 0x00, 0x01, 0x00, 0x04, 0x01};
byte data304[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data305[] = {0x01, 0x01, 0x01};
byte data306[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
byte data307[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data308[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data309[] = {0x11, 0x00, 0x8E, 0xEA, 0xFF, 0xF6, 0xFF, 0xF6};
byte data30A[] = {0xE2, 0x5A, 0x3D, 0x64, 0x00, 0x00, 0x00, 0x00};
byte data30B[] = {0x00, 0x00, 0x17, 0x0E, 0x00, 0x00, 0x15, 0x69};
byte data30C[] = {0x00, 0x00, 0x00};
byte data30D[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data30E[] = {0x99, 0xD4, 0x99, 0xCA, 0x00, 0x00, 0x00, 0x00};
byte data310[] = {0x9E, 0x7A, 0x9E, 0x7A, 0x9E, 0x7A, 0x9E, 0x7A};
byte data311[] = {0x9E, 0x7A, 0x9E, 0x7A, 0x9E, 0x7A, 0x9E, 0x7A};
byte data312[] = {0x9E, 0x7A, 0x9E, 0x7A, 0x9E, 0x7A, 0x9E, 0x7A};
byte data313[] = {0x9E, 0x7A, 0x9E, 0x7A, 0x9E, 0x7A, 0x9E, 0x7A};
byte data314[] = {0x9E, 0x7A, 0x9E, 0x7A, 0x00, 0x00, 0x00, 0x00};
byte data315[] = {0x00, 0x00, 0x00};
byte data316[] = {0x00, 0x00, 0x00, 0x00};
byte data317[] = {0x00, 0x00};
byte data318[] = {0x00, 0x00, 0x00, 0x00};
byte data319[] = {0x0, 0x6F, 0x5B, 0x25, 0x00, 0x00, 0x00, 0x00};
byte data320[] = {0x1F, 0x1F, 0x02, 0x1F, 0x02};
byte data321[] = {0x20, 0x20, 0x20};
byte data322[] = {0x20, 0x20, 0x20, 0x20};
byte data326[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
byte data31A[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data31B[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte data330[] = {0x02, 0x21, 0x00, 0x00, 0x05};
byte data33F[] = {0x05, 0x01, 0x02, 0x51, 0x17, 0x19, 0x81, 0xEA};
byte data340[] = {0x76, 0x55, 0x49, 0x62, 0x00, 0x3F, 0x00, 0x00};
byte data341[] = {0x06, 0x00, 0x11, 0x0C, 0x00, 0x64, 0x5A, 0x9C};
byte data342[] = {0xFF, 0xFF, 0xAB, 0x26, 0x00, 0x00, 0xB5, 0xE0};
byte data343[] = {0xE1, 0xAD, 0xF0, 0xC4, 0xFF, 0xFF, 0x00, 0x00};
byte data500[] = {0x20, 0x01, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00};
byte data501[] = {0x20, 0x34, 0x00, 0x46, 0x00, 0x02, 0xA1, 0x23};
byte data502[] = {0x23, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00};
byte data503[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

String read_batt = " ";

void printCellSection(float *vols, int totalCells, int startNumber)
{
  tft.setTextSize(1);
  int startX = 5;
  int startY = 5 + yLCD;
  int xSpace = 52;
  int ySpace = 14;

  for (int i = 0; i < totalCells; i++)
  {
    int col = i % 3;
    int row = i / 3;
    int currentX = startX + col * xSpace;
    int currentY = startY + row * ySpace;

    tft.setCursor(currentX, currentY);
    int currentCellNum = startNumber + i;
    tft.setTextColor(MAU_XANH_LA);
    if (currentCellNum < 10)
      tft.print("0");
    tft.print(currentCellNum);
    tft.print(":");

    int volX = currentX + 18;
    tft.fillRect(volX, currentY, 30, 8, MAU_DEN);
    tft.setCursor(volX, currentY);
    tft.setTextColor(MAU_TRANG);
    tft.print(vols[i], 3);
  }
}

void displayPage(int page, bool clearScreen)
{
  dienAp20Cell[0] = var3_5 * 0.0001;
  dienAp20Cell[1] = var3_6 * 0.0001;
  dienAp20Cell[2] = var3_7 * 0.0001;
  dienAp20Cell[3] = var3_8 * 0.0001;
  dienAp20Cell[4] = var3_9 * 0.0001;
  dienAp20Cell[5] = var3_10 * 0.0001;
  dienAp20Cell[6] = var3_11 * 0.0001;
  dienAp20Cell[7] = var3_12 * 0.0001;
  dienAp20Cell[8] = var3_13 * 0.0001;
  dienAp20Cell[9] = var3_14 * 0.0001;
  dienAp20Cell[10] = var3_15 * 0.0001;
  dienAp20Cell[11] = var3_16 * 0.0001;
  dienAp20Cell[12] = var3_17 * 0.0001;
  dienAp20Cell[13] = var3_18 * 0.0001;
  dienAp20Cell[14] = var3_19 * 0.0001;
  dienAp20Cell[15] = var3_20 * 0.0001;
  dienAp20Cell[16] = var21 * 0.0001;
  dienAp20Cell[17] = var22 * 0.0001;
  dienAp20Cell[18] = var23 * 0.0001;
  dienAp20Cell[19] = var24 * 0.0001;

  if (clearScreen)
  {
    tft.fillRect(0, 5, 160, 103, MAU_DEN);
  }

  switch (page)
  {
  case 1:
    if (clearScreen)
    {
      tft.setTextColor(MAU_XANH_LA);
      tft.setTextSize(1);
      tft.setCursor(15, 5 + yLCD);
      tft.print("SOC");
    }
    tft.fillRect(25, 40 + yLCD, 100, 16, MAU_DEN);
    tft.setTextColor(MAU_TRANG);
    tft.setTextSize(2);
    tft.setCursor(25, 40 + yLCD);
    tft.print(var2_32, 1);
    tft.print(" %");
    break;

  case 2:
    if (clearScreen)
    {
      tft.setTextColor(MAU_XANH_LA);
      tft.setTextSize(1);
      tft.setCursor(15, 5 + yLCD);
      tft.print("AMPE");
    }
    tft.fillRect(25, 40 + yLCD, 100, 16, MAU_DEN);
    tft.setTextColor(MAU_TRANG);
    tft.setTextSize(2);
    tft.setCursor(25, 40 + yLCD);
    tft.print(a_total, 1);
    tft.print(" A");
    break;

  case 3:
    if (clearScreen)
    {
      tft.setTextColor(MAU_XANH_LA);
      tft.setTextSize(1);
      tft.setCursor(15, 5 + yLCD);
      tft.print("CONG SUAT");
    }
    tft.fillRect(25, 40 + yLCD, 120, 16, MAU_DEN);
    tft.setTextColor(MAU_VANG);
    tft.setTextSize(2);
    tft.setCursor(25, 40 + yLCD);
    tft.print(a_total * volpin, 1);
    tft.print(" W");
    break;

  case 4:
    if (clearScreen)
    {
      tft.setTextColor(MAU_XANH_LA);
      tft.setTextSize(1);
      tft.setCursor(15, 5 + yLCD);
      tft.print("VOLPIN");
    }
    tft.fillRect(25, 40 + yLCD, 100, 16, MAU_DEN);
    tft.setTextColor(MAU_TRANG);
    tft.setTextSize(2);
    tft.setCursor(25, 40 + yLCD);
    tft.print(volpin, 1);
    tft.print(" V");
    break;

  case 5:
    if (clearScreen)
    {
      tft.setTextColor(MAU_XANH_LA);
      tft.setTextSize(1);
      tft.setCursor(15, 5 + yLCD);
      tft.print("Cycle");
      tft.setCursor(65, 5 + yLCD);
      tft.print("Delta_vol");
    }
    tft.fillRect(15, 40 + yLCD, 100, 16, MAU_DEN);
    tft.fillRect(65, 40 + yLCD, 100, 16, MAU_DEN);
    tft.setTextColor(MAU_TRANG);
    tft.setTextSize(2);
    tft.setCursor(15, 40 + yLCD);
    tft.print(var33, 1);
    tft.setCursor(65, 40 + yLCD);
    tft.print((var3_1 * 0.0001) - (var3_2 * 0.0001), 4);
    break;

  case 6:
    if (clearScreen)
    {
      tft.setTextColor(MAU_XANH_LA);
      tft.setTextSize(1);
      tft.setCursor(15, 5 + yLCD);
      tft.print("Temp");
    }
    tft.fillRect(25, 40 + yLCD, 100, 16, MAU_DEN);
    tft.setTextColor(MAU_TRANG);
    tft.setTextSize(2);
    tft.setCursor(25, 40 + yLCD);
    tft.print(var_100, 1);
    tft.print("  ");
    tft.print(var_101, 1);
    tft.print("  ");
    tft.print(var_102, 1);
    break;

  case 7:
    printCellSection(dienAp20Cell, 12, 1);
    break;

  case 8:
    printCellSection(&dienAp20Cell[12], 8, 13);
    break;
  }
}

void loop()
{
  if ((USART1->SR & USART_SR_RXNE) != 0)
  {
    read_batt = (uint8_t)(USART1->DR & 0xFF);
  }

  if (millis() - lastTime >= 200)
  {
    CAN_Transmit_Data(0x201, data201, 8);
    if (read_batt == "1")
    {
      CAN_Transmit_Data(0x201, data201, 8);
    }
    lastTime = millis();
  }

  if ((CAN1->RF0R & CAN_RF0R_FOVR0) != 0)
  {
    CAN1->RF0R = CAN_RF0R_FOVR0;
  }

  for (int i = 0; i < 50; i++)
  {
    if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0)
    {
      uint32_t received_id = (CAN1->sFIFOMailBox[0].RIR) >> 21;
      uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
      uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

      uint8_t buf[8];
      buf[0] = (uint8_t)(rdlr & 0xFF);
      buf[1] = (uint8_t)((rdlr >> 8) & 0xFF);
      buf[2] = (uint8_t)((rdlr >> 16) & 0xFF);
      buf[3] = (uint8_t)(rdlr >> 24);
      buf[4] = (uint8_t)(rdhr & 0xFF);
      buf[5] = (uint8_t)((rdhr >> 8) & 0xFF);
      buf[6] = (uint8_t)((rdhr >> 16) & 0xFF);
      buf[7] = (uint8_t)(rdhr >> 24);

      switch (received_id)
      {
      case 0x30A:
      {
        uint8_t old_soc = var2_32;
        var2_31 = buf[0];
        var1_31 = buf[1];
        var2_32 = buf[2];
        phantram = buf[2];
        var1_32 = buf[3];
        temp1 = buf[3];
        var2_33 = buf[4];
        var1_33 = buf[5];
        var2_34 = buf[6];
        var1_34 = buf[7];

        rcap_lfp = word(var2_31, var1_31);
        soh_lfp = var1_32;

        if (currentPage == 0 && var2_32 > 0)
        {
          systemReady = true;
          currentPage = 1;
          displayPage(currentPage, true);
          lastPageTime = millis();
        }
        else if (currentPage == 1 && old_soc != var2_32)
        {
          displayPage(currentPage, false);
          lastPageTime = millis();
        }
        break;
      }
      case 0x321:
        tempbms = buf[0];
        var1_35 = buf[1];
        var2_35 = buf[0];
        var1_36 = buf[3];
        var2_36 = buf[2];
        var1_37 = buf[5];
        var2_37 = buf[4];
        var1_38 = buf[7];
        var2_38 = buf[6];
        break;
      case 0x309:
        a4 = buf[4];
        a5 = buf[5];
        a6 = buf[6];
        a7 = buf[7];
        break;
      case 0x320:
        var_100 = buf[1];
        var_101 = buf[0];
        var_102 = buf[3];
        var_103 = buf[2];
        break;
      case 0x30E:
        var1_27 = buf[1];
        var2_27 = buf[0];
        var1_28 = buf[3];
        var2_28 = buf[2];
        var1_29 = buf[5];
        var2_29 = buf[4];
        var1_30 = buf[7];
        var2_30 = buf[6];
        break;
      case 0x311:
        var1_5 = buf[1];
        var2_5 = buf[0];
        var1_6 = buf[3];
        var2_6 = buf[2];
        var1_7 = buf[5];
        var2_7 = buf[4];
        var1_8 = buf[7];
        var2_8 = buf[6];
        break;
      case 0x312:
        var1_9 = buf[1];
        var2_9 = buf[0];
        var1_10 = buf[3];
        var2_10 = buf[2];
        var1_11 = buf[5];
        var2_11 = buf[4];
        var1_12 = buf[7];
        var2_12 = buf[6];
        break;
      case 0x313:
        var1_13 = buf[1];
        var2_13 = buf[0];
        var1_14 = buf[3];
        var2_14 = buf[2];
        var1_15 = buf[5];
        var2_15 = buf[4];
        var1_16 = buf[7];
        var2_16 = buf[6];
        break;
      case 0x314:
        var1_17 = buf[1];
        var2_17 = buf[0];
        var1_18 = buf[3];
        var2_18 = buf[2];
        var1_19 = buf[5];
        var2_19 = buf[4];
        var1_20 = buf[7];
        var2_20 = buf[6];
        break;
      case 0x31A:
        var1_21 = buf[1];
        var2_21 = buf[0];
        var1_22 = buf[3];
        var2_22 = buf[2];
        var1_23 = buf[5];
        var2_23 = buf[4];
        var1_24 = buf[7];
        var2_24 = buf[6];
        break;
      case 0x31B:
        var1_25 = buf[1];
        var2_25 = buf[0];
        var1_26 = buf[3];
        var2_26 = buf[2];
        break;
      case 0x310:
        var1_1 = buf[1];
        var2_1 = buf[0];
        var1_2 = buf[3];
        var2_2 = buf[2];
        var1_3 = buf[5];
        var2_3 = buf[4];
        var1_4 = buf[7];
        var2_4 = buf[6];
        break;
      }

      v_max_lfp = word(var1_3, var2_4);
      v_min_lfp = word(var2_2, var1_2);

      volpin = var3_5 * 0.0001 + var3_6 * 0.0001 + var3_7 * 0.0001 + var3_8 * 0.0001 + var3_9 * 0.0001 + var3_10 * 0.0001 + var3_11 * 0.0001 + var3_12 * 0.0001 + var3_13 * 0.0001 + var3_14 * 0.0001 + var3_15 * 0.0001 + var3_16 * 0.0001 + var3_17 * 0.0001 + var3_18 * 0.0001 + var3_19 * 0.0001 + var3_20 * 0.0001 + var21 * 0.0001 + var22 * 0.0001 + var23 * 0.0001 + var24 * 0.0001;

      a_total = word(a6, a7) * 2 * 0.01;
      var3_1 = word(var2_1, var1_1);
      var3_2 = word(var2_2, var1_2);
      var3_3 = word(var2_3, var1_3);
      var3_4 = word(var2_4, var1_4);
      var3_5 = word(var2_5, var1_5);
      var3_6 = word(var2_6, var1_6);
      var3_7 = word(var2_7, var1_7);
      var3_8 = word(var2_8, var1_8);
      var3_9 = word(var2_9, var1_9);
      var3_10 = word(var2_10, var1_10);
      var3_11 = word(var2_11, var1_11);
      var3_12 = word(var2_12, var1_12);
      var3_13 = word(var2_13, var1_13);
      var3_14 = word(var2_14, var1_14);
      var3_15 = word(var2_15, var1_15);
      var3_16 = word(var2_16, var1_16);
      var3_17 = word(var2_17, var1_17);
      var3_18 = word(var2_18, var1_18);
      var3_19 = word(var2_19, var1_19);
      var3_20 = word(var2_20, var1_20);
      var21 = word(var2_21, var1_21);
      var22 = word(var2_22, var1_22);
      var23 = word(var2_23, var1_23);
      var24 = word(var2_24, var1_24);
      var33 = word(var2_33, var1_33);

      CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
  }

  if (var2_32 < 100)
  {
    if ((unsigned long)(millis() - thoigian) > 200)
    {
      dem++;
      if (dem == 1)
        data303[0] = 0x10;
      if (dem == 2)
        data303[0] = 0x11;
      if (dem == 3)
        data303[0] = 0x12;
      if (dem == 4)
        data303[0] = 0x13;
      if (dem > 5)
        dem = 0;

      CAN_Transmit_Data(0x033, data033, 8);
      CAN_Transmit_Data(0x100, data100, 8);
      CAN_Transmit_Data(0x101, data101, 3);
      data30A[2] = var2_32;
      CAN_Transmit_Data(0x30A, data30A, 8);
      CAN_Transmit_Data(0x300, data300, 8);
      CAN_Transmit_Data(0x301, data301, 8);
      CAN_Transmit_Data(0x303, data303, 8);
      CAN_Transmit_Data(0x304, data304, 8);
      CAN_Transmit_Data(0x33F, data33F, 8);
      CAN_Transmit_Data(0x305, data305, 8);
      CAN_Transmit_Data(0x306, data306, 8);
      CAN_Transmit_Data(0x307, data307, 8);
      CAN_Transmit_Data(0x308, data308, 8);
      CAN_Transmit_Data(0x309, data309, 8);
      CAN_Transmit_Data(0x30B, data30B, 8);
      CAN_Transmit_Data(0x30C, data30C, 8);
      CAN_Transmit_Data(0x30D, data30D, 8);
      CAN_Transmit_Data(0x310, data310, 8);
      CAN_Transmit_Data(0x311, data311, 8);
      CAN_Transmit_Data(0x312, data312, 8);
      CAN_Transmit_Data(0x313, data313, 8);
      CAN_Transmit_Data(0x314, data314, 8);
      CAN_Transmit_Data(0x315, data315, 8);
      CAN_Transmit_Data(0x316, data316, 8);
      CAN_Transmit_Data(0x317, data317, 8);
      CAN_Transmit_Data(0x318, data318, 8);
      CAN_Transmit_Data(0x319, data319, 8);
      CAN_Transmit_Data(0x320, data320, 8);
      CAN_Transmit_Data(0x321, data321, 8);
      CAN_Transmit_Data(0x322, data322, 8);
      CAN_Transmit_Data(0x326, data326, 8);
      CAN_Transmit_Data(0x31A, data31A, 8);
      CAN_Transmit_Data(0x31B, data31B, 8);
      CAN_Transmit_Data(0x30E, data30E, 8);
      CAN_Transmit_Data(0x330, data330, 8);
      CAN_Transmit_Data(0x340, data340, 8);
      CAN_Transmit_Data(0x341, data341, 8);
      CAN_Transmit_Data(0x342, data342, 8);
      CAN_Transmit_Data(0x343, data343, 8);

      thoigian = millis();
    }
  }
  else
  {
    if ((unsigned long)(millis() - thoigian1) > 200)
    {
      data30A[2] = 100;
      CAN_Transmit_Data(0x30A, data30A, 8);
      thoigian1 = millis();
    }
  }

  // Xử lý nút bấm chuyển trang
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && currentButtonState == LOW)
  {
    delay(100);
    currentPage++;
    if (currentPage > 8)
    {
      currentPage = 1;
    }
    displayPage(currentPage, true);
  }

  if (millis() - lastPageTime > 1500)
  {
    displayPage(currentPage, false);
    lastPageTime = millis();
  }

  lastButtonState = currentButtonState;
}