#include "AD770X.h"

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>

#define LOG_ENABLED

#ifdef LOG_ENABLED
#define LOG(x) Serial.print(x)
#define LOG_LN(x) Serial.println(x)
#else
#define LOG(x)
#define LOG_LN(x)
#endif

//#define DEVBOARD_ESP8266_WEMOS_D1
#define DEVBOARD_ESP32_S2
//#define DEVBOARD_CH552

#ifdef DEVBOARD_ESP8266_WEMOS_D1
#define PIN_SDA (D4)
#define PIN_SCL (D3)
#define PIN_CS (D10)
#define PIN_MOSI (D11)
#define PIN_MISO (D12)
#define PIN_SPIClock (D13)
#define PIN_RST (D9)
#define PIN_DRDY (D2)
#define BUTTON_CALIBRATION (D8) // calibration button pin
#define DEVBOARD_DEFINED
#endif
#ifdef DEVBOARD_ESP32_S2
#define PIN_SDA (8)
#define PIN_SCL (9)
#define PIN_CS (12)
#define PIN_MOSI (11)
#define PIN_MISO (9)
#define PIN_SPIClock (7)
#define PIN_RST (5)
#define PIN_DRDY (40)
#define BUTTON_CALIBRATION (3) // calibration button pin
#define DEVBOARD_DEFINED
#endif
#ifdef DEVBOARD_CH552
#define PIN_SDA (30)
#define PIN_SCL (31)
#define PIN_CS (11)
#define PIN_MOSI (10)
#define PIN_MISO (17)
#define PIN_SPIClock (16)
#define PIN_RST (15)
#define PIN_DRDY (14)
#define BUTTON_CALIBRATION (32) // calibration button pin
#define DEVBOARD_DEFINED
#endif

#ifndef DEVBOARD_DEFINED
#error "No board supported"
#endif

#define AdcValuesWindowsCount (10)
// define the number of bytes you want to access
#define EEPROM_SIZE 2 //short type

typedef enum
{
  NORMAL,
  CALIBRATION
} ModeType;

typedef struct
{
  unsigned short minEarthAdcIndex = 0xffff;
  unsigned short maxEarthAdcIndex = 0;
} CalibrationInfoType;

typedef struct
{
    unsigned short middleAdcIndex = 0;
    unsigned short absAdcValueForEarthField = 0;
} CalibrationBasedInfoType;

ushort adcValue = 0;
//int adcValueAverage = 0;
//byte adcValueByte1 = 0;
//byte adcValueByte2 = 0;
uint adcValuesSum = 0;
uint adcValuesAverage = 0;
uint adcValuesWindow[AdcValuesWindowsCount] = {0};
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2);
//bool adcDataAsked = false;
unsigned long currentMillis = 0;
unsigned long readAdcNormalModeDelay = 50; // period for reading ADC in normal mode
unsigned long LastAdcReadMillis = 0; // when ADC was read
unsigned long drawDelay = 300; // wait to draw
unsigned long LastDrawMillis = 0; // when last draw happened
unsigned long buttonCalibrationDelay = 50;
bool          buttonCalibrationPushed = false;
unsigned long buttonCalibrationPushedMillis = 0; // when button for calibration was released
unsigned long readAdcCalModeDelay = 100; // wait 
unsigned long LastAdcReadCalModeMillis = 0; // period for reading ADC in calibration mode
ModeType workingMode = NORMAL;
CalibrationInfoType calibrationInfo;
CalibrationBasedInfoType calibrationBasedInfo;


AD770X ad7705(5.0, PIN_CS, PIN_MOSI, PIN_MISO, PIN_SPIClock, PIN_RST, PIN_DRDY);

void u8g2_prepare()
{
  u8g2.begin();
}

void convertCalibationInfo()
{
  calibrationBasedInfo.middleAdcIndex = calibrationInfo.minEarthAdcIndex + (calibrationInfo.maxEarthAdcIndex - calibrationInfo.minEarthAdcIndex) / 2;
  calibrationBasedInfo.absAdcValueForEarthField = (calibrationInfo.maxEarthAdcIndex - calibrationBasedInfo.middleAdcIndex);
  calibrationBasedInfo.absAdcValueForEarthField = calibrationBasedInfo.absAdcValueForEarthField < 1 ? 1 : calibrationBasedInfo.absAdcValueForEarthField;
}

void setup() {
  #ifdef LOG_ENABLED
  // put your setup code here, to run once:
  Serial.begin(115200);    // set to ESP8266 bootloader baudrate, so that you can see the boot info
#endif
  Wire.begin();//PIN_SDA, PIN_SCL

  u8g2_prepare();
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pinMode(BUTTON_CALIBRATION, INPUT_PULLUP);

  ad7705.resetHard();
  ad7705.reset();

  ad7705.init(AD770X::CHN_AIN1, AD770X::CLKDIV_0, AD770X::CLK_1MHz, AD770X::UNIPOLAR, AD770X::GAIN_1, AD770X::UPDATE_RATE_20);
  LOG_LN("--Init Done--");

  currentMillis = millis();
  LastAdcReadMillis = currentMillis;
  LastDrawMillis = LastAdcReadMillis;

  // read the 2 bytes of middleAdcIndex from flash memory
  calibrationInfo.minEarthAdcIndex = EEPROM.read(0) | (EEPROM.read(1) << 8);
  calibrationInfo.maxEarthAdcIndex = EEPROM.read(2) | (EEPROM.read(3) << 8);
  convertCalibationInfo();
  LOG("calibrationBasedInfo.middleAdcIndex=");
  LOG_LN(calibrationBasedInfo.middleAdcIndex);
  LOG_LN();
  LOG_LN("--Start--");
}
/*
void setup() {
    Serial.begin(115200);    // set to ESP8266 bootloader baudrate, so that you can see the boot info
    pinMode(PIN_DRDY, INPUT);

    u8g2_prepare();

    SPI.transfer(0x20); // Active Channel is Ain1(+)/Ain1(-), next operation as write to the clock register 
    delay(1);
    SPI.transfer(0x04); // ????? master clock enabled, 1 MHz Clock, set output rate to 25Hz
    delay(1);
    SPI.transfer(0x10); // Active Channel is Ain1(+)/Ain1(-), next operation as write to the setup register 
    delay(1);
//    SPI.transfer(0x44); // gain = 1, unipolar mode, buffer off, clear FSYNC and perform a Self Calibration 
    SPI.transfer(0x40); // gain = 1, unipolar mode, buffer off, clear FSYNC and perform a Self Calibration
//    SPI.transfer(0x7c); // gain = 128, unipolar mode, buffer off, clear FSYNC and perform a Self Calibration 
    delay(1);

    Serial.println("--Start waiting PIN_DRDY--");
    while(digitalRead(PIN_DRDY));
    Serial.println("--Init Done--");
    
}
*/
void adcWindow_push(int value)
{
  static int window_position = 0;
  static int adcValuesCount = 0;
  static char isAdcValuesNotFull = true;
  if(true == isAdcValuesNotFull)
  {
    isAdcValuesNotFull = ((adcValuesCount + 1) <= AdcValuesWindowsCount);
  }
  switch(isAdcValuesNotFull)
  {
    case true:
    // LOG("array not full");
      ++adcValuesCount;
      adcValuesSum += value;
      adcValuesAverage = adcValuesSum / adcValuesCount;
      adcValuesWindow[adcValuesCount - 1] = value;
    break;
    case false:
    // LOG("array full");
      adcValuesSum -= adcValuesWindow[window_position];
      adcValuesSum += value;
      adcValuesAverage = adcValuesSum / AdcValuesWindowsCount;
      adcValuesWindow[window_position] = value;
      ++window_position;
      window_position = window_position % AdcValuesWindowsCount;
    break;
  }
}

int adcWindow_get_avg()
{
  return adcValuesAverage;
}

double getZeroCalibrationLinearAproximation(double value)
{
  return value * 2.2057 - 0.00016;
}

double getZeroCalibrationQuadraticAproximation(double value)
{
  return (- 0.004685 * value * value) + 2.214885 * value;// + 0.000288;
}

void drawNormalMode(const int adc)
{
  static uint i = 0;
  const uint magn_B_index_zero = calibrationBasedInfo.middleAdcIndex;
  const int magn_B_index = adc - magn_B_index_zero;
  const int magn_B_index_abs = magn_B_index < 0 ? magn_B_index * -1 : magn_B_index;
  LOG("magn_B_index= ");
  LOG_LN(magn_B_index);
  LOG("calibrationBasedInfo.absAdcValueForEarthField= ");
  LOG_LN(calibrationBasedInfo.absAdcValueForEarthField);
  // const int magn_B_earth_max_index = calibrationBasedInfo.absAdcValueForEarthField;
  const float magn_B_earth_ratio = ((float)magn_B_index) / (float)calibrationBasedInfo.absAdcValueForEarthField;
  const int magn_B_u_tesla = (int)(magn_B_earth_ratio * 50.0); //50uT = 1.0 ratio

  u8g2.firstPage();
  for(int i = 0 ; i <= 2 ; ++i)//workaround to not redraw eight times same content, but 8-3=5 times
  {
    // u8g2.nextPage();
  }
  do {
    u8g2.setFont(u8g2_font_7x14_tf);
    //u8g2.drawStr(0, 24, "Hello World!");
    u8g2.setCursor(0, 10);
    u8g2.printf("#%d: ADC=%d", i, adc);
    u8g2.setCursor(0, 25);
    //      u8g2.printf("Volts=%f", getZeroCalibrationQuadraticAproximation(v1_d * 1.0 / 65536.0 * 5.0));//For MODE_ZERO_SCALE_CAL
    u8g2.printf("Volts=%f", adc * 1.0 / 65536.0 * 5.0 - 0.0);//2.5//For MODE_SELF_CAL
    u8g2.setCursor(0, 40);
    if(abs(magn_B_u_tesla) < 1000)//
    {
      u8g2.printf("B: %d uT", (int)magn_B_u_tesla);
    }
    else
    {
      u8g2.printf("B: %d.%d%d%d mT", magn_B_u_tesla / 1000, (magn_B_index_abs % 1000) / 100, (magn_B_index_abs % 100) / 10, (magn_B_index_abs % 10));
    }
        u8g2.setCursor(0, 50);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.printf("github.com/vadimostanin");
    u8g2.setCursor(0, 57);
    u8g2.printf("/GaussMeter_AD7705_OLED");
  } while ( u8g2.nextPage() );
//  u8g2.sendBuffer();
  ++i;
}

void drawCalibrationMode()
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14_tf);
    //u8g2.drawStr(0, 24, "Hello World!");
    u8g2.setCursor(0, 10);
    u8g2.printf("Calibration...");

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 25);
    u8g2.printf("TURN THE SENSOR");
    u8g2.setCursor(0, 40);
    u8g2.printf("RANDOMLY ORDER");
    u8g2.setCursor(0, 55);
    u8g2.printf("AROUND ALL AXIS");
  } while ( u8g2.nextPage() );
}

void ModeNormalHandler()
{
  if ((unsigned long)(currentMillis - LastAdcReadMillis) >= readAdcNormalModeDelay)
  {
    while (!ad7705.dataReady(AD770X::CHN_AIN1)) {
        ;
    }
//    adcValue = ad7705.readADResult(AD770X::CHN_AIN1, 2.5);
    adcValue = ad7705.readADResultRaw(AD770X::CHN_AIN1);
    // LOG("readadcValue=");
    // LOG_LN(adcValue);
    adcWindow_push(adcValue);
    adcValue = adcWindow_get_avg();
    // LOG("adcValue=");
    // LOG_LN(adcValue);
    LastAdcReadMillis = currentMillis;
  }

  if ((unsigned long)(currentMillis - LastDrawMillis) >= drawDelay)
  {
    drawNormalMode(adcValue);
    LastDrawMillis = currentMillis;
  }

  // check the button for calibration
  if ((unsigned long)(currentMillis - buttonCalibrationPushedMillis) >= buttonCalibrationDelay )
  {
    if(digitalRead(BUTTON_CALIBRATION) == LOW) {
      LOG_LN("Button is pressed");
      // update the time when button was pushed
      buttonCalibrationPushed = true;
    }
    else if(buttonCalibrationPushed && digitalRead(BUTTON_CALIBRATION) == HIGH)
    {
      LOG_LN("Button is released");
      // update the time when button was released
      buttonCalibrationPushed = false;
      workingMode = CALIBRATION;
      calibrationInfo = CalibrationInfoType();
      calibrationBasedInfo = CalibrationBasedInfoType();
    }
    buttonCalibrationPushedMillis = currentMillis;
  }
}

void ModeCalibrationHandler()
{
  if ((unsigned long)(currentMillis - LastAdcReadCalModeMillis) >= readAdcCalModeDelay)
  {
    while (!ad7705.dataReady(AD770X::CHN_AIN1));
    adcValue = ad7705.readADResultRaw(AD770X::CHN_AIN1);

    if(calibrationInfo.minEarthAdcIndex > adcValue)
    {
      calibrationInfo.minEarthAdcIndex = adcValue;
    }
    if(calibrationInfo.maxEarthAdcIndex < adcValue)
    {
      calibrationInfo.maxEarthAdcIndex = adcValue;
    }
    
    LastAdcReadCalModeMillis = currentMillis;
  }
  if ((unsigned long)(currentMillis - LastDrawMillis) >= drawDelay)
  {
    drawCalibrationMode();
    LastDrawMillis = currentMillis;
  }
  // check the button for calibration
  if ((unsigned long)(currentMillis - buttonCalibrationPushedMillis) >= buttonCalibrationDelay )
  {
    if(digitalRead(BUTTON_CALIBRATION) == LOW) {
      LOG_LN("Button is pressed");
      // update the time when button was pushed
      buttonCalibrationPushed = true;
    }
    else if(buttonCalibrationPushed && digitalRead(BUTTON_CALIBRATION) == HIGH)
    {
      LOG_LN("Button is released");
      // update the time when button was released
      buttonCalibrationPushed = false;
      workingMode = NORMAL;
      convertCalibationInfo();
      LOG("calibrationBasedInfo.middleAdcIndex=");
      LOG_LN(calibrationBasedInfo.middleAdcIndex);
      // save the LED state in flash memory
      EEPROM.write(0, (calibrationInfo.minEarthAdcIndex & 0xff));//lower byte
      EEPROM.write(1, (calibrationInfo.minEarthAdcIndex >> 8));//higher byte
      EEPROM.write(2, (calibrationInfo.maxEarthAdcIndex & 0xff));//lower byte
      EEPROM.write(3, (calibrationInfo.maxEarthAdcIndex >> 8));//higher byte
      EEPROM.commit();
    }
    buttonCalibrationPushedMillis = currentMillis;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();

  switch(workingMode)
  {
    case NORMAL:
      ModeNormalHandler();
    break;
    case CALIBRATION:
      ModeCalibrationHandler();
    break;
  }  
}

/*
void loop() {
  
    static int i = 0;

//    Serial.print("#");
//    Serial.print(i);
//    Serial.print(": ");
//    
//    Serial.print("CH1: ");

    if(false == adcDataAsked)
    {
      SPI.transfer(0x38); // next operation - read from the data register
      adcDataAsked = true;
    }
//    while(digitalRead(PIN_DRDY)); // wait for /PIN_DRDY to go low  
    if(true == adcDataAsked && LOW == digitalRead(PIN_DRDY))
    {
      adcDataAsked = false;;
      adcValueByte1 = SPI.transfer(0x0);
      delayMicroseconds(10);
      adcValueByte2 = SPI.transfer(0x0);
      adcValue = adcValueByte1 << 8 | adcValueByte2;
  //    Serial.print(adcValue, 5);
  //    Serial.println();
  //    Serial.print(", ");
      adcWindow_push( adcValue );
      adcValueAverage = adcWindow_get_avg();
  
  //    Serial.println(adcValueAverage, 5);
  
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_7x14_tf);
        //u8g2.drawStr(0,24, "Hello World!");
        u8g2.setCursor(0,24);
        u8g2.printf("#%d: %d, %d", i, adcValue, adcValueAverage);
    //u8g2.drawBox(0, 0, 128, 64 );
      } while ( u8g2.nextPage() );
      delay(100);
      ++i;
    }

}
*/
