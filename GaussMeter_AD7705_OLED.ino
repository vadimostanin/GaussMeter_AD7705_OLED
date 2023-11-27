#include "AD770X.h"

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
//#include <Wire.h>

#define LOG_ENABLED

#ifdef LOG_ENABLED
#define LOG(x) Serial.print(x)
#define LOG_LN(x) Serial.println(x)
#else
#define LOG(x)
#define LOG_LN(x)
#endif

typedef enum
{
  NORMAL,
  CALIBRATION
} ModeType;

typedef struct
{
  uint minEarthAdcIndex;
  uint maxEarthAdcIndex;
  uint middleAdcIndex;
} CalInfoType;

uint adcValue = 0;
//int adcValueAverage = 0;
//byte adcValueByte1 = 0;
//byte adcValueByte2 = 0;
uint adcValuesSum = 0;
uint adcValuesAverage = 0;
#define AdcValuesWindowsCount (10)
uint adcValuesWindow[AdcValuesWindowsCount] = {0};
#define DRDY (D2)
#define BUTTON_CALIBRATION (D8) // calibration button pin
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2);
//bool adcDataAsked = false;
unsigned long currentMillis = 0;
unsigned long readAdcNormalModeDelay = 100; // period for reading ADC in normal mode
unsigned long LastAdcReadMillis = 0; // when ADC was read
unsigned long drawDelay = 100; // wait to draw
unsigned long LastDrawMillis = 0; // when last draw happened
unsigned long buttonCalibrationPushedMillis = 0; // when button for calibration was released
unsigned long readAdcCalModeDelay = 100; // wait 
unsigned long LastAdcReadCalModeMillis = 0; // period for reading ADC in calibration mode
ModeType workingMode = NORMAL;
CalInfoType calibrationInfo = {0};


AD770X ad7705(5.0, D10, D11, D12, D13, D9, DRDY);

void u8g2_prepare()
{
  u8g2.begin();
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);    // set to ESP8266 bootloader baudrate, so that you can see the boot info
    u8g2_prepare();
    pinMode(BUTTON_CALIBRATION, INPUT_PULLUP);

    ad7705.resetHard();
    ad7705.reset();
    
    LOG_LN();
    LOG_LN("--Start--");
    
    ad7705.init(AD770X::CHN_AIN1, AD770X::CLKDIV_0, AD770X::CLK_1MHz, AD770X::UNIPOLAR, AD770X::GAIN_1, AD770X::UPDATE_RATE_20);
//    TM7705.init(AD770X::CHN_AIN2, AD770X::CLK_DIV_1, AD770X::BIPOLAR, AD770X::GAIN_1, AD770X::UPDATE_RATE_50);
    LOG_LN("--Init Done--");

    currentMillis = millis();
    LastAdcReadMillis = currentMillis;
    LastDrawMillis = LastAdcReadMillis;
}
/*
void setup() {
    Serial.begin(115200);    // set to ESP8266 bootloader baudrate, so that you can see the boot info
    pinMode(DRDY, INPUT);

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

    Serial.println("--Start waiting DRDY--");
    while(digitalRead(DRDY));
    Serial.println("--Init Done--");
    
}
*/
void adcWindow_push(int value)
{
  static int window_position = 0;
  static int adcValuesCount = 0;
  static char isAdcValuesNotFull = 0;
  if(1 == isAdcValuesNotFull)
  {
    isAdcValuesNotFull = ((adcValuesCount + 1) <= AdcValuesWindowsCount);
  }
  switch(isAdcValuesNotFull)
  {
    case true:
      ++adcValuesCount;
      adcValuesSum += value;
      adcValuesAverage = adcValuesSum / adcValuesCount;
      adcValuesWindow[adcValuesCount - 1] = value;
    break;
    case false:
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
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x14_tf);
    //u8g2.drawStr(0, 24, "Hello World!");
    u8g2.setCursor(0, 10);
    u8g2.printf("#%d: ADC=%d", i, adc);
    u8g2.setCursor(0, 25);
    //      u8g2.printf("Volts=%f", getZeroCalibrationQuadraticAproximation(v1_d * 1.0 / 65536.0 * 5.0));//For MODE_ZERO_SCALE_CAL
    u8g2.printf("Volts=%f", adc * 1.0 / 65536.0 * 5.0 - 0.0);//2.5//For MODE_SELF_CAL
    u8g2.setCursor(0, 40);
    const uint magn_B_index_zero = calibrationInfo.middleAdcIndex;
    LOG("magn_B_index_zero= ");
    LOG_LN(magn_B_index_zero);
    const int magn_B_index = adc - magn_B_index_zero;
//    LOG("magn_B_index= ");
//    LOG_LN(magn_B_index);
    const int magn_B_earth_max_index = 10;
    const float magn_B_earth_ratio = ((float)magn_B_index) / (float)magn_B_earth_max_index;
    const float magn_B_tesla = magn_B_earth_ratio * 50.0; //50uT = 1.0 ratio
//    LOG("magn_B_tesla= ");
//    LOG_LN(magn_B_tesla);
    u8g2.printf("B: %d uT", (int)magn_B_tesla);
    //u8g2.drawBox(0, 0, 128, 64 );
  } while ( u8g2.nextPage() );
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

    u8g2.setCursor(0, 30);
    u8g2.printf("MOVE THE SENSOR RANDOMLY ORDER AROUND ALL AXIS");
  } while ( u8g2.nextPage() );
}

void ModeNormalHandler()
{
  if ((unsigned long)(currentMillis - LastAdcReadMillis) >= readAdcNormalModeDelay)
  {
    LOG("CH1: ");
    while (!ad7705.dataReady(AD770X::CHN_AIN1)) {
        ;
    }
//    adcValue = ad7705.readADResult(AD770X::CHN_AIN1, 2.5);
    adcValue = ad7705.readADResultRaw(AD770X::CHN_AIN1);
    LOG_LN(adcValue);
    adcWindow_push(adcValue);
    adcValue = adcWindow_get_avg();
    LastAdcReadMillis = currentMillis;
  }

  if ((unsigned long)(currentMillis - LastDrawMillis) >= drawDelay)
  {
    drawNormalMode(adcValue);
    LastDrawMillis = currentMillis;
  }

  // check the button for calibration
  if (digitalRead(BUTTON_CALIBRATION) == LOW) {
    // update the time when button was pushed
    buttonCalibrationPushedMillis = currentMillis;
    workingMode = CALIBRATION;
    memset(&calibrationInfo, 0, sizeof(calibrationInfo));
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
  // check the button for switching to normal mode
  if (digitalRead(BUTTON_CALIBRATION) == LOW) {
    // update the time when button was pushed
    buttonCalibrationPushedMillis = currentMillis;
    workingMode = NORMAL;
    calibrationInfo.middleAdcIndex = (calibrationInfo.minEarthAdcIndex + calibrationInfo.maxEarthAdcIndex) / 2;
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
//    while(digitalRead(DRDY)); // wait for /DRDY to go low  
    if(true == adcDataAsked && LOW == digitalRead(DRDY))
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
