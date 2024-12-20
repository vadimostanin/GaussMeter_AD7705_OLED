#include "AD770X.h"

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>

// #define LOG_ENABLED

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
#define PIN_BATTERY_INTERNAL_ADC (???)
#define BUTTON_CALIBRATION (D8) // calibration button pin
#define DEVBOARD_DEFINED
#endif
#ifdef DEVBOARD_ESP32_S2
#define PIN_SDA (SDA)//33
#define PIN_SCL (SCL)//35
#define PIN_CS (12)
#define PIN_MOSI (11)
#define PIN_MISO (9)
#define PIN_SPIClock (7)
#define PIN_RST (5)
#define PIN_DRDY (40)
#define PIN_BATTERY_INTERNAL_ADC (1)
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
#define PIN_BATTERY_INTERNAL_ADC (???)
#define BUTTON_CALIBRATION (32) // calibration button pin
#define DEVBOARD_DEFINED
#endif

#ifndef DEVBOARD_DEFINED
#error "No board supported"
#endif

#define AdcValuesWindowsCount (10)
#define AdcValuesWindowsCountVoltageInternal (1)
// define the number of bytes you want to access
#define EEPROM_SIZE (4) //short type

typedef enum
{
  NORMAL,
  CALIBRATION
} ModeType;

typedef struct
{
  uint32_t minEarthAdcIndex = 0xffff;
  uint32_t maxEarthAdcIndex = 0;
} CalibrationInfoType;

typedef struct
{
    uint32_t middleAdcIndex = 0;
    uint32_t absAdcValueForEarthField = 0;
} CalibrationBasedInfoType;

uint32_t adcValue = 0;
uint32_t adcBatteryChannelValue = 0;
uint32_t adcValuesSum = 0;
uint32_t adcValuesAverage = 0;
uint32_t adcValuesWindow[AdcValuesWindowsCount] = {0};
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2);
unsigned long currentMillis = 0;
unsigned long readAdcNormalModeDelay = 20; // period for reading ADC in normal mode
unsigned long readAdcBatteryChannelDelay = 1000; // period for reading ADC for battery voltage
unsigned long LastAdcReadMillis = 0; // when ADC channel 1 was read
unsigned long LastAdcChannel2ReadMillis = 0; // when ADC channel 2 was read
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

#include "soc/rtc.h"
#include "freertos/xtensa_timer.h"
void esp_timer_impl_update_apb_freq(uint32_t apb_ticks_per_us); //private in IDF
static uint32_t calculateApb(struct rtc_cpu_freq_config_s * conf){
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
	return APB_CLK_FREQ;
#else
    if(conf->freq_mhz >= 80){
        return 80 * MHZ;
    }
    return (conf->source_freq_mhz * MHZ) / conf->div;
#endif
}

bool my_setCpuFrequencyMhz(uint32_t cpu_freq_mhz){
    struct rtc_cpu_freq_config_s conf, cconf;
    uint32_t capb, apb;
    //Get XTAL Frequency and calculate min CPU MHz
    rtc_xtal_freq_t xtal = rtc_clk_xtal_freq_get();
#if CONFIG_IDF_TARGET_ESP32
    if(xtal > RTC_XTAL_FREQ_AUTO){
        if(xtal < RTC_XTAL_FREQ_40M) {
            if(cpu_freq_mhz <= xtal && cpu_freq_mhz != xtal && cpu_freq_mhz != (xtal/2)){
                log_e("Bad frequency: %u MHz! Options are: 240, 160, 80, %u and %u MHz", cpu_freq_mhz, xtal, xtal/2);
                return false;
            }
        } else if(cpu_freq_mhz <= xtal && cpu_freq_mhz != xtal && cpu_freq_mhz != (xtal/2) && cpu_freq_mhz != (xtal/4)){
            log_e("Bad frequency: %u MHz! Options are: 240, 160, 80, %u, %u and %u MHz", cpu_freq_mhz, xtal, xtal/2, xtal/4);
            return false;
        }
    }
#endif
    if(cpu_freq_mhz > xtal && cpu_freq_mhz != 240 && cpu_freq_mhz != 160 && cpu_freq_mhz != 80){
        if(xtal >= RTC_XTAL_FREQ_40M){
            log_e("Bad frequency: %u MHz! Options are: 240, 160, 80, %u, %u and %u MHz", cpu_freq_mhz, xtal, xtal/2, xtal/4);
        } else {
            log_e("Bad frequency: %u MHz! Options are: 240, 160, 80, %u and %u MHz", cpu_freq_mhz, xtal, xtal/2);
        }
        return false;
    }
#if CONFIG_IDF_TARGET_ESP32
    //check if cpu supports the frequency
    if(cpu_freq_mhz == 240){
        //Check if ESP32 is rated for a CPU frequency of 160MHz only
        if (REG_GET_BIT(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_CPU_FREQ_RATED) &&
            REG_GET_BIT(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_CPU_FREQ_LOW)) {
            log_e("Can not switch to 240 MHz! Chip CPU frequency rated for 160MHz.");
            cpu_freq_mhz = 160;
        }
    }
#endif
    //Get current CPU clock configuration
    rtc_clk_cpu_freq_get_config(&cconf);
    //return if frequency has not changed
    if(cconf.freq_mhz == cpu_freq_mhz){
        return true;
    }
    //Get configuration for the new CPU frequency
    if(!rtc_clk_cpu_freq_mhz_to_config(cpu_freq_mhz, &conf)){
        log_e("CPU clock could not be set to %u MHz", cpu_freq_mhz);
        return false;
    }
    //Current APB
    capb = calculateApb(&cconf);
    //New APB
    apb = calculateApb(&conf);
    
    //Call peripheral functions before the APB change
    // if(apb_change_callbacks){
    //     triggerApbChangeCallback(APB_BEFORE_CHANGE, capb, apb);
    // }
    //Make the frequency change
    rtc_clk_cpu_freq_set_config_fast(&conf);
    if(capb != apb){
        //Update REF_TICK (uncomment if REF_TICK is different than 1MHz)
        //if(conf.freq_mhz < 80){
        //    ESP_REG(APB_CTRL_XTAL_TICK_CONF_REG) = conf.freq_mhz / (REF_CLK_FREQ / MHZ) - 1;
        // }
        //Update APB Freq REG
        rtc_clk_apb_freq_update(apb);
        //Update esp_timer divisor
        // esp_timer_impl_update_apb_freq(apb / MHZ);
    }
    //Update FreeRTOS Tick Divisor
#if CONFIG_IDF_TARGET_ESP32C3

#elif CONFIG_IDF_TARGET_ESP32S3

#else
    uint32_t fcpu = (conf.freq_mhz >= 80)?(conf.freq_mhz * MHZ):(apb);
    _xt_tick_divisor = fcpu / XT_TICK_PER_SEC;
#endif
    //Call peripheral functions after the APB change
    // if(apb_change_callbacks){
    //     triggerApbChangeCallback(APB_AFTER_CHANGE, capb, apb);
    // }
    log_d("%s: %u / %u = %u Mhz, APB: %u Hz", (conf.source == RTC_CPU_FREQ_SRC_PLL)?"PLL":((conf.source == RTC_CPU_FREQ_SRC_APLL)?"APLL":((conf.source == RTC_CPU_FREQ_SRC_XTAL)?"XTAL":"8M")), conf.source_freq_mhz, conf.div, conf.freq_mhz, apb);
    return true;
}

#include "esp_pm.h"
void setup() {
  uint32_t Freq = 0;
#ifdef LOG_ENABLED
  // put your setup code here, to run once:
  Serial.begin(115200);    // set to ESP8266 bootloader baudrate, so that you can see the boot info
  Serial.flush();  // wait to empty the UART FIFO before changing the CPU Freq.
#endif
  // delay(2000);
  my_setCpuFrequencyMhz(40);
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
  LOG("calibrationInfo.minEarthAdcIndex=");
  LOG_LN(calibrationInfo.minEarthAdcIndex);
  LOG("calibrationInfo.maxEarthAdcIndex=");
  LOG_LN(calibrationInfo.maxEarthAdcIndex);
  convertCalibationInfo();
  LOG("calibrationBasedInfo.middleAdcIndex=");
  LOG_LN(calibrationBasedInfo.middleAdcIndex);
  LOG_LN();
  
  adcBatteryChannelValue = analogRead(PIN_BATTERY_INTERNAL_ADC);
  LOG("adcBatteryChannelValue = ");
  LOG_LN(adcBatteryChannelValue);
  ad7705.selectChannel(AD770X::CHN_AIN1);
  Freq = getCpuFrequencyMhz();
  LOG("CPU Freq = ");
  LOG(Freq);
  LOG_LN(" MHz");
  Freq = getXtalFrequencyMhz();
  LOG("XTAL Freq = ");
  LOG(Freq);
  LOG_LN(" MHz");
  Freq = getApbFrequency();
  LOG("APB Freq = ");
  LOG(Freq);
  LOG_LN(" Hz");

  analogReadResolution(13);
  analogSetAttenuation(ADC_11db);

  LOG_LN("--Start--");
  LOG_LN(__LINE__);
}

void adcWindow_push(int value)
{
  static uint32_t window_position = 0;
  static uint32_t adcValuesCount = 0;
  static unsigned char isAdcValuesNotFull = true;
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

uint32_t adcWindow_get_avg()
{
  return adcValuesAverage;
}

// double getZeroCalibrationLinearAproximation(double value)
// {
//   return value * 2.2057 - 0.00016;
// }

// double getZeroCalibrationQuadraticAproximation(double value)
// {
//   return (- 0.004685 * value * value) + 2.214885 * value;// + 0.000288;
// }

void drawNormalMode(const uint32_t adc)
{
  static uint32_t i = 0;
  const uint32_t magn_B_index_zero = calibrationBasedInfo.middleAdcIndex;
  const int32_t magn_B_index = adc - magn_B_index_zero;
  const uint32_t magn_B_index_abs = magn_B_index < 0 ? magn_B_index * -1 : magn_B_index;
  LOG("magn_B_index= ");
  LOG_LN(magn_B_index);
  LOG("calibrationBasedInfo.absAdcValueForEarthField= ");
  LOG_LN(calibrationBasedInfo.absAdcValueForEarthField);
  // const int magn_B_earth_max_index = calibrationBasedInfo.absAdcValueForEarthField;
  const float magn_B_earth_ratio = ((float)magn_B_index) / (float)calibrationBasedInfo.absAdcValueForEarthField;
  const int32_t magn_B_u_tesla = (int32_t)(magn_B_earth_ratio * 50.0); //50uT = 1.0 ratio
  const uint32_t magn_B_u_tesla_abs = abs(magn_B_u_tesla);
  const char sign = magn_B_u_tesla > 0 ? ' ' : '-';
  const float batteryVolt = adcBatteryChannelValue * 2 * 2.55 / 8192.0;

  u8g2.firstPage();
  // for(int i = 0 ; i <= 2 ; ++i)//workaround to not redraw eight times same content, but 8-3=5 times
  // {
  //   // u8g2.nextPage();
  // }
  do {
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.setCursor(0, 10);
    u8g2.printf("#%03d: ADC=%05d", i, adc);
    u8g2.setCursor(0, 25);
    //      u8g2.printf("Volts=%f", getZeroCalibrationQuadraticAproximation(v1_d * 1.0 / 65536.0 * 5.0));//For MODE_ZERO_SCALE_CAL
    u8g2.printf("Battery=%.2f", batteryVolt);
    u8g2.setFont(u8g2_font_battery19_tn);
    u8g2.setCursor(100, 30);
    if(uint32_t(batteryVolt) > 1.0)
    {
      u8g2.print(int(abs(batteryVolt - 3.2) * 10 * 1.25) / 2);
    }
    else
    {
      u8g2.print(6);
    }
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.setCursor(0, 40);
    if(magn_B_u_tesla_abs < 1000)
    {
      u8g2.printf("B: %c%03u uT", sign, magn_B_u_tesla_abs);
    }
    else
    {
      u8g2.printf("B: %c%03d.%03d mT", sign, magn_B_u_tesla_abs / 1000, magn_B_u_tesla_abs % 1000);
    }
  } while ( u8g2.nextPage() );
  ++i;
}

void drawCalibrationMode()
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 7);
    u8g2.printf("Calibration...");

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 16);
    u8g2.printf("TURN SENSOR AROUND");

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 26);
    u8g2.printf("ADC: %05hu", adcValue);
    u8g2.setCursor(0, 35);
    u8g2.printf("MIN: %05hu", calibrationInfo.minEarthAdcIndex);
    u8g2.setCursor(0, 44);
    u8g2.printf("MAX: %05hu", calibrationInfo.maxEarthAdcIndex);
    {
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(0, 50);
      u8g2.printf("github.com/vadimostanin");
      u8g2.setCursor(0, 57);
      u8g2.printf("/GaussMeter_AD7705_OLED");
    }
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

  if ((unsigned long)(currentMillis - LastAdcChannel2ReadMillis) >= readAdcBatteryChannelDelay)
  {
    adcBatteryChannelValue = analogRead(PIN_BATTERY_INTERNAL_ADC);
    LOG(", Battery Voltage=");
    LOG_LN(adcBatteryChannelValue);

    LastAdcChannel2ReadMillis = currentMillis;
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
