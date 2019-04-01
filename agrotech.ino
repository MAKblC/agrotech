#include <Wire.h>
#include <I2C_graphical_LCD_display.h>
#include <VL53L0X.h>
#include <BH1750FVI.h>
#include <VEML6075.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <SPI.h>

int var = 0;

#include <Adafruit_MCP4725.h>                           // динамик
Adafruit_MCP4725 buzzer;
int ton;
int vol1 = 1000; // Уровень громкости = vol1-vol2
int vol2 = 900;  //

#include "TLC59108.h"
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 0); // Без перемычек добавляется 3 бита адреса
TLC59108 leds2(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса

#define SOIL_MOISTURE    32
#define SOIL_TEMPERATURE 33
const float air_value1 = 1522.0;
const float water_value1 = 688.0;
const float moisture_01 = 0.0;
const float moisture_1001 = 100.0;

char ssid[] = "IOTIK";                            // Логин Wi-Fi  // Wi-Fi login
char pass[] = "Terminator812";                    // Пароль от Wi-Fi // Wi-Fi password
char auth[] = "fb7eac83c7d044f18d49d9434efd5167"; // Токен // Authorization token
IPAddress blynk_ip(139, 59, 206, 133);            // конфигурация блинка // Blynk configuration


VL53L0X lox;
BH1750FVI LightSensor_1;
Adafruit_BME280 bme280;
I2C_graphical_LCD_display lcd;

#define HIGH_ACCURACY

#define UPDATE_TIMER 500
#define UPDATE_sound 25
BlynkTimer timer_update;


#include "PCA9536.h"
PCA9536 pca9536;

void setup ()
{
  
  
  timer_update.setInterval(UPDATE_TIMER, readSendData);
  timer_update.setInterval(UPDATE_TIMER, sound);
  Serial.begin(115200);
  // Инициализация датчика
  Wire.begin();
  Blynk.begin(auth, ssid, pass, blynk_ip, 8442);
  lox.init();
  lox.setTimeout(500);
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  lox.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  lox.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  lox.setMeasurementTimingBudget(200000);
#endif

  buzzer.begin(0x61); // С перемычкой адрес будет 0x60
  buzzer.setVoltage(0, false);   // выключение звука

  LightSensor_1.begin();
  LightSensor_1.setMode(Continuously_High_Resolution_Mode);

  bool bme_status = bme280.begin();
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");

pca9536.reset();
 pca9536.setMode(IO_OUTPUT);
 delay(1000);
 pca9536.setState(IO_LOW);
 // pca9536.setState(IO0, IO_LOW);
 // pca9536.setState(IO1, IO_LOW);
 // pca9536.setState(IO2, IO_LOW);
//  pca9536.setState(IO3, IO_LOW);

  lcd.begin();
  lcd.clear (0, 0, 128, 64, 0x00);
  

  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  leds2.init(HW_RESET_PIN);
  leds2.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);

}

void loop ()
{
  Blynk.run();
  timer_update.run();
}
void sound()
{
  float dist = lox.readRangeSingleMillimeters();
  Serial.println(dist);
  while (dist > 300) {
    note(14, 400); note(2, 100); note(9, 400); note(7, 500);
    buzzer.setVoltage(0, false);   // выключение звука
     lcd.clear (0, 0, 128, 64, 0x00);
    lcd.gotoxy (10, 53);
    lcd.string ("the door is open" , false);
    dist = lox.readRangeSingleMillimeters();
  }
}
void readSendData() {
  float adc01 = analogRead(SOIL_MOISTURE);
  float h11 = map(adc01, air_value1, water_value1, moisture_01, moisture_1001);
  Blynk.virtualWrite(V0, h11); delay(2);
Serial.println(h11);
  float l = LightSensor_1.getAmbientLight();
  Blynk.virtualWrite(V1, l); delay(2);

  float t = bme280.readTemperature();
  float h = bme280.readHumidity();
  float p = bme280.readPressure() / 100.0F;
  Blynk.virtualWrite(V2, t); delay(2);
  Blynk.virtualWrite(V3, h); delay(2);
  Blynk.virtualWrite(V4, p); delay(2);

  lcd.gotoxy (40, 53);
  lcd.string ("Temperature, C" , false);
  char buf[8];
  sprintf(buf, "%d", (int)t);
  lcd.clear (10, 53, 38, 60, 0x00);
  lcd.gotoxy (10, 53);
  lcd.string (buf, false);

  lcd.gotoxy (40, 39);
  lcd.string ("Humidity, %" , false);
  char buf1[8];
  sprintf(buf1, "%d", (int)h);
  lcd.clear (10, 39, 32, 46, 0x00);
  lcd.gotoxy (10, 39);
  lcd.string (buf1, false);

  lcd.gotoxy (40, 22);
  lcd.string ("Pressure, hPa" , false);
  char buf2[8];
  sprintf(buf2, "%d", (int)p);
  lcd.clear (10, 22, 32, 32, 0x00);
  lcd.gotoxy (10, 22);
  lcd.string (buf2, false);
}
BLYNK_WRITE(V5)
{
  int buttonstate1 = param.asInt ();
  if (buttonstate1 == 1) {
    pca9536.setState(IO2, IO_HIGH);
    leds.setBrightness(6, 0xff);
    leds.setBrightness(3, 0xff);
    leds.setBrightness(0, 0xff);
    // включить если нажата кнопка "Насос" // turn on the pump if button = 1
  }
  else    {
    pca9536.setState(IO2, IO_LOW);
    leds.setBrightness(6, 0);
    leds.setBrightness(3, 0);
    leds.setBrightness(0, 0);
  }
}
BLYNK_WRITE(V6)
{
  int buttonstate1 = param.asInt ();
  if (buttonstate1 == 1) {
    pca9536.setState(IO3, IO_HIGH);
    leds2.setBrightness(4, 0xff);
    leds2.setBrightness(1, 0xff);
    leds2.setBrightness(5, 0xff);
    // включить если нажата кнопка "Насос" // turn on the pump if button = 1
  }
  else    {
    pca9536.setState(IO3, IO_LOW);
    leds2.setBrightness(4, 0);
    leds2.setBrightness(1, 0);
    leds2.setBrightness(5, 0);
  }
}
int note( int type, int duration) {   // нота (какая нота, длительность)
  switch (type) {
    case 1:   ton = 1000; break;
    case 2:   ton = 860;  break;
    case 3:   ton = 800;  break;
    case 4:   ton = 700;  break;
    case 5:   ton = 600;  break;
    case 6:   ton = 525;  break;
    case 7:   ton = 450;  break;
    case 8:   ton = 380;  break;
    case 9:   ton = 315;  break;
    case 10:  ton = 250;  break;
    case 11:  ton = 190;  break;
    case 12:  ton = 130;  break;
    case 13:  ton = 80;   break;
    case 14:  ton = 30;   break;
    case 15:  ton = 1;   break;
  }
  delay(10);
  for (int i = 0; i < duration; i++) {
    buzzer.setVoltage(vol1, false);
    buzzer.setVoltage(vol2, false);
    delayMicroseconds(ton);
  }
}
