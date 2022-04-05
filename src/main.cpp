#include "state_machine.h"
#include "display_actions.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include <SPI.h>
#include <Wire.h>
// Arduino таймер CTC прерывание
// avr-libc library includes
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// подключение библиотек для работы с DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
// #define DEBUG_SERIAL
//  enoder
const short ENCODER_CLK = 2; // do not change
const short ENCODER_DT = 3; // do not change
const short ENCODER_SW_BUTTON = 4; // encoder_sw
//--encoder
const short ADDITIONAL_BUTTON = 5; // encoder_sw
const short RELAY_PIN = 7;
const short PIEZO_PIN = 6;
const short ONE_WIRE_BUS = 11; // вывод, к которому подключён DS18B20
const short TEMPERATURE_PRECISION = 12; // точность измерений (9 ... 12)
volatile int counter = 0;   // счётчик
const int defaultTimeoutSec = 150;
const unsigned long pressTimeoutMS = 500;
//  eeprom
const short EEPROM_INIT_ADDR = 1023;
const char EEPROM_INIT_VALUE = 10;
const short EEPROM_TIMEOUT_ADDR = 0;
//--eeprom


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DisplayActions da(&display);
StateMachine stm(&da, 30, defaultTimeoutSec, RELAY_PIN, PIEZO_PIN);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);
DeviceAddress Thermometer;


void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


volatile byte lastEnc = 0b11, lastEncSw = 0b1, lastAddSw = 0b1;
volatile bool encMiddleAngle = false;
volatile unsigned long lastPressTimeAddSw = 0;
volatile unsigned long lastPressTimeEncSw = 0;
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  cli();
  byte encState = (PIND & 0b111100) >> 2;  // D2 + D3 + D4 + D5
  byte swState = (encState & 0b100) >> 2;
  byte addSwState = (encState & 0b1000) >> 3;
  encState = encState & 0b11; 
  if (encState != lastEnc){
    if (!encState) encMiddleAngle = true;
    if (encMiddleAngle && encState == 0b01) {
      if (stm.checkAction()) stm.incLedTimerSec();
      encMiddleAngle = false;
      }
    else if (encMiddleAngle && encState == 0b10) {
      if (stm.checkAction()) stm.decLightTimerSec();
      encMiddleAngle = false;
    }
    lastEnc = encState;
  }else if (swState != lastEncSw){
    // action to letting go
    if (lastEncSw == 0b0 && swState == 0b1){
      // encoder button (start/stop)
      if (millis()-lastPressTimeEncSw >= pressTimeoutMS) {
        if (!stm.isLightOn()){
          stm.resetLightTimerSec();
        }
      }else if(stm.checkAction()) stm.toggleTimer(true);
    }else if (lastEncSw == 0b1 && swState == 0b0){
      lastPressTimeEncSw = millis();
    }
    lastEncSw = swState;
  }else if (addSwState != lastAddSw){
      // additional button (start/pause)
      if (lastAddSw == 0b0 && addSwState == 0b1){
        if (millis()-lastPressTimeAddSw >= pressTimeoutMS) {
          if (!stm.isLightOn()){
            EEPROM.put(EEPROM_TIMEOUT_ADDR, stm.getLightTimerSecLeft());
          }
        }
        else if(stm.checkAction()) stm.toggleTimer(false);
      }else if(lastAddSw == 0b1 && addSwState == 0b0){ // start press
        lastPressTimeAddSw = millis();
      } 
      lastAddSw = addSwState;
  }
  sei();
}


volatile bool isSecond = false;
volatile float prev;
ISR(TIMER1_COMPA_vect)
{
  if (isSecond){
    stm.tickLightTimer();
    stm.tickDisplayTimerSec();
    stm.setTemperature(sensor.getTempCByIndex(0));
  }else{
    sensor.requestTemperatures();
  }
  isSecond = !isSecond;
}


void setup()
{
  #ifdef DEBUG_SERIAL
    Serial.begin(9600);
    Serial.println("Started with debug");
  #endif

  int initVal, initTimeout;
  EEPROM.get(EEPROM_INIT_ADDR, initVal);
  if (initVal != EEPROM_INIT_VALUE){
    EEPROM.put(EEPROM_INIT_ADDR, EEPROM_INIT_VALUE);
  }else{
    EEPROM.get(EEPROM_TIMEOUT_ADDR, initTimeout);
    stm.setDefaultLightTimeout(initTimeout);
  }

  sensor.begin();
  sensor.getAddress(Thermometer, 0);
  sensor.setWaitForConversion(false);  // Async requests
  sensor.setResolution(Thermometer, TEMPERATURE_PRECISION);

  pinMode(A0, INPUT);
  pinMode(ENCODER_SW_BUTTON, INPUT);
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  pinMode(ADDITIONAL_BUTTON, INPUT_PULLUP);

  // инициализация Timer1
  cli();  // отключить глобальные прерывания
  TCCR1A = 0;   // установить регистры в 0
  TCCR1B = 0;
  //OCR1A = 15624; // установка регистра совпадения 1C 16mhz
  // OCR1A = 7811; // установка регистра совпадения 0.5C 16mhz
  OCR1A = 3905; // установка регистра совпадения 0.5C 8Mhz
  TCCR1B |= (1 << WGM12);  // включить CTC режим 
  TCCR1B |= (1 << CS10); // Установить биты на коэффициент деления 1024
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 

  sei(); // включить глобальные прерывания


  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    #ifdef DEBUG_SERIAL
      Serial.println(F("SSD1306 allocation failed"));
    #endif
    for(;;); // Don't proceed, loop forever
  }
  #ifdef DEBUG_SERIAL
    Serial.println(F("SSD1306 allocation success"));
  #endif

  // encoder
  cli();
  pciSetup(ENCODER_CLK);
  pciSetup(ENCODER_DT);
  pciSetup(ENCODER_SW_BUTTON);
  pciSetup(ADDITIONAL_BUTTON);
  sei();

  display.clearDisplay();
  da.drawLightStatus(stm.isLightOn());
}


void loop()
{
  stm.refreshState();
}

