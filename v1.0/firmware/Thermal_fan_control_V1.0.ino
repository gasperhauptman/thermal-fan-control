//libraries for using Dallas temperature sensor DS18B20 and lcd
#include <OneWire.h>
#include <DallasTemperature.h>
#include "lib_I2CLCD.h"
#include <avr/wdt.h>

//********************** USER SET PARAMETERS ***********************************************************************************************************************************

#define SERIAL_FREQ 115200  //set a frequnecy that serial commnunication runs on, set to 0 to disable serial monitoring

//conected devices, fans and temperature sensors
#define NUM_SENSORS 2    // number of connected temperature sensors (1..2), when only 1 sensor is connected, all 4 fan slots (pwm outputs) are controlled by that sensor
#define FAN_MASK 0b0101  // which slots have fan connected, "0b(fan4)(fan3)(fan2)(fan1)", 1 if fan is present, 0 if slot is empty, for example 0b0111

//defines arduino pins for I/O devices (do not change when using V1.0 PCB)
#define ONEWIREBUS 2  //pin temperature sensors are connected to
#define FAN1PWM 9     //PWM1 output pin, controls fan 1 and 2
#define FAN2PWM 10    //PWM2 output pin, controls fan 3 and 4
#define ALARM 3       //pin that outputs "0" when alarm is triggered
#define FANPIN1 4     //pin that tacho from first fan is connected to
#define FANPIN2 5     //same for second fan
#define FANPIN3 6     //third
#define FANPIN4 7     //and fourth

//set a frequency of PWM required by driven fan
#define PWM_FREQ 25000  //in Hz (usually around 25kHz for PC fans)

//set a measururing/PWM refresh frequency
#define UPDATE_FREQ 1000  //period in miliseconds (limited by temperature to digital convertion speed of temperature sensors, which is ~750ms @ 12bits .. ~96ms @ 9bits)

//set diferent temperatures that define how duty cycle is calculated
#define UPPER_LIM 40   //temperature above which duty cycle is always 100%
#define LOWER_LIM 25   //temperature below which duty cycle is at it's minimum value      //between these 2 values, PWM duty cycle is linear from set MIN_DUTY value to 100%
#define MIN_DUTY 0.2   //minimum duty cycle value, set between 0 and 1 (0 and 100%)
#define ALARM_TEMP 35  //temperature at which alarm is triggered, usually same as UPPER_LIM but can be set at different value

//fan parameters
#define PULSE_COUNT 2  //set a number of pulses given by fan each rotation (usually 2 for computer fans)
#define MIN_RPM 1000    //lower limit for fan rpm, if lower than, alarm is triggered (set to 0 to disable low rpm alarm)

//when installing new sensor, you must paste sensor's addresses here, can be copied from serial monitor at start up of the system!
//if only using 1 sensor, put it's address under sensor1 !
DeviceAddress sensor1 = { 0x28, 0x92, 0xAD, 0x43, 0x51, 0x20, 0x01, 0x39 };
DeviceAddress sensor2 = { 0x28, 0xEE, 0x35, 0xB1, 0x0C, 0x00, 0x00, 0xD7 };

//***********************************************************************************************************************************************************************************

OneWire oneWire(ONEWIREBUS);
DallasTemperature T_sensor(&oneWire);

#define PWM_COUNTER ((F_CPU / PWM_FREQ) - 1)

float TempC[2];
bool alarmStatus = 0;  // 0=failure (high temp or low fan rpm), 1=OK
float duty[2] = {1.0, 1.0};  // start at full speed until first reading
bool errorTemp[2];
bool ErrorTemp = 0;
bool errorRPM[4];
bool ErrorRPM = 0;
int lcd = 0;

unsigned long lastTempUpdate = 0;
unsigned long lastDisplayUpdate = 0;
bool convRequested = 0;
unsigned long convStart = 0;

struct Fan {
  uint8_t pin;
  volatile unsigned long last;
  volatile unsigned long period;
  volatile int loopCount;
  float rpm;
};

Fan fans[4];

//definitions of custom characters for LCD
uint8_t degreeChar[8] = { 0x0E, 0x0A, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t fanChar[8] = { 0x00, 0x00, 0x19, 0x0B, 0x04, 0x1A, 0x13, 0x00 };
uint8_t breathing1[8] = { 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t breathing2[8] = { 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t breathing3[8] = { 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t breathing4[8] = { 0x10, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };

//code that only runs once at every start of the system
void setup() {
  wdt_disable();
  if (SERIAL_FREQ > 0) Serial.begin(SERIAL_FREQ);
  fans[0].pin = FANPIN1;
  fans[1].pin = FANPIN2;
  fans[2].pin = FANPIN3;
  fans[3].pin = FANPIN4;
  pinMode(ALARM, OUTPUT);
  for (int i = 0; i < 4; i++) {
    pinMode(fans[i].pin, INPUT);
    fans[i].last = micros();
    fans[i].period = 0;
    fans[i].loopCount = 0;
    fans[i].rpm = 0;
  }
  PCICR |= 0b00000100;
  PCMSK2 |= 0b11110000;
  T_sensor.begin();
  sensorCheck();
  T_sensor.setResolution(12);  // 9..12
  lcdBegin();
  InitPWM();
  writePWM(duty[0], duty[1]);
  interrupts();
  wdt_enable(WDTO_4S);
}

//code running continiously after setup() is complete
void loop() {
  // temp conversion request every UPDATE_FREQ miliseconds
  if (!convRequested && millis() - lastTempUpdate >= UPDATE_FREQ) {
    requestTemp();
    convStart = millis();
    convRequested = 1;
  }
  // read temp result after 750ms and calculate all important values
  if (convRequested && millis() - convStart >= 800) {
    convRequested = 0;
    lastTempUpdate = millis();
    readTemp();
    getFanSpeed();
    writePWM(duty[0] = getDuty(TempC[0]), duty[1] = getDuty(TempC[NUM_SENSORS > 1 ? 1 : 0]));
    updateLED();
    serialData();
  }
  // display updates independently of measuring frequency
  if (millis() - lastDisplayUpdate >= 400) {
    lastDisplayUpdate = millis();
    updateLCD();
  }
  wdt_reset();
}

ISR(PCINT2_vect) {  //checks for pin changes on tacho pins, interrupt routine
  static uint8_t lastPort = 0xFF;
  uint8_t port = PIND;
  unsigned long now = micros();

  if ((lastPort & (1 << PIND4)) && !(port & (1 << PIND4))) {
    fans[0].period += (now - fans[0].last);
    fans[0].loopCount++;
    fans[0].last = now;
  }

  if ((lastPort & (1 << PIND5)) && !(port & (1 << PIND5))) {
    fans[1].period += (now - fans[1].last);
    fans[1].loopCount++;
    fans[1].last = now;
  }

  if ((lastPort & (1 << PIND6)) && !(port & (1 << PIND6))) {
    fans[2].period += (now - fans[2].last);
    fans[2].loopCount++;
    fans[2].last = now;
  }

  if ((lastPort & (1 << PIND7)) && !(port & (1 << PIND7))) {
    fans[3].period += (now - fans[3].last);
    fans[3].loopCount++;
    fans[3].last = now;
  }

  lastPort = port;
}

void requestTemp(void) {  //sends request to temp sensors to start temp to digital convertion and returns without waiting for results
  T_sensor.setWaitForConversion(0);
  T_sensor.requestTemperatures();
}

void readTemp(void) {  //reads digital value of measured temperature once convertion is finished
  TempC[0] = T_sensor.getTempC(sensor1);
#if NUM_SENSORS >= 2
  TempC[1] = T_sensor.getTempC(sensor2);
#else
  TempC[1] = -127;  // mark as not connected
#endif
}

float getDuty(float temp) {  //returns duty cycle as a result of function of measured temperature and set temperature limits
  if (temp < -100) return 1.0;
  if (temp <= LOWER_LIM) return MIN_DUTY;
  if (temp >= UPPER_LIM) return 1.0;
  return MIN_DUTY + (temp - LOWER_LIM) * (1.0 - MIN_DUTY) / (UPPER_LIM - LOWER_LIM);
}

void getFanSpeed(void) {
  noInterrupts();
  for (int i = 0; i < 4; i++) {
    if (!(FAN_MASK & (1 << i))) continue;  // skip empty slots
    fans[i].rpm = fans[i].loopCount > 0 ? (1 / (((fans[i].period / 1000000.0) / fans[i].loopCount) * PULSE_COUNT)) * 60 : 0;
    fans[i].loopCount = 0;
    fans[i].period = 0;
  }
  interrupts();
}
void writePWM(float value1, float value2)  //writes given duty cycles into timer1 compare registers
{
  value1 = constrain(value1, 0.0, 1.0);
  value2 = constrain(value2, 0.0, 1.0);
  OCR1A = (uint16_t)(value1 * ICR1);
  OCR1B = (uint16_t)(value2 * ICR1);
}

void updateLED(void) {  //update alarm status based on system state
  //check RPM
  ErrorRPM = 0;
  for (int i = 0; i < 4; i++) {
    if (!(FAN_MASK & (1 << i))) {  // slot empty
      errorRPM[i] = 0;
      continue;
    }
    if (fans[i].rpm < MIN_RPM) {
      ErrorRPM = 1;
      errorRPM[i] = 1;
    } else errorRPM[i] = 0;
  }
  //check temperatures
  ErrorTemp = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (TempC[i] >= ALARM_TEMP || TempC[i] < -100) {
      ErrorTemp = 1;
      errorTemp[i] = 1;
    } else errorTemp[i] = 0;
  }
  //set or reset alarm
  if (ErrorRPM || ErrorTemp) alarmStatus = 0;
  else alarmStatus = 1;
  digitalWrite(ALARM, alarmStatus);
}

void serialData(void) {  //prints current values of fan speed, temperature and alarm status
  if (SERIAL_FREQ == 0) return;
  Serial.print("RPM: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(fans[i].rpm, 0);
    if (i < 4 - 1) Serial.print(", ");
  }
  Serial.print("    Temp('C): ");
  Serial.print(TempC[0]);
#if NUM_SENSORS >= 2
  Serial.print(", ");
  Serial.print(TempC[1]);
#endif
  Serial.print("     Fan_power(%): ");
  Serial.print(duty[0] * 100);
  Serial.print(", ");
  Serial.print(duty[1] * 100);
  if (alarmStatus) Serial.println("     OK");
  else {
    Serial.print("     Alarm");
    if (ErrorTemp && ErrorRPM) Serial.println(" - high measured temperature and low fan speed!");
    else if (ErrorTemp && !ErrorRPM) Serial.println(" - high measured temperature!");
    else if (!ErrorTemp && ErrorRPM) Serial.println(" - low fan speed, check fan(s)");
    else Serial.println("");
  }
}

void sensorCheck(void) {  //checks for connected and missing temperature sensors and reports to user using serial monitor
  if (SERIAL_FREQ == 0) return;
  Serial.println("----------------------------------------------------------------------------------------------------");
  Serial.println();
  Serial.print("Devices found: ");
  Serial.println(T_sensor.getDeviceCount());
  Serial.println();
  Serial.println("Paste this addresses into code when installing a new temperature sensor:");
  DeviceAddress sensorAddr;
  for (uint8_t i = 0; i < T_sensor.getDeviceCount(); i++) {
    if (T_sensor.getAddress(sensorAddr, i)) {
      Serial.print("Connected sensor ");
      Serial.print(i + 1);
      Serial.print(" address: ");
      printAddress(sensorAddr);
    } else {
      Serial.print("Failed to read sensor at index ");
      Serial.println(i);
    }
  }
  Serial.println();
  if (!T_sensor.isConnected(sensor1)) {
    Serial.print("Sensor1 not found! - ");
    printAddress(sensor1);
  }
#if NUM_SENSORS >= 2
  if (!T_sensor.isConnected(sensor2)) {
    Serial.print("Sensor2 not found! - ");
    printAddress(sensor2);
  }
#endif
  Serial.println();
  Serial.println("----------------------------------------------------------------------------------------------------");
}

void printAddress(DeviceAddress sensorAddr) {  //prints 64-bit temperature sensor address in format, that can be directly copied into code
  if (SERIAL_FREQ == 0) return;
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("0x");
    if (sensorAddr[i] < 16) Serial.print("0");
    Serial.print(sensorAddr[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println();
}

void lcdBegin(void) {  //begins LCD communication, defines custom characters and displays device name
  Wire.begin();
  initLCD();
  clearDisplayLCD();
  cursorOFF();
  returnHomeLCD();
  printDisplayLCD("Thermal fan ctrl");
  locateCursorLCD(0, 1);
  printDisplayLCD("IJS, 2026");
  createCharLCD(0, degreeChar);
  createCharLCD(1, fanChar);
  createCharLCD(3, breathing1);
  createCharLCD(4, breathing2);
  createCharLCD(5, breathing3);
  createCharLCD(6, breathing4);
  delay(2000);
  clearDisplayLCD();
}

static int prev_lcd = -1;
static float prev_TempC[2] = { 999.0f, 999.0f };
static float prev_duty[2] = { -1.0f, -1.0f };
static bool prev_errorTemp[2] = { 0, 0 };
static bool prev_errorRPM[4] = { 1, 1, 1, 1 };  // force first draw

void updateLCD(void) {  //updates lcd based on what data changed
  lcd++;
  if (lcd > 3) lcd = 0;
  // spinning icon on row 0 only
  locateCursorLCD(0, 0);
  printCustomCharLCD(lcd + 3);
  for (int i = 0; i < 2; i++) {
    // blank col 0 on row 1 (no icon there)
    if (i == 1) {
      locateCursorLCD(0, 1);
      printDisplayLCD(" ");
    }
    // --- Temperature column ---
    bool tempChanged = (TempC[i] != prev_TempC[i]) || (errorTemp[i] != prev_errorTemp[i]) || ((lcd % 2) != (prev_lcd % 2));
    if (tempChanged) {
      locateCursorLCD(1, i);
      if (i >= NUM_SENSORS) {
        printDisplayLCD("        ");
      } else if (TempC[i] == -127) {
        printDisplayLCD("No Con. ");
      } else {
        printFloatLCD(TempC[i], 5, 1);
        printCustomCharLCD(0);
        printDisplayLCD("C");
        if (lcd % 2 && errorTemp[i]) printDisplayLCD("!");
        else printDisplayLCD(" ");
      }
      prev_TempC[i] = TempC[i];
      prev_errorTemp[i] = errorTemp[i];
    }
    // --- Duty column ---
    int dutyIdx = i < NUM_SENSORS ? i : 0;
    int dutyPct = (int)round(duty[dutyIdx] * 100);
    int prevDutyPct = (int)round(prev_duty[i] * 100);  // <-- use i, not dutyIdx
    if (dutyPct != prevDutyPct) {
      locateCursorLCD(9, i);
      if (dutyPct >= 100) printDisplayLCD(" MAX ");
      else {
        printIntLCD(dutyPct, 3);
        printDisplayLCD("% ");
      }
      prev_duty[i] = duty[dutyIdx];  // <-- store into i slot, not dutyIdx slot
    }
    // --- Fan indicators ---
    for (int a = 0; a < 2; a++) {
      int fanIdx = i * 2 + a;
      bool fanChanged = (FAN_MASK & (1 << fanIdx)) && (errorRPM[fanIdx] != prev_errorRPM[fanIdx]);
      bool blinkChanged = ((lcd < 2) != (prev_lcd < 2)) && (FAN_MASK & (1 << fanIdx)) && errorRPM[fanIdx];

      if (fanChanged || blinkChanged) {
        locateCursorLCD(14 + a, i);
        if (!(FAN_MASK & (1 << fanIdx))) printDisplayLCD(" ");
        else if (errorRPM[fanIdx] && lcd < 2) printDisplayLCD(" ");
        else printCustomCharLCD(1);
        if (FAN_MASK & (1 << fanIdx)) prev_errorRPM[fanIdx] = errorRPM[fanIdx];
      }
    }
  }
  prev_lcd = lcd;
}

void InitPWM(void) {         //set timer1 as PWM generator with outputs on pins D9 and D1
  pinMode(FAN1PWM, OUTPUT);  // D9
  pinMode(FAN2PWM, OUTPUT);  // D10
  // Stop timer
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // Fast PWM, TOP = ICR1 (Mode 14)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);  // prescaler = 1
  ICR1 = PWM_COUNTER;
  OCR1A = 0;
  OCR1B = 0;
}