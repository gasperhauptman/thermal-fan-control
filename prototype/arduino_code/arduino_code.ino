
//libraries for using Dallas temperature sensor DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

//********************** USER SET PARAMETERS ************************************************************************************

#define serial_freq 115200  //set a frequnecy that serial commnunication runs on, set to 0 to disable serial monitoring

//defines arduino pins for I/O devices
#define OneWireBus 2  //pin temperature sensor is connected to
#define fanPWM 9      //do not modify
#define alarmLED 3    //pin that outputs logical high when alarm is triggered
#define fanPin1 4     //pin that tacho from first fan is connected to
#define fanPin2 5     //same for second fan

//set a frequency of pwm required by driven fan
#define frekvenca_pwm 25000  //in Hz

//set a measururing/PWM refresh frequency
#define updateFreq 1  //period in seconds, not including temp -> digital convertion time

//set diferent temperatures that define how duty cycle is controlled
#define upper_lim 45   //temperature above which duty cycle is always 100%
#define lower_lim 30   //temperature below which duty cycle is at it's minimum value      //between these 2 values, PWM duty cycle is linear from set min_duty value to 100%
#define min_duty 0.5   //minimum duty cycle value, between 0 and 1 (0 and 100%)
#define alarm_temp 50  //temerature at which alarm is triggered

//fan parameters
#define pulse_count 2  //set a number of pulses given by a fan per rotation
#define minSpeed 1000  //lower limit for fan rpm, if lower than, alarm is triggered

//*********************************************************************************************************************************

OneWire oneWire(OneWireBus);
DallasTemperature T_sensor(&oneWire);

float TempC = 0;
int counter = 16000000 / frekvenca_pwm - 1;
int duty = 0;
unsigned long lastUpdate = 0;
bool ledState = 0;  // 0=failure (high temp or low fan rpm), 1=OK

struct Fan {
  uint8_t pin;
  volatile unsigned long last;
  volatile unsigned long period;
  volatile int loopCount;
  float rpm;
};

Fan fans[2];

void setup() {

  if (serial_freq > 0) Serial.begin(serial_freq);

  fans[0].pin = fanPin1;
  fans[1].pin = fanPin2;

  pinMode(fanPWM, OUTPUT);
  pinMode(alarmLED, OUTPUT);

  for (int i = 0; i < 2; i++) {
    pinMode(fans[i].pin, INPUT);
    fans[i].last = micros();
    fans[i].period = 0;
    fans[i].loopCount = 0;
    fans[i].rpm = 0;
  }

  T_sensor.begin();
  if (serial_freq > 0) {
    Serial.print("Number of connected OneWire devices: ");
    Serial.println(T_sensor.getDeviceCount());
  }

  InitPWM();

  PCICR |= 0b00000100;   //enable portd interrupts
  PCMSK2 |= 0b00110000;  //enable pins d4 and d5

  interrupts();
}

void loop() {
  if (millis() - lastUpdate >= updateFreq * 1000) {
    lastUpdate = millis();
    getTemp();
    getFanSpeed();
    serialData();
    writePWM(getDuty(TempC) * counter);
    updateLED();
  }
}

ISR(PCINT2_vect) {
  static uint8_t lastPort = 0xFF;
  uint8_t port = PIND;
  unsigned long now = micros();

  // Detects falling edge on D4
  if ((lastPort & (1 << PIND4)) && !(port & (1 << PIND4))) {
    fans[0].period += (now - fans[0].last);
    fans[0].loopCount++;
    fans[0].last = now;
  }

  // Detects falling edge on D5
  if ((lastPort & (1 << PIND5)) && !(port & (1 << PIND5))) {
    fans[1].period += (now - fans[1].last);
    fans[1].loopCount++;
    fans[1].last = now;
  }
  lastPort = port;
}

void getTemp(void) {
  T_sensor.requestTemperatures();
  TempC = T_sensor.getTempCByIndex(0);
}

float getDuty(float temp) {
  if (temp <= lower_lim) return min_duty;
  if (temp >= upper_lim) return 1.0;
  return min_duty + (temp - lower_lim) * (1.0 - min_duty) / (upper_lim - lower_lim);
}

void getFanSpeed(void) {
  noInterrupts();

  for (int i = 0; i < 2; i++) {
    fans[i].rpm = fans[i].loopCount > 0 ? (1 / (((fans[i].period / 1000000.0) / fans[i].loopCount) * pulse_count)) * 60 : 0;
  }

  for (int i = 0; i < 2; i++) {
    fans[i].loopCount = 0;
    fans[i].period = 0;
  }
  interrupts();
}

void writePWM(uint16_t value) {
  if ((value >= 0) && (value < (counter + 1))) {
    OCR1A = value;
  }
}

void updateLED(void) {
  if (fans[0].rpm < minSpeed || fans[1].rpm < minSpeed) {
    ledState = 0;
  } else {
    if (TempC >= alarm_temp || TempC < -100) ledState = 0;
    else if (TempC < (alarm_temp - 1)) ledState = 1;
  }
  digitalWrite(alarmLED, ledState);
}

void serialData(void) {
  if (serial_freq > 0) {
    Serial.print("RPM_1 = ");
    Serial.print(fans[0].rpm);
    Serial.print(", RPM_2 = ");
    Serial.print(fans[1].rpm);
    Serial.print(", Temp = ");
    Serial.print(TempC);
    if (ledState) Serial.println(", OK");
    else Serial.println(", Alarm!");
  }
}
  
void InitPWM(void)  //get PWM on pin D9
{
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10); 

  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (0 << WGM10);  
  ICR1 = counter;
  TCNT1 = 0;
  OCR1B = 0;
  OCR1A = 0;
  TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);
  DDRB |= (1 << DDB1);
  
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
}