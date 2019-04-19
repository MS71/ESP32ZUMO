#include <avr/sleep.h>
#include <Zumo32U4.h>
#include <Wire.h>

#define CMD_BEEP                0x01
#define CMD_BATLEVEL            0x02
#define CMD_MOTORS_SET_SPEED    0x03
#define CMD_ENCODERS            0x04
#define CMD_LIDAR_SET_PWM       0x05
#define CMD_GET_STATUS          0x06

Zumo32U4Buzzer            buzzer;
Zumo32U4Motors            motors;
Zumo32U4Encoders          encoders;
Zumo32U4LineSensors       lineSensors;
Zumo32U4ProximitySensors  proxSensors;
#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];
bool useEmitters = true;

uint8_t proxSensorsValues[6] = {0};

volatile uint32_t loopcnt = 0;

volatile uint16_t bat = 0;

volatile int16_t enc_l = 0;
volatile int16_t enc_r = 0;

void setPwmDutyA(int val) {
  TC4H = val >> 8;
  OCR4A = 0xFF & val;
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  lineSensors.initFiveSensors();

  buzzer.playFrequency(100, 100, 10);

#ifdef USE_RPISLAVELIB
#else
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
#endif

#if 1
  PLLFRQ = (PLLFRQ & 0xCF) | 0x30;
#else
// Configure PLL.
  PLLCSR  = (1 << PINDIV);  // set if using 16MHz clock.
  PLLFRQ  = (1 << PLLUSB);  // divide PLL output by two.
  PLLFRQ |= (1 << PLLTM0);  // postscale by 1.
  PLLFRQ |= (10 << PDIV0);  // 96MHz output frequency.

  // Enable PLL.
  PLLCSR |= (1 << PLLE);    // enable PLL.
  
  // Wait for PLL lock.
  while (!(PLLCSR & (1 << PLOCK)));
#endif  

#if 1
  // 10-bit operation
  TC4H = 0x03; OCR4C = 0xFF;
  //Configuration of Timer 4 Registers, OC4A (D13) + 0C4B (D10)
  TCCR4A = (TCCR4A & 0x00111100) | 0b10000010;
  //Prescaler
  TCCR4B = (TCCR4B & 0b1110000) | 1;

  setPwmDutyA(0);
#endif

  lineSensors.initThreeSensors();
  proxSensors.pullupsOn();
  proxSensors.initThreeSensors();
  buzzer.playFrequency(1000, 50, 10);
}

void loop()
{
  int16_t tmp_l = encoders.getCountsAndResetLeft();
  int16_t tmp_r = encoders.getCountsAndResetRight();
  uint16_t tmp_bat = readBatteryMillivolts();

  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  proxSensors.read();

  proxSensorsValues[0] = proxSensors.countsLeftWithLeftLeds();
  proxSensorsValues[1] = proxSensors.countsLeftWithRightLeds();
  proxSensorsValues[2] = proxSensors.countsFrontWithLeftLeds();
  proxSensorsValues[3] = proxSensors.countsFrontWithRightLeds();
  proxSensorsValues[4] = proxSensors.countsRightWithLeftLeds();
  proxSensorsValues[5] = proxSensors.countsRightWithRightLeds();

  cli();
  loopcnt++;
  bat = tmp_bat;
  enc_l += tmp_l;
  enc_r += tmp_r;
  sei();

#if 1
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_mode();
  /** Das Programm läuft ab hier nach dem Aufwachen weiter. **/
  /** Es wird immer zuerst der Schlafmodus disabled.        **/
  sleep_disable();
#endif
}

#define MSG_UINT8(_p_)  (uint8_t)((((_p_)[1])<<0))
#define MSG_UINT16(_p_) (uint16_t)(((uint16_t)((_p_)[0])<<8)|((uint16_t)((_p_)[1])<<0))
#define MSG_UINT32(_p_) (uint32_t)(((uint32_t)((_p_)[0])<<24)|((uint32_t)((_p_)[1])<<16)|((uint32_t)((_p_)[2])<<8)|((uint32_t)((_p_)[3])<<0))

#define MSG_INT8(_p_)  (int8_t)((((_p_)[1])<<0))
#define MSG_INT16(_p_) (int16_t)(((int16_t)((_p_)[0])<<8)|((int16_t)((_p_)[1])<<0))
#define MSG_INT32(_p_) (int32_t)(((int32_t)((_p_)[0])<<24)|((int32_t)((_p_)[1])<<16)|((int32_t)((_p_)[2])<<8)|((int32_t)((_p_)[3])<<0))

uint8_t cmd = 0;

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  ledGreen(1);
  cmd = Wire.read();
  howMany--;
  if ( howMany > 0 )
  {
    const int L = 32;
    int n = 0;
    uint8_t msg[L];

    while (1 < Wire.available() && ((n + 1) < L))
    {
      // loop through all but the last
      msg[n++] = Wire.read(); // receive byte as a character
    }
    msg[n++] = Wire.read();    // receive byte as an integer

    switch (cmd)
    {
      case CMD_BEEP:
        if ( n >= (2 + 2 + 1) )
        {
          uint16_t freq = MSG_UINT16(&msg[0]);
          uint16_t duration = MSG_UINT16(&msg[2]);
          uint8_t volume = MSG_UINT8(&msg[2 + 2]);
          buzzer.playFrequency(freq, duration, volume);
        }
        break;
      case CMD_MOTORS_SET_SPEED:
        {
          int16_t l = MSG_INT16(&msg[0]);
          int16_t r = MSG_INT16(&msg[2]);
          motors.setLeftSpeed(l);
          motors.setRightSpeed(r);
        }
        break;
      case CMD_LIDAR_SET_PWM:
        {
          uint16_t v = MSG_UINT16(&msg[0]);
          setPwmDutyA(v);
        }
        break;
    }
  }
  ledGreen(0);
}

void requestEvent()
{
  ledGreen(1);
  uint8_t idx = 0;
  uint8_t buf[4 + 2 + 4 + 2*NUM_SENSORS + 6 ];

  buf[idx++] = (uint8_t)(loopcnt >> 24) & 0xff;
  buf[idx++] = (uint8_t)(loopcnt >> 16) & 0xff;
  buf[idx++] = (uint8_t)(loopcnt >> 8) & 0xff;
  buf[idx++] = (uint8_t)(loopcnt >> 0) & 0xff;

  buf[idx++] = (uint8_t)(bat >> 8) & 0xff;
  buf[idx++] = (uint8_t)(bat >> 0) & 0xff;

  buf[idx++] = (uint8_t)(enc_l >> 8) & 0xff;
  buf[idx++] = (uint8_t)(enc_l >> 0) & 0xff;
  enc_l = 0;

  buf[idx++] = (uint8_t)(enc_r >> 8) & 0xff;
  buf[idx++] = (uint8_t)(enc_r >> 0) & 0xff;
  enc_r = 0;

  {
    int i;
    for(i=0;i<NUM_SENSORS;i++)    
    {
      buf[idx++] = (uint8_t)(lineSensorValues[i] >> 8) & 0xff;
      buf[idx++] = (uint8_t)(lineSensorValues[i] >> 0) & 0xff;
    }
  }

  {
    int i = 0;
    buf[idx++] = proxSensorsValues[i++];
    buf[idx++] = proxSensorsValues[i++];
    buf[idx++] = proxSensorsValues[i++];
    buf[idx++] = proxSensorsValues[i++];
    buf[idx++] = proxSensorsValues[i++];
    buf[idx++] = proxSensorsValues[i++];
  }

  Wire.write(buf, sizeof(buf));
  ledGreen(0);
}
