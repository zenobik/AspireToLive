#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SparkFunBME280.h>

#define enablePin 12
#define dirPin 11
#define stepPin 10
#define stopButt 9
#define endStop 8

#define encA 2
#define encB 3
#define encS 4

#define buzzer 5

#define o2Sensor A0

//uncomment to enable oxygen sensor
//#define O2_SENSOR 1

float reduction = 1.0;
float stepsPerRev = 800;
float degreesPerFullStroke = 360 * 0.55;

//set -1 if motor connection is inverted
int inverter = 1;
#define acceleration 10000
//in steps per second
#define speedMax 2000

#define expiratoryTimeSubtract 0
#define inspiratoryTimeSubtract 0.03

float airVolume = 100.0; //in percents
float inspiratory_factor = 1;
float expiratory_factor = 2;
float freq = 40;

//this will be calcuilated
float nSteps;
float cycleTime;
float expiratoryTime;
float inspiratoryTime;
float speed;

//temporary variables
float pressureBuf = 1023.0;
unsigned long entryMillis = 0;

uint8_t alarmFlag = 0;
uint8_t changeFlag = 0;
uint8_t stopFlag = 1;

void recalculateParameters(void);
void calibrateMotor(void);
void checkEncoder(void);
void drawUItemplate(void);
void redrawUI(uint8_t stop, uint8_t nPos);
void updateO2(void);
void checkPressure(void);

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, enablePin);
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
BME280 mySensorA;

void setup()
{
  delay(2000);
  //set lower max speed for calibration
  stepper.setMaxSpeed(speedMax / 2.0);
  stepper.setAcceleration(acceleration);

  //config inputs
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encS, INPUT_PULLUP);

  pinMode(endStop, INPUT_PULLUP);
  pinMode(stopButt, INPUT_PULLUP);

  pinMode(buzzer, OUTPUT);
  //PNP transistor - set PWM to 255 to turn off
  analogWrite(buzzer, 255);

  Wire.begin();

  //initialize BME280
  mySensorA.setI2CAddress(0x77);
  mySensorA.beginI2C();

  //print calibration message on screen
  lcd.begin(20, 4);
  lcd.clear();
  lcd.home();
  lcd.print("Calibrating device,");
  lcd.setCursor(0, 1);
  lcd.print("please wait.");

  //calibrate motor - go to endstop
  calibrateMotor();
  recalculateParameters();
  drawUItemplate();
}

//main loop
void loop()
{
  //ALTER SETTINGS
  if (stopFlag)
  {
    checkEncoder();
    //if something has changed - recalculate
    if (changeFlag)
    {
      recalculateParameters();
      changeFlag = 0;
    }
    //if stop button pressed -
    if (digitalRead(stopButt) == LOW && (entryMillis + 1000) < millis())
    {
      stopFlag = 0;
      redrawUI(0, 0);
      entryMillis = millis();
    }
#ifdef O2_SENSOR
    static unsigned long lastMillis = 0;
    if (lastMillis + 1000 < millis())
    {
      updateO2();
      lastMillis = millis();
    }
#endif
  }
  //WORK
  else
  {
    //set target speed for inspiratory phase
    stepper.setMaxSpeed(speed);
    //set number of steps needed
    stepper.moveTo(inverter * nSteps);
    while (stepper.distanceToGo() != 0)
    {
      stepper.run();
    }
    unsigned long target = millis() + expiratoryTime;
    pressureBuf = mySensorA.readFloatPressure();

    //move to endstop
    stepper.setMaxSpeed(speedMax);
    //value should be larger than needed
    stepper.move(inverter * -5000);
    while (digitalRead(endStop) == HIGH)
    {
      stepper.run();
    }
    //zero steps to move
    stepper.moveTo(stepper.currentPosition());
    //set current location as zero point
    stepper.setCurrentPosition(0);
    checkPressure();
#ifdef O2_SENSOR
    updateO2();
#endif
    while (millis() < target)
    {
      //Stop button pressed
      if (digitalRead(stopButt) == LOW && (entryMillis + 1000) < millis())
      {
        alarmFlag = 0;
        analogWrite(buzzer, 255);
        lcd.setCursor(15, 0);
        lcd.print("     ");
        stopFlag = 1;
        redrawUI(1, 0);
        entryMillis = millis();
        break;
      }
    }
  }
}

//function draw user interface for the first time with labels
void drawUItemplate(void)
{
  lcd.clear();
  lcd.home();
#ifdef O2_SENSOR
  lcd.setCursor(1, 0);
  lcd.print("O2: ");
  lcd.setCursor(10, 0);
  lcd.print("%");
  updateO2();
#else
  lcd.setCursor(0, 0);
  lcd.print("Aspire to live");
#endif
  lcd.setCursor(1, 1);
  lcd.print("Rate: ");
  lcd.print((int)freq);
  lcd.print("/min");

  lcd.setCursor(1, 2);
  lcd.print("Volume: ");
  lcd.print((int)airVolume);
  lcd.print("%");

  lcd.setCursor(1, 3);
  lcd.print("I/E: ");
  lcd.print((int)inspiratory_factor);
  lcd.print("/");
  lcd.print((int)expiratory_factor);

  lcd.setCursor(0, 1);
  lcd.print('>');
  lcd.setCursor(16, 3);
  lcd.print("EDIT");
}

//function redraw (updates) values without clearing whole LCD
//stop - is the device stopped
//nPos - currently edited value
void redrawUI(uint8_t stop, uint8_t nPos)
{
#ifdef O2_SENSOR
  lcd.setCursor(5, 0);
  lcd.print("    ");
  lcd.setCursor(5, 0);
  updateO2();
#endif
  lcd.setCursor(7, 1);
  lcd.print("      ");
  lcd.setCursor(7, 1);
  lcd.print((int)freq);
  lcd.print("/min");

  lcd.setCursor(9, 2);
  lcd.print("    ");
  lcd.setCursor(9, 2);
  lcd.print((int)airVolume);
  lcd.print("%");

  lcd.setCursor(6, 3);
  lcd.print((int)inspiratory_factor);
  lcd.print("/");
  lcd.print((int)expiratory_factor);

  //if stop mode - print EDIT text and 
  //select currently edited value
  if (stop)
  {
    for (uint8_t i = 1; i < 4; i++)
    {
      lcd.setCursor(0, i);
      lcd.print(' ');
    }

    lcd.setCursor(0, nPos + 1);
    lcd.print('>');
    lcd.setCursor(16, 3);
    lcd.print("EDIT");
  }
  //else clear edit indicators
  else
  {
    for (uint8_t i = 1; i < 4; i++)
    {
      lcd.setCursor(0, i);
      lcd.print(' ');
    }
    lcd.setCursor(16, 3);
    lcd.print("    ");
  }
}

//function analyze expiratory pressure
//if pressure is too low, probably pacient is disconncted
//pressureBuf variable contains pressure right after inspiration (atmospheric pressure
void checkPressure(void)
{
  pressureBuf = mySensorA.readFloatPressure() - pressureBuf;
  if (pressureBuf > 30.0 && alarmFlag)
  {
    analogWrite(buzzer, 255);
    alarmFlag = 0;
    lcd.setCursor(15, 0);
    lcd.print("     ");
  }
  else if (pressureBuf <= 30.0 && !alarmFlag)
  {
    analogWrite(buzzer, 128);
    alarmFlag = 1;
    lcd.setCursor(15, 0);
    lcd.print("ALARM");
  }
}

//function updates reading of O2 sensor on the LCD screen
void updateO2(void)
{
  static int sum = 0;
  uint8_t i = 0;
  for (i = 0; i < 32; i++)
  {
    sum += analogRead(o2Sensor);
  }
  sum >>= 5;
  i = 0;
  float MeasuredVout = sum * (5.0 / 1023.0);
  float Concentration = MeasuredVout * 0.21 / 2.0;
  lcd.setCursor(5, 0);
  lcd.print("    ");
  lcd.setCursor(5, 0);
  lcd.print(Concentration * 100.0);
}

//function reads encoder and changes respirator parameters
void checkEncoder(void)
{
  static uint8_t nPos = 0;
  static int encLastS = HIGH;
  static unsigned long encLastMil = 0;

  int currState = digitalRead(encS);

  if (encLastS == HIGH && currState == LOW && encLastMil + 120 < millis())
  {
    encLastMil = millis();
    if (nPos < 2)
      nPos++;
    else
      nPos = 0;
    redrawUI(1, nPos);
  }
  encLastS = currState;

  static uint8_t iePos = 0;
  static int encLastA = HIGH;
  int change = 0;
  currState = digitalRead(encA);
  if (encLastA == HIGH && currState == LOW)
  {
    if (digitalRead(encB) == HIGH)
    {
      change = 1;
    }
    else
    {
      change = -1;
    }

    switch (nPos)
    {
    case 0:
    {
      if (change > 0 && freq < 40)
        freq++;
      else if (change < 0 && freq > 6)
        freq--;
      break;
    }
    case 1:
    {
      if (change > 0 && airVolume < 100)
        airVolume++;
      else if (change < 0 && airVolume > 10)
        airVolume--;
      break;
    }
    case 2:
    {
      if (change > 0 && iePos < 4)
        iePos++;
      else if (change < 0 && iePos > 0)
        iePos--;

      switch (iePos)
      {
      case 0:
        inspiratory_factor = 2;
        expiratory_factor = 1;
        break;
      case 1:
        inspiratory_factor = 1;
        expiratory_factor = 2;
        break;
      case 2:
        inspiratory_factor = 1;
        expiratory_factor = 3;
        break;
      case 3:
        inspiratory_factor = 1;
        expiratory_factor = 4;
        break;
      case 4:
        inspiratory_factor = 1;
        expiratory_factor = 5;
        break;
      }
      break;
    }
    }
    redrawUI(1, nPos);
    changeFlag = 1;
  }
  encLastA = currState;
}

//function calibrates motor
//if the endstop is active, move forward, and then backward to the endstop
//if the endstop is not active, moves backward to the endstop
void calibrateMotor(void)
{
  if (digitalRead(endStop) == LOW)
  {
    //we are probably at position 0, but move forward and return to recalibrate
    stepper.move(inverter * 2000);
    while (digitalRead(endStop) == LOW && stepper.isRunning())
    {
      stepper.run();
    }
    //zero the number of steps to move
    stepper.moveTo(stepper.currentPosition());
    //move backward until reach endstop
    stepper.move(inverter * -2000);
    while (digitalRead(endStop) == HIGH)
    {
      stepper.run();
    }
    //zero the number of steps to move
    stepper.moveTo(stepper.currentPosition());
    //set the current location as zero point
    stepper.setCurrentPosition(0);
  }
  else
  {
    //move backward until reach endstop
    stepper.move(inverter * -10000);
    while (digitalRead(endStop) == HIGH)
    {
      stepper.run();
    }
    //zero the number of steps to move
    stepper.moveTo(stepper.currentPosition());
    //set the current location as zero point
    stepper.setCurrentPosition(0);
  }
}

void recalculateParameters(void)
{
  //this will be calcuilated
  nSteps = (reduction) * (airVolume / 100.0) * stepsPerRev * (degreesPerFullStroke / 360.0);
  //calculate time of whole cycle (inspiratory and expiratory)
  cycleTime = 60 / freq; //in s
  inspiratoryTime = (cycleTime / (inspiratory_factor + expiratory_factor) * inspiratory_factor) - inspiratoryTimeSubtract;
  //target speed (steps/s) is number of steps divided by inspiratory time in s
  //acceleration time is insignificant
  speed = nSteps / inspiratoryTime;
  //calculate expiratory time and subtract motor return time (number of steps divided by max speed) [ms]
  expiratoryTime = (((cycleTime / (inspiratory_factor + expiratory_factor) * expiratory_factor)) * 1000.0) - expiratoryTimeSubtract;
}