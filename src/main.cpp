#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define enablePin 12
#define dirPin 11
#define stepPin 10
#define stopButt 9
#define endStop 8

#define encA 2
#define encB 3
#define encS 4

#define o2Sensor A0

float reduction = 5.18;
float stepsPerRev = 400;
float degreesPerFullStroke = 360*0.40;

//negative if motor connection is inverse
int inverter = 1;

#define ratioMAX 4
#define ratioMIN 1 //
#define airVolumeMAX 100 //procenty
#define airVolumeMIN 10 //procenty
#define freqMAX 20 //per minute
#define freqMIN 6  //per minutr

#define acceleration 100000
//in steps per second
#define speedMax 2500

#define expiratoryTimeSubtract 24
#define inspiratoryTimeSubtract 0.02

float airVolume = 100; //in percents
float inspiratory_factor = 1;
float expiratory_factor = 2;
float freq = 40;

//this will be calcuilated
float nSteps = (reduction) * (airVolume / 100.0) * stepsPerRev * (degreesPerFullStroke / 360.0);
float cycleTime;
float expiratoryTime;
float inspiratoryTime;
float speed;

unsigned long entryMillis = 0;
uint8_t changeFlag=0;
uint8_t stopFlag=1;

void recalculateParameters(void);
void calibrateMotor(void);
void checkUI(void);
void redrawUI(uint8_t stop, uint8_t nPos);
void updateSensor(void);

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, enablePin);
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);


void setup() {
  stepper.setMaxSpeed(speedMax);
  stepper.setAcceleration(acceleration);

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encS, INPUT_PULLUP);

  pinMode(endStop, INPUT_PULLUP);
  pinMode(stopButt, INPUT_PULLUP);

  Serial.begin(115200);

  lcd.begin(20,4);
  lcd.clear();
  lcd.home();
  lcd.print("Calibrating device,");
  lcd.setCursor(0,1);
  lcd.print("please wait.");

  calibrateMotor();
  recalculateParameters();
  redrawUI(1,0);
}

void loop() {
  if(stopFlag) {
    static unsigned long lastMillis=0;
    checkUI();
    if(changeFlag) {
      recalculateParameters();
      changeFlag=0;
    }
    if(digitalRead(stopButt)==LOW &&  (entryMillis+1000)<millis()) {
      stopFlag=0;
      redrawUI(0,0);
      entryMillis=millis();
    }
    if(lastMillis+1000<millis()) {
      updateSensor();
      lastMillis = millis();
    }
  } else {      
    //set target speed for inspiratory phase
    stepper.setMaxSpeed(speed);
    //set number of steps needed
    stepper.moveTo(inverter*nSteps);
    while(stepper.distanceToGo() != 0) {
      stepper.run();
    }

    stepper.setMaxSpeed(speedMax);
    stepper.moveTo(0);
    while(stepper.distanceToGo() != 0) {
      stepper.run();
    }
    unsigned long target = millis() + expiratoryTime;
    uint8_t once=0;
    while(millis()<target){
      if(digitalRead(stopButt)==LOW && (entryMillis+1000)<millis()) {
        stopFlag=1;
        redrawUI(1,0);
        entryMillis=millis();
        break;
      }
      if(!once) {
        updateSensor();
        once++;
      }
    }    
  }  
}

void redrawUI(uint8_t stop, uint8_t nPos) {
  lcd.clear();
  lcd.home();
  lcd.setCursor(1,0);  
  lcd.print("O2: ");
  lcd.setCursor(10,0);  
  lcd.print("%");
  updateSensor();
  lcd.setCursor(1,1);
  lcd.print("Rate: ");
  lcd.print((int)freq);
  lcd.print("/min");

  lcd.setCursor(1,2);
  lcd.print("Volume: ");
  lcd.print((int)airVolume);
  lcd.print("%");

  lcd.setCursor(1,3);
  lcd.print("I/E: ");
  lcd.print((int)inspiratory_factor);
  lcd.print("/");
  lcd.print((int)expiratory_factor);

  

  if(stop) {    
    lcd.setCursor(0,nPos+1);
    lcd.print('>');
    lcd.setCursor(16,3);
    lcd.print("EDIT");
  }
}

void updateSensor(void) {
  static int sum=0;
  uint8_t i=0;
  for(i=0;i<32;i++) {    
    sum+=analogRead(o2Sensor);
  }
  sum >>= 5;
  i=0;
  float MeasuredVout = sum * (5.0 / 1023.0);
  float Concentration = MeasuredVout * 0.21 / 2.0;
  lcd.setCursor(5,0);
  lcd.print("    ");
  lcd.setCursor(5,0);
  lcd.print(Concentration*100.0);  
}

void checkUI(void) {
  static uint8_t nPos = 0;
  static int encLastS = HIGH;
  static unsigned long encLastMil = 0;

  int currState = digitalRead(encS);

  if(encLastS == HIGH && currState == LOW && encLastMil+120<millis()) {
    encLastMil = millis();
    if(nPos<2)
      nPos++;
    else 
      nPos=0;
    redrawUI(1,nPos);
  }
  encLastS = currState;

  static uint8_t iePos=0;
  static int encLastA = HIGH;
  int change=0;
  currState = digitalRead(encA);
  if (encLastA == HIGH && currState == LOW){
    if (digitalRead(encB) == HIGH) { 
      change=1;
    } else {
      change=-1;
    }

    switch (nPos) {
      case 0: {
        if(change>0 && freq<40) 
          freq++;
        else if(change<0 && freq>6)
          freq--;
        break;
      }
      case 1: {
        if(change>0 && airVolume<100)
          airVolume++;
        else if(change<0 && airVolume>5)
          airVolume--;
        break;
      }
      case 2: {
        if(change>0 && iePos<4) 
          iePos++;
        else if(change<0 && iePos>0)
          iePos--;

        switch(iePos) {
          case 0: inspiratory_factor=2;expiratory_factor=1; break;
          case 1: inspiratory_factor=1;expiratory_factor=2; break;
          case 2: inspiratory_factor=1;expiratory_factor=3; break;
          case 3: inspiratory_factor=1;expiratory_factor=4; break;
          case 4: inspiratory_factor=1;expiratory_factor=5; break;
        }
        break;
      }
    }
    redrawUI(1,nPos);
    changeFlag=1;
  }  
  encLastA=currState;
}

void calibrateMotor(void) {
  if(digitalRead(endStop) == LOW) {
    //we are probably at position 0, but move forward and return
    stepper.move(inverter*1000);
    while (digitalRead(endStop) == HIGH && stepper.isRunning()) {
      stepper.run();
    }
    //zero steps to move
    stepper.moveTo(stepper.currentPosition());

    stepper.move(inverter*-2000);
    while (digitalRead(endStop) == HIGH) {
      stepper.run();
    }
    //zero steps to move
    stepper.moveTo(stepper.currentPosition());
    //set current location as zero point
    stepper.setCurrentPosition(0);
  } else {
    //move to 0 position
    stepper.move(inverter*-100000);
    while (digitalRead(endStop) == HIGH) {
      stepper.run();
    }
    //zero steps to move
    stepper.moveTo(stepper.currentPosition());
    //set current location as zero point
    stepper.setCurrentPosition(0);
  }
}

void recalculateParameters(void) {
  //this will be calcuilated
  nSteps = (reduction) * (airVolume / 100) * stepsPerRev * (degreesPerFullStroke / 360.0);
  //calculate time of whole cycle (inspiratory and expiratory)
  cycleTime = 60 / freq; //in s
  inspiratoryTime = (cycleTime/(inspiratory_factor+expiratory_factor)*inspiratory_factor)-inspiratoryTimeSubtract;
  //target speed (steps/s) is number of steps divided by inspiratory time in s
  //acceleration time is insignificant 
  speed = nSteps / inspiratoryTime;
  //calculate expiratory time and subtract motor return time (number of steps divided by max speed) [ms]
  expiratoryTime = (((cycleTime/(inspiratory_factor+expiratory_factor)*expiratory_factor)-(nSteps/speedMax))*1000.0) - expiratoryTimeSubtract;
}