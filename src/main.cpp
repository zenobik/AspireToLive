#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define dirPin 3
#define stepPin 6
#define endStop 1

#define encVolumeA 0
#define encVolumeB 1
#define encRatioA 2
#define encRatioB 3
#define encIeA 4
#define encIeB 5

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

uint8_t changeFlag=0;

void recalculateParameters(void);
void calibrateMotor(void);
void checkUI(void);
void redrawUI(void);

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, 8);
LiquidCrystal_I2C lcd(0x27);

void setup() {
  stepper.setMaxSpeed(speedMax);
  stepper.setAcceleration(acceleration);

  lcd.begin(20,4);
  lcd.home();
  lcd.println("Calibrating device,");
  lcd.print("please wait.");

  pinMode(endStop, INPUT_PULLUP);

  calibrateMotor();
  recalculateParameters();
  redrawUI();
}

void loop() {
  //set target speed for inspiratory phase
  stepper.setMaxSpeed(speed);
  //set number of steps needed
  stepper.moveTo(inverter*nSteps);
  while(stepper.distanceToGo() != 0) {
    stepper.run();
    checkUI();
  }

  stepper.setMaxSpeed(speedMax);
  stepper.moveTo(0);
  while(stepper.distanceToGo() != 0) {
    stepper.run();
    checkUI();
  }
  delay(expiratoryTime);
  if(changeFlag) {
    recalculateParameters();
  }
}

void redrawUI(void) {
  lcd.clear();
  lcd.home();
  lcd.print("Rate: ");
  lcd.print((int)freq);
  lcd.println("/min");

  lcd.print("Volume: ");
  lcd.print((int)airVolume);
  lcd.println("%");

  lcd.print("I/E: ");
  lcd.print(inspiratory_factor);
  lcd.print("/");
  lcd.println(expiratory_factor);
}

void checkUI(void) {
  static uint8_t iePos=0;

  static int encVolumeLastA = digitalRead(encVolumeA);
  static int encRatioLastA = digitalRead(encRatioA);
  static int encIeLastA = digitalRead(encIeA);

  int currState = digitalRead(encVolumeA);
  if (currState != encVolumeLastA){
    if (digitalRead(encVolumeB) != currState) { 
       if(airVolume<100) airVolume++;
     } else {       
       if(airVolume>5) airVolume--;
     }
     changeFlag=1;     
     redrawUI();
  }
  encVolumeLastA = currState;

  currState = digitalRead(encRatioA);
  if (currState != encRatioLastA){
    if (digitalRead(encRatioB) != currState) { 
       if(freq<40) freq++;
     } else {
       if(freq>6)freq--;
     }
     changeFlag=1;
     redrawUI(); 
  }
  encRatioLastA = currState;

  currState = digitalRead(encIeA);
  if (currState != encIeLastA){
    if (digitalRead(encIeB) != currState) { 
       if(iePos<4)iePos++;
     } else {
       if(iePos>0)iePos--;
     }

     switch(iePos) {
       case 0: inspiratoryTime=2;expiratoryTime=1; break;
       case 1: inspiratoryTime=1;expiratoryTime=2; break;
       case 2: inspiratoryTime=1;expiratoryTime=3; break;
       case 3: inspiratoryTime=1;expiratoryTime=4; break;
       case 4: inspiratoryTime=1;expiratoryTime=5; break;
     }
     changeFlag=1;
     redrawUI();
  }
  encIeLastA = currState;
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

    stepper.move(inverter*-1000);
    while (digitalRead(endStop) == HIGH) {
      stepper.run();
    }
    //zero steps to move
    stepper.moveTo(stepper.currentPosition());
    //set current location as zero point
    stepper.setCurrentPosition(0);
  } else {
    //move to 0 position
    stepper.move(inverter*-1000);
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
  nSteps = (reduction) * (airVolume / 100) * stepsPerRev * (degreesPerFullStroke / 360);
  //calculate time of whole cycle (inspiratory and expiratory)
  cycleTime = 60 / freq; //in s
  inspiratoryTime = cycleTime/(inspiratory_factor+expiratory_factor)*inspiratory_factor;
  //target speed (steps/s) is number of steps divided by inspiratory time in s
  //acceleration time is insignificant 
  speed = nSteps / inspiratoryTime;
  //calculate expiratory time and subtract motor return time (number of steps divided by max speed) [ms]
  expiratoryTime = ((cycleTime/(inspiratory_factor+expiratory_factor)*expiratory_factor)-(nSteps/speedMax))*1000;
}