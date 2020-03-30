#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define enablePin 12
#define dirPin 11
#define stepPin 10
#define stopButt 9
#define endStop 8


#define encVolumeA 2
#define encVolumeB 3
#define encRatioA 4
#define encRatioB 5
#define encIeA 6
#define encIeB 7

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
void redrawUI(uint8_t stop);

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin, enablePin);
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);


void setup() {
  stepper.setMaxSpeed(speedMax);
  stepper.setAcceleration(acceleration);

  pinMode(encVolumeA, INPUT_PULLUP);
  pinMode(encVolumeB, INPUT_PULLUP);
  pinMode(encRatioA, INPUT_PULLUP);
  pinMode(encRatioB, INPUT_PULLUP);
  pinMode(encIeA, INPUT_PULLUP);
  pinMode(encIeB, INPUT_PULLUP);

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
  redrawUI(1);
}

void loop() {
  if(stopFlag) {
    checkUI();
    delay(20);
    if(changeFlag) {
      recalculateParameters();
      changeFlag=0;
    }
    if(digitalRead(stopButt)==LOW &&  (entryMillis+1000)<millis()) {
      stopFlag=0;
      redrawUI(0);
      entryMillis=millis();
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
    unsigned long target = millis() + expiratoryTime - expiratoryTimeSubtract;
    while(millis()<target){
      if(digitalRead(stopButt)==LOW && (entryMillis+1000)<millis()) {
        stopFlag=1;
        redrawUI(1);
        entryMillis=millis();
        break;
      }
    }    
  }  
}

void redrawUI(uint8_t stop) {
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,1);
  lcd.print("Rate: ");
  lcd.print((int)freq);
  lcd.print("/min");

  lcd.setCursor(0,2);
  lcd.print("Volume: ");
  lcd.print((int)airVolume);
  lcd.print("%");

  lcd.setCursor(0,3);
  lcd.print("I/E: ");
  lcd.print((int)inspiratory_factor);
  lcd.print("/");
  lcd.print((int)expiratory_factor);

  if(stop) {    
    lcd.setCursor(16,3);
    lcd.print("EDIT");
  }
}

void checkUI(void) {
  static uint8_t iePos=0;

  static int encVolumeLastA = digitalRead(encVolumeA);
  static int encRatioLastA = digitalRead(encRatioA);
  static int encIeLastA = digitalRead(encIeA);
  static unsigned long lastVolumeMil=0;
  static unsigned long lastRatioMil=0;
  static unsigned long lastIeMil=0;


  if(millis()>lastVolumeMil+70) {
    int currState = digitalRead(encVolumeA);
    if (currState != encVolumeLastA){
      if (digitalRead(encVolumeB) != currState) { 
        if(airVolume<100) airVolume++;
      } else {       
        if(airVolume>5) airVolume--;
      }
      lastVolumeMil=millis();
      redrawUI(1);
      changeFlag=1;     
    }
    encVolumeLastA = currState;
  }

  if(millis()>lastRatioMil+70) {
    int currState = digitalRead(encRatioA);
    if (currState != encRatioLastA){
      if (digitalRead(encRatioB) != currState) { 
        if(freq<40) freq++;
      } else {
        if(freq>6)freq--;
      }
      lastRatioMil=millis();
      redrawUI(1);
      changeFlag=1;
    }
    encRatioLastA = currState;
  }

  
  if(millis()>lastIeMil+70) {
    int currState = digitalRead(encIeA);
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
      lastIeMil=millis();
      redrawUI(1);
      changeFlag=1;
    }
    encIeLastA = currState;
  }
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
  expiratoryTime = ((cycleTime/(inspiratory_factor+expiratory_factor)*expiratory_factor)-(nSteps/speedMax))*1000.0;
}