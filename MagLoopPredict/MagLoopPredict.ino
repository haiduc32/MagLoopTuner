/*
 * Built based on the samples from: https://github.com/teemuatlut/TMCStepper/tree/master/examples
 * Also very much googling and figuring stufff out.
 * 
 * This is configured for Elecraft radios. 
 * It might be enough just to change the Serial speed to accomodate other radios.
 * 
 * Its configured to home turning counter clock wise. If it's not turning CCW on start, switch stepper cable.
 * 
 * Pre-requisites:
 * TMC2130 - make sure to have an open jumper on the big pads, also the tiny jumper pads need to be soldered:
 *    - right - for the upper ones (considering the chip orientation)
 *    - left - for the lower ones
 * NEMA17 motor model: 
 * Motor wire length: 1m
 * (if you choose a different stepper model or different wire length you might neeed to adjust STALL_VALUE)
 * Arduino Uno (clone works just fine)
 * Pin Wiring: TODO
 * Compiled in Arduino IDE
 */

#include <TMCStepper.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>


#define EN_PIN           7 // Enable
#define DIR_PIN          9 // Direction
#define STEP_PIN         8 // Step
#define CS_PIN           10 // Chip select
#define BTN              19 // the button.. (A5 pin)

//just a guess as the maximum number of data points you'll need
#define MAX_DATA_POINTS_PER_BAND 20

#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000

#define STALL_VALUE      18 // [-64..63]
#define R_SENSE 0.11f // R Sense resistor value - can be measured on board (the biggest ones)
                      // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

#define MAGIC 0x4444 // a magic number to write to/read from EEPROM so we know 
                     // that it's populated and not random data
#define MAX_LONG 2147483647L

struct DataPoint {
  int pos;
  long freq;
};

// Select your stepper driver type
TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI

AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

SoftwareSerial softSerial(4, 5, true); // RX, TX, invert voltage

using namespace TMC2130_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

bool calibration = false;
int calibPoints = 0;
DataPoint *calib;

void setup() {
  pinMode(BTN, INPUT_PULLUP);
  delay(1);
  bool btnState = digitalRead(BTN);
  if (btnState == LOW) {
    calibration = true;
  } else {
    //load all calibration data points
    int magic;
    EEPROM.get(0, magic);
    if (magic == MAGIC) {
      EEPROM.get(2, calibPoints);
      calib = malloc(sizeof(DataPoint)*calibPoints);
      for (int i = 0; i < calibPoints; i++) {
        EEPROM.get(4+sizeof(DataPoint)*i, calib[i]);
      }
    }
  }
  
  delay(1000);
  stepper.setEnablePin(EN_PIN);
  SPI.begin();
  Serial.begin(38400);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  if (calibration) Serial.println("Calibration mode");

  softSerial.begin(38400);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(16);
  //driver.TCOOLTHRS(0); 
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);
  driver.diag1_stall(true);
  driver.diag1_pushpull(true);

//    driver.toff(3);
//    driver.tbl(1);
//    driver.hysteresis_start(4);
//    driver.hysteresis_end(-2);
//    driver.microsteps(16);
////    driver.coolstep_min_speed(0xFFFFF); // 20bit max

    stepper.setMaxSpeed(2000); // 100mm/s @ 80 steps/mm
    stepper.setAcceleration(200*120); // 2000mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(true, false, true);
    //TODO: test that works ok with removing the next two lines
    stepper.enableOutputs();
    delay(1);
    stepper.disableOutputs();
    delay(1);
    home();
}

//find home position
void home() {
  bool foundLimit = false;
  
  stepper.enableOutputs();
  int steps = 0;
  stepper.moveTo(-200*16*4);
  while (stepper.isRunning() ) {
    for (int i = 0; i < 16*3; i++) {
      stepper.run();
      steps++;
    }

    static uint32_t last_time=0;
    uint32_t ms = millis();
    if((ms-last_time) > 100) { //run every 0.1s
      last_time = ms;
//      DRV_STATUS_t drv_status{0};
//      drv_status.sr = driver.DRV_STATUS();
      
//      Serial.print(driver.stallguard());
//      Serial.print(" ");
//      Serial.println(driver.sg_result(), DEC);
//      Serial.print(" ");
//      Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);
    }

    if (!foundLimit && (stepper.currentPosition() < 500) && driver.stallguard()) {
      foundLimit = true;
      stepper.stop();
//      Serial.print(steps, DEC);
//      Serial.print(" stall ");
//      Serial.println(stepper.targetPosition(), DEC);
    }
  }
  stepper.disableOutputs();

//  // a desperate attempt to get the homing right
//  // seems to help to get repetitive results
//  delay(10);
//  stepper.enableOutputs();
//  stepper.setCurrentPosition(0);
//  stepper.moveTo(-32);
//  stepper.runToPosition();
//  delay(200);
//
//  delay(10);
//  stepper.enableOutputs();
//  stepper.setCurrentPosition(0);
//  stepper.moveTo(-32);
//  stepper.runToPosition();
//  delay(200);
//
//  delay(10);
//  stepper.enableOutputs();
//  stepper.setCurrentPosition(0);
//  stepper.moveTo(-32);
//  stepper.runToPosition();
//  delay(200);

  stepper.setCurrentPosition(0);
  stepper.disableOutputs();

  calibration = true;
}

long lastFreq = 0;
void loop() {
  if (calibration) {
    calibrate();
  }

  //to avoid errors, do nothing if there aren't at least 2 data points
  if (calibPoints < 2) {
    delay(1000);
    return;
  }
  
  long freq = readFreq();
  
  
  if (freq == lastFreq) {
    delay(500);
    return;
  }

  //find 2 datapoints that the frequency is in between
  DataPoint foundLower = findLowerDataPoint(freq);
  DataPoint foundHigher = findHigherDataPoint(freq);

  //check that both DPs have been found - 0 is not found
  if (foundLower.pos > 0 && foundHigher.pos > 0) {
    int dpBandwidth = foundHigher.freq - foundLower.freq;
    int steps = foundHigher.pos - foundLower.pos;
    double proportional = (double)dpBandwidth/steps;
    int newPos = (freq-foundLower.pos)/proportional;

    //TODO: correct for mechanical play
    moveTo(newPos);
  }

  //not sure how fast can we poll CAT
  delay(500);
}

void calibrate() {
  int dp = 0;
  DataPoint dataPoints[MAX_DATA_POINTS_PER_BAND];

  for (int i = 0;;i+=50) {
    int pos = i;
    moveTo(pos);

    //now we wait for the BTN to be pressed (LOW state), 
    //short for next, and long for save data point, longer for save
    bool btn = HIGH;
    do  {
      delay (100);
      btn = digitalRead(BTN);
    } while (btn);
    
    unsigned long pressTime = millis();
    //now wait till the button is depressed (HIGH state)
    do  {
      delay (100);
      btn = digitalRead(BTN);
    } while (!btn);

    //saving 4 bytes of ram..
    pressTime = millis() - pressTime;

    if (pressTime < 1000) {
      //short press - continue to next position
    } else if (pressTime < 2000) {
      // add data point
      Serial.println("Adding data point");
      dataPoints[dp].pos = i;
      dataPoints[dp].freq = readFreq();
    } else {
      //save band data and reset to 0 position
      //save total number of data points at address 0
      for (int j = 0; j < dp; j++) {
        int addr = 4 + calibPoints * sizeof(DataPoint);
        EEPROM.put(addr, dataPoints[j]);
        calibPoints++;
      }

      //write the calibration points count
      EEPROM.put(0, MAGIC);
      EEPROM.put(2, calibPoints);

      //finally, time to reset to position 0
      moveTo(0);
      break;
    }
  }
}

void moveTo(int pos) {
  Serial.print("Move to ");Serial.println(pos, DEC);
  stepper.enableOutputs();
  stepper.moveTo(pos);
  stepper.runToPosition();
  stepper.disableOutputs();
}

long readFreq() {
  softSerial.print("FA;");
  Serial.println("FA;");
  softSerial.flush();
  String response = softSerial.readStringUntil(';');
  Serial.println(response);
  long freq = response.substring(5).toInt();
  Serial.println(freq);

  return freq;
}

DataPoint findLowerDataPoint(long freq) {
  DataPoint foundDp;
  foundDp.pos = 0;
  for (int i = 0; i < calibPoints; i++) {
    DataPoint dp = calib[i];
    if (dp.freq <= freq && 
        //check that it's not more then 500khz off - sanity check
        (dp.freq + 5000000L > freq) && 
        dp.pos > foundDp.pos) {
          foundDp = dp;
        }
  }
  return foundDp;
}

DataPoint findHigherDataPoint(long freq) {
  DataPoint foundDp;
  foundDp.pos = MAX_LONG;
  for (int i = 0; i < calibPoints; i++) {
    DataPoint dp = calib[i];
    if (dp.freq >= freq && 
        //check that it's not more then 500khz off - sanity check
        (dp.freq - 5000000L < freq) && 
        dp.pos < foundDp.pos) {
          foundDp = dp;
        }
  }

  //check if any dp has been found, if not, reset it's pos to 0
  if (foundDp.pos == MAX_LONG) foundDp.pos = 0;
  return foundDp;
}
