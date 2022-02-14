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


#define EN_PIN           7 // Enable
#define DIR_PIN          9 // Direction
#define STEP_PIN         8 // Step
#define CS_PIN           10 // Chip select

#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000

#define STALL_VALUE      17 // [-64..63]
#define R_SENSE 0.11f // R Sense resistor value - can be measured on board (the biggest ones)
                      // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI

AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

SoftwareSerial softSerial(4, 5, true); // RX, TX, invert voltage

using namespace TMC2130_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

void setup() {
  delay(1000);
  stepper.setEnablePin(EN_PIN);
  SPI.begin();
  Serial.begin(38400);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

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
    stepper.setAcceleration(1000*120); // 2000mm/s^2
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

  // a desperate attempt to get the homing right
  // seems to help to get repetitive results
  delay(10);
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);
  stepper.moveTo(-32);
  stepper.runToPosition();
  delay(200);

  delay(10);
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);
  stepper.moveTo(-32);
  stepper.runToPosition();
  delay(200);

  delay(10);
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);
  stepper.moveTo(-32);
  stepper.runToPosition();
  delay(200);

  stepper.setCurrentPosition(0);
  stepper.disableOutputs();
}

bool isHoming = true;

long lastFreq = 0;
void loop() {
  softSerial.print("FA;");
  Serial.println("FA;");
  softSerial.flush();
  String response = softSerial.readStringUntil(';');
  Serial.println(response);
  long freq = response.substring(5).toInt();
  Serial.println(freq);
  if (freq == lastFreq) {
    delay(500);
    return;
  }
  
  if (freq >= 7000000l && freq <= 7200000l) {
    // base pos = 3180
    // band steps = 3610 - 3180 = 430
    double a = 430;
    double proportional = (double)(freq - 7000000) / 200000.0;
    double relative = a * proportional;

    //new position
    int pos = 3180 + (int)relative;
    Serial.print("Move to ");Serial.println(pos, DEC);
    stepper.enableOutputs();
    stepper.moveTo(pos);
    stepper.runToPosition();
    stepper.disableOutputs();
  }
  //7000000 .. 3180
  //7200000 .. 3610

  lastFreq = freq;

  //not sure how fast can we poll CAT
  delay(500);
}
