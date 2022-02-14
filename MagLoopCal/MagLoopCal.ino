/*
 * Built based on the samples from: https://github.com/teemuatlut/TMCStepper/tree/master/examples
 * Also very much googling and figuring stufff out.
 * 
 * Pre-requisites:
 * Arduino Unoma
 * TMC2130 - make sure to have an open jumper on the big pads, also the tiny jumper pads need to be soldered:
 *    - right - for the upper ones (orient the board so the text on the driver IC is in correct reading position )
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


#define EN_PIN           7 // Enable
#define DIR_PIN          9 // Direction
#define STEP_PIN         8 // Step
#define CS_PIN           10 // Chip select

//works at accel 1000
//#define STALL_VALUE      17 // [-64..63]

#define STALL_VALUE 18

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI

AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

using namespace TMC2130_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

void setup() {
  delay(1000);
  stepper.setEnablePin(EN_PIN);
  SPI.begin();
  Serial.begin(250000);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

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
      
      Serial.print(driver.stallguard());
      Serial.print(" ");
      Serial.println(driver.sg_result(), DEC);
//      Serial.print(" ");
//      Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);
    }

    if (!foundLimit && (stepper.currentPosition() < 500) && driver.stallguard()) {
      foundLimit = true;
      stepper.stop();
      Serial.print(steps, DEC);
      Serial.print(" stall ");
      Serial.println(stepper.targetPosition(), DEC);
    }
  }
  stepper.disableOutputs();

//  // a desperate attempt to get the homing right
//  // but results are mixed, so you can try it, just in case it helps
//  delay(10);
//  stepper.enableOutputs();
//  stepper.setCurrentPosition(0);
//  stepper.moveTo(16);
//  stepper.moveTo(-32);
//  stepper.runToPosition();
//  delay(200);


  stepper.setCurrentPosition(0);
  stepper.disableOutputs();
}

bool isHoming = true;

void loop() {
  // write to terminal "home 0" or "move xx" with a \n (new line character)
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil(' ');
    String strArg = Serial.readStringUntil('\n');

    int arg = strArg.toInt();

    if (cmd == "home") {
      home();
      Serial.println("Homing done");
    }

    else if (cmd == "move") {
      Serial.println("Moving.");
      stepper.enableOutputs();
      stepper.moveTo(arg);
      stepper.runToPosition();
      stepper.disableOutputs();
    }
  }
}
