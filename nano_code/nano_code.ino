/*
 * cheapStepper_move.ino
 * ///////////////////////////////////////////
 * using CheapStepper Arduino library v.0.2.0
 * created by Tyler Henry, 7/2016
 * ///////////////////////////////////////////
 * 
 * This sketch illustrates the library's
 * "blocking" move functions -
 * i.e. the move will "pause" the arduino sketch
 * -- for non-blocking moves, see cheapStepper_newMoveTo.ino example
 * 
 * This sketch also shows how to set the RPM
 * and shows a few different types of move functions
 * - by steps or by degrees.
 * 
 * Blocking moves are useful if you need a specific RPM
 * but don't need your arduino to perform other functions
 * while the stepper is moving.
 * 
 * //////////////////////////////////////////////////////
 */

// first, include the library :)

#include <CheapStepper.h>

// next, declare the stepper
// and connect pins 8,9,10,11 to IN1,IN2,IN3,IN4 on ULN2003 board

// Declare 3 CheapStepper objects, counting down from pin 13 to pin 2

CheapStepper stepper0 (13, 12, 11, 10);
CheapStepper stepper1 (9, 8, 7, 6);
CheapStepper stepper2 (5, 4, 3, 2); 
 // let's create a boolean variable to save the direction of our rotation

boolean moveClockwise = true;


void setup() {

  // let's set a custom speed of 20rpm (the default is ~16.25rpm)
  
  stepper0.setRpm(20); 
  stepper1.setRpm(20);
  stepper2.setRpm(20);
  /* Note: CheapStepper library assumes you are powering your 28BYJ-48 stepper
   * using an external 5V power supply (>100mA) for RPM calculations
   * -- don't try to power the stepper directly from the Arduino
   * 
   * accepted RPM range: 6RPM (may overheat) - 24RPM (may skip)
   * ideal range: 10RPM (safe, high torque) - 22RPM (fast, low torque)
   */

  // now let's set up a serial connection and print some stepper info to the console
  
  Serial.begin(9600); Serial.println();
  Serial.print(stepper0.getRpm()); // get the RPM of the stepper
  Serial.print(" rpm = delay of ");
  Serial.print(stepper0.getDelay()); // get delay between steps for set RPM
  Serial.print(" microseconds between steps");
  Serial.println();
  
  // stepper0.setTotalSteps(4076);
  /* you can uncomment the above line if you think your motor
   * is geared 63.68395:1 (measured) rather than 64:1 (advertised)
   * which would make the total steps 4076 (rather than default 4096)
   * for more info see: http://forum.arduino.cc/index.php?topic=71964.15
   */
}

void loop() {

    // let's do a clockwise move first
  
    moveClockwise = true;

    // let's move the stepper clockwise to position 2048
    // which is 180 degrees, a half-turn (if using default of 4096 total steps)
    
    stepper0.moveTo (moveClockwise, 2048/2);
    stepper1.moveTo (moveClockwise, 2048/2);
    stepper2.moveTo (moveClockwise, 2048/2);

    // now let's print the stepper position to the console
    
    Serial.print("step position: ");  
    Serial.print(stepper0.getStep()); // get the current step position
    Serial.print(" / 4096");
    Serial.println();

    // now let's wait one second
    
    delay(1000); // wait a sec

    // and now let's move another 90 degrees (a quarter-turn) clockwise

    stepper0.moveDegrees (moveClockwise, 45);
    stepper1.moveDegrees (moveClockwise, 45);
    stepper2.moveDegrees (moveClockwise, 45);
    // stepper0.moveDegreesCW (90); <--- another way to do a clockwise 90 degree turn

    // let's print the stepper position to the console again
    
    Serial.print("step position: ");
    Serial.print(stepper0.getStep());
    Serial.print(" / 4096");
    Serial.println();

    // and wait another second
    
    delay(1000);

    // ok, now let's reverse directions (to counter-clockwise)
    
    moveClockwise = false;

    // and move back to the start position (0 degree)
   
    stepper0.moveToDegree (moveClockwise, 0);
    stepper1.moveToDegree (moveClockwise, 0);
    stepper2.moveToDegree (moveClockwise, 0);
    // moveClockwise is now false, so move counter-clockwise back to start

    // let's print the position to the console once again
    
    Serial.print("step position: ");
    Serial.print(stepper0.getStep());
    Serial.print(" / 4096");
    Serial.println();

    // and wait another second before starting loop() over
    
    delay(1000);  
}
