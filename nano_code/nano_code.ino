
/*
 * cheapStepper_newMoveTo.ino
 * ///////////////////////////////////////////
 * using CheapStepper Arduino library v.0.2.0
 * created by Tyler Henry, 7/2016
 * ///////////////////////////////////////////
 * 
 * This sketch illustrates the library's
 * "non-blocking" move functions -
 * i.e. you can perform moves with the stepper over time
 * while still running other code in your loop()
 * 
 * This can be useful if your Arduino is multi-tasking,
 * but be careful: if the other code in your loop()
 * slows down your Arduino, the stepper motor may
 * slow down or move with a stutter
 * 
 * //////////////////////////////////////////////////////
 */

// first, include the library :)

#include <CheapStepper.h>

using namespace std;

// next, declare the stepper objects

CheapStepper stepper0 (13,12,11,10); // stepper0 is connected to pins 8,9,10,11
CheapStepper stepper1 (9,8,7,6); // stepper1 is connected to pins 8,9,10,11
CheapStepper stepper2 (5,4,3,2); // stepper2 is connected to pins 8,9,10,11


 // let's also create a boolean variable to save the direction of our rotation
 // and a timer variable to keep track of move times

bool moveClockwise = true;
unsigned long moveStartTime = 0; // this will save the time (millis()) when we started each new move
//float q[300][3] = {};

// Joint angle array
void setup() {

  // let's run the stepper at 12rpm (if using 5V power) - the default is ~16 rpm

  stepper0.setRpm(12);
  stepper1.setRpm(12);
  stepper2.setRpm(12);

  // let's print out the RPM to make sure the setting worked
  
  Serial.begin(9600);
//  Serial.print("stepper RPM: "); Serial.print(stepper.getRpm());
  Serial.println();

  while (!Serial) {
    delay(100); // wait for serial port to connect. Needed for native USB port only
  }

  // and let's print the delay time (in microseconds) between each step
  // the delay is based on the RPM setting:
  // it's how long the stepper will wait before each step

//  Serial.print("stepper delay (micros): "); Serial.print(stepper.getDelay());
  Serial.println();

  // now let's set up our first move...
  // let's move a half rotation from the start point

  // stepper0.newMoveTo(moveClockwise, 2048/4);
  // stepper1.newMoveTo(moveClockwise, 2048/4);
  // stepper2.newMoveTo(moveClockwise, 2048/4);

  /* this is the same as: 
   * stepper.newMoveToDegree(clockwise, 180);
   * because there are 4096 (default) steps in a full rotation
   */
  moveStartTime = millis(); // let's save the time at which we started this move
  
}

float * processReceivedData(String data);
float * q = {};
void loop() {

  if (Serial.available() > 0) {
    
    // Read the incoming data
    String receivedData = Serial.readStringUntil('\n');
    
    // Process the received data (assuming it's comma-separated)
    q = processReceivedData(receivedData);
  }

  // we need to call run() during loop() 
  // in order to keep the stepper moving
  // if we are using non-blocking moves
  
  stepper0.run();
  stepper1.run();
  stepper2.run();

  ////////////////////////////////
  // now the stepper is moving, //
  // let's do some other stuff! //
  ////////////////////////////////

  // let's check how many steps are left in the current move:
  
  int stepsLeft = stepper0.getStepsLeft();

  // if the current move is done...
  
  if (stepsLeft == 0){

    // let's print the position of the stepper to serial
    
    Serial.print("stepper position: "); Serial.print(stepper0.getStep());
    Serial.println();

    // and now let's print the time the move took

    unsigned long timeTook = millis() - moveStartTime; // calculate time elapsed since move start
    Serial.print("move took (ms): "); Serial.print(timeTook);
    Serial.println(); Serial.println();
    
    // let's start a new move in the reverse direction
    
    moveClockwise = !moveClockwise; // reverse direction
    
    stepper0.newMoveDegrees (moveClockwise, 45); // move 180 degrees from current position
    stepper1.newMoveDegrees (moveClockwise, 45); // move 180 degrees from current position
    stepper2.newMoveDegrees (moveClockwise, 45); // move 180 degrees from current position

    moveStartTime = millis(); // reset move start time

  }

}

float * processReceivedData(String data) {
  // Split the received data into individual elements
  String values[900];  // Assuming maximum of 300 rows and 3 columns
  int i = 0;

  while (data.length() > 0) {
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      values[i] = data.substring(0, commaIndex);
      data = data.substring(commaIndex + 1);
    }
    else {
      values[i] = data;
      data = "";
    }
    i++;
  }

  // Convert string values to floats and reconstruct the original 2D array
  float receivedArray[300][3];

  for (int row = 0; row < 300; row++) {
    for (int col = 0; col < 3; col++) {
      receivedArray[row][col] = values[row * 3 + col].toFloat();
    }
  }

  // Now you can use the 'receivedArray' for further processing
  // Add your code here to use the received data as needed
  return receivedArray;
}
