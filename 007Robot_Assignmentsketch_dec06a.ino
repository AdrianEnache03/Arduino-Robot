//Constants
/*
eprom.get(leftStopValue, 0);
eprom.get(rightStopValue, 1);
eprom.get(leftLDRWhite, 4);
eprom.get(leftLDRBlack, 6);
eprom.get(rightLDRRight, 8);
eprom.get(leftStopValue, 10);
eprom.get(leftStopValue, 12);
eprom.get(leftStopValue, 14);
*/

//LDR constants
int myVariableLeft;
int myVariableMid;
int myVariableRight;

//Left and right push buttons
#define PBL 4 // Left pushbutton pin number
#define PBR 2 // Right pushbutton pin number

//LED constants
const int GREEN = 7;  //Define green 7
const int YELLOW = 12; //Define Yellow 12
const int RED = 13; // Define Red

//Time variables for barcode scanning
unsigned long currentTime;
unsigned long startTime;
unsigned long endTime;
unsigned long diffTime;
unsigned long beginningTime;

unsigned long TimeOne = 0;
unsigned long TimeTwo = 0;
unsigned long TimeThree = 0;
int count = 0;

//Drive constanats
#include <Servo.h>
Servo leftServo;
Servo rightServo;
//Change speed later
const int CurrentSpeed = 1;
const double currentAngleTime = 5.28;

// Colour constants
int black;
int white;


void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT); // Set up left LDR as an input
  pinMode(A1, INPUT); // Set up middle LDR as an input
  pinMode(A2, INPUT); // Set up right LDR as an input

  //IR pin setup
  pinMode(3, OUTPUT); // setup IR LED pin as output

  pinMode(7, OUTPUT); // setup green LED pin as output
  pinMode(12, OUTPUT); // setup yellow LED pin as output
  pinMode(13, OUTPUT); // setup red LED pin as output


  rightServo.attach(6); // pin value assigned for right servo
  leftServo.attach(5); // pin value assigned for left servo


  Serial.begin(9600); // Setup Serial port
  Serial.println(); // Clear an empty line at the top of the monitor
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Please input the coressponding integer with the system you wish to calibrate: ");
  Serial.println("1)  Calibration");
  Serial.println("2)  Complete project (Obstacle detection works really well here)");
  Serial.println("3)  Line follower");
  Serial.println("4)  Barcode scanner");
  //Wait for the user to input a value into the serial monitor.
  while (!Serial.available())
  {
    ;
  }
  // Recieve an integer value from the user. The program then check each option and then runs the coressponding subroutine.
  int input = Serial.parseInt();
  delay(3000);
  lineFollow();

  if (input == 1)
  {
    CalibrationMenu();
  }

  else if (input == 2)
  {
    Serial.println("To exit the complete project, enter any value into the serial monitor.");
    tone(3, 38000); // Start sending IR pulses at 38kHz on pin 3
    while (!Serial.available())
    {


      if (digitalRead(2) == LOW) {
        //Serial.println("Obstacle detected");
        obstacleAvoidance();
      }
      else {
        //Serial.println("No obstacle detected");
        lineFollow();
      }
    }
  }
  else if (input == 3)
  {
    while (!Serial.available())
    {
      lineFollow();
    }
  }
  else if (input == 4)
  {
    while (!Serial.available())
    {
      stripeWidth();
    }
  }
  else
  {
    Serial.println("Invalid input, please enter one of the available digits");
  }

}

//LED changes

void LightPlay(char type)
{
  if (type == 'Y')
  {
    setLEDs(0, 1, 0);
  }
  else if (type == 'G')
  {
    setLEDs(1, 0, 0);
  }
  else
  {
    setLEDs(0, 0, 1);
  }
}

void setLEDs(int green_state, int yellow_state, int red_state) {
  /* Code to switch green LED on or off according to green_state */
  if (green_state == 0)
  {
    digitalWrite(GREEN, LOW);
  }
  else
  {
    digitalWrite(GREEN, HIGH);
  }

  /* Code to switch yellow LED on or off according to yellow_state */
  if (yellow_state == 0)
  {
    digitalWrite(YELLOW, LOW);
  }
  else
  {
    digitalWrite(YELLOW, HIGH);
  }


  /* Code to switch red LED on or off according to red_state */
  if (red_state == 0)
  {
    digitalWrite(RED, LOW);
  }
  else
  {
    digitalWrite(RED, HIGH);
  }
}


// Drive commands

void setSpeed(int leftSpeed, int rightSpeed)
{
  rightServo.write(96 + rightSpeed);
  leftServo.write(95 - leftSpeed);
}
void stop()
{
  setSpeed(0, 0);
}

void moveForward(int dist)
{
  double time = abs(dist / CurrentSpeed);
  time = milliSeconds(time);
  if (dist > 0)
  {
    setSpeed(40, 40);
  }
  else
  {
    setSpeed(-40, -40);
  }
  delay(time);
  stop();
  delay(time);
}

int milliSeconds(double seconds)
{
  return seconds * 1000;
}

void turnSpeed(int angle)
{

  if (angle > 0)
  {
    setSpeed(-40, 40);
  }
  else
  {
    setSpeed(40, -40);
  }
}


void turnAngle(double angle)
{
  double Totaltime = abs(angle) / 360 * currentAngleTime;
  Totaltime = milliSeconds(Totaltime);
  Serial.print(Totaltime);
  turnSpeed(angle);

  delay (Totaltime);
  stop();
  delay(Totaltime);
}


//Barcode scanning
void stripeWidth()
{
  startTime = millis();
  while (((white + 70) <= myVariableLeft <= black && (white + 70) <= myVariableMid <= black && (white + 70) <= myVariableRight <= black) || ((startTime + millis()) < 3000) )
  {
    ;
  }
  if ((white + 70) <= myVariableLeft <= black && (white + 70) <= myVariableMid <= black && (white + 70) <= myVariableRight <= black)
  {
    endTime = millis();
    diffTime = endTime - startTime;
    count++;
    if (count == 1 && TimeOne == 0)
    {
      TimeOne = diffTime;
    }
    else if (count == 2 && TimeTwo == 0)
    {
      TimeTwo  = diffTime;
    }
    else if (count == 3 && TimeThree == 0)
    {
      TimeThree = diffTime;
    }
    else
    {
      Serial.println("You have an error when calculating StripWidth");
    }
    Serial.println("Something was detected");
  }
  else
  {
    Serial.println("Nothing detectecd");
  }
}

void sign()
{
  diffTime = TimeTwo - TimeOne;
  if (count == 3)
  {
    turnAngle(180);
    LightPlay('Y');
  }
  else if (count == 2 && diffTime < 3000)
  {
    //turn right
    LightPlay('G');
    turnAngle(90);

  }
  else if (count == 2)
  {
    //turn left
    LightPlay('R');
    turnAngle(-90);
  }
  else
  {
    Serial.println("Unable to identify sign");
  }
}


void lineFollow()
{
  myVariableLeft = analogRead(A0);
  myVariableMid = analogRead(A1);
  myVariableRight = analogRead(A2);
  setSpeed(10, 10);
  if (myVariableMid > myVariableLeft)
  {
    //Turn left
    turnSpeed(4);


  }
  else if (myVariableMid > myVariableRight)
  {
    // Turn right
    turnSpeed(-5);

  }
  else if (myVariableMid < myVariableLeft && myVariableMid < myVariableRight)
  {
    delay(20);
  }
  else if ((white + 70) <= myVariableLeft <= black && (white + 70) <= myVariableMid <= black && (white + 70) <= myVariableRight <= black )
  {
    stripeWidth();
  }
  else
  {
    Serial.println("Error");
  }

}


void obstacleAvoidance()
{


  //Serial.println(analogRead(2));// For original before problem




  {
    Serial.println("Obstacle detected");
    moveForward(0.10);
    stop();
    turnAngle(90);
    moveForward(0.15);
    turnAngle(-90);
    moveForward(0.17);
    turnAngle(-90);
    moveForward(0.15);
    turnAngle(90);
    setSpeed(40, 40);
  }

  noTone(3); // Turn the IR LED off on pin 3


}

void CalibrationMenu()
{
  // Calibration menu
  Serial.println("Please input the coressponding integer with the system you wish to calibrate: ");
  Serial.println("1)  Left Motor calbiration");
  Serial.println("2)  Right motor calibration");
  Serial.println("3)  LDR Calibration");
  Serial.println("4)  IR Calibration");
  //Wait for the user to input a value into the serial monitor.
  while (!Serial.available())
  {
    ;
  }
  // Recieve an integer value from the user. The program then check each option and then runs the coressponding subroutine.
  int input = Serial.parseInt();
  if (input == 1)
  {
    leftServo.write(86);
    rightServo.write(106);
    findStopLeft();
  }
  else if (input == 2)
  {
    leftServo.write(86);
    rightServo.write(106);
    findStopRight();
  }
  else if (input == 3)
  {
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);

    Serial.println("Please press left button");
    black = LDRCalibrate();
    Serial.println(black);
    Serial.println("Please press right button");
    white = LDRCalibrate();
    Serial.println(white);

  }
  else
  {

    IRView();
  }
}


//Finds the stopping value for the left motor
void findStopLeft()
{
  rightServo.attach(6);
  leftServo.attach(5);

  int value = 75;

  while (!Serial.available())
  {
    rightServo.write(value);
    Serial.println(value);
    delay(1500);
    value++;
  }

}
//Finds the stopping value of right motor

void findStopRight()
{
  int value = 105;
  while (!Serial.available())
  {
    leftServo.write(value);
    delay(1500);
    Serial.println(value);
    value--;
  }

}

// Colour calibration for white and dark surfaces.
int LDRCalibrate()
{
  Serial.println();
  Serial.println("Press left button on black surface: ");
  Serial.println("Press right button on white surface");
  while (!Serial.available())
  {

  }
  if (digitalRead(PBL) == HIGH)
  {
    while (digitalRead(PBL) == HIGH) {} // Wait for press
    while (digitalRead(PBL) == LOW) {} // Wait for release
    myVariableLeft = analogRead(A0);
    myVariableMid = analogRead(A1);
    myVariableRight = analogRead(A2);
    int mean = (myVariableLeft + myVariableMid + myVariableRight) / 3;
    return mean;
  }
  else
  {
    while (digitalRead(PBR) == HIGH) {} // Wait for press
    while (digitalRead(PBR) == LOW) {} // Wait for release
    myVariableLeft = analogRead(A0);
    myVariableMid = analogRead(A1);
    myVariableRight = analogRead(A2);
    int mean = (myVariableLeft + myVariableMid + myVariableRight) / 3;
    return mean;
  }

}

//IR calibration to see if an object is detected
void IRView()
{
  while (!Serial.available())
  {
    tone(3, 38000); // Start sending IR pulses at 38kHz on pin 3
    if (digitalRead(2) == LOW) {
      Serial.println("Obstacle detected");
    }
    else {
      Serial.println("No obstacle detected");
    }
  }
}
