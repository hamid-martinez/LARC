#include <ezButton.h>
// Set up for stepper motor and limit switch
const int stepper_step = 2;
const int stepper_dir = 1;

const int steps_per_rev = 200;

// User input and communication variables
String readString;
String split_1;
String split_2;
int comma_index;

void setup()
{
    Serial.begin(9600);

    pinMode(stepper_step, OUTPUT);
    pinMode(stepper_dir, OUTPUT);
}

void loop()
{
    while(Serial.available())
    {
        delay(50);
        char c = Serial.read();
        readString += c;
    }

    // Create a bigger input string to take commands for all motors: M190,M2180,M375,M4130
    // Separte the string to create the different targets of each motor
    comma_index = readString.indexOf(",");
    split_1 = readString.substring(0, comma_index); // "F"
    split_2 = readString.substring(comma_index + 1);  // "360"

    if(split_1 == "RU")
  {
    // Set direction of both motors
    digitalWrite(stepper_dir, HIGH);

    // Call function to move the motors by how long the button is pressed
    move_motors_fast();
  }

}

void move_motors_fast()
{
  digitalWrite(stepper_step, HIGH);
  digitalWrite(stepper_step, LOW);
  delayMicroseconds(5000);  // This number determines the speed of the motors, the higher the slower the motor gets
}

void move_motors_same_dir_slow()
{
  digitalWrite(stepper_dir, LOW);

  digitalWrite(stepper_step, HIGH);
  delayMicroseconds(9000);
  digitalWrite(stepper_step, LOW);
  delayMicroseconds(9000);
}