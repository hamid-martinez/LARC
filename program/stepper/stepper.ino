// Set up for stepper motor and limit switch
const int stepper_step = 25;
const int stepper_dir = 22;
const int step_angle = 1.8; // from stepper data sheet
const int steps_per_rev = 360 / step_angle;

const int limit_switch = 24;
bool k;

// User input and communication variables
String readString;
String split_1;
String split_2;
int comma_index;
int user_input;

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

  if (split_1 == "STUS")
  {
    digitalWrite(stepper_dir, HIGH);
    user_input = split_2.toInt();
    Serial.println(user_input);

    while (k==true)
    {
      move_stepper();
    }


    split_1 = "";
    split_2 = "";
    user_input = 0;
    Serial.println(split_1);
  }
  else
  {
    Serial.println("Doing nothing");
  }

}

void move_stepper()
{
  for (int i = 0; i < user_input; i++) 
    {
      // These four lines result in 1 step:
      digitalWrite(stepper_step, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepper_step, LOW);
      delayMicroseconds(1000);
    }
}