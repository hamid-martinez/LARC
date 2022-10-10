// Set up for stepper motor and limit switch
const int stepper_step = 25;
const int stepper_dir = 22;
const int stepper_enable = 27;

const float step_angle = 1.8; // from stepper data sheet
const int steps_per_rev = 360 / step_angle;

const int limit_switch = 24;
int limit_state = 0;

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
  pinMode(stepper_enable, OUTPUT);
  pinMode(limit_switch, INPUT);
}

void loop()
{
  read_serial_port();
  
  limit_state = digitalRead(limit_switch);
  Serial.print("Limit state: ");
  Serial.println(limit_state);

  if(limit_switch == HIGH)
  {
    digitalWrite(stepper_enable, LOW);
  }

  if (split_1 == "PU")
  {
    Serial.print("split1: ");
    Serial.print(split_1);
    Serial.print(" ");
    digitalWrite(stepper_enable, LOW);
    digitalWrite(stepper_dir, HIGH);

    user_input = split_2.toInt();
    Serial.print("User: ");
    Serial.print(user_input);
    Serial.println(" ");

    for (int j = 0; j < user_input; j++)
    {
      stepper_one_turn();
    }

    Serial.println("Ouside stepper function");
    digitalWrite(stepper_enable, HIGH);
    split_1 = "";
    split_2 = "";
  }

  else if (split_1 == "PD")
  {
    Serial.print("split1: ");
    Serial.print(split_1);
    Serial.print(" ");
    digitalWrite(stepper_enable, LOW);
    digitalWrite(stepper_dir, LOW);

    user_input = split_2.toInt();
    Serial.print("User: ");
    Serial.print(user_input);
    Serial.println(" ");

    for (int j = 0; j < user_input; j++)
    {
      stepper_one_turn();
    }

    Serial.println("Ouside stepper function");
    digitalWrite(stepper_enable, HIGH);
    split_1 = "";
    split_2 = "";
  }

}

void stepper_one_turn()
{
  Serial.println("Inside stepper function");
  Serial.print("Steps per rev: ");
  Serial.println(steps_per_rev);
  for (int i = 0; i < steps_per_rev; i++) 
  {
    // These four lines result in 1 step:
    digitalWrite(stepper_step, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepper_step, LOW);
    delayMicroseconds(1000);
  }
}

void read_serial_port() 
{
  readString = "";

  if (Serial.available()) 
  {
    delay(10);

    while (Serial.available() > 0) 
    {
      readString += (char)Serial.read();
    }

    comma_index = readString.indexOf(",");
    split_1 = readString.substring(0, comma_index); // "F"
    split_2 = readString.substring(comma_index + 1);  // "360"

    Serial.flush();

  }
}