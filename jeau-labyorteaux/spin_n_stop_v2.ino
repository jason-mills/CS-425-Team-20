#define STEPPER_PIN_1 9
#define STEPPER_PIN_2 10
#define STEPPER_PIN_3 11
#define STEPPER_PIN_4 12

int step_number = 0;
bool stopMotor = false;

void OneStep(bool dir);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
}

//main loop for the program
void loop() {

if(Serial.available() > 0)
{
    char command = Serial.read();
    Serial.print(command);
    
    // 30 Degrees
    if(command == 'A')
    {
      for(int i = 0; i < 170; i++)
      {
        OneStep(false);
        delay(4);
      }
    }

    // 45 Degrees
    else if(command == 'B')
    {
      for(int i = 0; i < 256; i++)
      {
        OneStep(false);
        delay(4);
      }
    }

    // 60 Degrees
    else if(command == 'C')
    {
      for(int i = 0; i < 341; i++)
      {
        OneStep(false);
        delay(4);
      }
    }

    // 90 Degrees
    else if(command == 'D')
    {
      for(int i = 0; i < 512; i++)
      {
        OneStep(false);
        delay(4);
      }
    }

    // 180 Degrees
    else if(command == 'E')
    {
      for(int i = 0; i < 1024; i++)
      {
        OneStep(false);
        delay(4);
      }
    }
  }
  
}
 


void OneStep(bool dir)
{
  if(dir)
  {
    switch(step_number)
    {
      case 0: 
      digitalWrite(STEPPER_PIN_1, HIGH);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, LOW);
      break;
      case 1: 
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, HIGH);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, LOW);
      break;
      case 2: 
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, HIGH);
      digitalWrite(STEPPER_PIN_4, LOW);
      break;
      case 3: 
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, HIGH);
      break;
    }
  }
  else
  {
    switch(step_number)
    {
      case 0: 
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, HIGH);
      break;
      case 1: 
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, HIGH);
      digitalWrite(STEPPER_PIN_4, LOW);
      break;
      case 2: 
      digitalWrite(STEPPER_PIN_1, LOW);
      digitalWrite(STEPPER_PIN_2, HIGH);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, LOW);
      break;
      case 3: 
      digitalWrite(STEPPER_PIN_1, HIGH);
      digitalWrite(STEPPER_PIN_2, LOW);
      digitalWrite(STEPPER_PIN_3, LOW);
      digitalWrite(STEPPER_PIN_4, LOW);
      break;
    }
  }
  step_number++;
  if (step_number > 3)
  {
    step_number = 0;
  }
}
