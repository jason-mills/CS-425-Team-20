#define STEPPER_PIN_1 9
#define STEPPER_PIN_2 10
#define STEPPER_PIN_3 11
#define STEPPER_PIN_4 12

int step_number = 0;
bool stopMotor = false;

void OneStep(bool dir);

int calculate_degrees(char data_array[])
{
int total = 0;
int multiplier = 100;
for(int i = 0; i < 3; i++)
{
 switch (data_array[i]) {
    case 'a':
        total = total + (0 * multiplier);
        break;
    case 'b':
        total = total + (1 * multiplier);
        break;
    case 'c':
        total = total + (2 * multiplier);
        break;
    case 'd':
        total = total + (3 * multiplier);
        break;
    case 'e':
        total = total + (4 * multiplier);
        break;
    case 'f':
        total = total + (5 * multiplier);
        break;
    case 'g':
        total = total + (6 * multiplier);
        break;
    case 'h':
        total = total + (7 * multiplier);
        break;
    case 'i':
        total = total + (8 * multiplier);
        break;
    case 'j':
        total = total + (9 * multiplier);
        break;
    default:
        Serial.print("default");
        break;
}
multiplier = multiplier/10;
}
return total;
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
}

//main loop for the program
void loop() 
{
char data_array[3];
int iterator = 0;
bool LOOP = true;
while(LOOP)
{
  if(Serial.available() > 0)
  {
    char part_degrees = Serial.read();
    if (part_degrees == 'k')
    {
      LOOP = false;
    }
    else if (part_degrees != '\n')
    {
      data_array[iterator] = part_degrees;
      iterator++;
    }
  }
}

int degrees_to_rotate = calculate_degrees(data_array);
Serial.print(degrees_to_rotate);
for(int i = 0; i < degrees_to_rotate*5.68; i++)
{
  OneStep(false);
  delay(4);
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
