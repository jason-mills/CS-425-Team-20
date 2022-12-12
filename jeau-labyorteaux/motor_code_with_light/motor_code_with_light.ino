#define STEPPER_PIN_1 9
#define STEPPER_PIN_2 10
#define STEPPER_PIN_3 11
#define STEPPER_PIN_4 12
#define LED_PIN 13

int step_number = 0;

// 0 = nonstop
// 1 = fast speed
// 2 = medium speed
// 3 = slow speed
int modeNum = 2;

int spinTime = 3000;
int photoDelay = 2000;
bool stopMotor = false;
float oldtime = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
}

//main loop for the program
void loop() {
  // put your main code here, to run repeatedly
  //make sure to add however much time is needed for the photo delay to the spin buffer
 if ((millis()-oldtime) > (spinTime+photoDelay)) 
 {
  oldtime = millis();
  LED();
 }
 else
 {
  nonStop(4);
 }
}
void LED()
{
 digitalWrite(LED_PIN, HIGH);
 delay(photoDelay);
 digitalWrite(LED_PIN, LOW);
}
void speedFast()
{
  OneStep(false);
  delay(2);
}

void speedMedium()
{
  OneStep(false);
  delay(4);
}

void speedSlow()
{
  OneStep(false);
  delay(8);
}

void nonStop(int delaytime)
{
  OneStep(false);
  delay(delaytime);
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
