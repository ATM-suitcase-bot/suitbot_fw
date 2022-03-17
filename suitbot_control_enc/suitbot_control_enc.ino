//A simple script for manual control and mobility development

//Joystick analog pins for teleoperation
int joyX = A0;
int joyY = A1;

//Motor 1/2 PWN and DIR inputs
int m1PWM = 10;
int m2PWM = 11;
int m1DIR = 12;
int m2DIR = 13;

//Motor 1/2 Hall Effect Sensors 1/2
int m1H1 = 2;
int m1H2 = 4;
int m2H1 = 3;
int m2H2 = 5; //blue

//Default neutral value of analog joystick
int neu_val = 495;

//Logged left/right power values for motion smoothing
float last_l = 0.0;
float last_r = 0.0;

//Factor defining degree to which motion is smoothed
float smooth_k = 0.9;

long counterM1 = 0;
long counterM2 = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);

  pinMode(m1PWM, OUTPUT);
  pinMode(m2PWM, OUTPUT);
  pinMode(m1DIR, OUTPUT);
  pinMode(m2DIR, OUTPUT);

  pinMode(m1H1, INPUT);
  pinMode(m1H2, INPUT);
  pinMode(m2H1, INPUT);
  pinMode(m2H2, INPUT);

  attachInterrupt(digitalPinToInterrupt(m1H1), encM1, RISING);
  attachInterrupt(digitalPinToInterrupt(m2H1), encM2, RISING);

  delay(100);
  //Calibrate analog joystick input
  neu_val = analogRead(joyX);

}

void loop() {
  int x_val = analogRead(joyX);
  int y_val = analogRead(joyY);

  float left_out = float(y_val-neu_val) - float(x_val-neu_val)/2;
  float right_out = float(y_val-neu_val) + float(x_val-neu_val)/2;

  left_out = left_out / 1600.0;
  right_out = right_out / 1600.0;

  left_out = smooth_k*last_l + (1.0-smooth_k)*left_out;
  right_out = smooth_k*last_r + (1.0-smooth_k)*right_out;

  int left_pow = int(abs(left_out)*255);
  int right_pow = int(abs(right_out)*255);

  bool left_dir = left_out > 0;
  bool right_dir = right_out < 0;

  digitalWrite(m1DIR, left_dir);
  digitalWrite(m2DIR, right_dir);

  analogWrite(m1PWM, left_pow);
  analogWrite(m2PWM, right_pow);

  Serial.println(String(counterM1) + "\t" + String(counterM2));

  for(int i = 0; i < 1000; i++){ //avoid long delays for encoder purposes
    delayMicroseconds(100);
}
  

  last_l = left_out;
  last_r = right_out;

}

void encM1(){
  if(digitalRead(m1H2)){
    counterM1--;
  }
  else{
    counterM1++;
  }
}

void encM2(){
  if(digitalRead(m2H2)){
    counterM2++;
  }
  else{
    counterM2--;
  }
}
