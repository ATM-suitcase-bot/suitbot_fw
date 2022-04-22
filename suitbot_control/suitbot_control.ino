int joyX = A0;
int joyY = A1;

int m1PWM = 10;
int m2PWM = 11;
int m1DIR = 12;
int m2DIR = 13;

int neu_val = 495;

float last_l = 0.0;
float last_r = 0.0;

float smooth_k = 0.9;

void setup() {
  Serial.begin(9600);
  
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);

  pinMode(m1PWM, OUTPUT);
  pinMode(m2PWM, OUTPUT);
  pinMode(m1DIR, OUTPUT);
  pinMode(m2DIR, OUTPUT);

  delay(100);
  neu_val = analogRead(joyX);

}

void loop() {

  int x_val = analogRead(joyX);
  int y_val = analogRead(joyY);

  float left_out = float(y_val-neu_val) - float(x_val-neu_val)/3;
  float right_out = float(y_val-neu_val) + float(x_val-neu_val)/3;

  left_out = left_out / 1200.0;
  right_out = right_out / 1200.0;

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

  delay(20);

  last_l = left_out;
  last_r = right_out;

}
