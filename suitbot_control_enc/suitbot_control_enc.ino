/*
 * Main firmware for the Suitbot
 * 
 * 1) Receives velocity commands from the central computer
 * 2) Controls the motors using the velocity commands
 * 3) Sends current encoder readings (velocity) and force reading to the computer
 * 4) Reads battery voltage and sends the value to the computer
 */

#define MAX_VOLTAGE 25.0
#define ENC_REPORT_TIME 100000
#define BAT_REPORT_TIME 1000000
#define MS_2_S 1000000

#define MAX_SPEED 1.0; // max wheel speed. anything greater than this will be capped

#define AUTONOMOUS 0
#define MANUAL 1
#define DEBUG 2

//Joystick analog pins for teleoperation
int joyX = A0;
int joyY = A1;
int pressure1 = A2;
int pressure2 = A3;
int voltageSense = A4;

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
float smooth_k = 0.99;

long counterM1 = 0;
long counterM2 = 0;

long last_time = micros();
long last_time_voltage = micros();

float enc_2_m = 0.0025;// 0.0007; //encoder tick to meter travel conversion
float wb_m = 0.23; //wheelbase in meters

int last_c1 = 0;
int last_c2 = 0;

float v_forward = 0.0;
float v_angular = 0.0;

float angular_in = 0.0;
float velocity_in = 0.0;

//PD constants for velocity control
float k_p = 1.5;
float k_i = 0.04;
float k_d = 0.0;

float last_error_v = 0.0;
float last_error_ang = 0.0;

float int_error_v = 0.0;
float int_error_ang = 0.0;

float left_out = 0.0;
float right_out = 0.0;

bool is_debugging = false;

int mode = AUTONOMOUS;

void setup() {
  mode = AUTONOMOUS;
  //57600 baud serial comms
  Serial.setTimeout(50);
  Serial.begin(115200);

  //Init pinmodes Input/Output
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);

  pinMode(voltageSense, INPUT);

  pinMode(m1PWM, OUTPUT);
  pinMode(m2PWM, OUTPUT);
  pinMode(m1DIR, OUTPUT);
  pinMode(m2DIR, OUTPUT);

  pinMode(m1H1, INPUT);
  pinMode(m1H2, INPUT);
  pinMode(m2H1, INPUT);
  pinMode(m2H2, INPUT);


  //Attache interrupts for handling encoders
  attachInterrupt(digitalPinToInterrupt(m1H1), encM1, RISING);
  attachInterrupt(digitalPinToInterrupt(m2H1), encM2, RISING);

  //Pause for any electric transients to die down
  delay(100);
  //Calibrate analog joystick input 'neutral' value
  neu_val = analogRead(joyX);

}

void loop() {
  int voltage_val = analogRead(voltageSense);

  if (Serial.available() > 0) { //received velocity command from nuc
    // format: "a\tb"
    // where a is linear velocity and b is angular velocity
    String strIn = Serial.readString(); 

    // Process received string
    int n = strIn.length();
    char char_array[n + 1];
    strcpy(char_array, strIn.c_str());
    char * pch;
    pch = strtok(char_array,"\t");
    if (pch != NULL)
    {
      String x1_str(pch);
      velocity_in = x1_str.toFloat();
      pch = strtok(NULL, "\t");
      if (pch != NULL)
      {
        String x2_str(pch);
        angular_in = x2_str.toFloat();
        pch = strtok(NULL, "\t");
        if (pch != NULL)
        {
          String x3_str(pch);
          int mode_in = x3_str.toInt();
          if (mode_in == 1) {
            if (mode == AUTONOMOUS){
              reset_params(MANUAL);
            }
          }
          else if (mode == 0) {
            if (mode == MANUAL){
              reset_params(AUTONOMOUS);
            }
          }
        }
        
      }
    }
    
  }

  if (mode == AUTONOMOUS) {
    //Apply smoothing (anti-jerk control) to motor outputs
    float power_left_out = smooth_k*last_l + (1.0-smooth_k)*left_out;
    float power_right_out = smooth_k*last_r + (1.0-smooth_k)*right_out;
  
    //Convert floats to PWM power and boolean direction
    power_left_out = min(max(power_left_out, -1.0), 1.0);
    power_right_out = min(max(power_right_out, -1.0), 1.0);
    int left_pow = int(abs(power_left_out)*255);
    int right_pow = int(abs(power_right_out)*255);
    bool left_dir = power_left_out > 0;
    bool right_dir = power_right_out < 0;
  
    //Write motor powers
    digitalWrite(m1DIR, left_dir);
    digitalWrite(m2DIR, right_dir);
    analogWrite(m1PWM, left_pow);
    analogWrite(m2PWM, right_pow);

    //Log previous power output values
    last_l = power_left_out;
    last_r = power_right_out;
  }
  else if (mode == MANUAL) {
    teleop_control();
  }


  /* Report sensor values to PC */
  long microsec = micros();
  //long microsec_u = (unsigned long)microsec;
  int t_sec = floor(microsec / MS_2_S);
  long t_nano = long(microsec%MS_2_S)*1000;

    
  //Serial output encoder values- at most every 100 ms
  if(microsec - last_time > ENC_REPORT_TIME){

    float force1 = float(analogRead(pressure1)) * 50.0 / 1023.0;
    float force2 = float(analogRead(pressure2)) * 50.0 / 1023.0;
    
    
    int diff_1 = counterM1 - last_c1;
    int diff_2 = counterM2 - last_c2;
    float dt = float(micros() - last_time)/float(MS_2_S);
    float v_x = float(diff_1 + diff_2)*enc_2_m/(dt);
    float a_v_z = float(diff_1 - diff_2)*enc_2_m/(dt*wb_m);

    //log previous encoder vals
    last_c1 = counterM1;
    last_c2 = counterM2;
    last_time = micros();

    String enc_str = "encoder\t"+String(t_sec)+"\t"+String(t_nano)+"\t"+ 
                               String(v_x)+"\t"+String(a_v_z);
    Serial.println(enc_str);

    String force_str = "force\t"+String(t_sec)+"\t"+String(t_nano)+"\t"+ 
                               String(force1, 4)+"\t"+String(force2, 4);
    Serial.println(force_str);
    
    if (mode == AUTONOMOUS){
  
      //Calculate error on velocity and angular commands
      float error_v = velocity_in - v_x;
      float error_ang = angular_in - a_v_z;
  
      //Calculate power based on errors and derivative of errors
      float power_v = -1*(error_v*k_p+(error_v-last_error_v)/dt*k_d);
      power_v = power_v - int_error_v*k_i;
      float power_ang = -1*(error_ang*k_p+(error_ang-last_error_ang)/dt*k_d);
      power_ang = power_ang - int_error_ang*k_i;
  
      //PID constants should be reduced for radial control
      power_ang = power_ang/4.0;
  
      left_out = power_v + power_ang;
      right_out = power_v - power_ang;
  
      last_error_v = error_v;
      last_error_ang = error_ang;
  
      //accumulate but remove windup on integral terms
      int_error_v = int_error_v + error_v;
      int_error_v = min(max(int_error_v, -20), 20);
      int_error_ang = int_error_ang + error_ang;
      int_error_ang = min(max(int_error_ang, -20), 20);
    }
  }
  else if(micros() < last_time){ //Reset all counters upon overflow
    last_time = micros();
    last_c1 = counterM1;
    last_c2 = counterM2;
  }

  if(microsec - last_time_voltage > BAT_REPORT_TIME){ // report battery value every 1s
    int sensorValue = analogRead(A0);
    float voltage = float(sensorValue) * (MAX_VOLTAGE / 1023.0);
    String voltage_str = "voltage\t"+String(t_sec)+"\t"+String(t_nano)+"\t"+ 
                                          String(voltage, 4);
    Serial.println(voltage_str);
    last_time_voltage = micros();
  }
  else if (micros() < last_time_voltage){ // handle overflow
    last_time_voltage = micros();
  }
  
  //avoid long delays for encoder callback purposes
  delayMicroseconds(100);
}

//Callback functions for reading encoders
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

void teleop_control() {
  //Get joystick values (useful only in teleoperation mode)
  int x_val = analogRead(joyX);
  int y_val = analogRead(joyY);
  
  float left_out = float(y_val-neu_val) - float(x_val-neu_val)/3;
  float right_out = float(y_val-neu_val) + float(x_val-neu_val)/3;

  left_out = left_out / 1800.0;
  right_out = right_out / 1800.0;

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

  //delay(20);

  last_l = left_out;
  last_r = right_out;
}

void debug_control() {
  float left_out = 0.5;
  float right_out = 0.5;

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

  //delay(20);

  last_l = left_out;
  last_r = right_out;
}


void reset_params(int mode_in){
  mode = mode_in;

  last_l = 0.0;
  last_r = 0.0;
  
  last_time = micros();
  last_time_voltage = micros();
  
  v_forward = 0.0;
  v_angular = 0.0;
  
  angular_in = 0.0;
  velocity_in = 0.0;
  
  last_error_v = 0.0;
  last_error_ang = 0.0;
  
  int_error_v = 0.0;
  int_error_ang = 0.0;
  
  left_out = 0.0;
  right_out = 0.0;
}
