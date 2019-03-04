//the motor control in arduino mega 2560 used PID
//by srduitno 2018.10.30  No.000001
#include <Arduino.h>      //arduino library in platformio
#include <DueTimer.h>     //定时器库的头文件
#include <Coordinate.h>
#include <GcodeReading.h>
#define MOTOR_AMOUNT 3
#define MOTOR_1_A_PIN 22
#define MOTOR_2_A_PIN 24
#define MOTOR_3_A_PIN 26 
#define MOTOR_1_B_PIN 23
#define MOTOR_2_B_PIN 25
#define MOTOR_3_B_PIN 27

float fmoving_range = 1200;//边长定义待定义
float fmoving_height = 1000;//高度定义待定义（单位毫米）

float fxyzs[3][3] = {{-fmoving_range*pow(3,2)/2,fmoving_range/2,fmoving_height}
                     ,{fmoving_range*pow(3,2)/2,fmoving_range/2,fmoving_height}
                     ,{0,fmoving_range,fmoving_height}};
//int fxyzs[0][3] = {-fmoving_range*pow(3,2)/2,fmoving_range/2,fmoving_height};
//int fxyzs[1][3] = {fmoving_range*pow(3,2)/2,fmoving_range/2,fmoving_height};
//int fxyzs[2][3] = {0,fmoving_range,fmoving_height};//定义三个坐标基准点的坐标

float fma_goal = 0;
float fmb_goal = 0;
float fmc_goal = 0;//三个电机的目标收放量
//High applicability lines drive robot Coordinate calculation
float intent_coordinate[3] = {30,30,30}; //(x,y,z)目标位置
float current_coordinate[3] = {0,0,80}; //(x,y,z)too当前位置
//the coordinate input for HBldR
int system_time = 0;
int P_control_pins[] = {28,30,32};
int S_control_pins[] = {29,31,33};
int pwm_control_output[] = {2,3,4};

//P正转控制接线集合
//S反转控制接线集合
//Pwm控制接线集合

static float Bias[MOTOR_AMOUNT],
             Pwmoutput[MOTOR_AMOUNT],
             Intergral_bias[MOTOR_AMOUNT],
             Last_bias[MOTOR_AMOUNT];//pid的各项定义组
#define PID_debug 6
float P = 9;
float I = 0.1;
float D = 14;
int Endcoder[MOTOR_AMOUNT];//电机位置量
int Target[] = {-400,-400,-400};//电机的目标容器，好像也不是容器随便啦

void system_time_clock() {
system_time ++;
}

void motor_1_speed_measurement() { //电机脉冲检测中断
   if(digitalRead(MOTOR_1_A_PIN) == 1)
   {
     if(digitalRead(MOTOR_1_B_PIN) == 1)
     {
       Endcoder[0]++;
     }
     else if(digitalRead(MOTOR_1_B_PIN) == 0)
     {
       Endcoder[0]--;
     }
   }
   else if(digitalRead(MOTOR_1_A_PIN) == 0)
   {
     if(digitalRead(MOTOR_1_B_PIN) == 0)
     {
       Endcoder[0]++;
     }
     else if(digitalRead(MOTOR_1_B_PIN) == 1)
     {
       Endcoder[0]--;
     }
   }
}

void motor_2_speed_measurement() { //电机脉冲检测中断
   if(digitalRead(MOTOR_2_A_PIN) == 1)
   {
     if(digitalRead(MOTOR_2_B_PIN) == 1)
     {
       Endcoder[1]++;
     }
     else if(digitalRead(MOTOR_2_B_PIN) == 0)
     {
       Endcoder[1]--;
     }
   }
   else if(digitalRead(MOTOR_2_A_PIN) == 0)
   {
     if(digitalRead(MOTOR_2_B_PIN) == 0)
     {
       Endcoder[1]++;
     }
     else if(digitalRead(MOTOR_2_B_PIN) == 1)
     {
       Endcoder[1]--;
     }
   }
}

void motor_3_speed_measurement() { //电机脉冲检测中断
   if(digitalRead(MOTOR_3_A_PIN) == 1)
   {
     if(digitalRead(MOTOR_3_B_PIN) == 1)
     {
       Endcoder[2]++;
     }
     else if(digitalRead(MOTOR_3_B_PIN) == 0)
     {
       Endcoder[2]--;
     }
   }
   else if(digitalRead(MOTOR_3_A_PIN) == 0)
   {
     if(digitalRead(MOTOR_3_B_PIN) == 0)
     {
       Endcoder[2]++;
     }
     else if(digitalRead(MOTOR_3_B_PIN) == 1)
     {
       Endcoder[2]--;
     }
   }
}

void PID_debug_setup() {
  
}

void motor_PID_control() {
  for(int i = 0; i < MOTOR_AMOUNT; i++) {
    Bias[i] = Endcoder[i] - Target[i];
    Intergral_bias[i] += Bias[i]; 
    Pwmoutput[i] = P * Bias[i] + I * Intergral_bias[i] + D * (Bias[i] - Last_bias[i]);
    Last_bias[i] = Bias[i];
    if(Pwmoutput[i] >= 0){
      digitalWrite(P_control_pins[i],HIGH);
      digitalWrite(S_control_pins[i],LOW);
      analogWrite(pwm_control_output[i],Pwmoutput[i]);     
    }
    if(Pwmoutput[i] < 0){
      digitalWrite(P_control_pins[i],LOW);
      digitalWrite(S_control_pins[i],HIGH);
      Pwmoutput[i] = -Pwmoutput[i];
      analogWrite(pwm_control_output[i],Pwmoutput[i]);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_1_A_PIN,INPUT);
  pinMode(MOTOR_2_A_PIN,INPUT);
  pinMode(MOTOR_3_A_PIN,INPUT);
  pinMode(MOTOR_1_B_PIN,INPUT);
  pinMode(MOTOR_2_B_PIN,INPUT);
  pinMode(MOTOR_3_B_PIN,INPUT);

  float coordinate_goal[3] = {0,0,-100};
  float coordinate_now[3] = {0,0,0};
  float precision_mm = 5;

  for(int i = 0 ; i < MOTOR_AMOUNT ; i++) {
    pinMode(P_control_pins[i],OUTPUT);
    pinMode(S_control_pins[i],OUTPUT);
    pinMode(pwm_control_output[i],OUTPUT);
  }

  attachInterrupt( digitalPinToInterrupt(MOTOR_1_A_PIN), motor_1_speed_measurement, CHANGE);
  attachInterrupt( digitalPinToInterrupt(MOTOR_2_A_PIN), motor_2_speed_measurement, CHANGE);
  attachInterrupt( digitalPinToInterrupt(MOTOR_3_A_PIN), motor_3_speed_measurement, CHANGE);
  Timer4.attachInterrupt(motor_PID_control).setFrequency(100).start();
}

void loop() {
  if(Endcoder != Target) {
    float distance_ntg = distance_get(coordinate_now,coordinate_goal);
    float step_amount = distance_ntg / precision_mm;
    float step_xmove_every = (coordinate_goal[0] - coordinate_now[0]) / step_amount;
    float step_ymove_every = (coordinate_goal[1] - coordinate_now[1]) / step_amount;
    float step_zmove_every = (coordinate_goal[2] - coordinate_now[2]) / step_amount;
    float coordinate_
    while(Endcoder != Target) {
      if(Endcoder[0] == next_target[0] && (coordinate_goal[0] - next_target[0]) < step_xmove_every
         Endcoder[1] == next_target[1] && (coordinate_goal[1] - next_target[1]) < step_ymove_every
         Endcoder[2] == next_target[2] && (coordinate_goal[2] - next_target[2]) < step_zmove_every ) {
         Target = coordinate_goal;
      }
      else {
        
      }
      

    }
  }
}
