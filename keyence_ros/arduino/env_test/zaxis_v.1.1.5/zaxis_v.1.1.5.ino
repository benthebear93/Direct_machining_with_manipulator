#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#define STEP_P 8
#define DIR_P 9
#define stepsPerRevolution 3200
#define f LOW
#define b HIGH
int thr_val = 50;
ros::NodeHandle nh;

float error =0; //pid feedback error
float z_t = 10.0; // tool z
float z_m = 0; // measurment z 

int s_switch = 0;
int s_goback = 0;
int s_reset = 0;
float error_ = 0;

float p_val = 0;
float i_val = 0;
int pid_val = 0;
float kp = 15;
float ki = 0;
float dv = 0;
bool dir = 0;
unsigned int vel = 0;
int vel_pid = 0;
int step_num = 0;
int debug_flag =0;

std_msgs::Float32 z_tool;
std_msgs::Float32 error_val;
std_msgs::Int16 debug_vel;
ros::Publisher pub("z_distacne", &z_tool);
ros::Publisher pub2("error", &error_val);
ros::Publisher pub3("debug_vel", &debug_vel);


void Switch_button( const std_msgs::Int16& swt1){
  s_switch = swt1.data;
  if(s_switch == 1){
    nh.loginfo("1.start switch");
  }else{
    nh.loginfo("0.stop switch");
  }
}

void Switch_go_back( const std_msgs::Int16& swt2){
  s_goback = swt2.data;
  if(s_goback == 1){
    nh.loginfo("1.go");
  }else{
    nh.loginfo("0.back");
  }
}

void Switch_reset( const std_msgs::Int16& swt3){
  nh.loginfo("reset function");
  s_reset = swt3.data;
  if(s_reset == 1){
    nh.loginfo("1.start");
  }else{
    nh.loginfo("0.reset");
  }
}

void Laser_data(const std_msgs::Float32& laser){
  //nh.loginfo("laser callback?");
  z_m = laser.data;
}

void PID_Control(){
  nh.loginfo("control?");
  if(s_switch==1){
    error_ = z_t - z_m;
    if(error_ !=0){
      step_num = step_num +1;
      //error_ = z_t - z_m;
      if(error_ <= 0.00125){ // tool need to go up!s
        nh.loginfo("go up?");
        vel = (unsigned int)thr_val;
        digitalWrite(DIR_P, f);//set dir c 
        z_t = z_t + 0.000625;
        stepper_move(vel);
      }
      else if(error_ >=0.00125){ // tool need to go down!
        nh.loginfo("go down?");
        vel = (unsigned int)thr_val;
        digitalWrite(DIR_P, b);//set dir c 
        z_t = z_t - 0.000625;
        stepper_move(vel);
      }
      else{
        digitalWrite(DIR_P, 0);//backward(HIGH)
      }
      //i_val = i_val +ki*error*cycle_time;
      //d_val = kd*(error-error_pre)/cycle_time;
      if(step_num%2 == 0){
        pubtest();
        step_num = 0;
      }
    }
  }else{
    stepper_stop(0, f); //speed = delay Ms
  }
}
void pubtest(){
  z_tool.data = z_t;
  pub.publish(&z_tool);
  error_val.data = error_;
  pub2.publish(&error_val);
  debug_vel.data = vel;
  pub3.publish(&debug_vel);
}

ros::Subscriber<std_msgs::Int16> sub("start_button", Switch_button);
ros::Subscriber<std_msgs::Int16> sub2("go_back_button", Switch_go_back);
ros::Subscriber<std_msgs::Int16> sub4("reset_button", Switch_reset);
ros::Subscriber<std_msgs::Float32> sub3("kf_z", Laser_data);

void backnforth(){ // for the test only
  if(s_reset == 1){
    if(s_goback == 0){
      nh.loginfo("0.straight");
      stepper_set(100, f); //speed = delay Ms
    }else{
      nh.loginfo("1.back");
      stepper_set(100, b); //speed = delay Ms
    }
  }
  else{
    stepper_stop(0, b); //speed = delay Ms
  }
}

void stepper_set(int speed_val, int dir){
  digitalWrite(DIR_P, dir);//set dir c 
  for (int i = 0; i < stepsPerRevolution; i++){
    if(dir==LOW){
      z_t = z_t + 0.000625;
    }else{
      z_t = z_t - 0.000625;
    }
    digitalWrite(STEP_P, HIGH);
    delayMicroseconds(speed_val);
    digitalWrite(STEP_P, LOW);
    delayMicroseconds(speed_val);
  }
}

void stepper_move(unsigned int speed_val){
  digitalWrite(STEP_P, HIGH);
  delayMicroseconds(vel);
  digitalWrite(STEP_P, LOW);
  delayMicroseconds(vel);
}

void stepper_stop(int speed_val, int dir){
}

void setup() {
  pinMode(STEP_P,OUTPUT);
  pinMode(DIR_P,OUTPUT);
  nh.advertise(pub);
  nh.advertise(pub2);
  nh.advertise(pub3);
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
}

void loop() {
  nh.loginfo("start");
  if(s_reset ==1){
    backnforth();
    debug_flag = 0;
  }
  else{
    if(debug_flag==0){
      nh.loginfo("stop reset and start control");
      nh.loginfo("start PID control func");
      debug_flag = 1;
    }
    PID_Control();
  }
  nh.spinOnce();
}
