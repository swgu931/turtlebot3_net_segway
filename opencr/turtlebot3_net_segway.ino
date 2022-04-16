
#include "turtlebot3_segway.h"

#include <ros.h>
#include <std_msgs/Float64.h>

std_msgs::Float64 mAngle;

ros::NodeHandle nh;

ros::Publisher pub_angle("angle",&mAngle);

//ros::Rate loop_rate(10);

void messageControlCb(const std_msgs::Float64& msg){
  controlNetSegway(msg.data);
}

ros::Subscriber<std_msgs::Float64> sub_control("control", &messageControlCb);

/*******************************************************************************
* Declaration for Hardware Timer (Interrupt control)
*******************************************************************************/
//HardwareTimer Timer(TIMER_CH1);

uint last_time_action_done_millis = 0;

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
cIMU IMU;

/*******************************************************************************
* Declaration for Filter
*******************************************************************************/
float filterFrequency = 1.0;    // Hz
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

float angle[3] = {0.0,0.0,0.0}; //roll, pitch, yaw

// initial angle offset
float angle_offset = 0.0;

// keyboard input
char keyboard;

// Set PID gain
float p_gain = 4000.0;
float i_gain = 2.0;
float d_gain = 78.0;

float control_output = 0.0;


void setup()
{
  nh.initNode();  
  // Setting for Dynamixel motors
  motor_driver.init();
  
  delay(100);
  controlNetSegway(600.0);
  delay(2000);
  controlNetSegway(0.0);
  // Initialization IMU
  imuInit();

  nh.advertise(pub_angle);
  nh.subscribe(sub_control);
}

void loop()
{
  /*
  while(!nh.connected()){
    nh.spinOnce();
  }
*/
/*
  if ( millis() > last_time_action_done_millis + 100 )
  {
     last_time_action_done_millis = millis();

     if (IMU.update() > 0){
      getAngle(angle);
      mAngle.data = angle[0];
      }
      
      pub_angle.publish(&mAngle);

     // do stuff you want to do once per second
  }
  

  nh.spinOnce();
  */
  //loop_rate.sleeo();
  
  if (IMU.update() > 0){
    getAngle(angle);
    mAngle.data = angle[0];
  }

    pub_angle.publish(&mAngle);
    nh.spinOnce();
    delay(10);
    
}

/*******************************************************************************
* Get angle from IMU
*******************************************************************************/
void getAngle(float angle[3])
{
  float roll, pitch, yaw;

  roll  = IMU.rpy[0];
  pitch = IMU.rpy[1];
  yaw   = IMU.rpy[2];

  angle[0] = lowpassFilter.input(pitch) + angle_offset;
  angle[1] = pitch + angle_offset;
}

/*
void startDynamixelControlInterrupt()
{
  Timer.pause();
  Timer.setPeriod(CONTOL_PERIOD);           // in microseconds
  Timer.attachInterrupt(controlSegway);
  Timer.refresh();
  Timer.resume();
}
*/
/*******************************************************************************
* Initialization of IMU
*******************************************************************************/
void imuInit()
{
  IMU.begin();

  IMU.SEN.acc_cali_start();
  while( IMU.SEN.acc_cali_get_done() == false )
  {
    IMU.update();
  }

  // Start Dynamixel Control Interrupt
  
  //startDynamixelControlInterrupt();
}

/*******************************************************************************
* Control segway PWM
*******************************************************************************/
void controlNetSegway(float control_output){
    bool dxl_comm_result = false;
    dxl_comm_result = motor_driver.controlMotor((int64_t)control_output, (int64_t)control_output);
    if (dxl_comm_result == false)
    return;
}
/*
void controlSegway(void)
{
  bool dxl_comm_result = false;
  static float control_input = 0.0;

  static float cur_error = 0.0, pre_error = 0.0, integral = 0.0, derivative = 0.0;
  static float diff_time = 0.007;

  static int16_t cnt = 0;           // timer counter

  cur_error  = control_input - angle[0];
  integral   = integral + (cur_error * diff_time);
  derivative = (cur_error - pre_error) / diff_time;

  if (cnt > 500)
  {
    integral = 0.0;
    cnt = 0;
  }
  else
  {
    cnt++;
  }

  control_output = p_gain * cur_error +
                   i_gain * integral  +
                   d_gain * derivative;

  if (control_output >= PWM_LIMIT)
  {
    control_output = PWM_LIMIT;
  }
  else if (control_output <= (-1) * PWM_LIMIT)
  {
    control_output = (-1) * PWM_LIMIT;
  }
  pre_error = cur_error;

  dxl_comm_result = motor_driver.controlMotor((int64_t)control_output, (int64_t)control_output);
  if (dxl_comm_result == false)
    return;
}
*/
