#include "ros/ros.h"
#include "std_msgs/Float64.h"

import 


#define PWM_LIMIT  885

static float cur_error = 0.0, pre_error = 0.0, integral = 0.0, derivative = 0.0;
static float diff_time = 0.010;
static float control_input = 0.0;
static int16_t cnt = 0;

// static float p_gain = 4000.0, i_gain = 2.0, d_gain = 78.0;
static float p_gain = 55000.0;
static float i_gain = 10000.0;
static float d_gain = 30000.0;


void controlSegway(float angle)
{
  //bool dxl_comm_result = false;
  //static float control_input = 0.0;

  //static float cur_error = 0.0, pre_error = 0.0, integral = 0.0, derivative = 0.0;
  //static float diff_time = 0.007;

  //static int16_t cnt = 0;           // timer counter

  cur_error  = control_input - angle; //angle[0];
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

}


void sensorCallback(const std_msgs::Float64::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%f]", msg->data);

    //std_msgs::Float64 controlValue;
    controlSegway((float)msg->data);
    
    //control_pub.publish(controlValue);

    //ros::spinOnce();
}

int main(int argc, char **argv){
    ros::init(argc,argv,"netsegway");
    ros::NodeHandle n;

    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber angle_sub = n.subscribe("angle", 1000, sensorCallback);

    ros::Publisher control_pub = n.advertise<std_msgs::Float64>("control", 1);
    ros::Rate loop_rate(10);

    std_msgs::Float64 controlValue;

    int count = 0;
    while (ros::ok()){
        //++count;
        //if (count == 1000) controlValue.data = -400.0f;
        //else 
        controlValue.data = control_output;//400.0f;///control_output;
        control_pub.publish(controlValue);
        ros::spinOnce();
        loop_rate.sleep();

        /*
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce()
        
        loop_rate.sleep()
        ++count;
        */

    }

    return 0;
}
