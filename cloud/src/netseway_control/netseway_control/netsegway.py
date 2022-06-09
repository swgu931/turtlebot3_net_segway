import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32

P_gain = 55000.0
I_gain = 10000.0
D_gain = 30000.0

Control_Topic = '/netsegway/control_input'

class NetSegwayController(Node):

  def __init__(self):
    super().__init__('netsegway_controller')
    self.qos_profile = QoSProfile(depth=10)
    self.publisher_ = self.create_publisher(Float32, Control_Topic, self.qos_profile)
    timer_period = 0.01  # seconds
    self.i = 0
    
    self.subscription = self.create_subscription(
        Float32,
        'topic',
        self.Controller_callback,
        self.qos_profile)
    # self.subscription  # prevent unused variable warning
    
    self.pre_error = 0
    self.diff_time = 0.007   # how to make decision of diff_time ???????????    
    self.cnt = 0
    self.p_gain = 55000.0
    self.i_gain = 10000.0
    self.d_gain = 30000.0
      
  def Controller_callback(self, msg):
      
    control_command = Calculate_pid_control(msg)
    
    self.publisher_.publish(control_command)
    
    if i % 100 == 0
          self.get_logger().info('Publishing: "%f"' % msg)
    self.i += 1
      
      
  def Calculate_pid_control(self, msg):
    cur_error  = control_input - msg; #angle[0];
    integral   = integral + (cur_error * diff_time);
    derivative = (cur_error - pre_error) / diff_time;
    
    if self.cnt > 500:
      integral = 0.0
      self.cnt = 0
    else:
      self.__init__cnt += 1
    
    control_output = p_gain * cur_error + i_gain * integral + d_gain * derivative

    if control_output >= PWM_LIMIT:
      control_output = PWM_LIMIT
    elif control_output <= (-1) * PWM_LIMIT:
      control_output = (-1) * PWM_LIMIT
    else:
      print ("==================== ??? ")
    
    pre_error = cur_error
      
    return control_output
    
    
    
    
  def Calculate_lqr_control(self, msg):
    control_output = 1
    return control_output
    
    
    
  def Calculate_lqg_control(self, msg):
    control_output = 1
    return control_output
      
  
  def Calculate_H_inf_control(self, msg):
    control_output = 1
    return control_output    
        
  

        
def main(args=None):
    rclpy.init(args=args)

    netsegway_controller = NetSegwayController()

    rclpy.spin(netsegway_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    netsegway_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
   




# ==============================
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
