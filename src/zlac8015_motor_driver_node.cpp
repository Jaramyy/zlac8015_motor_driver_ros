#include <ros/ros.h>
#include <modbus/modbus.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
//#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
struct wheel_travel
{
   float left = 0.0 ,right = 0.0;
};

class zlac8015_controller
{
   public:
      zlac8015_controller();
      void enable_motor();
      void disable_motor();
      void setMode(uint16_t mode);
      void set_accel_time(uint16_t accel_time[2]);
      void set_decel_time(uint16_t decel_time[2]);
      void set_rpm(int16_t rpm_left,int16_t rpm_right);
      
      void get_mode();
      void get_rpm();
      wheel_travel get_wheel_travelled();
      void cmdVel_callback(const geometry_msgs::Twist::ConstPtr& msg);
      void diffDriveController();
      void poseUpdate(zlac8015_controller &data);

      float l_meter = 0.0 ,r_meter = 0.0 ,l_meter_old = 0.0 ,r_meter_old = 0.0;
      wheel_travel wheel_meas;
      float cycleDistance,cycleAngle,avgAngle,orientation_z;
      nav_msgs::Odometry odom_old;
      nav_msgs::Odometry odom;


      ros::NodeHandle node;
      ros::Subscriber sub_cmdVel = node.subscribe("cmd_vel",100,&zlac8015_controller::cmdVel_callback,this);
      ros::Publisher pub_odom = node.advertise<nav_msgs::Odometry>("odom", 10); 

   private:

      geometry_msgs::Twist vel;
      int32_t l_meter_meas = 0 ,r_meter_meas = 0;

      const int32_t CONTROL_REG = 0x200E;
      const int32_t OPR_MODE = 0x200E;
      const int32_t L_ACL_TIME = 0x2080;
		const int32_t R_ACL_TIME = 0x2081;
		const int32_t L_DCL_TIME = 0x2082;
		const int32_t R_DCL_TIME = 0x2083;

		//Velocity control
		const int32_t L_CMD_RPM = 0x2088;
		const int32_t R_CMD_RPM = 0x2089;
		const int32_t L_FB_RPM = 0x20AB;
		const int32_t R_FB_RPM = 0x20AC;

		//Position control
		const int32_t POS_CONTROL_TYPE = 0x200F;

		const int32_t L_MAX_RPM_POS = 0x208E;
		const int32_t R_MAX_RPM_POS = 0x208F;

		const int32_t L_CMD_REL_POS_HI = 0x208A;
		const int32_t L_CMD_REL_POS_LO = 0x208B;
		const int32_t R_CMD_REL_POS_HI = 0x208C;
		const int32_t R_CMD_REL_POS_LO = 0x208D;

		const int32_t L_FB_POS_HI = 0x20A7;
		const int32_t L_FB_POS_LO = 0x20A8;
		const int32_t R_FB_POS_HI = 0x20A9;
		const int32_t R_FB_POS_LO = 0x20AA;

		//Troubleshooting
		const int32_t L_FAULT = 0x20A5;
		const int32_t R_FAULT = 0x20A6;

		//########################
		//## Control CMDs (REG) ##
		//########################
		const int32_t EMER_STOP = 0x05;
		const int32_t ALRM_CLR = 0x06;
		const int32_t DOWN_TIME = 0x07;
		const int32_t ENABLE = 0x08;
		const int32_t POS_SYNC = 0x10;
		const int32_t POS_L_START = 0x11;
		const int32_t POS_R_START = 0x12;

		// ####################
		// ## Operation Mode ##
		// ####################
		const int32_t POS_REL_CONTROL = 1;
		const int32_t POS_ABS_CONTROL = 2;
		const int32_t VEL_CONTROL = 3;

		const int32_t ASYNC = 0;
		const int32_t SYNC = 1;

		// #################
		// ## Fault codes ##
		// #################
		const int32_t NO_FAULT = 0x0000;
		const int32_t OVER_VOLT = 0x0001;
		const int32_t UNDER_VOLT = 0x0002;
		const int32_t OVER_CURR = 0x0004;
		const int32_t OVER_LOAD = 0x0008;
		const int32_t CURR_OUT_TOL = 0x0010;
		const int32_t ENCOD_OUT_TOL = 0x0020;
		const int32_t MOTOR_BAD = 0x0040;
		const int32_t REF_VOLT_ERROR = 0x0080;
		const int32_t EEPROM_ERROR = 0x0100;
		const int32_t WALL_ERROR = 0x0200;
		const int32_t HIGH_TEMP = 0x0400;

      // ##############
		// ## Odometry ##
		// ##############
		// 8 inches wheel
		float travel_in_one_rev = 0.33615041393; //0.33615041393
		int32_t cpr = 4096; //16385
		float R_Wheel = 0.107; //meter

      float rps_2_rpm = 9.54929658551;
      float base = 0.4;
      float wheel = 0.0535;
   

      modbus_t *ctx;

      int16_t int16Dec_to_int16Hex(int16_t data)
      {
         uint8_t low_byte;
         uint8_t high_byte;
         int16_t all_bytes;
         
         low_byte = (data & 0x00FF);
         high_byte = (data & 0xFF00) >> 8;
         all_bytes = (high_byte << 8) | low_byte;

         return all_bytes;
      }
};

zlac8015_controller::zlac8015_controller()
{  
   ROS_WARN("motor driver connecting");
   int device_ID = 1;
   ctx = modbus_new_rtu("/dev/ttyUSB1", 115200, 'N', 8, 1);
   if (ctx == NULL) {
      ROS_FATAL("Unable to create the libmodbus context");
      return;
   }

   modbus_set_slave(ctx,device_ID);

   if (modbus_connect(ctx) == -1) {
      ROS_FATAL("Connection failed: %s\n", modbus_strerror(errno));
      modbus_free(ctx);
      return;
   }
   else{
      ROS_WARN("motor driver connedcted");
   }
}
void zlac8015_controller::enable_motor()
{
   if(modbus_write_register(ctx, CONTROL_REG, ENABLE)!=1)
   {
      ROS_FATAL("enable error");
   }
}
void zlac8015_controller::disable_motor()
{
   if(modbus_write_register(ctx, CONTROL_REG, DOWN_TIME)!=1)
   {
      ROS_FATAL("disable error");
   }
}
void zlac8015_controller::setMode(uint16_t mode)
{
   if(mode == 1)
   {
      ROS_INFO("Set relative position control");
   }
   else if(mode == 2)
   {
      ROS_INFO("Set absolute position control");
   }
   else if(mode == 3)
   {
      ROS_INFO("Set speed rpm control");
   }
   else
   {
      ROS_INFO("set_mode ERROR: set only 1, 2, or 3");
   }
   modbus_write_register(ctx, CONTROL_REG, mode);
}
void zlac8015_controller::set_accel_time(uint16_t accel_time[2])
{
   if(accel_time[0] > 32767)
   {
      accel_time[0] = 32767;
   }
   else
   {
      accel_time[0] = 0;
   }

   if(accel_time[1] > 32767)
   {
      accel_time[1] = 32767;
   }
   else
   {
      accel_time[1] = 0;
   }
      
   modbus_write_registers(ctx, L_ACL_TIME, 2, accel_time);
}
void zlac8015_controller::set_decel_time(uint16_t decel_time[2])
{
   if(decel_time[0] > 32767)
   {
      decel_time[0] = 32767;
   }
   else
   {
      decel_time[0] = 0;
   }

   if(decel_time[1] > 32767)
   {
      decel_time[1] = 32767;
   }
   else
   {
      decel_time[1] = 0;
   }
      
   modbus_write_registers(ctx, L_DCL_TIME, 2, decel_time);
}
void zlac8015_controller::set_rpm(int16_t rpm_left,int16_t rpm_right)
{  
   // ROS_WARN("rpm_l = %d, rpm_r = %d",rpm_left,rpm_right);
   int16_t rpm[2] = {rpm_left,rpm_right};
   if(rpm[0] > 3000)
   {  
      rpm[0] = 3000;
   }
   else if(rpm[0] < -3000)
   {
      rpm[0] = -3000;
   }

   if(rpm[1] > 3000)
   {
      rpm[1] = 3000;
   }
   else if(rpm[1] < -3000)
   {
      rpm[1] = -3000;
   }

   u_int16_t rpm_conmand[2];
   rpm_conmand[0] = int16Dec_to_int16Hex(rpm[0]);
   rpm_conmand[1] = int16Dec_to_int16Hex(rpm[1]);
   // ROS_WARN("rpm_l = %d, rpm_r = %d",rpm_conmand[0],rpm_conmand[1]);
   modbus_write_registers(ctx, L_CMD_RPM, 2, rpm_conmand);
}

void zlac8015_controller::get_mode()
{
   uint16_t tab_reg[1];
   if(modbus_read_registers(ctx, OPR_MODE, 1, tab_reg)!= -1)
   {
      ROS_WARN("%d",tab_reg[0]);
   }
   else
   {
      ROS_WARN("can't get data");
   }
}

void zlac8015_controller::get_rpm()
{ 
   u_int16_t tab_reg[2];
   if(modbus_read_registers(ctx, L_FB_RPM, 2, tab_reg)!= -1)
   {
      ROS_WARN("Left speed = %d, Right speed = %d",tab_reg[0]/10,tab_reg[1]/10);
   }
   else
   {
      ROS_WARN("can't get data");
   }
}

wheel_travel zlac8015_controller::get_wheel_travelled()
{  
   u_int16_t tab_reg[4];
   int32_t l_pulse,r_pulse,l_travelled,r_travelled ;
   wheel_travel wheel_travelled;

   if(modbus_read_registers(ctx, L_FB_POS_HI, 4, tab_reg)!= -1)
   {
      // u_int16_t l_pul_hi = tab_reg[0];
      // u_int16_t l_pul_lo = tab_reg[1];
      // u_int16_t r_pul_hi = tab_reg[2];
      // u_int16_t r_pul_lo = tab_reg[3];

      l_pulse = ((tab_reg[0] & 0xFFFF) << 16) | (tab_reg[1] & 0xFFFF);
      r_pulse = ((tab_reg[2] & 0xFFFF) << 16) | (tab_reg[3] & 0xFFFF);
      
      wheel_travelled.left = (float_t(l_pulse)/cpr)*travel_in_one_rev;
      wheel_travelled.right = (float_t(r_pulse)/cpr)*travel_in_one_rev;
      // ROS_INFO("Left speed = %d, Right speed = %d", l_pulse, r_pulse);
      
      // ROS_INFO("Left speed = %.3f, Right speed = %.3f", wheel_travelled.left, wheel_travelled.right);
      return wheel_travelled;
   }
   else
   {
      ROS_WARN("can't get data");
   }
}

void zlac8015_controller::cmdVel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
   vel.linear.x = msg->linear.x;
   vel.angular.z = msg->angular.z;
}

void zlac8015_controller::diffDriveController()
{
   int command_left,command_right;
   command_left = (vel.linear.x - (vel.angular.z * base/2)) / wheel * rps_2_rpm;
   command_right = (vel.linear.x + (vel.angular.z * base/2)) / wheel * rps_2_rpm;  // RPS-> RPM (1/(2*pi))*60
   set_rpm(command_left,-command_right);
}

void zlac8015_controller::poseUpdate(zlac8015_controller &data)
{
   wheel_meas =  data.get_wheel_travelled();
   // self.l_meter_meas, self.r_meter_meas = self.motors.get_wheels_travelled()
   l_meter = wheel_meas.left - l_meter_old;
   r_meter = (-1*wheel_meas.right) - r_meter_old;
   ROS_INFO("l = %f , r = %f",l_meter,r_meter);
   // # print("L = ",self.l_meter,"R = ",self.r_meter)

    
   
   cycleDistance = (r_meter + l_meter) / 2.0;
   cycleAngle = asin((r_meter - l_meter) / base);       
   avgAngle = (cycleAngle / 2.0) + odom_old.pose.pose.orientation.z;
   
   ROS_INFO("x = %f , y = %f",avgAngle  ,cycleDistance);
   // # Calculate the new pose (x, y, and theta)
   odom.pose.pose.position.x = odom_old.pose.pose.position.x + cos(avgAngle)*cycleDistance;
   odom.pose.pose.position.y = odom_old.pose.pose.position.y + sin(avgAngle)*cycleDistance;
   orientation_z = (cycleAngle + odom_old.pose.pose.orientation.z);

   
   //odom_quat = tf.transformations.quaternion_from_euler(0, 0, (orientation_z));
   tf2::Quaternion q;      
   q.setRPY(0, 0, orientation_z);
   odom.pose.pose.orientation.x = q.x();
   odom.pose.pose.orientation.y = q.y();
   odom.pose.pose.orientation.z = q.z();
   odom.pose.pose.orientation.w = q.w();

   // #odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

   odom.header.stamp = ros::Time::now();
   odom.header.frame_id = "odom";
   odom.child_frame_id = "base_footprint";
   odom.twist.twist.linear.x  = cycleDistance/(odom.header.stamp.toSec() - odom_old.header.stamp.toSec());
   // # print(self.avgAngle)
   odom.twist.twist.angular.z = cycleAngle/(odom.header.stamp.toSec() - odom_old.header.stamp.toSec());

   // #Save the pose data for the next cycle
   odom_old.pose.pose.position.x = odom.pose.pose.position.x;
   odom_old.pose.pose.position.y = odom.pose.pose.position.y;
   odom_old.pose.pose.orientation.z =  orientation_z;
   odom_old.header.stamp = odom.header.stamp;
   l_meter_old = wheel_meas.left;
   r_meter_old = (-1*wheel_meas.right);

   static tf2_ros::TransformBroadcaster br;
   geometry_msgs::TransformStamped transformStamped;

   transformStamped.header.stamp = ros::Time::now();
   transformStamped.header.frame_id = "odom";
   transformStamped.child_frame_id =  "base_footprint";
   transformStamped.transform.translation.x = odom.pose.pose.position.x;
   transformStamped.transform.translation.y = odom.pose.pose.position.y;
   transformStamped.transform.translation.z = 0.0;
   transformStamped.transform.rotation.x = q.x();
   transformStamped.transform.rotation.y = q.y();
   transformStamped.transform.rotation.z = q.z();
   transformStamped.transform.rotation.w = q.w();
   br.sendTransform(transformStamped);

   // #Publish the odometry message
   pub_odom.publish(odom);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "zlac8015_motor_driver");
   zlac8015_controller driver;
   uint16_t accel_time_driver[2] = {(uint16_t)1000,(uint16_t)1000};
   uint16_t decel_time_driver[2] = {(uint16_t)1000,(uint16_t)1000};
   driver.disable_motor();
   driver.set_accel_time(accel_time_driver);
   driver.set_decel_time(accel_time_driver);
   driver.setMode(3);
   driver.enable_motor();

   ros::Rate loop_rate(50); 
   while(ros::ok())
   {
      driver.diffDriveController();
      driver.poseUpdate(driver);
      ros::spinOnce();
      loop_rate.sleep();
   }
   driver.disable_motor();

   // driver.disable_motor();
   // driver.set_accel_time(accel_time_driver1);
   // driver.set_decel_time(decel_time_driver1);
   // driver.setMode(3);
   // ROS_WARN("Get mode operation");
   // sleep(1.5);
   // driver.get_mode();
   // driver.enable_motor();
   // driver.set_rpm(90,50);
   // while(ros::ok())
   // {  
      
   // }
   // driver.disable_motor();
   // driver.set_rpm(-100,-100);
   // sleep(3);
   // driver.set_rpm(0,0);
   return 0;
}
