#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneArmControl.h>
//#include <Eigen>
//#include <dji_sdk_demo/spacetrex_kalman_filter.h>
#include <iostream>
//#include <future>
#include <cstdio>
#include <dlib/optimization.h>
//using namespace kf;
//#include "dji_sdk_demo/spherex.h"
#include "math.h"

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

//ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient drone_arm_service;
ros::ServiceClient query_version_service;
//ros::ServiceClient query_version_service;

uint8_t flight_status = 255;
ros::Publisher ctrlVelYawratePub;
ros::Publisher hopStatus;

Mission hop;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spherex");
  ros::NodeHandle nh;

  ros::Subscriber getVelocity = nh.subscribe("dji_sdk/velocity" ,100, &getVelocity_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ctrlVelYawratePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
  //note
  //make a hop status advertiser


  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  drone_arm_service          = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

  bool obtain_control_flag = obtain_control();
  if(obtain_control_flag)
  {
  ROS_INFO("Obtain Control Success");
  }
  else ROS_INFO("Obtain Control Failed");

  double g, d, theta, phi, t_fac;
  bool hopped = true;

  while(ros::ok())
  {
    if(hopped)
    {
      std::cout<<"Please Enter d, theta, phi, and landing factor; g is = 1.62";
      std::cin>>d>>theta>>phi>>t_fac;
      hop.set_mission(d,theta,phi,t_fac);
      hopped = false;
    }
    hopped  =  hop.finished;
    ros::spin();
  }

}

void Mission::hop_ex()
{
        double tc = hop.t;
        double z_vel_c = hop.z_vel-1.62*tc;


        if(z_vel_c<=(-0.8)*hop.z_vel)
        {
          if(hop.land_flag_init)
          {
            hop.acc_land = z_vel_c/hop.t_fac;
            hop.land_flag_init = false;
            ROS_INFO("landing acceleration %f", hop.acc_land);
          }
          double hop_land_vel = z_vel_c - hop.acc_land*hop.land_t;
          hop.land_t = hop.land_t + 0.01;
          if(hop_land_vel >= -0.25)
          {
            hop.touchdown_counter++;
            if(hop.touchdown_counter >= 25)
            {
              hop_fill_vel(0,0,0,0);
              hop.finished = true;
            }
            else
            {
              hop_fill_vel(0,0,-0.20,0);
               hop.finished = false;    //return true;
            }
          }
          else
          {
            hop_fill_vel(0,0, hop_land_vel,0);
          }
        }
        else
        {
          hop_fill_vel(hop.x_vel,hop.y_vel,z_vel_c,0);
          ROS_INFO("%lf, %lf, %lf", hop.x_vel,hop.y_vel,z_vel_c);
          hop.land_flag_init = true;
          hop.land_t = 0;
          hop.finished = false;
        }
}

void Mission::hop_fill_vel(double Vx, double Vy, double Vz, double yaw)
{

  sensor_msgs::Joy controlVelYawRate;
  controlVelYawRate.axes.push_back(Vx);
  controlVelYawRate.axes.push_back(Vy);
  controlVelYawRate.axes.push_back(Vz);
  controlVelYawRate.axes.push_back(yaw);
  ctrlVelYawratePub.publish(controlVelYawRate);
  ROS_INFO("PUBLISHING VELOCITY");
}

bool Mission::set_mission(double d_, double theta_, double phi_, double t_fac_)
{
    grav = 1.62;
    d = d_;
    theta = theta_;
    phi = phi_;
    t = 0.00;
    t_fac = t_fac_;
    acc_land = 1.62;
    finished =false;
    touchdown_counter = 0;
    hold_counter = 0;
    double theta_rad = (theta*3.14)/180;
    double phi_rad = (phi*3.14)/180;
    ROS_INFO("Calculating the intial velocity vector");
    v = sqrt((d*grav)/sin(2*theta_rad));
    x_vel = v*cos(theta_rad)*sin(phi_rad);
    y_vel = v*cos(theta_rad)*cos(phi_rad);
    z_vel = v*sin(theta_rad);
    return true;
}


void getVelocity_callback(const geometry_msgs::Vector3Stamped& vel_from_sdk) // prototyped in the flight control header
{
      hop.hold_counter++;
      if(hop.hold_counter>=200)
      {
      hop.t = hop.t + hop.dt ;
      hop.hop_ex();
      }
      else ROS_INFO("I am getting ready, Please Wait");
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}
bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}
