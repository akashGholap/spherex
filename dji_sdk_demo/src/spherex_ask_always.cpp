//#include "dji_sdk_demo/demo_flight_control.h"
#include <SphereX/spherex_flight_control.h>
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneArmControl.h>
#include "std_msgs/Bool.h"
#include <dji_sdk/setNextHop.h>
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
ros::ServiceClient set_next_hop_service;
//ros::ServiceClient query_version_service;

uint8_t flight_status = 255;
ros::Publisher ctrlVelYawratePub;
ros::Publisher hopStatusPub;


Mission hop;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spherex");
  ros::NodeHandle nh;

  ros::Subscriber getVelocity = nh.subscribe("dji_sdk/velocity" ,100, &getVelocity_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber  icpppStatusSub = nh.subscribe("spherex/icp_pp_status", 10, &icp_pp_status_callback);
  ctrlVelYawratePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
  hopStatusPub = nh.advertise<std_msgs::Bool>("spherex/hopStatus",100);
  //note
  //make a hop status advertiser


  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  drone_arm_service          = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_next_hop_service       = nh.serviceClient<dji_sdk::setNextHop>("dji_sdk/set_next_hop_service");

  bool nextHopSet = true;

  bool obtain_control_flag = obtain_control();
  if(obtain_control_flag)
  {
  ROS_INFO("Obtain Control Success");
  }
  else ROS_INFO("Obtain Control Failed");

  double g, d, theta, phi, t_fac;
  std::cout<<"Please Enter d, theta, phi, and landing factor; g is = 1.62";
  std::cin>>d>>theta>>phi>>t_fac;
  bool is_set_start = hop.set_mission(d,theta,phi,t_fac);

  bool hopped = true;

  while(ros::ok())
  {
    if(hopped&&nextHopSet&&!is_set_start)
    {


      hop.set_mission(hop.nd, hop.ntheta, hop.nphi, t_fac);
      hopped = false;

    }
    hopped  =  hop.finished;
    if(hop.icp_pp&&hopped)
    {
      std::cout<<"is it in icp_pp"<<std::endl;
      nextHopSet = set_next_hop();
      is_set_start = false;
    }
    hop.hop_status_publish();
    ros::spinOnce();
  }

}

void Mission::hop_ex()
{
        double tc = hop.t;
        double z_vel_c = hop.z_vel-1.62*tc;
        if(z_vel_c<=(-0.8)*hop.z_vel && (!hop.land_flag))
        {
          hop.land_flag = true;
          hop.acc_land = z_vel_c/hop.t_fac;
          hop.init_vel_land = z_vel_c;
          //hop.land_flag = false;
          ROS_INFO("landing acceleration %f", hop.acc_land);
        }

        if(hop.land_flag)
        {
          hop.land_t = hop.land_t + 0.01;
          double hop_land_vel = hop.init_vel_land - hop.acc_land*hop.land_t;
          if(hop_land_vel >= -0.25)
          {
            hop.touchdown_counter++;
            if(hop.touchdown_counter >= 25)
            {
              hop_fill_vel(0,0,0,0);
              hop.finished = true;
              ROS_INFO("SphereX Landed");
              disarm_motors();
            }
            else
            {
              hop_fill_vel(0,0,-0.20,0);
               hop.finished = false;
               ROS_INFO("In touchdown");   //return true;
            }
          }
          else
          {
            hop_fill_vel(0,0, hop_land_vel,0);
            ROS_INFO("Pre touchdown");
          }
        }
        else
        {
          hop_fill_vel(hop.x_vel,hop.y_vel,z_vel_c,0);
          ROS_INFO("%lf, %lf, %lf", hop.x_vel,hop.y_vel,z_vel_c);
          hop.land_t = 0;
          hop.finished = false;
        }
        std_msgs::Bool hopStatus;
        hopStatus.data = hop.finished;
        hopStatusPub.publish(hopStatus);
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
void Mission::hop_status_publish(void)
{
  std_msgs::Bool hopStatus;
  hopStatus.data = hop.finished;
  hopStatusPub.publish(hopStatus);
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
    wait_counter = 0;
    up_counter = 0;
    hold_counter = 0;
    land_flag = false;
    arm = false;
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
      if(hop.hold_counter==100)
      {
        hop.arm = arm_motors();
      }
      if(!hop.arm && (hop.hold_counter==200))
      {
        hop.arm = arm_motors();
        if(!hop.arm)
        {
          ROS_INFO("SphereX is not arming");
        }
      }
      else if(hop.arm && (hop.hold_counter==200))
      {
        ROS_INFO("SphereX Arming Successful");
      }
      if(hop.hold_counter>=400)
      {
        if(hop.wait_counter<=100)
        {
          hop.hop_fill_vel(0,0,0,0);
          hop.wait_counter++;
          hop.finished = false;
        }
        else if(hop.wait_counter>100&&hop.up_counter<=50)
        {
          hop.hop_fill_vel(0,0,3,0);
          hop.up_counter++;
          hop.finished = false;
        }
        else
        {
          hop.t = hop.t + hop.dt ;
          hop.hop_ex();
        }

      }

}

void icp_pp_status_callback(const std_msgs::Bool& status)
{
  hop.icp_pp = status.data;
}

bool set_next_hop()
{
  dji_sdk::setNextHop nhop;
  nhop.request.ifpped = true;
  set_next_hop_service.call(nhop);
  if(!nhop.response.vset)
  {
    ROS_INFO("Parameters not set yet");
    return false;
  }
  else
  {
    int userinput;
    int t_fac;
    std::cout<<"Please Verify the response from the icp service"<<std::endl;
    std::cout<<" d "<<nhop.response.d<<" Theta "<<nhop.response.theta<<" Phi "<<nhop.response.phi<<std::endl;
    std::cout<<"Your Response 1 or 0 "<<std::endl;
    std::cin>>userinput;
    if(userinput)
    {
      hop.nd = nhop.response.d;
      hop.ntheta = nhop.response.theta;
      hop.nphi = nhop.response.phi;
      return true;
    }
    else
    {
      std::cout<<"Me kya bolu, aap hi dalo ab input d, theta, phi, and landing factor; g is = 1.62";
      std::cin>>hop.nd>>hop.ntheta>>hop.nphi>>t_fac;

      return true;
    }
  }

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

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

bool arm_motors()
{
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 1;
  drone_arm_service.call(droneArmControl);
  if(!droneArmControl.response.result)
  {
    ROS_ERROR("Arming Failed");
      return false;
  }
  return true;
}

bool disarm_motors()
{
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 0;
  drone_arm_service.call(droneArmControl);
  if(!droneArmControl.response.result)
  {
    ROS_ERROR("Disarming Failed");
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
