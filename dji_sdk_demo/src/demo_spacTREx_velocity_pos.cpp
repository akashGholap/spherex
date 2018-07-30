
#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneArmControl.h>
//#include <Eigen>
#include <dji_sdk_demo/spacetrex_kalman_filter.h>
#include <iostream>
//#include <future>
#include <cstdio>
#include <dlib/optimization.h>
using namespace kf;
//#include "dji_sdk_demo/spherex.h"
#include "math.h"

const float gravity = -2.0;
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient drone_arm_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlVelYawratePub;


// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
//geometry_msgs::Vector3 velocity_from_sdk;


Mission hop_pos;
Mission hopper_vel;
Mission hop;
kalman_filter kfs(3,3,0.01);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  //ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 100, &attitude_callback);
  //ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 100, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  //ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 100, &display_mode_callback);
  //ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 100, &local_position_callback);
  ros::Subscriber getVelocity = nh.subscribe("dji_sdk/velocity" ,100, &getVelocity_callback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 100);

  ctrlVelYawratePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  drone_arm_service          = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool hopping_result = true;
  hop.finished = false;
  bool landing_flag =false;


    double xi,yi,zi; //initial position
    double xf,yf,zf; //final position

    float x=2 ,y=3,z=2,yaw=2;
    int state_action=1;



  while(ros::ok())
  {

    if(hopping_result)
    {
      std::cout<<"Please Enter next Velocity vector";
      std::cin>>xf>>yf>>zf>>xi>>yi>>zi;
      hop.set_mission(xf,yf,zf,xi,yi,zi);
      bool opt_vel_set  =  set_optimum_velocity();
      if(opt_vel_set)
      {
        set_filter_main();
      }

      hopping_result = false;
    }
    hopping_result = hop.finished;
    landing_flag = hop.land;
    ROS_INFO(hop.land?"HRtrue":"HRfalse");
    if(landing_flag)
    {
      ROS_INFO("initiating landing");

    }

    ros::spin();
}
// Helper Functions
}
/////////////////////////////////////////////////////////////"hopex"///////////////////////////////////////////////////////////////////////////////////////

bool Mission::hopex(double x, double y, double z, double yaw)
{

  float Vz_start = z;
  float Vz_current = z;
  ros::Time start_time = ros::Time::now();
   while(((-1)*(Vz_current) <= 0.90*Vz_start))
    {
    sensor_msgs::Joy controlVelYawRate;
    controlVelYawRate.axes.push_back(x);
    controlVelYawRate.axes.push_back(y);
    controlVelYawRate.axes.push_back(Vz_current);
    controlVelYawRate.axes.push_back(yaw);
    ctrlVelYawratePub.publish(controlVelYawRate);
    ros::Duration elapsed_time = ros::Time::now() - start_time;

    Vz_current = Vz_start + gravity*(elapsed_time.toSec());
    ROS_INFO("x y and z %f %f %f",x,y,Vz_current);
    }


   landing_initiate();
   //arm_motors();
   return true;


}

///////////////////////////////////////////////////////////////////////////"hop_step"////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Mission::hop_step(double xcr, double ycr, double zcr, double tc)

{
  ROS_INFO("Executing Step");
  double z_vel_c = hop.z_vel-1.62*tc;
  ROS_INFO("%lf, %lf, %lf", xcr,ycr,zcr);
  // if((xcr>0.2)||(ycr>0.2)||(zcr>0.2))
  //   {
        ROS_INFO("In Bound");
        hop_fill_vel(hop.x_vel,hop.y_vel,hop.z_vel-1.62*tc,0);
        ROS_INFO("%lf, %lf, %lf", hop.x_vel,hop.y_vel,z_vel_c);
        if(z_vel_c<=(-0.8)*hop.z_vel) {
          ROS_INFO("to landing");
          landing_initiate();
          return true;

        }
        else
        {
          return false;
        }
  //   }
  // else {return true;}
  // return false;
}
//////////////////////////////////////////////////////////////////////////"custom landing sequence"////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////"hop_fill_vel"///////////////////////////////////////////////////////////////////////////////////////

void Mission::hop_fill_vel(double Vx, double Vy, double Vz, double yaw)
{

  sensor_msgs::Joy controlVelYawRate;
  controlVelYawRate.axes.push_back(Vx);
  controlVelYawRate.axes.push_back(Vy);
  controlVelYawRate.axes.push_back(Vz);
  controlVelYawRate.axes.push_back(yaw);
  ctrlVelYawratePub.publish(controlVelYawRate);
  ROS_INFO("PUBLISHING vELOCITY");
}

/////////////////////////////////////////////////////////////"hop_vel_pos"///////////////////////////////////////////////////////////////////////////////////////

int Mission::hop_vel_pos(float x, float y, float z, float yaw)
{
  ROS_INFO("hopex_vel_pos called");
  std::vector<std::vector<float>> pos_matrix;
  hop_pos.reset();
  hop_pos.start_gps_location = current_gps;
  hop_pos.start_local_position = current_local_pos;
  int steps =  create_position_matrix(pos_matrix, x, y, z, yaw);
  hop_pos.setTarget(x,y,z,yaw);
  hop_pos.state = 2;
  hop_pos.start_flag = true;
  while(!hop_pos.finished)
  {
   //ROS_INFO(" still in finish loop");
    ros::spinOnce();
  }
  hop_pos.finished = false;
  landing_initiate();


}

/////////////////////////////////////////////////////////////"hopex_to_pos"///////////////////////////////////////////////////////////////////////////////////////

int Mission::hopex_to_pos(float x, float y, float z, float yaw)
{
  ROS_INFO("hopex_to_pos called");
  std::vector<std::vector<float>> pos_matrix;
  hop_pos.start_gps_location = current_gps;
  hop_pos.start_local_position = current_local_pos;
  int steps =  create_position_matrix(pos_matrix, x, y, z, yaw);

  for(int i=0; i<steps; i++)
  {

    ROS_INFO("vectors %f, %f, %f,%f ",pos_matrix[i][0], pos_matrix[i][1], pos_matrix[i][2], pos_matrix[i][3] );

  }
  ROS_INFO("step %d", steps);
  for(int i=0 ;  i<(steps -4)  ; i++)
  {

    hop_pos.reset();
    hop_pos.start_gps_location = current_gps;
    hop_pos.start_local_position = current_local_pos;
    hop_pos.setTarget(pos_matrix[i][0], pos_matrix[i][1], pos_matrix[i][2], pos_matrix[i][3]);
    hop_pos.state = 1;
    while(!hop_pos.finished)
    {
     //ROS_INFO(" still in finish loop");
      ros::spinOnce();
    }
    landing_initiate();
    hop_pos.finished = false;

  }
return true;
}

////////////////////////////////////////////////////////////////////////////"create_position_matrix"/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Mission::create_position_matrix(std::vector<std::vector<float>> &pos_matrix, float x, float y, float z, float yaw)  // not defined
{
  ROS_INFO("hopex_to_pos called");
//  hop_pos.start_local_position = current_local_pos;
  float del_x = x -  start_local_position.x;
  float del_y = y -  start_local_position.y;
  float del_z = z -  start_local_position.z;
  int   hop_steps_count= 0;
  float step_min = (abs(del_x)>=abs(del_y)) ? abs(del_y) : abs(del_x);
  hop_steps_count = 2*step_min;
  float x_sstep = del_x/hop_steps_count;
  float y_sstep = del_y/hop_steps_count;
  float z_sstep = del_z/hop_steps_count;
  ROS_INFO("%d steps count", hop_steps_count);
  pos_matrix.resize(hop_steps_count);
  for(int i=0; i<hop_steps_count; i++)
  {
    //pos_matrix[i].resize(4);
    pos_matrix[i].push_back(x_sstep);
    pos_matrix[i].push_back(y_sstep);
    pos_matrix[i].push_back(z_sstep);
    pos_matrix[i].push_back(0);
    x_sstep = x_sstep + del_x/hop_steps_count;
    y_sstep = y_sstep + del_y/hop_steps_count;
    z_sstep = (hop_steps_count/2 > i) ? (z_sstep + del_z/hop_steps_count) : (z_sstep - del_z/hop_steps_count);
    ROS_INFO("vectors %f, %f, %f,%f ",pos_matrix[i][0], pos_matrix[i][1], pos_matrix[i][2], pos_matrix[i][3] );

  }
  return hop_steps_count;


}


/////////////////////////////////////////////////////////////"landing_initiate"///////////////////////////////////////////////////////////////////////////////////////
bool landing_initiate(void)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = 6;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("landing failed");
    return false;
  }

  return true;
}
/////////////////////////////////////////////////////////////"arm_motors and disarm_motors"///////////////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////"optimization_function"///////////////////////////////////////////////////////////////////////////////////////

double optimization_function(double x) // not yet prototyped
{
  //this function is intentended to call from the
  return (((hop.Ry)/x)*((hop.Ry)/x) + ((hop.Rx)/x)*((hop.Rx)/x) + (hop.Rz/x + 1.62*x/2)*(hop.Rz/x + 1.62*x/2));
}

/////////////////////////////////////////////////////////////////"set_filter_main////////////////////////////////////////////////////////////////////////////////////////////"

void set_filter_main()    // prototyped in the kalman_filter_spacetrex header
{ int m = 3, n = 3;
  double dt = 0.01;
  ROS_INFO("to set up KF");
  Eigen::Matrix3d A(m,n); A << 1,0,0,
                               0,1,0,
                               0,0,1; //system matrix

  Eigen::MatrixXd B(n,1); B << 0, 0, -0.0162;   //Control Input Matrix

  Eigen::MatrixXd C(m,n); C << 1,0,0,
                               0,1,0,
                               0,0,1; //Output Matrix

  Eigen::MatrixXd K(m,n); K <<0.3229,    0.3229,    0.3229,
                              0.3229,    0.3229,    0.3229,
                              0.3229,    0.3229,    0.3229;

  Eigen::MatrixXd Q(m,n); Q << 1e-1, 1e-1, 1e-1,
                               1e-1, 1e-1, 1e-1,
                               1e-1, 1e-1, 1e-1;

  Eigen::MatrixXd R(m,n); R << 0.01, 0, 0,
                               0, 0.01, 0,
                               0, 0, 0.01;
  Eigen::MatrixXd P(m,n); P << 0.1, 0, 0,
                               0, 0.1, 0,
                               0, 0, 0.1;
  Eigen::MatrixXd P0(m,n); P0 << 0.1, 0, 0,
                               0, 0.1, 0,
                               0, 0, 0.1;


  kfs.set_filter(dt,A,B,C,Q,K,R,P,P0);

  ROS_INFO("filter is now set");
  std::cout<<kfs.setup_done;

}

//////////////////////////////////////////////////////////////////////////"getVelocity_callback"///////////////////////////////////////////////////////////////////////////////////////

void getVelocity_callback(const geometry_msgs::Vector3Stamped& vel_from_sdk) // prototyped in the flight control header
{
  ROS_INFO("In Velocity Callback loop");


  if(kfs.setup_done)
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
      ROS_INFO("kalman_filter Setup-done is now up");
      kfs.t_sec = vel_from_sdk.header.stamp.sec;
      kfs.t_nsec = vel_from_sdk.header.stamp.nsec;
      double t_c = kfs.t_sec + 0.000000001*kfs.t_nsec;
      Eigen::Vector3d vel_rtk(vel_from_sdk.vector.x, vel_from_sdk.vector.y ,vel_from_sdk.vector.z);
      kfs.predict();
      kfs.estimate(vel_rtk);
      kfs.t= kfs.t + kfs.dt_ ;

      ROS_INFO("%lf,%lf,%lf", kfs.xhat[0],kfs.xhat[1],kfs.xhat[2]);

      double xcr = hop.Rx - kfs.xhat[0]*kfs.t;
      double ycr = hop.Ry - kfs.xhat[1]*kfs.t;
      double zcr = hop.Rz - (kfs.xhat[2]+1.62*kfs.t)*kfs.t + (1/2)*1.62*kfs.t*kfs.t;
      bool to_stop = hop.hop_step(xcr,ycr,zcr,kfs.t);
      ROS_INFO(to_stop? "true":"FALSE");
      if(to_stop)
      {
        kfs.setup_done = false;
        hop.finished = true;
        hop.land = true;
      }
    else
    {hop.finished = false;}


    kfs.t_pre = kfs.t_c;
    //hopper_vel.hop_fill_vel(hop.x_vel, hop.y_vel, hop.z_vel);
   }
  }


}

//////////////////////////////////////////////////////////////////////////////////"set_optimum_velocity"/////////////////////////////////////////////////////////////////////////////// b

bool set_optimum_velocity()   //not yet prototyped
{
    if(!hop.start_flag)
    {
      ROS_INFO("Please Set Object hop from mission class first");
      return false;
    }
    else
    {
      ROS_INFO("Optimization starting");
      double t;
      const double begin = 0.1;
      //double begin1 = 0.1;
      double& starting_point = hop.optimum_time;
      const double end = 10;
      const double eps = 0.1;
      const long max_iter = 100;
      const double initial_search_radius = 0.1;
      t = dlib::find_min_single_variable(optimization_function, starting_point, begin, end, eps, max_iter, initial_search_radius);
      hop.x_vel = (hop.Rx/hop.optimum_time);
      hop.y_vel = (hop.Ry/hop.optimum_time);
      hop.z_vel = (hop.Rz/hop.optimum_time + 1.62*hop.optimum_time/2);
      ROS_INFO("optimization over %lf", hop.optimum_time);
      ROS_INFO("initial velocity %lf,%lf,%lf", hop.x_vel,hop.y_vel,hop.z_vel);
      return true;
    }



}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
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

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */
void Mission::Hop_step()
{
  ROS_INFO("entered step");
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  zCmd = start_local_position.z + target_offset_z;


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter ==1)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.25 &&
      std::abs(yOffsetRemaining) < 0.25 &&
      std::abs(zOffsetRemaining) < 0.25
    /*std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad*/)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter

    inbound_counter ++;
  ROS_INFO("%d break counter", inbound_counter);
 }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 10)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;

  }
}

/////////////////////////////////////////////////////////////////////////////////////"Hop_step_vel_pos"////////////////////////////////////////////////////////////////////////////////////

void Mission::Hop_step_vel_pos()
{
  ROS_INFO("entered step");
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  zCmd = start_local_position.z + target_offset_z;

if(start_flag)
{
  x_vel = xOffsetRemaining;
  y_vel = yOffsetRemaining;
  z_vel = 2*sqrt(x_vel*x_vel + y_vel*y_vel);
//  yaw =0;
  start_flag = false;
//  ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", x_vel, y_vel, z_vel);

}
if(abs(xOffsetRemaining)>=0.5||abs(yOffsetRemaining)>=0.5&&abs(zOffsetRemaining)>=0.5)
{
sensor_msgs::Joy controlVelYawRate;
controlVelYawRate.axes.push_back(x_vel);
controlVelYawRate.axes.push_back(y_vel);
controlVelYawRate.axes.push_back(z_vel_current);
controlVelYawRate.axes.push_back(0);
ctrlVelYawratePub.publish(controlVelYawRate);

//ros::Duration elapsed_time = ros::Time::now() - start_time;

z_vel_current = z_vel + gravity*(0.01*vel_counter);
vel_counter++;
//ROS_INFO("x y and z %f %f %f",x,y,Vz_current);
}
else{
  if (break_counter ==1)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.25 &&
      std::abs(yOffsetRemaining) < 0.25 &&
      std::abs(zOffsetRemaining) < 0.25
    /*std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad*/)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter

    inbound_counter ++;
  ROS_INFO("%d break counter", inbound_counter);
 }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 10)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;

  }
}
}


//-------------------xxxxxx---------------------xxxxxx-----------------------xxxxxxx------------------------xxxxxxxx-----------------------xxxxxx----------------------------xxxxxxx
void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;
  ROS_INFO("-----x=%f, y=%f, z=%f, target..", target.latitude,target.longitude, target.altitude);
  ROS_INFO("-----x=%f, y=%f, z=%f, origin..", origin.latitude,origin.longitude, origin.altitude);
  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


/////////////////////////////////////////////////////////////"quternions to euler angles"///////////////////////////////////////////////////////////////////////////////////////


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)   //from gps_callback
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;
  ROS_INFO("GPS callback called");
  // Down sampled to 50Hz loop
  if(true)
  {
    ROS_INFO("GPS in 1st if");
    start_time = ros::Time::now();
    switch(hop_pos.state)
    {
      ROS_INFO("GPS callback in switch case");
      case 0:
        break;

      case 1:
        if(!hop_pos.finished)
        {
          ROS_INFO("calling step loop");
          hop_pos.Hop_step();
        }
        else
        {
          hop_pos.reset();
          hop_pos.start_gps_location = current_gps;
          hop_pos.start_local_position = current_local_pos;
          //hop_pos.setTarget(20, 0, 0, 0);
          hop_pos.state = 1;
          ROS_INFO("##### Start route %d ....", hop_pos.state);
        }
        break;

      case 2:
        if(!hop_pos.finished)
        {
          hop_pos.Hop_step_vel_pos();
        }
        else
        {
          hop_pos.reset();
          hop_pos.start_gps_location = current_gps;
          hop_pos.start_local_position = current_local_pos;
          hop_pos.setTarget(0, -20, 0, 0);
          hop_pos.state = 2;
          ROS_INFO("##### Start route %d ....", hop_pos.state);
        }
        break;
      case 3:
        if(!hop_pos.finished)
        {
          hop_pos.Hop_step();
        }
        else
        {
          hop_pos.reset();
          hop_pos.start_gps_location = current_gps;
          hop_pos.start_local_position = current_local_pos;
          hop_pos.setTarget(-20, 0, 0, 0);
          hop_pos.state = 4;
          ROS_INFO("##### Start route %d ....", hop_pos.state);
        }
        break;
      case 4:
        if(!hop_pos.finished)
        {
          hop_pos.Hop_step();
        }
        else
        {
          ROS_INFO("##### Mission %d Finished ....", hop_pos.state);
          hop_pos.state = 0;
        }
        break;
    }
  }
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */

bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
