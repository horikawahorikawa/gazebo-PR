#ifndef  OCULUS_BAR_H
#define  OCULUS_BAR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tms_msg_db/TmsdbStamped.h>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <sstream>
#include <fstream>
#include <time.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <sstream>
#include <string>
#include "OculusBar.h"
#include "../include/OculusBar.h"

//------------------------------------------------------------------------------
#define deg2rad(x) ((x)*M_PI/180.0)

//------------------------------------------------------------------------------
using std::string;
using namespace std;
//-----------

//------------------------------------------------------------------------------
class OculusDb
{
//------------------------------------------------------------------------------
private:
  // ROS NodeHandle
  ros::NodeHandle nh;
  // ROS Topic Subscriber
  ros::Subscriber data_sub;
  //Using Moverio?
  char choice;
  bool resetflg;
  OculusBar* ocubar;

//------------------------------------------------------------------------------
public:
//------------------------------------------------------------------------------
  //In order to avoid some display gliches if Vicon cannot track Oculus anymore
  float x_old;
  float y_old;
  float z_old;

  //------------------------------------------------------------------------------
  //filter for Vicon angles data
  float lpfilter(float data, float filterVal, float filteredVal){
    filteredVal = (data * (1 - filterVal)) + (filteredVal * filterVal);
    return filteredVal;
  }

  void reset() { resetflg = true;}

  OculusDb(char choice, OculusBar* pocubar) :
      choice(choice)
  {
    //Init Vicon Stream
//    ROS_ASSERT(init_oculusdb());
    ocubar = pocubar;
    x_old = 2;
    y_old = 2;
    z_old = 1.5;

    resetflg = true;

    // Subscriber for tms_db_data topic
    data_sub = nh.subscribe("tms_db_data", 100, &OculusDb::ocMoveCallback, this);
  }

  //----------------------------------------------------------------------------
  ~OculusDb()
  {
//    ROS_ASSERT(shutdown_oculusdb());
    nh.shutdown();
  }

//------------------------------------------------------------------------------
private:
  bool init_oculusdb()
  {
    return true;
  }

  //----------------------------------------------------------------------------
  bool shutdown_oculusdb()
  {
    return true;
  }

  //----------------------------------------------------------------------------
  // ocMoveCallback function
  void ocMoveCallback(const tms_msg_db::TmsdbStampedConstPtr& msg){

    static tf::TransformBroadcaster brOc;
    //Oculus or Moverio
    float x  = 0;
    float y  = 0;
    float z  = 1.5;
    float rr = 0;
    float rp = 0;
    float ry = 0;

    //id of the device used (Moverio or Oculus)
    int id;

    if(choice == 0x01) { id = 1002; } // moverio
    else if (choice == 0x02) { id = 1001; } // Oculus
    else if (choice == 0x04) { id = 1001; } // Oculus DK2
    else std::cerr << "Unexpected choice" << std::endl;

    int32_t msg_size = msg->tmsdb.size();
//    std::cout << msg_size <<","<< id << std::endl;

    for(int32_t i=0; i<msg_size; i++)
    {
        if( msg->tmsdb[i].id == id ){
          //Get position form the data basis
          x = msg->tmsdb[i].x / 1000;
          y = msg->tmsdb[i].y / 1000;
          z = msg->tmsdb[i].z / 1000;

          if(choice == 0x01){ // if moverio chosen
            //filter angles' data
            ry = round(lpfilter((msg->tmsdb[i].ry)*.9+(ry*.1), 0.005 , ry));
            rp = round(lpfilter((msg->tmsdb[i].rr)*.9+(rp*.1), 0.5 , rp)); // there is a problem I don't understand in the angles equivalence;

            //reajustment, otherwise, we cannot reach every angle between 0 and 360 degrees
            ry = 180*ry/161;
            if(ry > 0) { rp = -180*rp/80; }
            else {rp = 180*rp/80; }
          } else {
            rr = msg->tmsdb[i].rr;
            rp = msg->tmsdb[i].rp;
            ry = msg->tmsdb[i].ry;
          }

          //if Vicon cannot track Oculus' markers anymore. In order to avoid some display glitches

          if( x == 0 && y == 0 && z == 0 )
          {
            x = x_old / 1000;
            y = y_old / 1000;
            z = z_old / 1000;
          }

          //transformation to send to Rviz
          tf::Transform transform;
          transform.setOrigin( tf::Vector3(x, y, z) );

          double adjustAngle = 0;
          if (choice == 0x04) {
            adjustAngle = -50.0; //2台目の Oculus DK2 の軸がずれている
            //ry = ry + deg2rad(adjustAngle);
            if(ry + deg2rad(adjustAngle) < -3.14)
              ry = M_PI + (ry + deg2rad(adjustAngle) + 3.14);
            else
              ry = ry + deg2rad(adjustAngle);
          }

          tf::Quaternion q;

          q.setRPY(-rr, -rp, ry);

          transform.setRotation(q);

          brOc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "oculus_plugin"));
          brOc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "start_position", "oculus_plugin"));

          ocubar->x = x * 1000;
          ocubar->y = y * 1000;
          ocubar->z = z * 1000;
          ocubar->rr = -rr;
          ocubar->rp = -rp;
          ocubar->ry = ry;
          ocubar->qw = q.getAngle();
          ocubar->qx = q.getAxis()[0];
          ocubar->qy = q.getAxis()[1];
          ocubar->qz = q.getAxis()[2];
          x_old = ocubar->x;
          y_old = ocubar->y;
          z_old = ocubar->z;
          ocubar->x_old = x_old;
          ocubar->y_old = y_old;
          ocubar->z_old = z_old;
        }
    }
  }
};


#endif //OCULUS_PLUGIN_BAR_H
