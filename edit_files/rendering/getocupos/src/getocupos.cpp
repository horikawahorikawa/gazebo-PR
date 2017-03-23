#include "../include/OculusBar.h"
#include "../include/getocupos.h"
//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
bool OculusBar::isRosInit = false;

//------------------------------------------------------------------------------
OculusBar::OculusBar(): argc(), argv()
{
  try
  {
    if (!isRosInit)
    {
      ros::init(argc, argv, "oculus_plugin");
      isRosInit=true;
      cout << "Success: connecting roscore.." << endl;
    }
  }
  catch(...)
  {
    cout << "Error: ros init" << endl;
  }

  // ros nodehandle, topic, service init
  static ros::NodeHandle nh;

  //------------------------------------------------------------------------------
  choice_in = 0x00;
  reset = false;
}

//------------------------------------------------------------------------------
OculusBar::~OculusBar()
{
}

//------------------------------------------------------------------------------
#if 1

void OculusBar::onOculusReset()
{
  reset = true;
}


void OculusBar::onOculusDK2()
{
  choice_in = 0x04;
  static boost::thread t(boost::bind(&OculusBar::rosOn, this));
}

#endif
//------------------------------------------------------------------------------
void OculusBar::rosOn()
{
  static ros::Rate loop_rate(10); // 0.1sec

 //Moverio Or Oculus ?
  string choice;
/*
  do{
    cout << "Oculus (O/O2) or Moverio (M)?" << endl;
    getline(cin,choice);
  }while(choice != "O" && choice != "O2" && choice != "M");

  if(choice == "M") { choice_in = 0x01; }
  else if(choice == "O") { choice_in = 0x02; }
  else if(choice == "O2") { choice_in = 0x04; }
*/
  if(choice_in == 0x00) choice_in = 0x04;

  switch(choice_in){
  case 0x01:
    cout << "Start traking movirio" << endl;
    break;
  case 0x02:
    cout << "Start traking oculus" << endl;
    break;
  case 0x04:
    cout << "Start traking oculus DK2" << endl;
    break;
  case 0x08:
    cout << "Start traking oculus DK3" << endl;
    break;
  }

  //Init ROS node
  OculusDb od(choice_in, this);
  std::cout << "aa" << std::endl;
  while (ros::ok())
  {
    if(reset) {
      od.reset();
      reset = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//------------------------------------------------------------------------------
