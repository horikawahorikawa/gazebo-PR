/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <math.h>
#include <unistd.h>

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/skyx/include/SkyX.h"
#include "gazebo/rendering/selection_buffer/SelectionBuffer.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/OculusCameraPrivate.hh"
#include "gazebo/rendering/OculusCamera.hh"
#include "ovr_include/OVR_Math.h"
#include "OVR_CAPI.h"

#include "opencv2/calib3d/calib3d.hpp"

using namespace gazebo;
using namespace rendering;
using namespace OVR;

const float g_defaultNearClip = 0.01f;
const float g_defaultFarClip = 500.0f;

//////////////////////////////////////////////////
OculusCamera::OculusCamera(const std::string &_name, ScenePtr _scene)
  : Camera(_name, _scene), dataPtr(new OculusCameraPrivate)
{
  this->dataPtr->dist0 = cv::Mat(720, 960, CV_8UC3);
  this->dataPtr->dist1 = cv::Mat(720, 960, CV_8UC3);
  //this->dataPtr->dist0 = cv::Mat(480, 640, CV_8UC3);
  //this->dataPtr->dist1 = cv::Mat(480, 640, CV_8UC3);
  this->dataPtr->rightCapture.setDesiredSize(640, 480);
  this->dataPtr->rightCapture.setDesiredFrameRate(30);
  this->dataPtr->rightCapture.setVerbose(true);
  this->dataPtr->rightCapture.open(0);

  this->dataPtr->leftCapture.setDesiredSize(640, 480);
  this->dataPtr->leftCapture.setDesiredFrameRate(30);
  this->dataPtr->leftCapture.setVerbose(true);
  this->dataPtr->leftCapture.open(1);

  if (!this->dataPtr->rightCapture.isOpen()){
    std::cerr << "Failed to open camera0" << std::endl;
  }

  if (!this->dataPtr->leftCapture.isOpen()){
    std::cerr << "Failed to open camera1" << std::endl;
    stereocam = false;
  }
  else{
    for (int i = 0; i < 20; ++i){
      this->dataPtr->rightCapture.grab();
      usleep(1000);
    }
    for (int i = 0; i < 20; ++i){
      this->dataPtr->leftCapture.grab();
      usleep(1000);
    }
      stereocam = true;
  }
  if(stereocam == true){
        cv::FileStorage cvfsR("cameraRparam.xml", CV_STORAGE_READ);
        cv::FileStorage cvfsL("cameraLparam.xml", CV_STORAGE_READ);
 		std::cout<<"Read camera parameters"<<std::endl;

        cvfsR["camera_matrix"] >> cameraMatrixR;
        cvfsR["distortion_coefficients"] >> distCoeffsR;
        cvfsL["camera_matrix"] >> cameraMatrixL;
        cvfsL["distortion_coefficients"] >> distCoeffsL;

        cvfsR.release();
        cvfsL.release();

        cv::Size imageSize(640, 480);

		char dir[255];
		getcwd(dir,255);
		std::cout<<"Current Directory : "<<dir<<std::endl;
		std::cout<<cameraMatrixR<<std::endl;
        cv::initUndistortRectifyMap(cameraMatrixR, distCoeffsR, cv::Mat(),
        cv::getOptimalNewCameraMatrix(cameraMatrixR, distCoeffsR, imageSize, 1, imageSize, 0),
                                    imageSize, CV_16SC2, map1R, map2R);

        cv::initUndistortRectifyMap(cameraMatrixL, distCoeffsL, cv::Mat(),
        cv::getOptimalNewCameraMatrix(cameraMatrixL, distCoeffsL, imageSize, 1, imageSize, 0),
                                    imageSize, CV_16SC2, map1L, map2L);

    // prev_time = std::chrono::system_clock::now();
		std::cout<<"Prepared distorting camera"<<std::endl;
  }

    //cv::namedWindow("MJPEG0", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("MJPEG1", CV_WINDOW_AUTOSIZE);
  temp = 0;
  reset_ovr_position = true;

  ovr_Initialize();
  this->dataPtr->hmd = ovrHmd_Create(0);
  if (!this->dataPtr->hmd)
  {
    gzerr << "Oculus Rift not detected. "
          << "Oculus error["
          << ovrHmd_GetLastError(NULL) << "]. "
          << "Is the oculusd service running?\n"
          << "Did you copy the udev rules from the oculussdk repo?\n"
          << "See: http://gazebosim.org/tutorials?tut=oculus"
          << std::endl;
    return;
  }

  if (this->dataPtr->hmd->ProductName[0] == '\0')
  {
    gzerr << "Oculus Rift detected, display not enabled. "
          << "Oculus error["
          << ovrHmd_GetLastError(NULL) << "]. "
          << std::endl;
    return;
  }

  // These are the suggested refresh rates for dk1 and dk2
  //   dk1: 60Hz
  //   dk2: 75Hz
  switch (this->dataPtr->hmd->Type)
  {
    case ovrHmd_DK1:
      // A little bit extra for safety
      this->SetRenderRate(70.0);
      break;
    case ovrHmd_DK2:
      // A little bit extra for safety
      this->SetRenderRate(80.0);
      break;
    case ovrHmd_None:
      gzerr << "Unable to handle Oculus with type 'None'\n";
      return;
    case ovrHmd_DKHD:
      gzerr << "Unable to handle Oculus with type 'DKHD'\n";
      return;
    case ovrHmd_Other:
      gzerr << "Unable to handle Oculus with type 'Other'\n";
      return;
    default:
      gzerr << "Unknown Oculus type '" << this->dataPtr->hmd->Type << "'\n";
      return;
  };

  // Log some useful information
  gzmsg << "Oculus Rift found." << std::endl;
  gzmsg << "\tType: " << this->dataPtr->hmd->Type << std::endl;
  gzmsg << "\tProduct Name: " << this->dataPtr->hmd->ProductName << std::endl;
  gzmsg << "\tProduct ID: " << this->dataPtr->hmd->ProductId << std::endl;
  gzmsg << "\tFirmware: " << this->dataPtr->hmd->FirmwareMajor << "."
    << this->dataPtr->hmd->FirmwareMinor << std::endl;
  gzmsg << "\tResolution: " << this->dataPtr->hmd->Resolution.w << "x"
    << this->dataPtr->hmd->Resolution.h << std::endl;
  gzmsg << "\tPosition tracking: "
    << (this->dataPtr->hmd->TrackingCaps & ovrTrackingCap_Position)
    << std::endl;

  // Start the sensor which informs of the Rift's pose and motion
  if (!ovrHmd_ConfigureTracking(this->dataPtr->hmd, ovrTrackingCap_Orientation
      | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0))
  {
    gzerr << "Tracking capability not found. "
      << "The Oculus viewpoint will not move.\n";
  }

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->controlSub = this->dataPtr->node->Subscribe("~/world_control",
                                           &OculusCamera::OnControl, this);

  // Oculus is now ready.
  this->dataPtr->ready = true;
}

//////////////////////////////////////////////////
OculusCamera::~OculusCamera()
{
  if(stereocam == true){
    this->dataPtr->rightCapture.close();
    this->dataPtr->leftCapture.close();
  }
  if(vicontracker == true)
  {
    delete ocubar;
  }

  RenderEngine::Instance()->root->destroySceneManager(
      this->dataPtr->externalSceneManager);

  ovrHmd_Destroy(this->dataPtr->hmd);
  ovr_Shutdown();

  this->connections.clear();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
void OculusCamera::Load(sdf::ElementPtr _sdf)
{
  if (this->Ready())
    Camera::Load(_sdf);
}

//////////////////////////////////////////////////
void OculusCamera::OnControl(ConstWorldControlPtr &_data)
{
  if (_data->has_reset() && _data->reset().has_all() && _data->reset().all())
  {
    this->ResetSensor();
  }
}

//////////////////////////////////////////////////
void OculusCamera::Load()
{
  if (this->Ready())
    Camera::Load();
}

//////////////////////////////////////////////////
void OculusCamera::Init()
{
  if (!this->Ready())
    return;

  Camera::Init();

  // Oculus
    this->dataPtr->rightCamera = this->scene->GetManager()->createCamera(
      "OculusUserRight");
    this->dataPtr->rightCamera->pitch(Ogre::Degree(90));

    // Don't yaw along variable axis, causes leaning
    this->dataPtr->rightCamera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
    this->dataPtr->rightCamera->setDirection(1, 0, 0);

    this->sceneNode->attachObject(this->dataPtr->rightCamera);

    this->dataPtr->rightCamera->setAutoAspectRatio(false);
    this->camera->setAutoAspectRatio(false);

    this->dataPtr->rightCamera->setNearClipDistance(g_defaultNearClip);
    this->dataPtr->rightCamera->setFarClipDistance(g_defaultFarClip);

    this->camera->setNearClipDistance(g_defaultNearClip);
    this->camera->setFarClipDistance(g_defaultFarClip);

  // Careful when setting this value.
  // A far clip that is too close will have bad side effects on the
  // lighting. When using deferred shading, the light's use geometry that
  // trigger shaders. If the far clip is too close, the light's geometry is
  // clipped and wholes appear in the lighting.
  switch (RenderEngine::Instance()->GetRenderPathType())
  {
    case RenderEngine::VERTEX:
      this->SetClipDist(g_defaultNearClip, g_defaultFarClip);
      break;

    case RenderEngine::DEFERRED:
    case RenderEngine::FORWARD:
      this->SetClipDist(g_defaultNearClip, g_defaultFarClip);
      break;

    default:
      this->SetClipDist(g_defaultNearClip, g_defaultFarClip);
      break;
  }

}
std::chrono::system_clock::time_point Oculus_head_time;
//////////////////////////////////////////////////
void OculusCamera::RenderImpl()
{
  //Video images
  if(stereocam == true){
    if (this->dataPtr->rightCapture.grab() && this->dataPtr->leftCapture.grab()){
      this->dataPtr->rightCapture.mjpeg(this->dataPtr->mjpeg0);
      this->dataPtr->leftCapture.mjpeg(this->dataPtr->mjpeg1);
    }

    cv::flip(this->dataPtr->mjpeg0, this->dataPtr->mjpeg0, -1);
    cv::flip(this->dataPtr->mjpeg1, this->dataPtr->mjpeg1, -1);

    // cv::Mat temp_resize(480, 640, CV_8UC3);
    // cv::resize(this->dataPtr->mjpeg0, this->dataPtr->mjpeg0, temp_resize.size(), 0, 0);
    // cv::resize(this->dataPtr->mjpeg1, this->dataPtr->mjpeg1, temp_resize.size(), 0, 0);

    cv::remap(this->dataPtr->mjpeg0, this->dataPtr->mjpeg0, map1R, map2R, cv::INTER_LINEAR);
    cv::remap(this->dataPtr->mjpeg1, this->dataPtr->mjpeg1, map1L, map2L, cv::INTER_LINEAR);

    //Ogre::PixelFormat ogrePixelFormat = Ogre::PF_BYTE_BGR;
    cv::resize(this->dataPtr->mjpeg0, this->dataPtr->dist0, this->dataPtr->dist0.size(), 0, 0);
    cv::resize(this->dataPtr->mjpeg1, this->dataPtr->dist1, this->dataPtr->dist1.size(), 0, 0);

    Ogre::TexturePtr Right_texture = Ogre::TextureManager::getSingleton().createManual(
          "RTexture",
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          Ogre::TEX_TYPE_2D,
          this->dataPtr->ovrtextureRightWidth, this->dataPtr->ovrtextureRightHeight, 0, Ogre::PF_BYTE_BGR,
          Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    Ogre::TexturePtr Left_texture = Ogre::TextureManager::getSingleton().createManual(
          "LTexture",
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          Ogre::TEX_TYPE_2D,
          this->dataPtr->ovrtextureLeftWidth, this->dataPtr->ovrtextureLeftHeight, 0, Ogre::PF_BYTE_BGR,
          Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    // OGRE TEXTURE LOCK
    // get the texture pixel buffer
    Ogre::HardwarePixelBufferSharedPtr R_pixelBuffer = Right_texture->getBuffer();
    Ogre::HardwarePixelBufferSharedPtr L_pixelBuffer = Left_texture->getBuffer();

   // lock the pixel buffer and get a pixel box
   R_pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
   L_pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
   const Ogre::PixelBox& R_pixelBox = R_pixelBuffer->getCurrentLock();
   const Ogre::PixelBox& L_pixelBox = L_pixelBuffer->getCurrentLock();

   Ogre::uint8* R_pDest = static_cast<Ogre::uint8*>(R_pixelBox.data);
   Ogre::uint8* L_pDest = static_cast<Ogre::uint8*>(L_pixelBox.data);

   // create some temp variables
   int x=0;
   int y=0;
   int x1=0;
   int y1=0;
   unsigned long byteSRC=0;
   unsigned long byteDST=0;


       // traverse all pixels...
       for (y=0; y<this->dataPtr->dist0.rows; y++)
          for (x=0; x<this->dataPtr->dist0.cols; x++)
          {
             x1 = this->dataPtr->ovrtextureLeftWidth / 2  - this->dataPtr->dist0.cols / 2 + x;
             y1 = this->dataPtr->ovrtextureLeftHeight / 2 - this->dataPtr->dist0.rows / 2 + y;
             byteSRC = x*3 + y*this->dataPtr->dist0.cols*3; // original has only 3 channels
             byteDST = x1*4 + y1*this->dataPtr->ovrtextureRightWidth*4; // destination has alpha channel too

             // copy 3 bytes from source to destination + 1 invented byte
             R_pDest[byteDST]=this->dataPtr->dist0.data[byteSRC];//b
             R_pDest[byteDST+1]=this->dataPtr->dist0.data[byteSRC+1];//g
             R_pDest[byteDST+2]=this->dataPtr->dist0.data[byteSRC+2];//r
             R_pDest[byteDST+3]=255;//a

             L_pDest[byteDST]=this->dataPtr->dist1.data[byteSRC];//b
             L_pDest[byteDST+1]=this->dataPtr->dist1.data[byteSRC+1];//g
             L_pDest[byteDST+2]=this->dataPtr->dist1.data[byteSRC+2];//r
             L_pDest[byteDST+3]=255;//a
          }
       // UNLOCK EVERYTHING!
       // unlock the pixel buffer
       R_pixelBuffer->unlock();
       L_pixelBuffer->unlock();

       ovrHmd_BeginFrameTiming(this->dataPtr->hmd, this->dataPtr->frameIndex);

       // Create background material
       Ogre::MaterialPtr leftcam_material = Ogre::MaterialManager::getSingleton().create("leftBackground", "General");
       leftcam_material->getTechnique(0)->getPass(0)->createTextureUnitState("LTexture");
       leftcam_material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
       leftcam_material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
       leftcam_material->getTechnique(0)->getPass(0)->setLightingEnabled(false);

       // Create background rectangle covering the whole screen
       Ogre::Rectangle2D* leftrect = new Ogre::Rectangle2D(true);
       leftrect->setCorners(-1.0, 1.0, 1.0, -1.0);
       leftrect->setMaterial("leftBackground");

       // Render the background before everything else
       leftrect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);

       // Use infinite AAB to always stay visible
       Ogre::AxisAlignedBox aabInf;
       aabInf.setInfinite();
       leftrect->setBoundingBox(aabInf);

       // Attach background to the scene
       this->sceneNode->attachObject(leftrect);
       this->dataPtr->renderTextureLeft->getBuffer()->getRenderTarget()->update();

       delete leftrect;
       Ogre::MaterialManager::getSingleton().remove("leftBackground");
       Ogre::TextureManager::getSingleton().remove("LTexture");

       // Create background material
       Ogre::MaterialPtr rightcam_material = Ogre::MaterialManager::getSingleton().create("rightBackground", "General");
       rightcam_material->getTechnique(0)->getPass(0)->createTextureUnitState("RTexture");
       rightcam_material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
       rightcam_material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
       rightcam_material->getTechnique(0)->getPass(0)->setLightingEnabled(false);

       // Create background rectangle covering the whole screen
       Ogre::Rectangle2D* rightrect = new Ogre::Rectangle2D(true);
       rightrect->setCorners(-1.0, 1.0, 1.0, -1.0);
       rightrect->setMaterial("rightBackground");

       // Render the background before everything else
       rightrect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
       rightrect->setBoundingBox(aabInf);

       // Attach background to the scene
       this->sceneNode->attachObject(rightrect);
       this->dataPtr->renderTextureRight->getBuffer()->getRenderTarget()->update();

       delete rightrect;
       Ogre::MaterialManager::getSingleton().remove("rightBackground");
       Ogre::TextureManager::getSingleton().remove("RTexture");

       this->renderTarget->update();
       ovrHmd_EndFrameTiming(this->dataPtr->hmd);
       this->dataPtr->frameIndex++;


      // std::chrono::duration<double> sec = std::chrono::system_clock::now() - prev_time;
      // prev_time = std::chrono::system_clock::now();
      // double dur = sec.count();
      // //  total_time += dur;
      // //  frame_rate = this->dataPtr->frameIndex / render_time;
      // //  std::cout<<"total_time :"<<total_time<<std::endl;
      // //  std::cout<<"frame_Index :"<<this->dataPtr->frameIndex<<std::endl;
      // std::cout<<"duration_sec :"<<dur<<std::endl;
      // //  std::cout<<"frame_rate :"<<frame_rate<<std::endl<<std::endl;

      // std::cout<<"Render_frame_index :"<<this->dataPtr->frameIndex<<std::endl;
      // std::chrono::duration<double> sec = std::chrono::system_clock::now() - prev_time;
      // double dur = sec.count();
      // std::cout<<"duration_sec :"<<dur<<std::endl;
      // std::chrono::duration<double> sec = std::chrono::system_clock::now() - prev_time;
      // double dur = sec.count();
      // std::cout<<"duration_sec :"<<dur<<std::endl;
      // prev_time = std::chrono::system_clock::now();
  }
  else{
    ovrHmd_BeginFrameTiming(this->dataPtr->hmd, this->dataPtr->frameIndex);

    this->dataPtr->renderTextureLeft->getBuffer()->getRenderTarget()->update();
    this->dataPtr->renderTextureRight->getBuffer()->getRenderTarget()->update();
    this->renderTarget->update();

    ovrHmd_EndFrameTiming(this->dataPtr->hmd);

    this->dataPtr->frameIndex++;
  }
}
//////////////////////////////////////////////////
void OculusCamera::Update()
{
  if (!this->Ready())
    return;

  Camera::Update();

  // std::cout<<"OcuCamUpdate_frame_index :"<<this->dataPtr->frameIndex<<std::endl;

  ovrFrameTiming frameTiming = ovrHmd_GetFrameTiming(this->dataPtr->hmd,
      this->dataPtr->frameIndex);

  ovrTrackingState ts = ovrHmd_GetTrackingState(
      this->dataPtr->hmd, frameTiming.ScanoutMidpointSeconds);

  // Only doing orientation tracking for now. Position tracking is an option
  // for dk2
  if(ts.StatusFlags & ovrStatus_OrientationTracked)
  {
    if (this->dataPtr->oculusTrackingWarned)
    {
      gzmsg << "Oculus: Head tracking enabled.\n";
      this->dataPtr->oculusTrackingWarned = false;
    }

    ovrPosef ovrpose = ts.HeadPose.ThePose;
    Posef ovrpose_ =  ts.HeadPose.ThePose;


    float yaw, pitch, roll;
    ovrpose_.Rotation.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&yaw, &pitch, &roll);

    if(vicontracker){
      current_ovr_roll = roll + M_PI;
      current_ovr_pitch = pitch + M_PI;
      current_ovr_yaw = yaw + M_PI;

      if(reset_ovr_position){
        prev_ovr_x = ocubar->x;
        prev_ovr_y = ocubar->y;
        ovr_sim_roll = ocubar->rr;
        ovr_sim_pitch = ocubar->rp;
        ovr_sim_yaw = ocubar->ry - (10.0 / 180.0 * M_PI);
        current_vicon_yaw = ocubar->ry - (10.0 / 180.0 * M_PI) + init_yaw_offset;
        prev_vicon_yaw = current_vicon_yaw;
        //debug
        init_vicon_roll = ovr_sim_roll;
        init_vicon_pitch = ovr_sim_pitch;
        init_vicon_yaw = ovr_sim_yaw;

        prev_ovr_roll = current_ovr_roll;
        prev_ovr_pitch = current_ovr_pitch;
        prev_ovr_yaw = current_ovr_yaw;

        if(init_vicon_roll != 0 && init_vicon_pitch != 0 && init_vicon_yaw != 0){
          reset_ovr_position = false;
          ovr_sim_roll = ovr_sim_roll + init_roll_offset;
          ovr_sim_pitch = ovr_sim_pitch + init_pitch_offset;
          ovr_sim_yaw = ovr_sim_yaw + init_yaw_offset;

          if(ovr_sim_roll < -M_PI)
            ovr_sim_roll = M_PI + (M_PI + ovr_sim_roll);
          else if(ovr_sim_roll > M_PI)
            ovr_sim_roll = -M_PI + (ovr_sim_roll - M_PI);

          if(ovr_sim_pitch < -M_PI)
            ovr_sim_pitch = M_PI + (M_PI + ovr_sim_pitch);
          else if(ovr_sim_pitch > M_PI)
            ovr_sim_pitch = -M_PI + (ovr_sim_pitch - M_PI);

          if(ovr_sim_yaw < -M_PI)
            ovr_sim_yaw = M_PI + (M_PI + ovr_sim_yaw);
          else if(ovr_sim_yaw > M_PI)
            ovr_sim_yaw = -M_PI + (ovr_sim_yaw - M_PI);
        }
      }
      else{
        //calc_roll_offset
        if(current_ovr_roll == prev_ovr_roll)
          ovr_roll_offset = 0;
        else if(current_ovr_roll > prev_ovr_roll){
          if(prev_ovr_roll < (M_PI / 2) && current_ovr_roll > (3 * M_PI / 2))
            ovr_roll_offset = -(2 * M_PI + prev_ovr_roll - current_ovr_roll);
          else
            ovr_roll_offset = current_ovr_roll - prev_ovr_roll;
        }
        else{
          if(prev_ovr_roll > (3 * M_PI / 2) && current_ovr_roll < (M_PI / 2))
            ovr_roll_offset = 2 * M_PI - prev_ovr_roll + current_ovr_roll;
          else
            ovr_roll_offset = current_ovr_roll - prev_ovr_roll;
        }

        //calc_pitch_offset
        if(current_ovr_pitch == prev_ovr_pitch)
          ovr_pitch_offset = 0;
        else if(current_ovr_pitch > prev_ovr_pitch){
          if(prev_ovr_pitch < (M_PI / 2) && current_ovr_pitch > (3 * M_PI / 2))
            ovr_pitch_offset = -(2 * M_PI + prev_ovr_pitch - current_ovr_pitch);
          else
            ovr_pitch_offset = current_ovr_pitch - prev_ovr_pitch;
        }
        else{
          if(prev_ovr_pitch > (3 * M_PI / 2) && current_ovr_pitch < (M_PI / 2))
            ovr_pitch_offset = 2 * M_PI - prev_ovr_pitch + current_ovr_pitch;
          else
            ovr_pitch_offset = current_ovr_pitch - prev_ovr_pitch;
        }

        //calc_yaw_offset
        if(current_ovr_yaw == prev_ovr_yaw)
          ovr_yaw_offset = 0;
        else if(current_ovr_yaw > prev_ovr_yaw){
          if(prev_ovr_yaw < (M_PI / 2) && current_ovr_yaw > (3 * M_PI / 2))
            ovr_yaw_offset = -(2 * M_PI + prev_ovr_yaw - current_ovr_yaw);
          else
            ovr_yaw_offset = current_ovr_yaw - prev_ovr_yaw;
        }
        else{
          if(prev_ovr_yaw > (3 * M_PI / 2) && current_ovr_yaw < (M_PI / 2))
            ovr_yaw_offset = 2 * M_PI - prev_ovr_yaw + current_ovr_yaw;
          else
            ovr_yaw_offset = current_ovr_yaw - prev_ovr_yaw;
        }

        ovr_sim_roll = ovr_sim_roll + ovr_roll_offset;
        ovr_sim_pitch = ovr_sim_pitch + ovr_pitch_offset;
        ovr_sim_yaw = ovr_sim_yaw + ovr_yaw_offset;

        if(ovr_sim_roll < -M_PI)
          ovr_sim_roll = M_PI + (M_PI + ovr_sim_roll);
        else if(ovr_sim_roll > M_PI)
          ovr_sim_roll = -M_PI + (ovr_sim_roll - M_PI);

        if(ovr_sim_pitch < -M_PI)
          ovr_sim_pitch = M_PI + (M_PI + ovr_sim_pitch);
        else if(ovr_sim_pitch > M_PI)
          ovr_sim_pitch = -M_PI + (ovr_sim_pitch - M_PI);

        if(ovr_sim_yaw < -M_PI)
          ovr_sim_yaw = M_PI + (M_PI + ovr_sim_yaw);
        else if(ovr_sim_yaw > M_PI)
          ovr_sim_yaw = -M_PI + (ovr_sim_yaw - M_PI);

        current_vicon_yaw = ocubar->ry - (10.0 / 180.0 * M_PI) + init_yaw_offset;
        if(current_vicon_yaw < -M_PI)
          current_vicon_yaw = M_PI + (M_PI + current_vicon_yaw);
        else if(current_vicon_yaw > M_PI)
          current_vicon_yaw = -M_PI + (current_vicon_yaw - M_PI);

        if(abs(current_vicon_yaw - prev_vicon_yaw) < 0.174 ||
            abs(current_vicon_yaw - prev_vicon_yaw) > 1.57 * 3)
          prev_vicon_yaw = current_vicon_yaw;
        else
          current_vicon_yaw = prev_vicon_yaw;

        gazebo::math::Quaternion ovr_sim_q(ovr_sim_roll, ovr_sim_pitch, current_vicon_yaw);

        if(abs(ocubar->x - prev_ovr_x) > 1.0 || abs(ocubar->y - prev_ovr_y) > 1.0)
          ovr_sim_q.SetFromEuler(ovr_sim_roll, ovr_sim_pitch, prev_vicon_yaw);

        // std::cout<<"current_vicon_yaw :"<<current_vicon_yaw<<std::endl;
        // std::cout<<"prev_vicon_yaw :"<<prev_vicon_yaw<<std::endl;
        // std::cout<<"ovr_sim_q.yaw :"<<ovr_sim_q.GetYaw()<<std::endl;

        prev_ovr_roll = current_ovr_roll;
        prev_ovr_pitch = current_ovr_pitch;
        prev_ovr_yaw = current_vicon_yaw;

        math::Vector3 oculus_position(ocubar->x, ocubar->y, ocubar->z);
        this->SetWorldPosition(oculus_position);
        this->sceneNode->setOrientation(Ogre::Quaternion(
            ovr_sim_q.w,
            -ovr_sim_q.x,
            -ovr_sim_q.y,
            ovr_sim_q.z));

        prev_ovr_x = ocubar->x;
        prev_ovr_y = ocubar->y;
      }
    }
    else{
      this->sceneNode->setOrientation(Ogre::Quaternion(
          ovrpose.Orientation.w,
          -ovrpose.Orientation.z,
          -ovrpose.Orientation.x,
          ovrpose.Orientation.y));
    }

  }
  else if (!this->dataPtr->oculusTrackingWarned)
  {
    gzwarn << "Oculus: No head tracking.\n\t"
      << "If you do not see a following message about 'Head tracking enabled'"
      << ", then try rebooting while leaving the Oculus turned on."
      << std::endl;
    this->dataPtr->oculusTrackingWarned = true;
  }

  this->sceneNode->needUpdate();
}

//////////////////////////////////////////////////
void OculusCamera::ResetSensor()
{
}

//////////////////////////////////////////////////
bool OculusCamera::Ready()
{
  return this->dataPtr->ready;
}

//////////////////////////////////////////////////
void OculusCamera::PostRender()
{
  Camera::PostRender();
}

//////////////////////////////////////////////////
void OculusCamera::Fini()
{
  Camera::Fini();
}

/////////////////////////////////////////////////
bool OculusCamera::AttachToVisualImpl(VisualPtr _visual,
    bool _inheritOrientation,
    double /*_minDist*/, double /*_maxDist*/)
{
  Camera::AttachToVisualImpl(_visual, _inheritOrientation);
  if (_visual)
  {
    math::Pose origPose = this->GetWorldPose();
    double yaw = _visual->GetWorldPose().rot.GetAsEuler().z;

    double zDiff = origPose.pos.z - _visual->GetWorldPose().pos.z;
    double pitch = 0;

    if (fabs(zDiff) > 1e-3)
    {
      double dist = _visual->GetWorldPose().pos.Distance(
          this->GetWorldPose().pos);
      pitch = acos(zDiff/dist);
    }

    this->Yaw(yaw);
    this->Pitch(pitch);

    math::Box bb = _visual->GetBoundingBox();
    math::Vector3 pos = bb.GetCenter();
    pos.z = bb.max.z;
  }

  return true;
}

//////////////////////////////////////////////////
bool OculusCamera::TrackVisualImpl(VisualPtr _visual)
{
  Camera::TrackVisualImpl(_visual);

  return true;
}

//////////////////////////////////////////////////
unsigned int OculusCamera::GetImageWidth() const
{
  return this->viewport->getActualWidth();
}

//////////////////////////////////////////////////
unsigned int OculusCamera::GetImageHeight() const
{
  return this->viewport->getActualHeight();
}

//////////////////////////////////////////////////
void OculusCamera::Resize(unsigned int /*_w*/, unsigned int /*_h*/)
{
  if (this->viewport)
  {
    this->viewport->setDimensions(0, 0, 0.5, 1);
    this->dataPtr->rightViewport->setDimensions(0.5, 0, 0.5, 1);

    delete [] this->saveFrameBuffer;
    this->saveFrameBuffer = NULL;
  }
}

//////////////////////////////////////////////////
bool OculusCamera::MoveToPosition(const math::Pose &_pose, double _time)
{
  return Camera::MoveToPosition(_pose, _time);
}

//////////////////////////////////////////////////
void OculusCamera::MoveToVisual(const std::string &_name)
{
  VisualPtr visualPtr = this->scene->GetVisual(_name);
  if (visualPtr)
    this->MoveToVisual(visualPtr);
  else
    gzerr << "MoveTo Unknown visual[" << _name << "]\n";
}

//////////////////////////////////////////////////
void OculusCamera::MoveToVisual(VisualPtr _visual)
{
  if (!_visual)
    return;

  if (this->scene->GetManager()->hasAnimation("cameratrack"))
  {
    this->scene->GetManager()->destroyAnimation("cameratrack");
  }

  math::Box box = _visual->GetBoundingBox();
  math::Vector3 size = box.GetSize();
  double maxSize = std::max(std::max(size.x, size.y), size.z);

  math::Vector3 start = this->GetWorldPose().pos;
  start.Correct();
  math::Vector3 end = box.GetCenter() + _visual->GetWorldPose().pos;
  end.Correct();
  math::Vector3 dir = end - start;
  dir.Correct();
  dir.Normalize();

  double dist = start.Distance(end) - maxSize;

  math::Vector3 mid = start + dir*(dist*.5);
  mid.z = box.GetCenter().z + box.GetSize().z + 2.0;

  dir = end - mid;
  dir.Correct();

  dist = mid.Distance(end) - maxSize;

  double yawAngle = atan2(dir.y, dir.x);
  double pitchAngle = atan2(-dir.z, sqrt(dir.x*dir.x + dir.y*dir.y));
  Ogre::Quaternion yawFinal(Ogre::Radian(yawAngle), Ogre::Vector3(0, 0, 1));
  Ogre::Quaternion pitchFinal(Ogre::Radian(pitchAngle), Ogre::Vector3(0, 1, 0));

  dir.Normalize();

  double scale = maxSize / tan((this->GetHFOV()/2.0).Radian());

  end = mid + dir*(dist - scale);

  // dist = start.Distance(end);
  // double vel = 5.0;
  double time = 0.5;  // dist / vel;

  Ogre::Animation *anim =
    this->scene->GetManager()->createAnimation("cameratrack", time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0, this->sceneNode);

  Ogre::TransformKeyFrame *key;

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = strack->createNodeKeyFrame(time);
  key->setTranslate(Ogre::Vector3(end.x, end.y, end.z));
  key->setRotation(yawFinal);

  this->animState =
    this->scene->GetManager()->createAnimationState("cameratrack");

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
  this->prevAnimTime = common::Time::GetWallTime();
}

//////////////////////////////////////////////////
void OculusCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  this->renderTarget = _target;
  this->Oculus();

  Ogre::RenderTexture *rt =
    this->dataPtr->renderTextureLeft->getBuffer()->getRenderTarget();

  rt->addViewport(this->camera);
  rt->getViewport(0)->setClearEveryFrame(true);
  rt->getViewport(0)->setOverlaysEnabled(false);
  rt->getViewport(0)->setShadowsEnabled(true);
  rt->getViewport(0)->setBackgroundColour(
        Conversions::Convert(this->scene->GetBackgroundColor()));
  rt->getViewport(0)->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  RTShaderSystem::AttachViewport(rt->getViewport(0), this->GetScene());

  if (this->GetScene()->GetSkyX() != NULL)
    rt->addListener(this->GetScene()->GetSkyX());

  rt = this->dataPtr->renderTextureRight->getBuffer()->getRenderTarget();
  rt->addViewport(this->dataPtr->rightCamera);
  rt->getViewport(0)->setClearEveryFrame(true);
  rt->getViewport(0)->setShadowsEnabled(true);
  rt->getViewport(0)->setOverlaysEnabled(false);
  rt->getViewport(0)->setBackgroundColour(
        Conversions::Convert(this->scene->GetBackgroundColor()));
  rt->getViewport(0)->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
  RTShaderSystem::AttachViewport(rt->getViewport(0), this->GetScene());

  if (this->GetScene()->GetSkyX() != NULL)
    rt->addListener(this->GetScene()->GetSkyX());

  ovrFovPort fovLeft = this->dataPtr->hmd->DefaultEyeFov[ovrEye_Left];
  ovrFovPort fovRight = this->dataPtr->hmd->DefaultEyeFov[ovrEye_Right];

  float combinedTanHalfFovHorizontal =
    std::max(fovLeft.LeftTan, fovLeft.RightTan);
  float combinedTanHalfFovVertical = std::max(fovLeft.UpTan, fovLeft.DownTan);

  float aspectRatio = combinedTanHalfFovHorizontal / combinedTanHalfFovVertical;

  this->camera->setAspectRatio(aspectRatio);
  this->dataPtr->rightCamera->setAspectRatio(aspectRatio);

  ovrMatrix4f projL = ovrMatrix4f_Projection(fovLeft, 0.001, 500.0, true);
  ovrMatrix4f projR = ovrMatrix4f_Projection(fovRight, 0.001, 500.0, true);

  this->camera->setCustomProjectionMatrix(true,
    Ogre::Matrix4(
      projL.M[0][0], projL.M[0][1], projL.M[0][2], projL.M[0][3],
      projL.M[1][0], projL.M[1][1], projL.M[1][2], projL.M[1][3],
      projL.M[2][0], projL.M[2][1], projL.M[2][2], projL.M[2][3],
      projL.M[3][0], projL.M[3][1], projL.M[3][2], projL.M[3][3]));

  this->dataPtr->rightCamera->setCustomProjectionMatrix(true,
    Ogre::Matrix4(
      projR.M[0][0], projR.M[0][1], projR.M[0][2], projR.M[0][3],
      projR.M[1][0], projR.M[1][1], projR.M[1][2], projR.M[1][3],
      projR.M[2][0], projR.M[2][1], projR.M[2][2], projR.M[2][3],
      projR.M[3][0], projR.M[3][1], projR.M[3][2], projR.M[3][3]));

  // This seems like a mistake, but it's here on purpose.
  // Problem: Shadows get rendered incorrectly with
  // setCustomProjectionMatrix.
  // Solution (HACK): Pass false to setCustomProjectionMatrix. Found this
  // solution here: http://www.ogre3d.org/forums/viewtopic.php?f=2&t=60904
  // and here: http://www.ogre3d.org/forums/viewtopic.php?f=2&t=78461
  this->camera->setCustomProjectionMatrix(false,
    Ogre::Matrix4(
      projL.M[0][0], projL.M[0][1], projL.M[0][2], projL.M[0][3],
      projL.M[1][0], projL.M[1][1], projL.M[1][2], projL.M[1][3],
      projL.M[2][0], projL.M[2][1], projL.M[2][2], projL.M[2][3],
      projL.M[3][0], projL.M[3][1], projL.M[3][2], projL.M[3][3]));

  this->dataPtr->rightCamera->setCustomProjectionMatrix(false,
    Ogre::Matrix4(
      projR.M[0][0], projR.M[0][1], projR.M[0][2], projR.M[0][3],
      projR.M[1][0], projR.M[1][1], projR.M[1][2], projR.M[1][3],
      projR.M[2][0], projR.M[2][1], projR.M[2][2], projR.M[2][3],
      projR.M[3][0], projR.M[3][1], projR.M[3][2], projR.M[3][3]));

  this->initialized = true;
}

//////////////////////////////////////////////////
void OculusCamera::Oculus()
{
  if (!this->Ready())
    return;

  // Create a separate scene manager to holds a distorted mesh and a camera.
  // The distorted mesh receives the left and right camera images, and the
  // camera in the externalSceneManager renders the distorted meshes.
  this->dataPtr->externalSceneManager =
    RenderEngine::Instance()->root->createSceneManager(Ogre::ST_GENERIC);
  this->dataPtr->externalSceneManager->setAmbientLight(
      Ogre::ColourValue(0.5, 0.5, 0.5));

  // Get the texture sizes
  ovrSizei textureSizeLeft = ovrHmd_GetFovTextureSize(this->dataPtr->hmd,
       ovrEye_Left, this->dataPtr->hmd->DefaultEyeFov[0], 1.0f);
  ovrSizei textureSizeRight = ovrHmd_GetFovTextureSize(this->dataPtr->hmd,
      ovrEye_Right, this->dataPtr->hmd->DefaultEyeFov[1], 1.0f);
    this->dataPtr->ovrtextureLeftWidth = textureSizeLeft.w;
    this->dataPtr->ovrtextureLeftHeight = textureSizeLeft.h;
    this->dataPtr->ovrtextureRightWidth = textureSizeRight.w;
    this->dataPtr->ovrtextureRightHeight = textureSizeRight.h;


  // Create the left and right render textures.
  this->dataPtr->renderTextureLeft =
    Ogre::TextureManager::getSingleton().createManual(
      "OculusRiftRenderTextureLeft",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      textureSizeLeft.w,
      textureSizeLeft.h,
      0,
      Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET);

  this->dataPtr->renderTextureRight =
    Ogre::TextureManager::getSingleton().createManual(
      "OculusRiftRenderTextureRight",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      textureSizeRight.w,
      textureSizeRight.h,
      0,
      Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET);

  // Create the left and right materials.
  Ogre::MaterialPtr matLeft =
    Ogre::MaterialManager::getSingleton().getByName("Oculus/LeftEye");
  Ogre::MaterialPtr matRight =
    Ogre::MaterialManager::getSingleton().getByName("Oculus/RightEye");

  // Attach materials to the render textures.
  matLeft->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(
      this->dataPtr->renderTextureLeft);
  matRight->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(
      this->dataPtr->renderTextureRight);

  ovrFovPort fovLeft = this->dataPtr->hmd->DefaultEyeFov[ovrEye_Left];
  ovrFovPort fovRight = this->dataPtr->hmd->DefaultEyeFov[ovrEye_Right];

  // Get eye description information
  ovrEyeRenderDesc eyeRenderDesc[2];
  eyeRenderDesc[0] = ovrHmd_GetRenderDesc(
      this->dataPtr->hmd, ovrEye_Left, fovLeft);
  eyeRenderDesc[1] = ovrHmd_GetRenderDesc(
      this->dataPtr->hmd, ovrEye_Right, fovRight);

  double combinedTanHalfFovHorizontal = std::max(
    std::max(fovLeft.LeftTan, fovLeft.RightTan),
    std::max(fovRight.LeftTan, fovRight.RightTan));
  double combinedTanHalfFovVertical = std::max(
    std::max(fovLeft.UpTan, fovLeft.DownTan),
    std::max(fovRight.UpTan, fovRight.DownTan));

  // Hold some values that are needed when creating the distortion meshes.
  ovrVector2f uvScaleOffset[2];
  ovrRecti viewports[2];
  viewports[0].Pos.x = 0;
  viewports[0].Pos.y = 0;
  viewports[0].Size.w = textureSizeLeft.w;
  viewports[0].Size.h = textureSizeLeft.h;
  viewports[1].Pos.x = textureSizeLeft.w;
  viewports[1].Pos.y = 0;
  viewports[1].Size.w = textureSizeRight.w;
  viewports[1].Size.h = textureSizeRight.h;

  // Create a scene node in the external scene to hold the distortion
  // meshes.
  Ogre::SceneNode *meshNode =
    this->dataPtr->externalSceneManager->getRootSceneNode()
    ->createChildSceneNode();

  // Create the Distortion Meshes:
  for (int eyeIndex = 0; eyeIndex < 2; ++eyeIndex)
  {
    ovrDistortionMesh meshData;

    // Make the FOV symmetrical. Refer to Section 8.5.2 of the oculus SDF
    // developers manual.
    eyeRenderDesc[eyeIndex].Fov.RightTan = combinedTanHalfFovHorizontal;
    eyeRenderDesc[eyeIndex].Fov.LeftTan = combinedTanHalfFovHorizontal;
    eyeRenderDesc[eyeIndex].Fov.UpTan = combinedTanHalfFovVertical;
    eyeRenderDesc[eyeIndex].Fov.DownTan = combinedTanHalfFovVertical;

    ovrHmd_CreateDistortionMesh(
        this->dataPtr->hmd,
        eyeRenderDesc[eyeIndex].Eye,
        eyeRenderDesc[eyeIndex].Fov,
        0,
        &meshData);

    Ogre::GpuProgramParametersSharedPtr params;

    if (eyeIndex == 0)
    {
      ovrHmd_GetRenderScaleAndOffset(eyeRenderDesc[eyeIndex].Fov,
        textureSizeLeft, viewports[eyeIndex], uvScaleOffset);

      params =
        matLeft->getTechnique(0)->getPass(0)->getVertexProgramParameters();
    }
    else
    {
      ovrHmd_GetRenderScaleAndOffset(eyeRenderDesc[eyeIndex].Fov,
        textureSizeRight, viewports[eyeIndex], uvScaleOffset);
      params =
        matRight->getTechnique(0)->getPass(0)->getVertexProgramParameters();
    }

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR  >= 9
    params->setNamedConstant("eyeToSourceUVScale",
        Ogre::Vector2(uvScaleOffset[0].x, uvScaleOffset[0].y));
    params->setNamedConstant("eyeToSourceUVOffset",
        Ogre::Vector2(uvScaleOffset[1].x, uvScaleOffset[1].y));
#else
    params->setNamedConstant("eyeToSourceUVScale",
        Ogre::Vector4(uvScaleOffset[0].x, uvScaleOffset[0].y, 0, 0));
    params->setNamedConstant("eyeToSourceUVOffset",
        Ogre::Vector4(uvScaleOffset[1].x, uvScaleOffset[1].y, 0, 0));
#endif
    Ogre::ManualObject *externalObj;

    // create ManualObject
    if (eyeIndex == 0)
    {
      externalObj =
        this->dataPtr->externalSceneManager->createManualObject(
            "OculusRiftRenderObjectLeft");
      externalObj->begin("Oculus/LeftEye",
          Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }
    else
    {
      externalObj =
        this->dataPtr->externalSceneManager->createManualObject(
            "OculusRiftRenderObjectRight");
      externalObj->begin("Oculus/RightEye",
          Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    for (unsigned int i = 0; i < meshData.VertexCount; ++i)
    {
      ovrDistortionVertex v = meshData.pVertexData[i];
      externalObj->position(v.ScreenPosNDC.x, v.ScreenPosNDC.y, 0);
      externalObj->textureCoord(v.TanEyeAnglesR.x, v.TanEyeAnglesR.y);
      externalObj->textureCoord(v.TanEyeAnglesG.x, v.TanEyeAnglesG.y);
      externalObj->textureCoord(v.TanEyeAnglesB.x, v.TanEyeAnglesB.y);

      float vig = std::max(v.VignetteFactor, 0.0f);
      externalObj->colour(vig, vig, vig, vig);
    }

    for (unsigned int i = 0; i < meshData.IndexCount; ++i)
    {
      externalObj->index(meshData.pIndexData[i]);
    }

    // Manual render object complete
    externalObj->end();

    // Attach externalObj object to the node
    meshNode->attachObject(externalObj);

    // Free up memory
    ovrHmd_DestroyDistortionMesh(&meshData);
  }

  // Position the node in the scene
  meshNode->setPosition(0, 0, -1);
  meshNode->setScale(1, 1, -1);

  // Create the external camera
  this->dataPtr->externalCamera =
    this->dataPtr->externalSceneManager->createCamera(
        "_OculusRiftExternalCamera_INTERNAL_");
  this->dataPtr->externalCamera->setFarClipDistance(50);
  this->dataPtr->externalCamera->setNearClipDistance(0.001);
  this->dataPtr->externalCamera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  this->dataPtr->externalCamera->setOrthoWindow(2, 2);
  this->dataPtr->externalSceneManager->getRootSceneNode()->attachObject(
      this->dataPtr->externalCamera);

  // Create the external viewport
  this->dataPtr->externalViewport = this->renderTarget->addViewport(
      this->dataPtr->externalCamera);
  this->dataPtr->externalViewport->setBackgroundColour(
      Ogre::ColourValue::Black);
  this->dataPtr->externalViewport->setOverlaysEnabled(true);

  // Set up IPD in meters:
  float ipd = ovrHmd_GetFloat(this->dataPtr->hmd, OVR_KEY_IPD,  0.064f);
  this->camera->setPosition(-ipd * 0.5, 0, 0);
  this->dataPtr->rightCamera->setPosition(ipd * 0.5, 0, 0);

  this->camera->setFOVy(Ogre::Radian(OculusFOV * M_PI / 180.0));
  this->dataPtr->rightCamera->setFOVy(Ogre::Radian(OculusFOV * M_PI / 180.0));
}

/////////////////////////////////////////////////
void OculusCamera::AdjustAspect(double _v)
{
  if (!this->Ready())
    return;

  for (int i = 0; i < 2; ++i)
  {
    Ogre::Camera *cam = i == 0 ? this->camera : this->dataPtr->rightCamera;
    cam->setAspectRatio(cam->getAspectRatio() + _v);
  }
}
void OculusCamera::SetOculusFOV(float oculusfov)
{
  this->camera->setFOVy(Ogre::Radian(oculusfov * M_PI / 180.0));
  this->dataPtr->rightCamera->setFOVy(Ogre::Radian(oculusfov * M_PI / 180.0));
}
void OculusCamera::ConnectROS()
{
    ocubar = new OculusBar;
    ocubar->onOculusDK2();
    vicontracker = true;
}
void OculusCamera::OculusAddrpy(float Input_roll, float Input_pitch, float Input_yaw){
  init_roll_offset = Input_roll;
  init_pitch_offset = Input_pitch;
  init_yaw_offset = Input_yaw;
  reset_ovr_position = true;
}
