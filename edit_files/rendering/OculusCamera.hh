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

#ifndef _GAZEBO_OCULUS_CAMERA_HH_
#define _GAZEBO_OCULUS_CAMERA_HH_

#include <string>
#include <chrono>
#include "gazebo/rendering/Camera.hh"
#include "getocupos/include/OculusBar.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace gazebo
{
  namespace rendering
  {
    class OculusCameraPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class OculusCamera OculusCamera.hh rendering/rendering.hh
    /// \brief A camera used for user visualization of a scene
    class GZ_RENDERING_VISIBLE OculusCamera : public Camera
    {
      /// \brief Constructor
      /// \param[in] _name Name of the camera.
      /// \param[in] _scene Scene to put the camera in.
      public: OculusCamera(const std::string &_name, ScenePtr _scene);

      /// \brief Destructor
      public: virtual ~OculusCamera();

      /// \brief Load the user camera.
      /// \param[in] _sdf Parameters for the camera.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Generic load function
      public: void Load();

      /// \brief Initialize
      public: void Init();

      /// \brief Render the camera
      public: virtual void Update();

      /// \brief Post render
      public: virtual void PostRender();

      /// \brief Finialize
      public: void Fini();

      /// \brief Resize the camera.
      /// \param[in] _w Width of the camera image.
      /// \param[in] _h Height of the camera image.
      public: void Resize(unsigned int _w, unsigned int _h);

      /// \brief Move the camera to focus on a visual.
      /// \param[in] _visual Visual to move the camera to.
      public: void MoveToVisual(VisualPtr _visual);

      // Doxygen automatically pulls in the correct documentation.
      public: virtual bool MoveToPosition(const math::Pose &_pose,
                                          double _time);

      /// \brief Move the camera to focus on a visual.
      /// \param[in] _visualName Name of the visual to move the camera to.
      public: void MoveToVisual(const std::string &_visualName);

      /// \brief Set to true to enable rendering
      ///
      /// Use this only if you really know what you're doing.
      /// \param[in] _target The new rendering target.
      public: virtual void SetRenderTarget(Ogre::RenderTarget *_target);

      /// \brief Change screen aspect ratio.
      /// \param[in] _v Aspect ratio.
      public: void AdjustAspect(double _v);

      // Documentation inherited
      public: virtual unsigned int GetImageWidth() const;

      // Documentation inherited
      public: virtual unsigned int GetImageHeight() const;

      /// \brief Reset the Oculus Rift sensor orientation.
      public: void ResetSensor();

      /// \brief Used to check if Oculus is plugged in and can be used.
      /// \return True when Oculus is ready to use.
      public: bool Ready();

      // Documentation inherited
      protected: virtual void RenderImpl();

      /// \brief Set the camera to be attached to a visual.
      ///
      /// This causes the camera to move in relation to the specified visual.
      /// \param[in] _visual The visual to attach to.
      /// \param[in] _inheritOrientation True if the camera should also
      /// rotate when the visual rotates.
      /// \param[in] _minDist Minimum distance the camera can get to the
      /// visual.
      /// \param[in] _maxDist Maximum distance the camera can get from the
      /// visual.
      /// \return True if successfully attach to the visual.
      protected: virtual bool AttachToVisualImpl(VisualPtr _visual,
                     bool _inheritOrientation, double _minDist = 0,
                     double _maxDist = 0);

      /// \brief Set the camera to track a scene node.
      ///
      /// Tracking just causes the camera to rotate to follow the visual.
      /// \param[in] _visual Visual to track.
      /// \return True if the camera is now tracking the visual.
      protected: virtual bool TrackVisualImpl(VisualPtr _visual);

      /// \brief Receive world control messages. Used to reset the oculus
      /// sensor.
      private: void OnControl(ConstWorldControlPtr &_data);

      /// \brief Apply distorsion to the render target.
      private: void Oculus();

      private: void UpdateBackgroundImages();

      /// \internal
      /// \brief Pointer to private data.

      //OculusPosition
      private: OculusCameraPrivate *dataPtr;
      private: float ovr_rr, ovr_rp, ovr_ry, init_ovr_rr, init_ovr_rp, init_ovr_ry, ovr_rr_offset, ovr_rp_offset, ovr_ry_offset;
      private: float ovr_sim_roll, ovr_sim_pitch, ovr_sim_yaw;
      private: float init_vicon_roll, init_vicon_pitch, init_vicon_yaw;
      private: float current_ovr_roll, current_ovr_pitch, current_ovr_yaw;
      private: float prev_ovr_roll, prev_ovr_pitch, prev_ovr_yaw;
      private: float ovr_roll_offset, ovr_pitch_offset, ovr_yaw_offset;
      private: float init_roll_offset = 0.0, init_pitch_offset = 0.0, init_yaw_offset = 0.0;
      private: float prev_ovr_x, prev_ovr_y;
      private: float prev_ovr_yaw_offset = 0;
      private: float current_vicon_yaw;
      private: float prev_vicon_yaw;
      private: bool reset_ovr_position;
      private: std::chrono::system_clock::time_point prev_time;
      private: double frame_rate;
      private: double total_time;
      public: OculusBar* ocubar;
      public: int temp, temp2;
      public: bool stereocam = false, vicontracker = false, debug = false;
      public: float OculusFOV = 115;
      public: void SetOculusFOV(float oculusfov);
      public: void ConnectROS();
      public: void OculusAddrpy(float Input_roll, float Input_pitch, float Input_yaw);

      //Right and Left Cams Calibration
      private: cv::Mat map1R, map2R, map1L, map2L;
      private: cv::Mat cameraMatrixR, distCoeffsR, cameraMatrixL, distCoeffsL;
    };
    /// \}
  }
}
#endif

/*

Size imageSize(640, 480);
38    Mat map1R, map2R;
39    Mat cameraMatrixR, distCoeffsR;
40
41    Mat map1L, map2L;
42    Mat cameraMatrixL, distCoeffsL;
43
44    cv::FileStorage cvfsR("cameraRparam.xml", CV_STORAGE_READ);
45    cv::FileStorage cvfsL("cameraLparam.xml", CV_STORAGE_READ);
46
47    cvfsR["camera_matrix"] >> cameraMatrixR;
48    cvfsR["distortion_coefficients"] >> distCoeffsR;
49
50    cvfsL["camera_matrix"] >> cameraMatrixL;
51    cvfsL["distortion_coefficients"] >> distCoeffsL;
52
53    cvfsR.release();
54    cvfsL.release();
57    initUndistortRectifyMap(cameraMatrixR, distCoeffsR, Mat(),
58    getOptimalNewCameraMatrix(cameraMatrixR, distCoeffsR, imageSize, 1, imageSize, 0),
59    imageSize, CV_16SC2, map1R, map2R);
60    //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
61
62    initUndistortRectifyMap(cameraMatrixL, distCoeffsL, Mat(),
63    getOptimalNewCameraMatrix(cameraMatrixL, distCoeffsL, imageSize, 1, imageSize, 0),

117    ¦    ¦  remap(destImgR, destImgR, map1R, map2R, INTER_LINEAR);
118         ¦  remap(destImgL, destImgL, map1L, map2L, INTER_LINEAR);
*/
