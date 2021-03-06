/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _VISUAL_CONFIG_HH_
#define _VISUAL_CONFIG_HH_

#include <map>
#include <string>

#include "gazebo/math/Pose.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/ModelData.hh"

namespace gazebo
{
  namespace gui
  {
    class ConfigWidget;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class VisualConfigData VisualConfig.hh
    /// \brief A class of widgets used for configuring visual properties.
    class VisualConfigData : public QWidget
    {
      Q_OBJECT

      /// \brief Qt callback when this item's button has been pressed.
      /// \param[in] _checked Whether it was checked or unchecked.
      private slots: void OnToggleItem(bool _checked);

      /// \brief Unique ID of this visual config.
      public: int id;

      /// \brief Name of the visual.
      public: std::string name;

      /// \brief Config widget for configuring visual properties.
      public: ConfigWidget *configWidget;

      /// \brief Widget associated with this data.
      public: QWidget *widget;
    };

    /// \class VisualConfig VisualConfig.hh
    /// \brief A tab for configuring visual properties of a link.
    class VisualConfig : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: VisualConfig();

      /// \brief Destructor
      public: ~VisualConfig();

      /// \brief Add a visual widget to the tab.
      /// \param[in] _name Name of visual added.
      /// \param[in] _visualMsg Msg containing information of the visual
      /// to be added.
      public: void AddVisual(const std::string &_name,
          const msgs::Visual *_visualMsg = NULL);

      /// \brief Update a visual widget from a visual msg.
      /// \param[in] _name Name of visual to be updated.
      /// \param[in] _visualMsg Msg used to update the visual widget values.
      public: void UpdateVisual(const std::string &_name,
          ConstVisualPtr _visualMsg);

      /// \brief Reset the visual tab.
      public: void Reset();

      /// \brief Get the number of visuals.
      /// \return Number of visuals.
      public: unsigned int GetVisualCount() const;

      /// \brief Get the msg containing all visual data.
      /// \param[in] _name Name of visual.
      /// \return Visual msg.
      public: msgs::Visual *GetData(const std::string &_name) const;

      /// \brief Set the geometry data of a visual
      /// \param[in] _name Name of visual.
      /// \param[in] _size Size of the geometry.
      /// \param[in] _uri URI of the geometry.
      public: void SetGeometry(const std::string &_name,
          const math::Vector3 &_size, const std::string &_uri = "");

      /// \brief Set the material of a visual
      /// \param[in] _name Name of visual.
      /// \param[in] _materialName Name of material.
      /// \param[in] _ambient Ambient color of visual.
      /// \param[in] _diffuse Diffuse color of visual.
      /// \param[in] _specular Specular color of visual.
      /// \param[in] _emissive Emissive color of visual.
      public: void SetMaterial(const std::string &_name,
          const std::string &_materialName,
          const common::Color &_ambient, const common::Color &_diffuse,
          const common::Color &_specular, const common::Color &_emissive);

      /// \brief Qt signal emitted when a visual is removed.
      /// \param[in] _name Name of visual removed.
      Q_SIGNALS: void VisualRemoved(const std::string &_name);

      /// \brief Qt signal emitted when a visual is added.
      /// \param[in] _name Name of visual added.
      Q_SIGNALS: void VisualAdded(const std::string &_name);

      /// \brief Qt callback when a visual is to be added.
      private slots: void OnAddVisual();

      /// \brief Qt callback when a visual is to be removed.
      /// \param[in] _id Id of item to be removed.
      private slots: void OnRemoveVisual(int _id);

      /// \brief Map of id to visual config widget.
      private: std::map<int, VisualConfigData *> configs;

      /// \brief Counter for the number of visuals.
      private: int counter;

      /// \brief Qt signal mapper for mapping remove button signals.
      private:  QSignalMapper *signalMapper;

      /// \brief Layout which holds all visual items.
      private: QVBoxLayout *listLayout;
    };
    /// \}
  }
}
#endif
