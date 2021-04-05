/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _VIDEO_STREAM_WIDGET_H_
#define _VIDEO_STREAM_WIDGET_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#   if GAZEBO_MAJOR_VERSION >= 9
#include <gazebo/transport/transport.hh>
#   else
#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>
#   endif
#endif

namespace gazebo
{
    class GAZEBO_VISIBLE VideoStreamWidget : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: VideoStreamWidget();

      /// \brief Destructor
      public: virtual ~VideoStreamWidget();

      /// \brief Callback trigged when the button is pressed.
      protected slots: void OnButton();

      /// \brief Counter used to create unique model names
      private: unsigned int counter;

      /// \brief Node used to establish communication with gzserver.
      private: transport::NodePtr node;

      /// \brief Publisher of video streaming control messages.
      private: transport::PublisherPtr videoPub;

      private: QPushButton *mButton;
      private: bool mVideoON;
      private: void enable();
      private: void disable();
    };
}
#endif

