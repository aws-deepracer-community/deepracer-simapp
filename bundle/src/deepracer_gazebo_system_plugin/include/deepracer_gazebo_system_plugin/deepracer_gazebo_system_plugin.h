///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#ifndef __DEEPRACER_GAZEBO_SYSTEM_PLUGIN_H__
#define __DEEPRACER_GAZEBO_SYSTEM_PLUGIN_H__

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_options.h>
#include <std_msgs/ColorRGBA.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// Gazebo
#define protected public
#include <gazebo/physics/physics.hh>
#undef protected
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "std_srvs/Empty.h"
#include "deepracer_msgs/GetLightNames.h"
#include "deepracer_msgs/GetVisualNames.h"
#include "deepracer_msgs/GetVisual.h"
#include "deepracer_msgs/GetVisuals.h"
#include "deepracer_msgs/GetModelStates.h"
#include "deepracer_msgs/GetLinkStates.h"
#include "deepracer_msgs/SetVisualColor.h"
#include "deepracer_msgs/SetVisualColors.h"
#include "deepracer_msgs/SetVisualTransparency.h"
#include "deepracer_msgs/SetVisualTransparencies.h"
#include "deepracer_msgs/SetVisualVisible.h"
#include "deepracer_msgs/SetVisualVisibles.h"
#include "deepracer_msgs/SetVisualPose.h"
#include "deepracer_msgs/SetVisualPoses.h"
#include "deepracer_msgs/SetVisualMesh.h"
#include "deepracer_msgs/SetVisualMeshes.h"
#include "deepracer_msgs/SetModelStates.h"
#include "deepracer_msgs/SetLinkStates.h"

namespace gazebo
{
    class DeepRacerGazeboSystemPlugin : public SystemPlugin
    {
        public: DeepRacerGazeboSystemPlugin();

        public: ~DeepRacerGazeboSystemPlugin();

        public: virtual void Load(int argc, char** argv);

        /// \brief call back handler for SigInt signal
        private: void processSigInt();

        /// \brief Non-blocking plugin initialize handler
        private: void loadDeepRacerGazeboSystemPlugin(std::string world_name);

        /// \brief Advertise the ROS services
        private: void advertiseServices();

        /// \brief ROS callback queue loop
        private: void update();

        private: void publishVisualMsg(physics::LinkPtr &link,
                                       const msgs::Visual &visualMsg,
                                       bool block=false);
        private: void updateVisualMsg(physics::LinkPtr &link, const msgs::Visual &visualMsg);

        /// \brief set msgs::Color from std_msgs::ColorRGBA
        private: void set(msgs::Color *_msg, const std_msgs::ColorRGBA &c);
        /// \brief set std_msgs::ColorRGBA from msgs::Color
        private: void set(std_msgs::ColorRGBA &std_c, const msgs::Color &c);

        /// \brief set msgs::Vector3d from geometry_msgs::Point
        private: void set(msgs::Vector3d *_msg, const geometry_msgs::Point &p);
        /// \brief set geometry_msgs::Point from msgs::Vector3d
        private: void set(geometry_msgs::Point &geo_p, const msgs::Vector3d &v);

        /// \brief set msgs::Quaternion from geometry_msgs::Quaternion
        private: void set(msgs::Quaternion *_msg, const geometry_msgs::Quaternion &q);
        /// \brief set geometry_msgs::Quaternion from msgs::Quaternion
        private: void set(geometry_msgs::Quaternion &geo_q, const msgs::Quaternion &q);

        /// \brief set msgs::Pose from geometry_msgs::Pose
        private: void set(msgs::Pose *_msg, const geometry_msgs::Pose &p);
        /// \brief set geometry_msgs::Pose from msgs::Pose
        private: void set(geometry_msgs::Pose &geo_p, const msgs::Pose &p);

        /// \brief set msgs::Vector3d from geometry_msgs::Vector3
        private: void set(msgs::Vector3d *_msg, const geometry_msgs::Vector3 &p);
        /// \brief set geometry_msgs::Vector3 from msgs::Vector3d
        private: void set(geometry_msgs::Vector3 &geo_v, const msgs::Vector3d &v);

        private: std_msgs::ColorRGBA defaultColorRGBA();


        /// \brief callback handle to return Light names
        private: bool getLightNamesCallback(deepracer_msgs::GetLightNames::Request &req,
                                            deepracer_msgs::GetLightNames::Response &res);
        /// \brief callback handle to return visual names
        private: bool getVisualNamesCallback(deepracer_msgs::GetVisualNames::Request &req,
                                             deepracer_msgs::GetVisualNames::Response &res);

        /// \brief callback handle to return the color of visual
        private: bool getVisualCallback(deepracer_msgs::GetVisual::Request &req,
                                             deepracer_msgs::GetVisual::Response &res);
        /// \brief callback handle to return the colors of visuals
        private: bool getVisualsCallback(deepracer_msgs::GetVisuals::Request &req,
                                         deepracer_msgs::GetVisuals::Response &res);

        /// \brief callback handle to set the color of visual
        private: bool setVisualColorCallback(deepracer_msgs::SetVisualColor::Request &req,
                                             deepracer_msgs::SetVisualColor::Response &res);
        /// \brief callback handle to set the colors of visuals
        private: bool setVisualColorsCallback(deepracer_msgs::SetVisualColors::Request &req,
                                              deepracer_msgs::SetVisualColors::Response &res);

        /// \brief callback handle to set the transparency of visual
        private: bool setVisualTransparencyCallback(deepracer_msgs::SetVisualTransparency::Request &req,
                                                    deepracer_msgs::SetVisualTransparency::Response &res);

        /// \brief callback handle to set the transparencies of visuals
        private: bool setVisualTransparenciesCallback(deepracer_msgs::SetVisualTransparencies::Request &req,
                                                      deepracer_msgs::SetVisualTransparencies::Response &res);

        /// \brief callback handle to set the visible of visual
        private: bool setVisualVisibleCallback(deepracer_msgs::SetVisualVisible::Request &req,
                                               deepracer_msgs::SetVisualVisible::Response &res);

        /// \brief callback handle to set the visibles of visuals
        private: bool setVisualVisiblesCallback(deepracer_msgs::SetVisualVisibles::Request &req,
                                                deepracer_msgs::SetVisualVisibles::Response &res);

        /// \brief callback handle to set the pose of visual
        private: bool setVisualPoseCallback(deepracer_msgs::SetVisualPose::Request &req,
                                            deepracer_msgs::SetVisualPose::Response &res);

        /// \brief callback handle to set the poses of visuals
        private: bool setVisualPosesCallback(deepracer_msgs::SetVisualPoses::Request &req,
                                             deepracer_msgs::SetVisualPoses::Response &res);

        /// \brief callback handle to set the Mesh of visual
        private: bool setVisualMeshCallback(deepracer_msgs::SetVisualMesh::Request &req,
                                            deepracer_msgs::SetVisualMesh::Response &res);

        /// \brief callback handle to set the Meshes of visuals
        private: bool setVisualMeshesCallback(deepracer_msgs::SetVisualMeshes::Request &req,
                                              deepracer_msgs::SetVisualMeshes::Response &res);

        /// \brief callback handle to get model states
        private: bool getModelStatesCallback(deepracer_msgs::GetModelStates::Request &req,
                                             deepracer_msgs::GetModelStates::Response &res);

        /// \brief callback handle to set model states
        private: bool setModelStatesCallback(deepracer_msgs::SetModelStates::Request &req,
                                             deepracer_msgs::SetModelStates::Response &res);

        /// \brief callback handle to get link states
        private: bool getLinkStatesCallback(deepracer_msgs::GetLinkStates::Request &req,
                                            deepracer_msgs::GetLinkStates::Response &res);

        /// \brief callback handle to set link states
        private: bool setLinkStatesCallback(deepracer_msgs::SetLinkStates::Request &req,
                                            deepracer_msgs::SetLinkStates::Response &res);

        /// \brief callback handle to pause physics
        private: bool pausePhysicsCallback(std_srvs::Empty::Request &req,
                                           std_srvs::Empty::Response &res);

        /// \brief callback handle to unpause physics
        private: bool unpausePhysicsCallback(std_srvs::Empty::Request &req,
                                             std_srvs::Empty::Response &res);

        /// \brief flag whether plugin is loaded or not
        private: bool m_pluginLoaded;

        /// \brief flag whether SigInt received or not
        private: bool m_shouldStop;

        /// \brief flag whether world is created or not
        private: bool m_isWorldCreated;

        /// \brief mutex to lock the access to member variables from callback.
        private: std::mutex m_lock;

        /// \brief SigInt event connection
        private: gazebo::event::ConnectionPtr m_sigintConnection;

        /// \brief Update event connection
        private: gazebo::event::ConnectionPtr m_updateConnection;

        /// \brief World Creation event connection
        private: gazebo::event::ConnectionPtr m_loadDeepRacerGazeboPluginEvent;

        /// \brief pointer to Gazebo world.
        private: physics::WorldPtr m_world;

        /// \brief pointer to ROS node.
        private: boost::shared_ptr<ros::NodeHandle> m_rosNode;

        /// \brief pointer to Gazebo node.
        private: transport::NodePtr m_gazeboNode;

        /// \brief pointer to gazebo's ~/visual topic publisher
        private: transport::PublisherPtr m_visualPub;

        /// \brief getLightNames Server
        private: ros::ServiceServer m_getLightNamesSrv;
        /// \brief getVisualNames Server
        private: ros::ServiceServer m_getVisualNamesSrv;

        /// \brief getVisualColor Server
        private: ros::ServiceServer m_getVisualSrv;
        /// \brief getVisualColors Server
        private: ros::ServiceServer m_getVisualsSrv;

        /// \brief setVisualColor Server
        private: ros::ServiceServer m_setVisualColorSrv;
        /// \brief setVisualColors Server
        private: ros::ServiceServer m_setVisualColorsSrv;

        /// \brief setVisualTransparency Server
        private: ros::ServiceServer m_setVisualTransparencySrv;
        /// \brief setVisualTransparencies Server
        private: ros::ServiceServer m_setVisualTransparenciesSrv;

        /// \brief setVisualVisible Server
        private: ros::ServiceServer m_setVisualVisibleSrv;
        /// \brief setVisualVisibles Server
        private: ros::ServiceServer m_setVisualVisiblesSrv;

        /// \brief setVisualPose Server
        private: ros::ServiceServer m_setVisualPoseSrv;
        /// \brief setVisualPoses Server
        private: ros::ServiceServer m_setVisualPosesSrv;

        /// \brief setVisualMesh Server
        private: ros::ServiceServer m_setVisualMeshSrv;
        /// \brief setVisualMeshes Server
        private: ros::ServiceServer m_setVisualMeshesSrv;

        /// \brief setModelStates Server
        private: ros::ServiceServer m_setModelStatesSrv;
        /// \brief getModelStates Server
        private: ros::ServiceServer m_getModelStatesSrv;

        /// \brief setLinkStates Server
        private: ros::ServiceServer m_setLinkStatesSrv;
        /// \brief getLinkStates Server
        private: ros::ServiceServer m_getLinkStatesSrv;

        /// \brief pausePhysics Server
        private: ros::ServiceServer m_pausePhysicsSrv;
        /// \brief unpausePhysics Server
        private: ros::ServiceServer m_unpausePhysicsSrv;

        /// \brief ROS comm
        boost::shared_ptr<ros::AsyncSpinner> m_asyncRosSpin;

        /// \brief ROS callback queue
        private: ros::CallbackQueue m_rosCallbackQueue;
        private: boost::shared_ptr<boost::thread> m_gazeboCallbackQueueThread;
    };
}

#endif