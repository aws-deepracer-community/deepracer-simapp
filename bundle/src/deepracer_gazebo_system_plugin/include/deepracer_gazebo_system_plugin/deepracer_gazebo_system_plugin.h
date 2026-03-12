// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DEEPRACER_GAZEBO_SYSTEM_PLUGIN_H__
#define __DEEPRACER_GAZEBO_SYSTEM_PLUGIN_H__

// Gazebo Harmonic System includes
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

// Gazebo Components
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/Conversions.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/sim/components/PhysicsCmd.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visibility.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Static.hh>

// Gazebo Messages and Transport
#include <gz/msgs/physics.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/material.pb.h>
#include <gz/transport/Node.hh>

// SDF includes for Material manipulation
#include <sdf/Material.hh>
#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <gz/math/Color.hh>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_srvs/srv/empty.hpp>

// DeepRacer message includes
#include "deepracer_msgs/srv/get_light_names.hpp"
#include "deepracer_msgs/srv/get_visual_names.hpp"
#include "deepracer_msgs/srv/get_visual.hpp"
#include "deepracer_msgs/srv/get_visuals.hpp"
#include "deepracer_msgs/srv/get_model_states.hpp"
#include "deepracer_msgs/srv/get_link_states.hpp"
#include "deepracer_msgs/srv/set_visual_color.hpp"
#include "deepracer_msgs/srv/set_visual_colors.hpp"
#include "deepracer_msgs/srv/set_visual_transparency.hpp"
#include "deepracer_msgs/srv/set_visual_transparencies.hpp"
#include "deepracer_msgs/srv/set_visual_visible.hpp"
#include "deepracer_msgs/srv/set_visual_visibles.hpp"
#include "deepracer_msgs/srv/set_visual_pose.hpp"
#include "deepracer_msgs/srv/set_visual_poses.hpp"
#include "deepracer_msgs/srv/set_visual_mesh.hpp"
#include "deepracer_msgs/srv/set_visual_meshes.hpp"
#include "deepracer_msgs/srv/set_model_states.hpp"
#include "deepracer_msgs/srv/set_link_states.hpp"
#include "deepracer_msgs/srv/get_model_properties.hpp"
#include "deepracer_msgs/srv/set_light_properties.hpp"
#include "deepracer_msgs/srv/spawn_model.hpp"
#include "deepracer_msgs/srv/delete_model.hpp"

// Standard includes
#include <memory>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <atomic>

namespace gz::sim::systems
{
    /// \brief DeepRacer Gazebo System Plugin for ROS 2 + Gazebo Harmonic
    /// 
    /// This system provides comprehensive ROS 2 services for:
    /// - Visual manipulation (colors, transparency, visibility, poses, meshes)
    /// - Model and link state management  
    /// - Physics control (pause/unpause)
    /// - Light management
    /// 
    /// ARCHITECTURE (Thread-Safe Design):
    /// - PostUpdate: Builds snapshot from ECM (simulation thread)
    /// - PreUpdate: Processes command queue (simulation thread)
    /// - Service callbacks: Read snapshot, queue commands (ROS thread)
    /// 
    /// CRITICAL: Never accesses ECM from ROS callbacks (thread safety)
    class DeepRacerGazeboSystemPlugin : public gz::sim::System,
                                       public gz::sim::ISystemConfigure,
                                       public gz::sim::ISystemPreUpdate,
                                       public gz::sim::ISystemPostUpdate
    {
        public: 
            /// \brief Constructor
            DeepRacerGazeboSystemPlugin();

            /// \brief Destructor
            ~DeepRacerGazeboSystemPlugin() override;

            /// \brief Configure the system
            /// \param[in] _entity The entity this plugin is attached to
            /// \param[in] _sdf The SDF Element associated with this system plugin
            /// \param[in] _ecm The EntityComponentManager of the given simulation instance
            /// \param[in] _eventMgr The EventManager of the given simulation instance
            void Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr) override;

            /// \brief Called every PreUpdate cycle
            /// Processes command queue from ROS callbacks
            /// \param[in] _info Simulation update information
            /// \param[in] _ecm The EntityComponentManager of the given simulation instance
            void PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm) override;

            /// \brief Called every PostUpdate cycle  
            /// Builds snapshot for ROS callbacks to read
            /// \param[in] _info Simulation update information
            /// \param[in] _ecm The EntityComponentManager of the given simulation instance
            void PostUpdate(const gz::sim::UpdateInfo &_info,
                           const gz::sim::EntityComponentManager &_ecm) override;

        private:
            /// \brief Initialize ROS 2 node and services
            void initializeROS2();

            /// \brief Create all ROS 2 services
            void createServices();

            /// \brief ROS 2 executor thread function
            void executorThread();

            /// \brief Shutdown ROS 2 components
            void shutdownROS2();

            // ========== SERVICE CALLBACK DECLARATIONS ==========
            
            // Visual Manipulation Services
            void getLightNamesCallback(
                const std::shared_ptr<deepracer_msgs::srv::GetLightNames::Request> request,
                std::shared_ptr<deepracer_msgs::srv::GetLightNames::Response> response);

            void getVisualNamesCallback(
                const std::shared_ptr<deepracer_msgs::srv::GetVisualNames::Request> request,
                std::shared_ptr<deepracer_msgs::srv::GetVisualNames::Response> response);

            void getVisualCallback(
                const std::shared_ptr<deepracer_msgs::srv::GetVisual::Request> request,
                std::shared_ptr<deepracer_msgs::srv::GetVisual::Response> response);

            void getVisualsCallback(
                const std::shared_ptr<deepracer_msgs::srv::GetVisuals::Request> request,
                std::shared_ptr<deepracer_msgs::srv::GetVisuals::Response> response);

            void setVisualColorCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualColor::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualColor::Response> response);

            void setVisualColorsCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualColors::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualColors::Response> response);

            void setVisualTransparencyCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualTransparency::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualTransparency::Response> response);

            void setVisualTransparenciesCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualTransparencies::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualTransparencies::Response> response);

            void setVisualVisibleCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualVisible::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualVisible::Response> response);

            void setVisualVisiblesCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualVisibles::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualVisibles::Response> response);

            void setVisualPoseCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualPose::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualPose::Response> response);

            void setVisualPosesCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualPoses::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualPoses::Response> response);

            void setVisualMeshCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualMesh::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualMesh::Response> response);

            void setVisualMeshesCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetVisualMeshes::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetVisualMeshes::Response> response);

            // Model/Link State Services
            void getModelStatesCallback(
                const std::shared_ptr<deepracer_msgs::srv::GetModelStates::Request> request,
                std::shared_ptr<deepracer_msgs::srv::GetModelStates::Response> response);

            void setModelStatesCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetModelStates::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetModelStates::Response> response);

            void getLinkStatesCallback(
                const std::shared_ptr<deepracer_msgs::srv::GetLinkStates::Request> request,
                std::shared_ptr<deepracer_msgs::srv::GetLinkStates::Response> response);

            void setLinkStatesCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetLinkStates::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetLinkStates::Response> response);

            // Physics Control Services
            void pausePhysicsCallback(
                const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                std::shared_ptr<std_srvs::srv::Empty::Response> response);

            void unpausePhysicsCallback(
                const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                std::shared_ptr<std_srvs::srv::Empty::Response> response);

            // Model Properties Service
            void getModelPropertiesCallback(
                const std::shared_ptr<deepracer_msgs::srv::GetModelProperties::Request> request,
                std::shared_ptr<deepracer_msgs::srv::GetModelProperties::Response> response);

            // Light Properties Service
            void setLightPropertiesCallback(
                const std::shared_ptr<deepracer_msgs::srv::SetLightProperties::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SetLightProperties::Response> response);

            // Spawn/Delete Model Services
            void spawnSDFModelCallback(
                const std::shared_ptr<deepracer_msgs::srv::SpawnModel::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SpawnModel::Response> response);

            void spawnURDFModelCallback(
                const std::shared_ptr<deepracer_msgs::srv::SpawnModel::Request> request,
                std::shared_ptr<deepracer_msgs::srv::SpawnModel::Response> response);

            void deleteModelCallback(
                const std::shared_ptr<deepracer_msgs::srv::DeleteModel::Request> request,
                std::shared_ptr<deepracer_msgs::srv::DeleteModel::Response> response);

        private:
            // ========== SNAPSHOT DATA STRUCTURES (Thread-Safe Caching) ==========
            
            /// \brief Snapshot of model state (updated in PostUpdate, read in callbacks)
            struct ModelStateSnapshot
            {
                gz::math::Pose3d worldPose;
                gz::math::Vector3d worldLinearVel;   // WORLD frame
                gz::math::Vector3d worldAngularVel;  // WORLD frame
            };
            
            /// \brief Snapshot of link state (updated in PostUpdate, read in callbacks)
            struct LinkStateSnapshot
            {
                gz::math::Pose3d worldPose;
                gz::math::Vector3d worldLinearVel;   // WORLD frame
                gz::math::Vector3d worldAngularVel;  // WORLD frame
            };
            
            // ========== COMMAND QUEUE STRUCTURES (Thread-Safe Communication) ==========
            
            /// \brief Command to set model state (queued from ROS, processed in PreUpdate)
            struct SetModelStateCommand
            {
                std::string model_name;
                std::string reference_frame;
                gz::math::Pose3d target_pose;        // In reference frame
                gz::math::Vector3d target_linear_vel;  // In reference frame
                gz::math::Vector3d target_angular_vel; // In reference frame
            };
            
            /// \brief Command to set link state (queued from ROS, processed in PreUpdate)
            struct SetLinkStateCommand
            {
                std::string link_name;
                std::string reference_frame;
                gz::math::Pose3d target_pose;        // In reference frame
                gz::math::Vector3d target_linear_vel;  // In reference frame
                gz::math::Vector3d target_angular_vel; // In reference frame
            };

            // ========== VISUAL COMMAND STRUCTURES ==========

            /// \brief Command to set visual color
            struct SetVisualColorCommand
            {
                std::string link_name;
                std::string visual_name;
                std_msgs::msg::ColorRGBA ambient;
                std_msgs::msg::ColorRGBA diffuse;
                std_msgs::msg::ColorRGBA specular;
                std_msgs::msg::ColorRGBA emissive;
            };

            /// \brief Command to set visual transparency
            struct SetVisualTransparencyCommand
            {
                std::string link_name;
                std::string visual_name;
                float transparency;
            };

            /// \brief Command to set visual visibility
            struct SetVisualVisibleCommand
            {
                std::string link_name;
                std::string visual_name;
                bool visible;
            };

            /// \brief Command to set visual pose
            struct SetVisualPoseCommand
            {
                std::string link_name;
                std::string visual_name;
                geometry_msgs::msg::Pose pose;
            };

            /// \brief Command to set visual mesh
            struct SetVisualMeshCommand
            {
                std::string link_name;
                std::string visual_name;
                std::string filename;
                geometry_msgs::msg::Vector3 scale;
            };

            /// \brief Command to set light properties
            struct SetLightPropertiesCommand
            {
                std::string light_name;
                geometry_msgs::msg::Pose pose;
                std_msgs::msg::ColorRGBA diffuse;
                std_msgs::msg::ColorRGBA specular;
                geometry_msgs::msg::Vector3 direction;
                float attenuation_constant;
                float attenuation_linear;
                float attenuation_quadratic;
                bool cast_shadows;
            };

            /// \brief Visual state snapshot (for GET services)
            struct VisualStateSnapshot
            {
                std::string link_name;
                std_msgs::msg::ColorRGBA ambient;
                std_msgs::msg::ColorRGBA diffuse;
                std_msgs::msg::ColorRGBA specular;
                std_msgs::msg::ColorRGBA emissive;
                float transparency;
                bool visible;
                geometry_msgs::msg::Pose pose;
                std::string mesh_filename;
                geometry_msgs::msg::Vector3 mesh_scale;
                uint16_t geometry_type;
            };

            /// \brief Visual cache: "link_name::visual_name" -> snapshot
            std::unordered_map<std::string, VisualStateSnapshot> visual_states_cache_;

            /// \brief Light names cache
            std::vector<std::string> light_names_cache_;
                        
            // ========== THREAD SYNCHRONIZATION ==========
            
            /// \brief Mutex for general thread safety (service callbacks)
            std::mutex mutex_;
            
            /// \brief Mutex for snapshot access (read-write lock)
            /// - PostUpdate: unique_lock (exclusive write)
            /// - Callbacks: shared_lock (concurrent reads)
            mutable std::shared_mutex snapshot_mtx_;
            
            /// \brief Mutex for command queue access
            std::mutex command_queue_mtx_;
            
            // ========== STATE CACHING (Built in PostUpdate, Read in Callbacks) ==========
            
            /// \brief Model state cache (world frame positions and velocities)
            std::unordered_map<std::string, ModelStateSnapshot> model_states_cache_;
            
            /// \brief Link state cache (world frame positions and velocities)
            std::unordered_map<std::string, LinkStateSnapshot> link_states_cache_;
            
            // ========== COMMAND QUEUES (Written in Callbacks, Read in PreUpdate) ==========
            
            /// \brief Queue of model state commands
            std::queue<SetModelStateCommand> model_state_commands_;
            
            /// \brief Queue of link state commands
            std::queue<SetLinkStateCommand> link_state_commands_;

            // ========== VISUAL/LIGHT COMMAND QUEUES ==========

            std::queue<SetVisualColorCommand> visual_color_commands_;
            std::queue<SetVisualTransparencyCommand> visual_transparency_commands_;
            std::queue<SetVisualVisibleCommand> visual_visible_commands_;
            std::queue<SetVisualPoseCommand> visual_pose_commands_;
            std::queue<SetVisualMeshCommand> visual_mesh_commands_;
            std::queue<SetLightPropertiesCommand> light_commands_;
            
            // ========== ENTITY LOOKUPS (Built in PostUpdate) ==========
            
            /// \brief Entity name lookups
            std::unordered_map<gz::sim::Entity, std::string> name_by_entity_;
            std::unordered_map<std::string, gz::sim::Entity> model_by_name_;
            std::unordered_map<std::string, gz::sim::Entity> link_by_name_;
            
            /// \brief Entity hierarchy (parent-child relationships)
            std::unordered_map<gz::sim::Entity, gz::sim::Entity> parent_;              // child -> parent
            std::unordered_multimap<gz::sim::Entity, gz::sim::Entity> children_;       // parent -> children
            
            /// \brief Entity type sets (for efficient filtering)
            std::unordered_set<gz::sim::Entity> models_;
            std::unordered_set<gz::sim::Entity> links_;
            std::unordered_set<gz::sim::Entity> joints_;
            std::unordered_set<gz::sim::Entity> collisions_;
            std::unordered_set<gz::sim::Entity> static_models_;
      
            /// \brief Event Manager pointer (safe to store)
            gz::sim::EventManager *eventMgr_;

            /// \brief Atomic flags for physics control (thread-safe communication)
            std::atomic<bool> pause_req_{false};
            std::atomic<bool> unpause_req_{false};

            /// \brief Gazebo transport node for world control services
            gz::transport::Node gz_node_;
            
            /// \brief Cached world name for service calls
            std::string world_name_;
            
            /// \brief Flag to track if world name has been cached
            std::atomic<bool> world_name_cached_{false};

            // ========== ROS 2 COMPONENTS ==========
            
            /// \brief ROS 2 node
            rclcpp::Node::SharedPtr node_;

            /// \brief ROS 2 executor thread
            std::thread executor_thread_;

            /// \brief Flag to control executor thread
            std::atomic<bool> executor_running_;

            /// \brief Flag to track ROS 2 initialization status
            std::atomic<bool> ros_initialized_;

            // ========== ROS 2 SERVICE SERVERS ==========
            
            rclcpp::Service<deepracer_msgs::srv::GetLightNames>::SharedPtr get_light_names_srv_;
            rclcpp::Service<deepracer_msgs::srv::GetVisualNames>::SharedPtr get_visual_names_srv_;
            rclcpp::Service<deepracer_msgs::srv::GetVisual>::SharedPtr get_visual_srv_;
            rclcpp::Service<deepracer_msgs::srv::GetVisuals>::SharedPtr get_visuals_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualColor>::SharedPtr set_visual_color_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualColors>::SharedPtr set_visual_colors_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualTransparency>::SharedPtr set_visual_transparency_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualTransparencies>::SharedPtr set_visual_transparencies_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualVisible>::SharedPtr set_visual_visible_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualVisibles>::SharedPtr set_visual_visibles_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualPose>::SharedPtr set_visual_pose_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualPoses>::SharedPtr set_visual_poses_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualMesh>::SharedPtr set_visual_mesh_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetVisualMeshes>::SharedPtr set_visual_meshes_srv_;
            rclcpp::Service<deepracer_msgs::srv::GetModelStates>::SharedPtr get_model_states_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetModelStates>::SharedPtr set_model_states_srv_;
            rclcpp::Service<deepracer_msgs::srv::GetLinkStates>::SharedPtr get_link_states_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetLinkStates>::SharedPtr set_link_states_srv_;
            rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_physics_srv_;
            rclcpp::Service<std_srvs::srv::Empty>::SharedPtr unpause_physics_srv_;
            rclcpp::Service<deepracer_msgs::srv::GetModelProperties>::SharedPtr get_model_properties_srv_;
            rclcpp::Service<deepracer_msgs::srv::SetLightProperties>::SharedPtr set_light_properties_srv_;
            rclcpp::Service<deepracer_msgs::srv::SpawnModel>::SharedPtr spawn_sdf_model_srv_;
            rclcpp::Service<deepracer_msgs::srv::SpawnModel>::SharedPtr spawn_urdf_model_srv_;
            rclcpp::Service<deepracer_msgs::srv::DeleteModel>::SharedPtr delete_model_srv_;
    };
}

#endif
