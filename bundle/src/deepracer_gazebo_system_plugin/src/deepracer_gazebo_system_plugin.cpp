// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <deepracer_gazebo_system_plugin/deepracer_gazebo_system_plugin.h>
#include <gz/plugin/Register.hh>

using namespace gz::sim;
using namespace gz::sim::systems;

//////////////////////////////////////////////////
DeepRacerGazeboSystemPlugin::DeepRacerGazeboSystemPlugin()
    : eventMgr_(nullptr), executor_running_(false), ros_initialized_(false)
{
    RCLCPP_INFO(rclcpp::get_logger("deepracer_gazebo_system_plugin"), 
                "DeepRacer Gazebo System Plugin: Constructor");
}

//////////////////////////////////////////////////
DeepRacerGazeboSystemPlugin::~DeepRacerGazeboSystemPlugin()
{
    RCLCPP_INFO(rclcpp::get_logger("deepracer_gazebo_system_plugin"), 
                "DeepRacer Gazebo System Plugin: Destructor");
    shutdownROS2();
}

//////////////////////////////////////////////////
void DeepRacerGazeboSystemPlugin::Configure(const Entity &/*_entity*/,
                                           const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                                           EntityComponentManager &/*_ecm*/,
                                           EventManager &_eventMgr)
{
    RCLCPP_INFO(rclcpp::get_logger("deepracer_gazebo_system_plugin"), 
                "DeepRacer Gazebo System Plugin: Configure");

    // Store EventManager pointer (safe to store)
    this->eventMgr_ = &_eventMgr;
    
    ros_initialized_ = false;

    RCLCPP_INFO(rclcpp::get_logger("deepracer_gazebo_system_plugin"), 
                "DeepRacer Gazebo System Plugin: Configuration complete");
}

//////////////////////////////////////////////////
void DeepRacerGazeboSystemPlugin::PreUpdate(const UpdateInfo &/*_info*/,
                                           EntityComponentManager &_ecm)
{
    if (!ros_initialized_)
    {
        RCLCPP_INFO(rclcpp::get_logger("deepracer_gazebo_system_plugin"), 
                    "Initializing ROS 2 integration in PreUpdate");
        initializeROS2();
        ros_initialized_ = true;
    }

    // Cache world name once
    if (!world_name_cached_) {
        _ecm.Each<components::World, components::Name>(
            [&](const Entity&, const components::World*, 
                const components::Name* name) -> bool {
                world_name_ = name->Data();
                world_name_cached_ = true;
                RCLCPP_INFO(node_->get_logger(), "Cached world name: %s", world_name_.c_str());
                return false; // Stop after finding first world
            });
        
        if (!world_name_cached_) {
            return; // No world found yet
        }
    }

    // Handle pause request
    if (pause_req_.exchange(false)) {
        const std::string svc = "/world/" + world_name_ + "/control";
        gz::msgs::WorldControl req;
        req.set_pause(true);
        gz::msgs::Boolean rep;
        bool result = false;
        
        bool ok = gz_node_.Request(svc, req, 1000, rep, result);
        if (ok && result) {
            RCLCPP_DEBUG(node_->get_logger(), "Physics paused successfully via service %s", svc.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "WorldControl pause request failed for %s", svc.c_str());
        }
    }

    // Handle unpause request
    if (unpause_req_.exchange(false)) {
        const std::string svc = "/world/" + world_name_ + "/control";
        gz::msgs::WorldControl req;
        req.set_pause(false);
        gz::msgs::Boolean rep;
        bool result = false;
        
        bool ok = gz_node_.Request(svc, req, 1000, rep, result);
        if (ok && result) {
            RCLCPP_DEBUG(node_->get_logger(), "Physics unpaused successfully via service %s", svc.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "WorldControl unpause request failed for %s", svc.c_str());
        }
    }

    // ========== PROCESS MODEL STATE COMMANDS (FROM ROS CALLBACKS) ==========
    {
        std::lock_guard<std::mutex> lock(command_queue_mtx_);
        
        if (!model_state_commands_.empty()) {
            RCLCPP_DEBUG(node_->get_logger(), "PreUpdate: Processing %zu queued model state commands", 
                model_state_commands_.size());
            
            // Build entity maps once for all commands
            std::unordered_map<std::string, Entity> modelEntityMap;
            std::unordered_map<std::string, Entity> referenceEntityMap;
            
            _ecm.Each<components::Model, components::Name>(
                [&](const Entity &e, const components::Model*, const components::Name *n) {
                    modelEntityMap[n->Data()] = e;
                    referenceEntityMap[n->Data()] = e;
                    return true;
                });
            _ecm.Each<components::Link, components::Name>(
                [&](const Entity &e, const components::Link*, const components::Name *n) {
                    referenceEntityMap[n->Data()] = e;
                    return true;
                });
        
            while (!model_state_commands_.empty()) {
                auto cmd = model_state_commands_.front();
                model_state_commands_.pop();
                
                // Find model entity from cache
                auto modelIt = modelEntityMap.find(cmd.model_name);
                if (modelIt == modelEntityMap.end()) {
                    RCLCPP_WARN(node_->get_logger(), 
                        "Model '%s' not found in PreUpdate", cmd.model_name.c_str());
                    continue;
                }
                Entity modelEntity = modelIt->second;
                
                // Transform pose and velocities to WORLD frame
                gz::math::Pose3d worldPose = cmd.target_pose;
                gz::math::Vector3d worldLinearVel = cmd.target_linear_vel;
                gz::math::Vector3d worldAngularVel = cmd.target_angular_vel;
                
                if (!cmd.reference_frame.empty() && 
                    cmd.reference_frame != "world" && 
                    cmd.reference_frame != "map" && 
                    cmd.reference_frame != "/map") {
                    
                    auto refIt = referenceEntityMap.find(cmd.reference_frame);
                    if (refIt != referenceEntityMap.end()) {
                        Entity refEntity = refIt->second;
                        gz::math::Pose3d refWorldPose = gz::sim::worldPose(refEntity, _ecm);
                        
                        // Transform pose: reference → world
                        worldPose = refWorldPose * cmd.target_pose;
                        
                        // Transform velocities: reference → world
                        worldLinearVel = refWorldPose.Rot().RotateVector(cmd.target_linear_vel);
                        worldAngularVel = refWorldPose.Rot().RotateVector(cmd.target_angular_vel);
                    }
                }
                
                // Apply WorldPoseCmd (world frame)
                auto poseComp = _ecm.Component<components::WorldPoseCmd>(modelEntity);
                if (poseComp) {
                    *poseComp = components::WorldPoseCmd(worldPose);
                } else {
                    _ecm.CreateComponent(modelEntity, components::WorldPoseCmd(worldPose));
                }
                _ecm.SetChanged(modelEntity, components::WorldPoseCmd::typeId,
                            ComponentState::OneTimeChange);

                // Transform world → body frame and use LinearVelocityCmd
                // Physics system ignores WorldLinearVelocityCmd per gz-sim#2266
                // Must use LinearVelocityCmd (body frame) instead
                gz::math::Pose3d currentModelPose = gz::sim::worldPose(modelEntity, _ecm);
                
                // Transform world velocities → body velocities
                gz::math::Vector3d bodyLinearVel = 
                    currentModelPose.Rot().RotateVectorReverse(worldLinearVel);
                gz::math::Vector3d bodyAngularVel = 
                    currentModelPose.Rot().RotateVectorReverse(worldAngularVel);
                
                // Only apply velocity commands if non-zero
                // Zero velocity should remove commands to allow wheel controllers
                double velMagnitude = bodyLinearVel.Length() + bodyAngularVel.Length();
                
                if (velMagnitude > 0.0) {
                    // User explicitly wants to set velocity
                    auto linVelComp = _ecm.Component<components::LinearVelocityCmd>(modelEntity);
                    if (linVelComp) {
                        *linVelComp = components::LinearVelocityCmd(bodyLinearVel);
                    } else {
                        _ecm.CreateComponent(modelEntity, components::LinearVelocityCmd(bodyLinearVel));
                    }
                    _ecm.SetChanged(modelEntity, components::LinearVelocityCmd::typeId,
                                ComponentState::OneTimeChange);
                    
                    auto angVelComp = _ecm.Component<components::AngularVelocityCmd>(modelEntity);
                    if (angVelComp) {
                        *angVelComp = components::AngularVelocityCmd(bodyAngularVel);
                    } else {
                        _ecm.CreateComponent(modelEntity, components::AngularVelocityCmd(bodyAngularVel));
                    }
                    _ecm.SetChanged(modelEntity, components::AngularVelocityCmd::typeId,
                                ComponentState::OneTimeChange);
                } else {
                    // Zero velocity - remove commands to allow wheel control
                    _ecm.RemoveComponent<components::LinearVelocityCmd>(modelEntity);
                    _ecm.RemoveComponent<components::AngularVelocityCmd>(modelEntity);
                }
            }
        }
    }
    
    // ========== PROCESS LINK STATE COMMANDS ==========
    {
        std::lock_guard<std::mutex> lock(command_queue_mtx_);
        
        if (!link_state_commands_.empty()) {
            // Build entity maps once for all link commands
            std::unordered_map<std::string, Entity> linkEntityMap;
            std::unordered_map<std::string, Entity> referenceEntityMap;
            
            _ecm.Each<components::Link, components::Name>(
                [&](const Entity &e, const components::Link*, const components::Name *n) {
                    linkEntityMap[n->Data()] = e;
                    referenceEntityMap[n->Data()] = e;
                    return true;
                });
            _ecm.Each<components::Model, components::Name>(
                [&](const Entity &e, const components::Model*, const components::Name *n) {
                    referenceEntityMap[n->Data()] = e;
                    return true;
                });
        
            while (!link_state_commands_.empty()) {
                auto cmd = link_state_commands_.front();
                link_state_commands_.pop();
                
                // Find link entity from cache
                auto linkIt = linkEntityMap.find(cmd.link_name);
                if (linkIt == linkEntityMap.end()) {
                    RCLCPP_WARN(node_->get_logger(), 
                        "Link '%s' not found in PreUpdate", cmd.link_name.c_str());
                    continue;
                }
                Entity linkEntity = linkIt->second;
                
                // Transform to world frame if reference frame specified
                gz::math::Pose3d worldPose = cmd.target_pose;
                gz::math::Vector3d worldLinearVel = cmd.target_linear_vel;
                gz::math::Vector3d worldAngularVel = cmd.target_angular_vel;
                
                if (!cmd.reference_frame.empty() && 
                    cmd.reference_frame != "world" && 
                    cmd.reference_frame != "map" && 
                    cmd.reference_frame != "/map") {
                    
                    auto refIt = referenceEntityMap.find(cmd.reference_frame);
                    if (refIt != referenceEntityMap.end()) {
                        Entity refEntity = refIt->second;
                        
                        // Get reference entity world pose
                        // Get reference entity world pose with fallback
                        gz::math::Pose3d refWorldPose;
                        auto refWorldPoseComp = _ecm.Component<components::WorldPose>(refEntity);
                        if (refWorldPoseComp) {
                            refWorldPose = refWorldPoseComp->Data();
                        } else {
                            refWorldPose = gz::sim::worldPose(refEntity, _ecm);
}
                        
                        // Transform pose: reference → world
                        worldPose = refWorldPose * cmd.target_pose;
                        
                        // Transform velocities: reference → world
                        worldLinearVel = refWorldPose.Rot().RotateVector(cmd.target_linear_vel);
                        worldAngularVel = refWorldPose.Rot().RotateVector(cmd.target_angular_vel);
                    }
                }
                
                auto poseComp = _ecm.Component<components::WorldPoseCmd>(linkEntity);
                if (poseComp) {
                    *poseComp = components::WorldPoseCmd(worldPose);
                } else {
                    _ecm.CreateComponent(linkEntity, components::WorldPoseCmd(worldPose));
                }
                _ecm.SetChanged(linkEntity, components::WorldPoseCmd::typeId,
                               ComponentState::OneTimeChange);
                
                // Transform world → body frame and use LinearVelocityCmd
                // Get link's current pose for body transformation
                gz::math::Pose3d currentLinkPose = gz::sim::worldPose(linkEntity, _ecm);
                
                // Transform world velocities → body velocities
                // Physics system only processes LinearVelocityCmd (body frame)
                gz::math::Vector3d bodyLinearVel = 
                    currentLinkPose.Rot().RotateVectorReverse(worldLinearVel);
                gz::math::Vector3d bodyAngularVel = 
                    currentLinkPose.Rot().RotateVectorReverse(worldAngularVel);
                
                // Apply velocity commands (body frame)
                auto linVelComp = _ecm.Component<components::LinearVelocityCmd>(linkEntity);
                if (linVelComp) {
                    *linVelComp = components::LinearVelocityCmd(bodyLinearVel);
                } else {
                    _ecm.CreateComponent(linkEntity, components::LinearVelocityCmd(bodyLinearVel));
                }
                _ecm.SetChanged(linkEntity, components::LinearVelocityCmd::typeId,
                            ComponentState::OneTimeChange);
                
                auto angVelComp = _ecm.Component<components::AngularVelocityCmd>(linkEntity);
                if (angVelComp) {
                    *angVelComp = components::AngularVelocityCmd(bodyAngularVel);
                } else {
                    _ecm.CreateComponent(linkEntity, components::AngularVelocityCmd(bodyAngularVel));
                }
                _ecm.SetChanged(linkEntity, components::AngularVelocityCmd::typeId,
                            ComponentState::OneTimeChange);
            }
        }
    }

    // ========== PROCESS VISUAL COLOR COMMANDS ==========
    {
        std::lock_guard<std::mutex> lock(command_queue_mtx_);
        
        if (!visual_color_commands_.empty()) {
            // Build link entity map once for all visual commands
            std::unordered_map<std::string, Entity> linkEntityMap;
            _ecm.Each<components::Link, components::Name>(
                [&](const Entity &e, const components::Link*, const components::Name *n) {
                    linkEntityMap[n->Data()] = e;
                    return true;
                });
            
            while (!visual_color_commands_.empty()) {
                auto cmd = visual_color_commands_.front();
                visual_color_commands_.pop();
                
                auto linkIt = linkEntityMap.find(cmd.link_name);
                if (linkIt == linkEntityMap.end()) {
                    RCLCPP_WARN(node_->get_logger(), "Link '%s' not found", cmd.link_name.c_str());
                    continue;
                }
                Entity linkEntity = linkIt->second;
                
                Entity visualEntity = _ecm.EntityByComponents(
                    components::ParentEntity(linkEntity),
                    components::Visual(),
                    components::Name(cmd.visual_name));
                
                if (visualEntity == kNullEntity) {
                    RCLCPP_WARN(node_->get_logger(), "Visual '%s' not found in link '%s'", 
                               cmd.visual_name.c_str(), cmd.link_name.c_str());
                    continue;
                }
            
                // First, ensure Material component exists (required for rendering)
                auto materialComp = _ecm.Component<components::Material>(visualEntity);
                if (!materialComp) {
                    sdf::Material defaultMat;
                    defaultMat.SetAmbient(gz::math::Color(cmd.ambient.r, cmd.ambient.g, cmd.ambient.b, cmd.ambient.a));
                    defaultMat.SetDiffuse(gz::math::Color(cmd.diffuse.r, cmd.diffuse.g, cmd.diffuse.b, cmd.diffuse.a));
                    defaultMat.SetSpecular(gz::math::Color(cmd.specular.r, cmd.specular.g, cmd.specular.b, cmd.specular.a));
                    defaultMat.SetEmissive(gz::math::Color(cmd.emissive.r, cmd.emissive.g, cmd.emissive.b, cmd.emissive.a));
                    _ecm.CreateComponent(visualEntity, components::Material(defaultMat));
                }
                
                // Use VisualCmd (Gazebo's official pattern for visual updates)
                gz::msgs::Visual visualMsg;
                visualMsg.set_id(visualEntity);
                visualMsg.set_name(cmd.visual_name);
                
                auto *matMsg = visualMsg.mutable_material();
                matMsg->mutable_ambient()->set_r(cmd.ambient.r);
                matMsg->mutable_ambient()->set_g(cmd.ambient.g);
                matMsg->mutable_ambient()->set_b(cmd.ambient.b);
                matMsg->mutable_ambient()->set_a(cmd.ambient.a);
                
                matMsg->mutable_diffuse()->set_r(cmd.diffuse.r);
                matMsg->mutable_diffuse()->set_g(cmd.diffuse.g);
                matMsg->mutable_diffuse()->set_b(cmd.diffuse.b);
                matMsg->mutable_diffuse()->set_a(cmd.diffuse.a);
                
                matMsg->mutable_specular()->set_r(cmd.specular.r);
                matMsg->mutable_specular()->set_g(cmd.specular.g);
                matMsg->mutable_specular()->set_b(cmd.specular.b);
                matMsg->mutable_specular()->set_a(cmd.specular.a);
                
                matMsg->mutable_emissive()->set_r(cmd.emissive.r);
                matMsg->mutable_emissive()->set_g(cmd.emissive.g);
                matMsg->mutable_emissive()->set_b(cmd.emissive.b);
                matMsg->mutable_emissive()->set_a(cmd.emissive.a);
                
                auto visualCmdComp = _ecm.Component<components::VisualCmd>(visualEntity);
                if (visualCmdComp) {
                    visualCmdComp->Data() = visualMsg;
                    _ecm.SetChanged(visualEntity, components::VisualCmd::typeId,
                                   ComponentState::OneTimeChange);
                } else {
                    _ecm.CreateComponent(visualEntity, components::VisualCmd(visualMsg));
                }
                
                RCLCPP_DEBUG(node_->get_logger(), "Updated visual color for '%s'", cmd.visual_name.c_str());
            }
        }
    }

    // ========== PROCESS VISUAL TRANSPARENCY COMMANDS ==========
    {
        std::lock_guard<std::mutex> lock(command_queue_mtx_);
        
        if (!visual_transparency_commands_.empty()) {
            std::unordered_map<std::string, Entity> linkEntityMap;
            _ecm.Each<components::Link, components::Name>(
                [&](const Entity &e, const components::Link*, const components::Name *n) {
                    linkEntityMap[n->Data()] = e;
                    return true;
                });
            
            while (!visual_transparency_commands_.empty()) {
                auto cmd = visual_transparency_commands_.front();
                visual_transparency_commands_.pop();
                
                auto linkIt = linkEntityMap.find(cmd.link_name);
                if (linkIt == linkEntityMap.end()) {
                    RCLCPP_WARN(node_->get_logger(), "Link '%s' not found", cmd.link_name.c_str());
                    continue;
                }
                Entity linkEntity = linkIt->second;
            
                Entity visualEntity = _ecm.EntityByComponents(
                    components::ParentEntity(linkEntity),
                    components::Visual(),
                    components::Name(cmd.visual_name));
                
                if (visualEntity == kNullEntity) {
                    RCLCPP_WARN(node_->get_logger(), "Visual '%s' not found", cmd.visual_name.c_str());
                    continue;
                }
            
                auto materialComp = _ecm.Component<components::Material>(visualEntity);
                sdf::Material material;
                
                if (materialComp) {
                    material = materialComp->Data();
                } else {
                    material.SetAmbient(gz::math::Color(0.3, 0.3, 0.3, 1.0));
                    material.SetDiffuse(gz::math::Color(0.7, 0.7, 0.7, 1.0));
                }
                
                double alpha = cmd.transparency;
                
                auto diffuse = material.Diffuse();
                diffuse.A() = alpha;
                material.SetDiffuse(diffuse);
                
                auto ambient = material.Ambient();
                ambient.A() = alpha;
                material.SetAmbient(ambient);
                
                auto specular = material.Specular();
                specular.A() = alpha;
                material.SetSpecular(specular);
                
                auto emissive = material.Emissive();
                emissive.A() = alpha;
                material.SetEmissive(emissive);
                
                if (materialComp) {
                    materialComp->Data() = material;
                    _ecm.SetChanged(visualEntity, components::Material::typeId,
                                   ComponentState::OneTimeChange);
                } else {
                    _ecm.CreateComponent(visualEntity, components::Material(material));
                }
            }
        }
    }

    // ========== PROCESS VISUAL VISIBILITY COMMANDS ==========
    {
        std::lock_guard<std::mutex> lock(command_queue_mtx_);
        
        if (!visual_visible_commands_.empty()) {
            std::unordered_map<std::string, Entity> linkEntityMap;
            _ecm.Each<components::Link, components::Name>(
                [&](const Entity &e, const components::Link*, const components::Name *n) {
                    linkEntityMap[n->Data()] = e;
                    return true;
                });
            
            while (!visual_visible_commands_.empty()) {
                auto cmd = visual_visible_commands_.front();
                visual_visible_commands_.pop();
                
                auto linkIt = linkEntityMap.find(cmd.link_name);
                if (linkIt == linkEntityMap.end()) continue;
                Entity linkEntity = linkIt->second;
            
                Entity visualEntity = _ecm.EntityByComponents(
                    components::ParentEntity(linkEntity),
                    components::Visual(),
                    components::Name(cmd.visual_name));
                
                if (visualEntity == kNullEntity) continue;
                
                uint32_t visibilityFlags = cmd.visible ? 1u : 0u;
                
                auto visibilityComp = _ecm.Component<components::VisibilityFlags>(visualEntity);
                if (visibilityComp) {
                    visibilityComp->Data() = visibilityFlags;
                    _ecm.SetChanged(visualEntity, components::VisibilityFlags::typeId,
                                   ComponentState::OneTimeChange);
                } else {
                    _ecm.CreateComponent(visualEntity, components::VisibilityFlags(visibilityFlags));
                }
            }
        }
    }

    // ========== PROCESS VISUAL POSE COMMANDS ==========
    {
        std::lock_guard<std::mutex> lock(command_queue_mtx_);
        
        if (!visual_pose_commands_.empty()) {
            std::unordered_map<std::string, Entity> linkEntityMap;
            _ecm.Each<components::Link, components::Name>(
                [&](const Entity &e, const components::Link*, const components::Name *n) {
                    linkEntityMap[n->Data()] = e;
                    return true;
                });
            
            while (!visual_pose_commands_.empty()) {
                auto cmd = visual_pose_commands_.front();
                visual_pose_commands_.pop();
                
                auto linkIt = linkEntityMap.find(cmd.link_name);
                if (linkIt == linkEntityMap.end()) continue;
                Entity linkEntity = linkIt->second;
            
                Entity visualEntity = _ecm.EntityByComponents(
                    components::ParentEntity(linkEntity),
                    components::Visual(),
                    components::Name(cmd.visual_name));
                
                if (visualEntity == kNullEntity) continue;
                
                gz::math::Vector3d pos(cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z);
                gz::math::Quaterniond rot(cmd.pose.orientation.w,
                                          cmd.pose.orientation.x,
                                          cmd.pose.orientation.y,
                                          cmd.pose.orientation.z);
                rot.Normalize();
                gz::math::Pose3d newPose(pos, rot);
                
                auto poseComp = _ecm.Component<components::Pose>(visualEntity);
                if (poseComp) {
                    poseComp->Data() = newPose;
                    _ecm.SetChanged(visualEntity, components::Pose::typeId,
                                   ComponentState::OneTimeChange);
                } else {
                    _ecm.CreateComponent(visualEntity, components::Pose(newPose));
                }
            }
        }
    }

    // ========== PROCESS VISUAL MESH COMMANDS ==========
    {
        std::lock_guard<std::mutex> lock(command_queue_mtx_);
        
        if (!visual_mesh_commands_.empty()) {
            std::unordered_map<std::string, Entity> linkEntityMap;
            _ecm.Each<components::Link, components::Name>(
                [&](const Entity &e, const components::Link*, const components::Name *n) {
                    linkEntityMap[n->Data()] = e;
                    return true;
                });
            
            while (!visual_mesh_commands_.empty()) {
                auto cmd = visual_mesh_commands_.front();
                visual_mesh_commands_.pop();
                
                auto linkIt = linkEntityMap.find(cmd.link_name);
                if (linkIt == linkEntityMap.end()) continue;
                Entity linkEntity = linkIt->second;
            
                Entity visualEntity = _ecm.EntityByComponents(
                    components::ParentEntity(linkEntity),
                    components::Visual(),
                    components::Name(cmd.visual_name));
                
                if (visualEntity == kNullEntity) continue;
                
                auto geometryComp = _ecm.Component<components::Geometry>(visualEntity);
                sdf::Geometry newGeometry;
                
                if (geometryComp) {
                    newGeometry = geometryComp->Data();
                }
                
                newGeometry.SetType(sdf::GeometryType::MESH);
                
                sdf::Mesh mesh;
                mesh.SetUri(cmd.filename);
                mesh.SetScale(gz::math::Vector3d(cmd.scale.x, cmd.scale.y, cmd.scale.z));
                
                newGeometry.SetMeshShape(mesh);
                
                if (geometryComp) {
                    geometryComp->Data() = newGeometry;
                    _ecm.SetChanged(visualEntity, components::Geometry::typeId,
                                   ComponentState::OneTimeChange);
                } else {
                    _ecm.CreateComponent(visualEntity, components::Geometry(newGeometry));
                }
            }
        }
    }

    // ========== PROCESS LIGHT COMMANDS ==========
    {
        std::lock_guard<std::mutex> lock(command_queue_mtx_);
        
        if (!light_commands_.empty()) {
            // Build light entity map once for all commands
            std::unordered_map<std::string, Entity> lightEntityMap;
            _ecm.Each<components::Light, components::Name>(
                [&](const Entity &entity, const components::Light*, const components::Name *name) -> bool
                {
                    lightEntityMap[name->Data()] = entity;
                    return true;
                });
            
            while (!light_commands_.empty()) {
                auto cmd = light_commands_.front();
                light_commands_.pop();
                
                auto lightIt = lightEntityMap.find(cmd.light_name);
                if (lightIt == lightEntityMap.end()) {
                    RCLCPP_WARN(node_->get_logger(), "Light '%s' not found", cmd.light_name.c_str());
                    continue;
                }
                Entity lightEntity = lightIt->second;
            
                auto lightComp = _ecm.Component<components::Light>(lightEntity);
                if (!lightComp) continue;
                
                auto light = lightComp->Data();
                
                light.SetCastShadows(cmd.cast_shadows);
                light.SetConstantAttenuationFactor(cmd.attenuation_constant);
                light.SetLinearAttenuationFactor(cmd.attenuation_linear);
                light.SetQuadraticAttenuationFactor(cmd.attenuation_quadratic);
                
                light.SetDiffuse(gz::math::Color(
                    cmd.diffuse.r, cmd.diffuse.g, cmd.diffuse.b, cmd.diffuse.a));
                light.SetSpecular(gz::math::Color(
                    cmd.specular.r, cmd.specular.g, cmd.specular.b, cmd.specular.a));
                
                // Only update direction if explicitly set (non-zero)
                gz::math::Vector3d dir(cmd.direction.x, cmd.direction.y, cmd.direction.z);
                if (dir != gz::math::Vector3d::Zero) {
                    light.SetDirection(dir);
                }
                
                // Update Light component for state consistency
                lightComp->Data() = light;
                _ecm.SetChanged(lightEntity, components::Light::typeId,
                               ComponentState::OneTimeChange);
                
                // Use LightCmd to notify rendering system of the update
                // (Light component alone doesn't trigger visual refresh)
                gz::msgs::Light lightMsg = gz::sim::convert<gz::msgs::Light>(light);

                // The converted lightMsg may carry a default (zero) pose from sdf::Light.
                // Inject the current entity pose so the renderer doesn't reset it to origin.
                gz::math::Vector3d cmdPos(cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z);
                gz::math::Quaterniond cmdRot(cmd.pose.orientation.w, cmd.pose.orientation.x,
                                             cmd.pose.orientation.y, cmd.pose.orientation.z);
                // ROS 2 geometry_msgs/Pose defaults to position=(0,0,0) and
                // orientation=(w=1,x=0,y=0,z=0) i.e. identity quaternion.
                bool poseIsDefault = (cmdPos == gz::math::Vector3d::Zero &&
                                      (cmdRot == gz::math::Quaterniond::Identity ||
                                       cmdRot == gz::math::Quaterniond(0, 0, 0, 0)));
                gz::math::Pose3d poseForMsg = poseIsDefault ? currentPose
                    : gz::math::Pose3d(cmdPos, cmdRot.Normalized());
                gz::msgs::Set(lightMsg.mutable_pose(),
                              poseForMsg);

                RCLCPP_DEBUG(node_->get_logger(),
                    "SetLight '%s': LightCmd pose=(%f,%f,%f) poseIsDefault=%d",
                    cmd.light_name.c_str(),
                    poseForMsg.Pos().X(), poseForMsg.Pos().Y(), poseForMsg.Pos().Z(),
                    poseIsDefault);

                auto lightCmdComp = _ecm.Component<components::LightCmd>(lightEntity);
                if (lightCmdComp) {
                    lightCmdComp->Data() = lightMsg;
                    _ecm.SetChanged(lightEntity, components::LightCmd::typeId,
                                   ComponentState::OneTimeChange);
                } else {
                    _ecm.CreateComponent(lightEntity, components::LightCmd(lightMsg));
                }
                
                // Only update ECS pose if it was explicitly set (not the default all-zeros)
                if (!poseIsDefault) {
                    if (poseComp) {
                        poseComp->Data() = poseForMsg;
                        _ecm.SetChanged(lightEntity, components::Pose::typeId,
                                       ComponentState::OneTimeChange);
                    } else {
                        _ecm.CreateComponent(lightEntity, components::Pose(poseForMsg));
                    }
                    RCLCPP_INFO(node_->get_logger(),
                        "SetLight '%s': updated ECS pose to (%f,%f,%f)",
                        cmd.light_name.c_str(),
                        poseForMsg.Pos().X(), poseForMsg.Pos().Y(), poseForMsg.Pos().Z());
                } else {
                    RCLCPP_INFO(node_->get_logger(),
                        "SetLight '%s': pose was default, preserving current pose",
                        cmd.light_name.c_str());
                }
            }
        }
    }
}

//////////////////////////////////////////////////
void DeepRacerGazeboSystemPlugin::PostUpdate(const UpdateInfo &/*_info*/,
                                            const EntityComponentManager &_ecm)
{
    // ========== BUILD SNAPSHOT FOR ROS CALLBACKS (SIMULATION THREAD) ==========
    std::unique_lock<std::shared_mutex> lk(snapshot_mtx_);
    
    // Clear previous snapshot
    name_by_entity_.clear();
    model_by_name_.clear();
    link_by_name_.clear();
    parent_.clear();
    children_.clear();
    models_.clear();
    links_.clear();
    joints_.clear();
    collisions_.clear();
    static_models_.clear();
    model_states_cache_.clear();
    link_states_cache_.clear();
    visual_states_cache_.clear();
    light_names_cache_.clear();

    // Build entity name map
    _ecm.Each<components::Name>([&](const Entity e, const components::Name *n) {
        name_by_entity_[e] = n->Data();
        return true;
    });

    // Build parent/children relations
    _ecm.Each<components::ParentEntity>([&](const Entity e, const components::ParentEntity *p) {
        parent_[e] = p->Data();
        children_.emplace(p->Data(), e);
        return true;
    });

    // Build model map and cache states
    _ecm.Each<components::Model, components::Name>(
        [&](const Entity e, const components::Model*, const components::Name *n) {
            models_.insert(e);
            model_by_name_[n->Data()] = e;
            
            // Cache model state with WORLD-frame velocities
            ModelStateSnapshot snapshot;
            
            // Get world pose
            snapshot.worldPose = gz::sim::worldPose(e, _ecm);
            
            // Read WorldLinearVelocity (world frame)
            auto worldLinVelComp = _ecm.Component<components::WorldLinearVelocity>(e);
            if (worldLinVelComp) {
                snapshot.worldLinearVel = worldLinVelComp->Data();
            } else {
                // Fallback: transform body → world manually
                auto bodyLinVelComp = _ecm.Component<components::LinearVelocity>(e);
                if (bodyLinVelComp) {
                    snapshot.worldLinearVel = 
                        snapshot.worldPose.Rot().RotateVector(bodyLinVelComp->Data());
                }
            }
            
            // Read WorldAngularVelocity (world frame)
            auto worldAngVelComp = _ecm.Component<components::WorldAngularVelocity>(e);
            if (worldAngVelComp) {
                snapshot.worldAngularVel = worldAngVelComp->Data();
            } else {
                // Fallback: transform body → world manually
                auto bodyAngVelComp = _ecm.Component<components::AngularVelocity>(e);
                if (bodyAngVelComp) {
                    snapshot.worldAngularVel = 
                        snapshot.worldPose.Rot().RotateVector(bodyAngVelComp->Data());
                }
            }
            
            model_states_cache_[n->Data()] = snapshot;
            return true;
        });

    // Build link map and cache states
    _ecm.Each<components::Link, components::Name>(
        [&](const Entity e, const components::Link*, const components::Name *n) {
            links_.insert(e);
            link_by_name_[n->Data()] = e;
            
            // Cache link state with WORLD-frame velocities
            LinkStateSnapshot snapshot;
            
            // Use worldPose() helper to get world coordinates
            snapshot.worldPose = gz::sim::worldPose(e, _ecm);
            
            auto worldLinVelComp = _ecm.Component<components::WorldLinearVelocity>(e);
            if (worldLinVelComp) {
                snapshot.worldLinearVel = worldLinVelComp->Data();
            } else {
                auto bodyLinVelComp = _ecm.Component<components::LinearVelocity>(e);
                if (bodyLinVelComp) {
                    snapshot.worldLinearVel = 
                        snapshot.worldPose.Rot().RotateVector(bodyLinVelComp->Data());
                }
            }
            
            auto worldAngVelComp = _ecm.Component<components::WorldAngularVelocity>(e);
            if (worldAngVelComp) {
                snapshot.worldAngularVel = worldAngVelComp->Data();
            } else {
                auto bodyAngVelComp = _ecm.Component<components::AngularVelocity>(e);
                if (bodyAngVelComp) {
                    snapshot.worldAngularVel = 
                        snapshot.worldPose.Rot().RotateVector(bodyAngVelComp->Data());
                }
            }
            
            link_states_cache_[n->Data()] = snapshot;
            return true;
        });

    // Build visual cache
    _ecm.Each<components::Visual, components::Name, components::ParentEntity>(
        [&](const Entity e, const components::Visual*, 
            const components::Name *n, const components::ParentEntity *p) {
            
            auto linkNameIt = name_by_entity_.find(p->Data());
            if (linkNameIt == name_by_entity_.end()) return true;
            
            std::string key = linkNameIt->second + "::" + n->Data();
            
            VisualStateSnapshot snapshot;
            snapshot.link_name = linkNameIt->second;
            
            auto poseComp = _ecm.Component<components::Pose>(e);
            if (poseComp) {
                const auto &pose = poseComp->Data();
                snapshot.pose.position.x = pose.Pos().X();
                snapshot.pose.position.y = pose.Pos().Y();
                snapshot.pose.position.z = pose.Pos().Z();
                snapshot.pose.orientation.x = pose.Rot().X();
                snapshot.pose.orientation.y = pose.Rot().Y();
                snapshot.pose.orientation.z = pose.Rot().Z();
                snapshot.pose.orientation.w = pose.Rot().W();
            }
            
            auto materialComp = _ecm.Component<components::Material>(e);
            if (materialComp) {
                const auto &mat = materialComp->Data();
                snapshot.ambient.r = mat.Ambient().R();
                snapshot.ambient.g = mat.Ambient().G();
                snapshot.ambient.b = mat.Ambient().B();
                snapshot.ambient.a = mat.Ambient().A();
                
                snapshot.diffuse.r = mat.Diffuse().R();
                snapshot.diffuse.g = mat.Diffuse().G();
                snapshot.diffuse.b = mat.Diffuse().B();
                snapshot.diffuse.a = mat.Diffuse().A();
                
                snapshot.specular.r = mat.Specular().R();
                snapshot.specular.g = mat.Specular().G();
                snapshot.specular.b = mat.Specular().B();
                snapshot.specular.a = mat.Specular().A();
                
                snapshot.emissive.r = mat.Emissive().R();
                snapshot.emissive.g = mat.Emissive().G();
                snapshot.emissive.b = mat.Emissive().B();
                snapshot.emissive.a = mat.Emissive().A();
                
                snapshot.transparency = 1.0 - snapshot.diffuse.a;
            }
            
            auto geometryComp = _ecm.Component<components::Geometry>(e);
            if (geometryComp) {
                const auto &geom = geometryComp->Data();
                snapshot.geometry_type = static_cast<uint16_t>(geom.Type());
                
                const auto *mesh = geom.MeshShape();
                if (mesh) {
                    snapshot.mesh_filename = mesh->Uri();
                    snapshot.mesh_scale.x = mesh->Scale().X();
                    snapshot.mesh_scale.y = mesh->Scale().Y();
                    snapshot.mesh_scale.z = mesh->Scale().Z();
                }
            }
            
            auto visibilityComp = _ecm.Component<components::VisibilityFlags>(e);
            snapshot.visible = (visibilityComp && visibilityComp->Data() != 0);
            
            visual_states_cache_[key] = snapshot;
            return true;
        });

    // Cache light names
    _ecm.Each<components::Light, components::Name>(
        [&](const Entity /*e*/, const components::Light*, const components::Name *n) {
            light_names_cache_.push_back(n->Data());
            return true;
        });

    // Build other entity sets
    _ecm.Each<components::Joint>([&](const Entity e, const components::Joint*) {
        joints_.insert(e);
        return true;
    });

    _ecm.Each<components::Collision>([&](const Entity e, const components::Collision*) {
        collisions_.insert(e);
        return true;
    });

    _ecm.Each<components::Model, components::Static>(
        [&](const Entity e, const components::Model*, const components::Static*) {
            static_models_.insert(e);
            return true;
        });
}

//////////////////////////////////////////////////
void DeepRacerGazeboSystemPlugin::initializeROS2()
{
    RCLCPP_INFO(rclcpp::get_logger("deepracer_gazebo_system_plugin"), 
                "Initializing ROS 2 integration");

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("deepracer_gazebo_system", "deepracer");
    
    RCLCPP_INFO(node_->get_logger(), "ROS 2 node created: %s", node_->get_name());

    createServices();

    executor_running_ = true;
    executor_thread_ = std::thread(&DeepRacerGazeboSystemPlugin::executorThread, this);

    RCLCPP_INFO(node_->get_logger(), "ROS 2 integration initialized successfully");
}

//////////////////////////////////////////////////
void DeepRacerGazeboSystemPlugin::createServices()
{
    RCLCPP_INFO(node_->get_logger(), "Creating ROS 2 services");

    get_light_names_srv_ = node_->create_service<deepracer_msgs::srv::GetLightNames>(
        "get_light_names",
        std::bind(&DeepRacerGazeboSystemPlugin::getLightNamesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_visual_names_srv_ = node_->create_service<deepracer_msgs::srv::GetVisualNames>(
        "get_visual_names",
        std::bind(&DeepRacerGazeboSystemPlugin::getVisualNamesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_visual_srv_ = node_->create_service<deepracer_msgs::srv::GetVisual>(
        "get_visual",
        std::bind(&DeepRacerGazeboSystemPlugin::getVisualCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_visuals_srv_ = node_->create_service<deepracer_msgs::srv::GetVisuals>(
        "get_visuals",
        std::bind(&DeepRacerGazeboSystemPlugin::getVisualsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_color_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualColor>(
        "set_visual_color",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualColorCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_colors_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualColors>(
        "set_visual_colors",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualColorsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_transparency_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualTransparency>(
        "set_visual_transparency",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualTransparencyCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_transparencies_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualTransparencies>(
        "set_visual_transparencies",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualTransparenciesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_visible_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualVisible>(
        "set_visual_visible",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualVisibleCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_visibles_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualVisibles>(
        "set_visual_visibles",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualVisiblesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_pose_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualPose>(
        "set_visual_pose",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualPoseCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_poses_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualPoses>(
        "set_visual_poses",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualPosesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_mesh_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualMesh>(
        "set_visual_mesh",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualMeshCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_visual_meshes_srv_ = node_->create_service<deepracer_msgs::srv::SetVisualMeshes>(
        "set_visual_meshes",
        std::bind(&DeepRacerGazeboSystemPlugin::setVisualMeshesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_model_states_srv_ = node_->create_service<deepracer_msgs::srv::GetModelStates>(
        "get_model_states",
        std::bind(&DeepRacerGazeboSystemPlugin::getModelStatesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_model_states_srv_ = node_->create_service<deepracer_msgs::srv::SetModelStates>(
        "set_model_states",
        std::bind(&DeepRacerGazeboSystemPlugin::setModelStatesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_link_states_srv_ = node_->create_service<deepracer_msgs::srv::GetLinkStates>(
        "get_link_states",
        std::bind(&DeepRacerGazeboSystemPlugin::getLinkStatesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_link_states_srv_ = node_->create_service<deepracer_msgs::srv::SetLinkStates>(
        "set_link_states",
        std::bind(&DeepRacerGazeboSystemPlugin::setLinkStatesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    pause_physics_srv_ = node_->create_service<std_srvs::srv::Empty>(
        "pause_physics_dr",
        std::bind(&DeepRacerGazeboSystemPlugin::pausePhysicsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    unpause_physics_srv_ = node_->create_service<std_srvs::srv::Empty>(
        "unpause_physics_dr",
        std::bind(&DeepRacerGazeboSystemPlugin::unpausePhysicsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    get_model_properties_srv_ = node_->create_service<deepracer_msgs::srv::GetModelProperties>(
        "get_model_properties",
        std::bind(&DeepRacerGazeboSystemPlugin::getModelPropertiesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_light_properties_srv_ = node_->create_service<deepracer_msgs::srv::SetLightProperties>(
        "set_light_properties",
        std::bind(&DeepRacerGazeboSystemPlugin::setLightPropertiesCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    spawn_sdf_model_srv_ = node_->create_service<deepracer_msgs::srv::SpawnModel>(
        "spawn_sdf_model",
        std::bind(&DeepRacerGazeboSystemPlugin::spawnSDFModelCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    spawn_urdf_model_srv_ = node_->create_service<deepracer_msgs::srv::SpawnModel>(
        "spawn_urdf_model",
        std::bind(&DeepRacerGazeboSystemPlugin::spawnURDFModelCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    delete_model_srv_ = node_->create_service<deepracer_msgs::srv::DeleteModel>(
        "delete_model",
        std::bind(&DeepRacerGazeboSystemPlugin::deleteModelCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(), "All ROS 2 services created successfully");
}

//////////////////////////////////////////////////
void DeepRacerGazeboSystemPlugin::executorThread()
{
    RCLCPP_INFO(node_->get_logger(), "Starting ROS 2 executor thread");
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    
    while (executor_running_ && rclcpp::ok())
    {
        executor.spin_some(std::chrono::milliseconds(10));
    }
    
    RCLCPP_INFO(node_->get_logger(), "ROS 2 executor thread stopped");
}

//////////////////////////////////////////////////
void DeepRacerGazeboSystemPlugin::shutdownROS2()
{
    if (executor_running_)
    {
        RCLCPP_INFO(node_->get_logger(), "Shutting down ROS 2 integration");
        
        executor_running_ = false;
        
        if (executor_thread_.joinable())
        {
            executor_thread_.join();
        }
        
        node_.reset();
        
        RCLCPP_INFO(rclcpp::get_logger("deepracer_gazebo_system_plugin"), 
                    "ROS 2 integration shutdown complete");
    }
}

//////////////////////////////////////////////////
// ========== GET SERVICE CALLBACKS (Read from snapshot cache) ==========
//////////////////////////////////////////////////

void DeepRacerGazeboSystemPlugin::getLightNamesCallback(
    const std::shared_ptr<deepracer_msgs::srv::GetLightNames::Request> /*request*/,
    std::shared_ptr<deepracer_msgs::srv::GetLightNames::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "getLightNames service called");
    
    std::shared_lock<std::shared_mutex> lock(snapshot_mtx_);
    
    try {
        response->light_names = light_names_cache_;
        response->success = true;
        response->status_message = "Found " + std::to_string(light_names_cache_.size()) + " lights";
        
        RCLCPP_DEBUG(node_->get_logger(), "Found %zu lights in cache", light_names_cache_.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error retrieving light names: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in getLightNames: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::getVisualNamesCallback(
    const std::shared_ptr<deepracer_msgs::srv::GetVisualNames::Request> request,
    std::shared_ptr<deepracer_msgs::srv::GetVisualNames::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "getVisualNames service called for %zu links", 
                request->link_names.size());
    
    std::shared_lock<std::shared_mutex> lock(snapshot_mtx_);
    
    try {
        bool get_all = request->link_names.empty();
        
        for (const auto &[key, snapshot] : visual_states_cache_) {
            size_t separator = key.find("::");
            if (separator == std::string::npos) continue;
            
            std::string link_name = key.substr(0, separator);
            std::string visual_name = key.substr(separator + 2);
            
            if (!get_all) {
                bool found = false;
                for (const auto &requested_link : request->link_names) {
                    // Fix: Strip model prefix (e.g., "racecar::base_link" -> "base_link")
                    std::string requested_link_stripped = requested_link;
                    size_t prefix_sep = requested_link.find("::");
                    if (prefix_sep != std::string::npos) {
                        requested_link_stripped = requested_link.substr(prefix_sep + 2);
                    }
                    
                    if (requested_link_stripped == link_name) {
                        found = true;
                        break;
                    }
                }
                if (!found) continue;
            }
            
            response->visual_names.push_back(visual_name);
            response->link_names.push_back(link_name);
        }
        
        response->success = true;
        response->status_message = "Found " + std::to_string(response->visual_names.size()) + " visuals";
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error retrieving visual names: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in getVisualNames: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::getVisualCallback(
    const std::shared_ptr<deepracer_msgs::srv::GetVisual::Request> request,
    std::shared_ptr<deepracer_msgs::srv::GetVisual::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "getVisual service called for link: %s, visual: %s", 
                request->link_name.c_str(), request->visual_name.c_str());
    
    std::shared_lock<std::shared_mutex> lock(snapshot_mtx_);
    
    try {
        std::string key = request->link_name + "::" + request->visual_name;
        
        auto it = visual_states_cache_.find(key);
        if (it == visual_states_cache_.end()) {
            response->success = false;
            response->status_message = "Visual not found in cache";
            return;
        }
        
        const auto &snapshot = it->second;
        response->ambient = snapshot.ambient;
        response->diffuse = snapshot.diffuse;
        response->specular = snapshot.specular;
        response->emissive = snapshot.emissive;
        response->transparency = snapshot.transparency;
        response->visible = snapshot.visible;
        response->pose = snapshot.pose;
        response->mesh_geom_filename = snapshot.mesh_filename;
        response->mesh_geom_scale = snapshot.mesh_scale;
        response->geometry_type = snapshot.geometry_type;
        
        response->success = true;
        response->status_message = "Visual properties retrieved from cache";
        
        RCLCPP_DEBUG(node_->get_logger(), "Retrieved visual properties for '%s' in link '%s' from cache", 
                    request->visual_name.c_str(), request->link_name.c_str());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error retrieving visual properties: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in getVisual: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::getVisualsCallback(
    const std::shared_ptr<deepracer_msgs::srv::GetVisuals::Request> request,
    std::shared_ptr<deepracer_msgs::srv::GetVisuals::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "getVisuals service called for %zu visuals", 
                request->link_names.size());
    
    std::shared_lock<std::shared_mutex> lock(snapshot_mtx_);
    
    try {
        if (request->link_names.size() != request->visual_names.size()) {
            response->success = false;
            response->status_message = "GetVisuals: link_names, visual_names must be same size!";
            return;
        }
        
        response->status.reserve(request->link_names.size());
        response->messages.reserve(request->link_names.size());
        
        for (size_t i = 0; i < request->link_names.size(); ++i) {
            std::string key = request->link_names[i] + "::" + request->visual_names[i];
            
            auto it = visual_states_cache_.find(key);
            if (it == visual_states_cache_.end()) {
                // Visual not found - add default values
                response->visual_names.push_back(request->visual_names[i]);
                response->link_names.push_back(request->link_names[i]);
                
                std_msgs::msg::ColorRGBA defaultColor;
                defaultColor.r = defaultColor.g = defaultColor.b = 0.5;
                defaultColor.a = 1.0;
                
                response->ambients.push_back(defaultColor);
                response->diffuses.push_back(defaultColor);
                response->speculars.push_back(defaultColor);
                response->emissives.push_back(defaultColor);
                response->transparencies.push_back(0.0);
                response->visibles.push_back(1);
                response->geometry_types.push_back(0);
                response->mesh_geom_filenames.push_back("");
                
                geometry_msgs::msg::Pose defaultPose;
                defaultPose.orientation.w = 1.0;
                response->poses.push_back(defaultPose);
                
                geometry_msgs::msg::Vector3 defaultScale;
                defaultScale.x = defaultScale.y = defaultScale.z = 1.0;
                response->mesh_geom_scales.push_back(defaultScale);
                
                response->status.push_back(0);
                response->messages.push_back("Visual not found");
                continue;
            }
            
            const auto &snapshot = it->second;
            
            response->visual_names.push_back(request->visual_names[i]);
            response->link_names.push_back(request->link_names[i]);
            response->ambients.push_back(snapshot.ambient);
            response->diffuses.push_back(snapshot.diffuse);
            response->speculars.push_back(snapshot.specular);
            response->emissives.push_back(snapshot.emissive);
            response->transparencies.push_back(snapshot.transparency);
            response->visibles.push_back(snapshot.visible ? 1 : 0);
            response->geometry_types.push_back(snapshot.geometry_type);
            response->mesh_geom_filenames.push_back(snapshot.mesh_filename);
            response->poses.push_back(snapshot.pose);
            response->mesh_geom_scales.push_back(snapshot.mesh_scale);
            response->status.push_back(1);
            response->messages.push_back("");
        }
        
        response->success = true;
        response->status_message = "Retrieved " + std::to_string(response->visual_names.size()) + " visual properties";
        
        RCLCPP_DEBUG(node_->get_logger(), "Retrieved %zu visual properties from cache", response->visual_names.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error retrieving visual properties: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in getVisuals: %s", e.what());
    }
}

//////////////////////////////////////////////////
// ========== SET SERVICE CALLBACKS (Queue commands) ==========
//////////////////////////////////////////////////

void DeepRacerGazeboSystemPlugin::setVisualColorCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualColor::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualColor::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualColor service called for link: %s, visual: %s", 
                request->link_name.c_str(), request->visual_name.c_str());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        SetVisualColorCommand cmd;
        cmd.link_name = request->link_name;
        cmd.visual_name = request->visual_name;
        cmd.ambient = request->ambient;
        cmd.diffuse = request->diffuse;
        cmd.specular = request->specular;
        cmd.emissive = request->emissive;
        
        visual_color_commands_.push(cmd);
        
        response->success = true;
        response->status_message = "Visual color command queued";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued color update for visual '%s' in link '%s'", 
                    request->visual_name.c_str(), request->link_name.c_str());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual color command: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualColor: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualColorsCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualColors::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualColors::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualColors service called for %zu visuals", 
                request->link_names.size());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        if (request->link_names.size() != request->visual_names.size() ||
            request->link_names.size() != request->ambients.size() ||
            request->link_names.size() != request->diffuses.size() ||
            request->link_names.size() != request->speculars.size() ||
            request->link_names.size() != request->emissives.size()) {
            response->success = false;
            response->status_message = "Mismatched array sizes in request";
            return;
        }
        
        for (size_t i = 0; i < request->link_names.size(); ++i) {
            SetVisualColorCommand cmd;
            cmd.link_name = request->link_names[i];
            cmd.visual_name = request->visual_names[i];
            cmd.ambient = request->ambients[i];
            cmd.diffuse = request->diffuses[i];
            cmd.specular = request->speculars[i];
            cmd.emissive = request->emissives[i];
            
            visual_color_commands_.push(cmd);
        }
        
        response->success = true;
        response->status_message = "Queued " + std::to_string(request->link_names.size()) + " visual color commands";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued %zu visual color commands", request->link_names.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual color commands: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualColors: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualTransparencyCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualTransparency::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualTransparency::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualTransparency service called for link: %s, visual: %s, transparency: %f", 
                request->link_name.c_str(), request->visual_name.c_str(), request->transparency);
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        SetVisualTransparencyCommand cmd;
        cmd.link_name = request->link_name;
        cmd.visual_name = request->visual_name;
        cmd.transparency = request->transparency;
        
        visual_transparency_commands_.push(cmd);
        
        response->success = true;
        response->status_message = "Visual transparency command queued";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued transparency update for visual '%s' in link '%s' to %.2f", 
                    request->visual_name.c_str(), request->link_name.c_str(), request->transparency);
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual transparency command: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualTransparency: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualTransparenciesCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualTransparencies::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualTransparencies::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualTransparencies service called for %zu visuals", 
                request->link_names.size());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        if (request->link_names.size() != request->visual_names.size() ||
            request->link_names.size() != request->transparencies.size()) {
            response->success = false;
            response->status_message = "Mismatched array sizes in request";
            return;
        }
        
        for (size_t i = 0; i < request->link_names.size(); ++i) {
            SetVisualTransparencyCommand cmd;
            cmd.link_name = request->link_names[i];
            cmd.visual_name = request->visual_names[i];
            cmd.transparency = request->transparencies[i];
            
            visual_transparency_commands_.push(cmd);
        }
        
        response->success = true;
        response->status_message = "Queued " + std::to_string(request->link_names.size()) + " visual transparency commands";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued %zu visual transparency commands", request->link_names.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual transparency commands: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualTransparencies: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualVisibleCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualVisible::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualVisible::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualVisible service called for link: %s, visual: %s, visible: %s", 
                request->link_name.c_str(), request->visual_name.c_str(), 
                request->visible ? "true" : "false");
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        SetVisualVisibleCommand cmd;
        cmd.link_name = request->link_name;
        cmd.visual_name = request->visual_name;
        cmd.visible = request->visible;
        
        visual_visible_commands_.push(cmd);
        
        response->success = true;
        response->status_message = "Visual visibility command queued";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued visibility update for visual '%s' in link '%s' to %s", 
                    request->visual_name.c_str(), request->link_name.c_str(), 
                    request->visible ? "visible" : "invisible");
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual visibility command: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualVisible: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualVisiblesCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualVisibles::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualVisibles::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualVisibles service called for %zu visuals", 
                request->link_names.size());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        if (request->link_names.size() != request->visual_names.size() ||
            request->link_names.size() != request->visibles.size()) {
            response->success = false;
            response->status_message = "Mismatched array sizes in request";
            return;
        }
        
        response->status.resize(request->link_names.size(), 1);
        response->messages.resize(request->link_names.size(), "Command queued");
        
        for (size_t i = 0; i < request->link_names.size(); ++i) {
            SetVisualVisibleCommand cmd;
            cmd.link_name = request->link_names[i];
            cmd.visual_name = request->visual_names[i];
            cmd.visible = (request->visibles[i] != 0);
            
            visual_visible_commands_.push(cmd);
        }
        
        response->success = true;
        response->status_message = "Queued " + std::to_string(request->link_names.size()) + " visual visibility commands";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued %zu visual visibility commands", request->link_names.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual visibility commands: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualVisibles: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualPoseCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualPose::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualPose::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualPose service called for link: %s, visual: %s", 
                request->link_name.c_str(), request->visual_name.c_str());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        SetVisualPoseCommand cmd;
        cmd.link_name = request->link_name;
        cmd.visual_name = request->visual_name;
        cmd.pose = request->pose;
        
        visual_pose_commands_.push(cmd);
        
        response->success = true;
        response->status_message = "Visual pose command queued";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued pose update for visual '%s' in link '%s'", 
                   request->visual_name.c_str(), request->link_name.c_str());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual pose command: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualPose: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualPosesCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualPoses::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualPoses::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualPoses service called for %zu visual pose updates", 
                request->link_names.size());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        if (request->link_names.size() != request->visual_names.size() ||
            request->link_names.size() != request->poses.size()) {
            response->success = false;
            response->status_message = "Input arrays must have the same size";
            return;
        }
        
        for (size_t i = 0; i < request->link_names.size(); ++i) {
            SetVisualPoseCommand cmd;
            cmd.link_name = request->link_names[i];
            cmd.visual_name = request->visual_names[i];
            cmd.pose = request->poses[i];
            
            visual_pose_commands_.push(cmd);
        }
        
        response->success = true;
        response->status_message = "Queued " + std::to_string(request->link_names.size()) + " visual pose commands";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued %zu visual pose commands", request->link_names.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual pose commands: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualPoses: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualMeshCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualMesh::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualMesh::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualMesh service called for link: %s, visual: %s, mesh: %s", 
                request->link_name.c_str(), request->visual_name.c_str(), request->filename.c_str());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        SetVisualMeshCommand cmd;
        cmd.link_name = request->link_name;
        cmd.visual_name = request->visual_name;
        cmd.filename = request->filename;
        cmd.scale = request->scale;
        
        visual_mesh_commands_.push(cmd);
        
        response->success = true;
        response->status_message = "Visual mesh command queued";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued mesh update for visual '%s' in link '%s' to '%s'", 
                   request->visual_name.c_str(), request->link_name.c_str(), request->filename.c_str());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual mesh command: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualMesh: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setVisualMeshesCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetVisualMeshes::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetVisualMeshes::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setVisualMeshes service called for %zu visual mesh updates", 
                request->link_names.size());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        if (request->link_names.size() != request->visual_names.size() ||
            request->link_names.size() != request->filenames.size() ||
            request->link_names.size() != request->scales.size()) {
            response->success = false;
            response->status_message = "Input arrays must have the same size";
            return;
        }
        
        for (size_t i = 0; i < request->link_names.size(); ++i) {
            SetVisualMeshCommand cmd;
            cmd.link_name = request->link_names[i];
            cmd.visual_name = request->visual_names[i];
            cmd.filename = request->filenames[i];
            cmd.scale = request->scales[i];
            
            visual_mesh_commands_.push(cmd);
        }
        
        response->success = true;
        response->status_message = "Queued " + std::to_string(request->link_names.size()) + " visual mesh commands";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued %zu visual mesh commands", request->link_names.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing visual mesh commands: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setVisualMeshes: %s", e.what());
    }
}

//////////////////////////////////////////////////
// ========== MODEL/LINK STATE CALLBACKS ==========
//////////////////////////////////////////////////

void DeepRacerGazeboSystemPlugin::getModelStatesCallback(
    const std::shared_ptr<deepracer_msgs::srv::GetModelStates::Request> request,
    std::shared_ptr<deepracer_msgs::srv::GetModelStates::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "getModelStates service called for %zu models", 
                request->model_names.size());
    
    std::shared_lock<std::shared_mutex> lock(snapshot_mtx_);
    
    try {
        if (request->model_names.size() != request->relative_entity_names.size()) {
            response->success = false;
            response->status_message = "GetModelStates: model_names, relative_entity_names must be same size!";
            return;
        }
        
        response->status.reserve(request->model_names.size());
        response->messages.reserve(request->model_names.size());
        
        for (size_t i = 0; i < request->model_names.size(); ++i) {
            const std::string &model_name = request->model_names[i];
            const std::string &relative_entity_name = request->relative_entity_names[i];
            
            deepracer_msgs::msg::ModelState model_state;
            model_state.model_name = model_name;
            model_state.reference_frame = relative_entity_name;
            
            auto modelIt = model_states_cache_.find(model_name);
            if (modelIt == model_states_cache_.end()) {
                RCLCPP_ERROR(node_->get_logger(), "GetModelStates: model [%s] does not exist", model_name.c_str());
                response->model_states.push_back(model_state);
                response->status.push_back(0);
                response->messages.push_back("GetModelStates: model does not exist");
                continue;
            }
            
            const auto &snapshot = modelIt->second;
            
            gz::math::Pose3d resultPose = snapshot.worldPose;
            gz::math::Vector3d resultLinearVel = snapshot.worldLinearVel;
            gz::math::Vector3d resultAngularVel = snapshot.worldAngularVel;
            
            RCLCPP_DEBUG(node_->get_logger(), "DEBUG ECM: Raw orientation from cache for model '%s': x=%.6f, y=%.6f, z=%.6f, w=%.6f", 
                        model_name.c_str(), resultPose.Rot().X(), resultPose.Rot().Y(), resultPose.Rot().Z(), resultPose.Rot().W());
            
            // Apply reference frame transformation if specified
            RCLCPP_DEBUG(node_->get_logger(), "DEBUG TRANSFORM: Before transformation pose: x=%.6f, y=%.6f, z=%.6f", 
                        resultPose.Pos().X(), resultPose.Pos().Y(), resultPose.Pos().Z());
            
            if (!relative_entity_name.empty() && 
                relative_entity_name != "world" && 
                relative_entity_name != "map" && 
                relative_entity_name != "/map") {
                
                auto refModelIt = model_states_cache_.find(relative_entity_name);
                auto refLinkIt = link_states_cache_.find(relative_entity_name);
                
                if (refModelIt != model_states_cache_.end()) {
                    const auto &refSnapshot = refModelIt->second;
                    
                    // Transform pose from world to reference frame
                    resultPose = resultPose.CoordPoseSolve(refSnapshot.worldPose);
                    
                    // Transform velocities from world to reference frame
                    resultLinearVel = refSnapshot.worldPose.Rot().RotateVectorReverse(
                        resultLinearVel - refSnapshot.worldLinearVel);
                    resultAngularVel = refSnapshot.worldPose.Rot().RotateVectorReverse(
                        resultAngularVel - refSnapshot.worldAngularVel);
                        
                } else if (refLinkIt != link_states_cache_.end()) {
                    const auto &refSnapshot = refLinkIt->second;
                    
                    resultPose = resultPose.CoordPoseSolve(refSnapshot.worldPose);
                    resultLinearVel = refSnapshot.worldPose.Rot().RotateVectorReverse(
                        resultLinearVel - refSnapshot.worldLinearVel);
                    resultAngularVel = refSnapshot.worldPose.Rot().RotateVectorReverse(
                        resultAngularVel - refSnapshot.worldAngularVel);
                        
                } else {
                    response->model_states.push_back(model_state);
                    response->status.push_back(0);
                    response->messages.push_back("GetModelStates: reference relative_entity_name not found, did you forget to scope the body by model name?");
                    continue;
                }
            }
            
            RCLCPP_DEBUG(node_->get_logger(), "DEBUG TRANSFORM: After transformation pose: x=%.6f, y=%.6f, z=%.6f", 
                        resultPose.Pos().X(), resultPose.Pos().Y(), resultPose.Pos().Z());
            
            // Populate model state response
            model_state.pose.position.x = resultPose.Pos().X();
            model_state.pose.position.y = resultPose.Pos().Y();
            model_state.pose.position.z = resultPose.Pos().Z();
            
            RCLCPP_DEBUG(node_->get_logger(), "DEBUG RESPONSE: Final response values for model '%s': x=%.6f, y=%.6f, z=%.6f", 
                        model_name.c_str(), model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z);
            
            model_state.pose.orientation.w = resultPose.Rot().W();
            model_state.pose.orientation.x = resultPose.Rot().X();
            model_state.pose.orientation.y = resultPose.Rot().Y();
            model_state.pose.orientation.z = resultPose.Rot().Z();
            
            model_state.twist.linear.x = resultLinearVel.X();
            model_state.twist.linear.y = resultLinearVel.Y();
            model_state.twist.linear.z = resultLinearVel.Z();
            model_state.twist.angular.x = resultAngularVel.X();
            model_state.twist.angular.y = resultAngularVel.Y();
            model_state.twist.angular.z = resultAngularVel.Z();
            
            response->model_states.push_back(model_state);
            response->status.push_back(1);
            response->messages.push_back("");
        }
        
        response->success = true;
        response->status_message = "GetModelStates: got properties";
        
        RCLCPP_DEBUG(node_->get_logger(), "Retrieved %zu model states from cache", response->model_states.size());
        
    } catch (const std::runtime_error &e) {
        response->success = false;
        response->status_message = "Runtime error retrieving model states: " + std::string(e.what());
    } catch (const std::logic_error &e) {
        response->success = false;
        response->status_message = "Logic error retrieving model states: " + std::string(e.what());
    } catch (const std::bad_alloc &e) {
        response->success = false;
        response->status_message = "Memory allocation error retrieving model states: " + std::string(e.what());
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error retrieving model states: " + std::string(e.what());
    } catch (...) {
        response->success = false;
        response->status_message = "Unknown error retrieving model states";
    }
}

void DeepRacerGazeboSystemPlugin::setModelStatesCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetModelStates::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetModelStates::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setModelStates service called for %zu models", 
                request->model_states.size());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        response->status.resize(request->model_states.size());
        response->messages.resize(request->model_states.size());
        
        for (size_t i = 0; i < request->model_states.size(); ++i) {
            const auto &model_state = request->model_states[i];
            
            SetModelStateCommand cmd;
            cmd.model_name = model_state.model_name;
            cmd.reference_frame = model_state.reference_frame;
            
            cmd.target_pose.Pos().Set(
                model_state.pose.position.x,
                model_state.pose.position.y,
                model_state.pose.position.z);
            cmd.target_pose.Rot().Set(
                model_state.pose.orientation.w,
                model_state.pose.orientation.x,
                model_state.pose.orientation.y,
                model_state.pose.orientation.z);
            cmd.target_pose.Rot().Normalize();
            
            cmd.target_linear_vel.Set(
                model_state.twist.linear.x,
                model_state.twist.linear.y,
                model_state.twist.linear.z);
            cmd.target_angular_vel.Set(
                model_state.twist.angular.x,
                model_state.twist.angular.y,
                model_state.twist.angular.z);
            
            model_state_commands_.push(cmd);
            
            response->status[i] = 1;
            response->messages[i] = "Command queued";
            
            RCLCPP_DEBUG(node_->get_logger(), "Queued state update for model '%s'", 
                       cmd.model_name.c_str());
        }
        
        response->success = true;
        response->status_message = "SetModelStates: commands queued";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued %zu model state commands", request->model_states.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing model state commands: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setModelStates: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::getLinkStatesCallback(
    const std::shared_ptr<deepracer_msgs::srv::GetLinkStates::Request> request,
    std::shared_ptr<deepracer_msgs::srv::GetLinkStates::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "getLinkStates service called for %zu links", 
                request->link_names.size());
    
    std::shared_lock<std::shared_mutex> lock(snapshot_mtx_);
    
    try {
        if (request->link_names.size() != request->reference_frames.size()) {
            response->success = false;
            response->status_message = "GetLinkStates: link_names, reference_frames must be same size!";
            return;
        }
        
        response->status.reserve(request->link_names.size());
        response->messages.reserve(request->link_names.size());
        
        for (size_t i = 0; i < request->link_names.size(); ++i) {
            const std::string &link_name = request->link_names[i];
            const std::string &reference_frame = request->reference_frames[i];
            
            deepracer_msgs::msg::LinkState link_state;
            link_state.link_name = link_name;
            link_state.reference_frame = reference_frame;
            
            auto linkIt = link_states_cache_.find(link_name);
            if (linkIt == link_states_cache_.end()) {
                RCLCPP_ERROR(node_->get_logger(), "Link '%s' not found in cache", link_name.c_str());
                response->link_states.push_back(link_state);
                response->status.push_back(0);
                response->messages.push_back("GetLinkStates: link not found, did you forget to scope the link by model name?");
                continue;
            }
            
            const auto &snapshot = linkIt->second;
            
            gz::math::Pose3d resultPose = snapshot.worldPose;
            gz::math::Vector3d resultLinearVel = snapshot.worldLinearVel;
            gz::math::Vector3d resultAngularVel = snapshot.worldAngularVel;
            
            RCLCPP_DEBUG(node_->get_logger(), "DEBUG getLinkStates: worldPose from cache for link '%s': pos=[%.3f,%.3f,%.3f]", 
                        link_name.c_str(), resultPose.Pos().X(), resultPose.Pos().Y(), resultPose.Pos().Z());
            
            if (!reference_frame.empty() && 
                reference_frame != "world" && 
                reference_frame != "map" && 
                reference_frame != "/map") {
                
                auto refLinkIt = link_states_cache_.find(reference_frame);
                auto refModelIt = model_states_cache_.find(reference_frame);
                
                if (refLinkIt != link_states_cache_.end()) {
                    const auto &refSnapshot = refLinkIt->second;
                    
                    resultPose = resultPose.CoordPoseSolve(refSnapshot.worldPose);
                    resultLinearVel = refSnapshot.worldPose.Rot().RotateVectorReverse(
                        resultLinearVel - refSnapshot.worldLinearVel);
                    resultAngularVel = refSnapshot.worldPose.Rot().RotateVectorReverse(
                        resultAngularVel - refSnapshot.worldAngularVel);
                        
                } else if (refModelIt != model_states_cache_.end()) {
                    const auto &refSnapshot = refModelIt->second;
                    
                    resultPose = resultPose.CoordPoseSolve(refSnapshot.worldPose);
                    resultLinearVel = refSnapshot.worldPose.Rot().RotateVectorReverse(
                        resultLinearVel - refSnapshot.worldLinearVel);
                    resultAngularVel = refSnapshot.worldPose.Rot().RotateVectorReverse(
                        resultAngularVel - refSnapshot.worldAngularVel);
                        
                } else {
                    response->link_states.push_back(link_state);
                    response->status.push_back(0);
                    response->messages.push_back("GetLinkStates: reference reference_frame not found, did you forget to scope the link by model name?");
                    continue;
                }
            }
            
            link_state.pose.position.x = resultPose.Pos().X();
            link_state.pose.position.y = resultPose.Pos().Y();
            link_state.pose.position.z = resultPose.Pos().Z();
            link_state.pose.orientation.x = resultPose.Rot().X();
            link_state.pose.orientation.y = resultPose.Rot().Y();
            link_state.pose.orientation.z = resultPose.Rot().Z();
            link_state.pose.orientation.w = resultPose.Rot().W();
            
            link_state.twist.linear.x = resultLinearVel.X();
            link_state.twist.linear.y = resultLinearVel.Y();
            link_state.twist.linear.z = resultLinearVel.Z();
            link_state.twist.angular.x = resultAngularVel.X();
            link_state.twist.angular.y = resultAngularVel.Y();
            link_state.twist.angular.z = resultAngularVel.Z();
            
            response->link_states.push_back(link_state);
            response->status.push_back(1);
            response->messages.push_back("");
        }
        
        response->success = true;
        response->status_message = "GetLinkStates: got state";
        
        RCLCPP_DEBUG(node_->get_logger(), "Retrieved %zu link states from cache", response->link_states.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error retrieving link states: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in getLinkStates: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setLinkStatesCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetLinkStates::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetLinkStates::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setLinkStates service called for %zu links", 
                request->link_states.size());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        response->status.resize(request->link_states.size());
        response->messages.resize(request->link_states.size());
        
        for (size_t i = 0; i < request->link_states.size(); ++i) {
            const auto &link_state = request->link_states[i];
            
            SetLinkStateCommand cmd;
            cmd.link_name = link_state.link_name;
            cmd.reference_frame = link_state.reference_frame;
            
            cmd.target_pose.Pos().Set(
                link_state.pose.position.x,
                link_state.pose.position.y,
                link_state.pose.position.z);
            cmd.target_pose.Rot().Set(
                link_state.pose.orientation.w,
                link_state.pose.orientation.x,
                link_state.pose.orientation.y,
                link_state.pose.orientation.z);
            cmd.target_pose.Rot().Normalize();
            
            cmd.target_linear_vel.Set(
                link_state.twist.linear.x,
                link_state.twist.linear.y,
                link_state.twist.linear.z);
            cmd.target_angular_vel.Set(
                link_state.twist.angular.x,
                link_state.twist.angular.y,
                link_state.twist.angular.z);
            
            link_state_commands_.push(cmd);
            
            response->status[i] = 1;
            response->messages[i] = "Command queued";
        }
        
        response->success = true;
        response->status_message = "SetLinkStates: commands queued";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued %zu link state commands", request->link_states.size());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing link state commands: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setLinkStates: %s", e.what());
    }
}

//////////////////////////////////////////////////
// ========== OTHER SERVICES ==========
//////////////////////////////////////////////////

void DeepRacerGazeboSystemPlugin::pausePhysicsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    RCLCPP_DEBUG(node_->get_logger(), "pausePhysics service called");
    
    // Set atomic flag - actual pause will happen in PreUpdate
    pause_req_.store(true, std::memory_order_relaxed);
}

void DeepRacerGazeboSystemPlugin::unpausePhysicsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    RCLCPP_DEBUG(node_->get_logger(), "unpausePhysics service called");
    
    // Set atomic flag - actual unpause will happen in PreUpdate
    unpause_req_.store(true, std::memory_order_relaxed);
}

void DeepRacerGazeboSystemPlugin::getModelPropertiesCallback(
    const std::shared_ptr<deepracer_msgs::srv::GetModelProperties::Request> request,
    std::shared_ptr<deepracer_msgs::srv::GetModelProperties::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "getModelProperties service called for model: %s", 
                request->model_name.c_str());
    
    std::shared_lock<std::shared_mutex> lock(snapshot_mtx_);
    
    try {
        // Find model
        auto modelIt = model_by_name_.find(request->model_name);
        if (modelIt == model_by_name_.end()) {
            response->success = false;
            response->status_message = "GetModelProperties: model does not exist";
            return;
        }
        
        const auto modelEntity = modelIt->second;
        
        // Get parent model name
        response->parent_model_name.clear();
        auto parentIt = parent_.find(modelEntity);
        if (parentIt != parent_.end()) {
            auto parent_entity = parentIt->second;
            if (models_.count(parent_entity)) {
                auto nameIt = name_by_entity_.find(parent_entity);
                if (nameIt != name_by_entity_.end())
                    response->parent_model_name = nameIt->second;
            }
        }
        
        // Helper to push child entities for DFS
        auto push_children = [&](gz::sim::Entity e, std::vector<gz::sim::Entity> &stk) {
            auto range = children_.equal_range(e);
            for (auto it = range.first; it != range.second; ++it)
                stk.push_back(it->second);
        };
        
        // Find canonical body name - the first link in the model
        response->canonical_body_name.clear();
        {
            std::vector<gz::sim::Entity> stack;
            stack.push_back(modelEntity);
            while (!stack.empty() && response->canonical_body_name.empty()) {
                auto cur = stack.back(); 
                stack.pop_back();
                
                if (links_.count(cur)) {
                    auto nameIt = name_by_entity_.find(cur);
                    if (nameIt != name_by_entity_.end()) {
                        // Use model::link format
                        response->canonical_body_name = request->model_name + "::" + nameIt->second;
                        break;
                    }
                }
                push_children(cur, stack);
            }
        }

        // Collect body (link) names in the model subtree
        response->body_names.clear();
        {
            std::vector<gz::sim::Entity> stack;
            stack.push_back(modelEntity);
            while (!stack.empty()) {
                auto cur = stack.back(); 
                stack.pop_back();
                
                bool isLink = links_.count(cur) > 0;
                
                if (isLink) {
                    auto nameIt = name_by_entity_.find(cur);
                    if (nameIt != name_by_entity_.end())
                        response->body_names.push_back(request->model_name + "::" + nameIt->second);
                }
                push_children(cur, stack);
            }
        }
        
        // Collect geom (collision) names in the model subtree
        response->geom_names.clear();
        {
            std::vector<gz::sim::Entity> stack;
            stack.push_back(modelEntity);
            while (!stack.empty()) {
                auto cur = stack.back(); 
                stack.pop_back();
                
                if (collisions_.count(cur)) {
                    auto nameIt = name_by_entity_.find(cur);
                    if (nameIt != name_by_entity_.end())
                        response->geom_names.push_back(nameIt->second);
                }
                push_children(cur, stack);
            }
        }
        
        // Collect joint names in the model subtree
        response->joint_names.clear();
        {
            std::vector<gz::sim::Entity> stack;
            stack.push_back(modelEntity);
            while (!stack.empty()) {
                auto cur = stack.back(); 
                stack.pop_back();
                
                if (joints_.count(cur)) {
                    auto nameIt = name_by_entity_.find(cur);
                    if (nameIt != name_by_entity_.end())
                        response->joint_names.push_back(nameIt->second);
                }
                push_children(cur, stack);
            }
        }
        
        // Get immediate child model names
        response->child_model_names.clear();
        {
            auto range = children_.equal_range(modelEntity);
            for (auto it = range.first; it != range.second; ++it) {
                auto child = it->second;
                if (models_.count(child)) {
                    auto nameIt = name_by_entity_.find(child);
                    if (nameIt != name_by_entity_.end())
                        response->child_model_names.push_back(nameIt->second);
                }
            }
        }
        
        // Check if model is static
        response->is_static = static_models_.count(modelEntity) > 0;
        
        response->success = true;
        response->status_message = "GetModelProperties: got properties";
        
        RCLCPP_DEBUG(node_->get_logger(), "GetModelProperties for '%s': canonical='%s', %zu bodies, %zu geoms, %zu joints, %zu children, static=%s", 
                    request->model_name.c_str(),
                    response->canonical_body_name.c_str(),
                    response->body_names.size(),
                    response->geom_names.size(), 
                    response->joint_names.size(),
                    response->child_model_names.size(),
                    response->is_static ? "true" : "false");
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error retrieving model properties: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in getModelProperties: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::setLightPropertiesCallback(
    const std::shared_ptr<deepracer_msgs::srv::SetLightProperties::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SetLightProperties::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "setLightProperties service called for light: %s",
                request->light_name.c_str());
    
    std::lock_guard<std::mutex> lock(command_queue_mtx_);
    
    try {
        SetLightPropertiesCommand cmd;
        cmd.light_name = request->light_name;
        cmd.pose = request->pose;
        cmd.diffuse = request->diffuse;
        cmd.specular = request->specular;
        cmd.direction = request->direction;
        cmd.attenuation_constant = request->attenuation_constant;
        cmd.attenuation_linear = request->attenuation_linear;
        cmd.attenuation_quadratic = request->attenuation_quadratic;
        cmd.cast_shadows = request->cast_shadows;
        
        light_commands_.push(cmd);
        
        response->success = true;
        response->status_message = "SetLightProperties: command queued";
        
        RCLCPP_DEBUG(node_->get_logger(), "Queued light properties update for '%s'", 
                   request->light_name.c_str());
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error queueing light properties command: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in setLightProperties: %s", e.what());
    }
}

void DeepRacerGazeboSystemPlugin::spawnSDFModelCallback(
    const std::shared_ptr<deepracer_msgs::srv::SpawnModel::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SpawnModel::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "spawnSDFModel service called for model: %s", request->model_name.c_str());
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (world_name_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "World name not cached yet");
        response->success = false;
        response->status_message = "World name not cached yet";
        return;
    }

    try {
        std::string serviceTopic = "/world/" + world_name_ + "/create";
        
        gz::msgs::EntityFactory req;
        req.set_sdf(request->model_xml);
        
        gz::msgs::Pose *pose = req.mutable_pose();
        pose->mutable_position()->set_x(request->initial_pose.position.x);
        pose->mutable_position()->set_y(request->initial_pose.position.y);
        pose->mutable_position()->set_z(request->initial_pose.position.z);
        pose->mutable_orientation()->set_w(request->initial_pose.orientation.w);
        pose->mutable_orientation()->set_x(request->initial_pose.orientation.x);
        pose->mutable_orientation()->set_y(request->initial_pose.orientation.y);
        pose->mutable_orientation()->set_z(request->initial_pose.orientation.z);
        
        if (!request->model_name.empty()) {
            req.set_name(request->model_name);
        }
        
        gz::transport::Node transportNode;
        gz::msgs::Boolean result;
        bool success = false;
        bool executed = transportNode.Request(serviceTopic, req, 5000, result, success);
        
        if (executed && success && result.data()) {
            RCLCPP_INFO(node_->get_logger(), "Model spawned successfully: %s", request->model_name.c_str());
            response->success = true;
            response->status_message = "SpawnSDFModel: spawned successfully";
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to spawn model: %s", request->model_name.c_str());
            response->success = false;
            response->status_message = "Failed to spawn model via Gazebo service";
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while spawning model: %s", e.what());
        response->success = false;
        response->status_message = "Exception: " + std::string(e.what());
    }
}

void DeepRacerGazeboSystemPlugin::spawnURDFModelCallback(
    const std::shared_ptr<deepracer_msgs::srv::SpawnModel::Request> request,
    std::shared_ptr<deepracer_msgs::srv::SpawnModel::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "spawnURDFModel service called for model: %s",
                request->model_name.c_str());
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // TODO
    // reference: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L623
    RCLCPP_WARN(node_->get_logger(), "URDF spawning not yet implemented");
    response->success = false;
    response->status_message = "URDF spawning not yet implemented";
}

void DeepRacerGazeboSystemPlugin::deleteModelCallback(
    const std::shared_ptr<deepracer_msgs::srv::DeleteModel::Request> request,
    std::shared_ptr<deepracer_msgs::srv::DeleteModel::Response> response)
{
    RCLCPP_DEBUG(node_->get_logger(), "deleteModel service called for model: %s",
                request->model_name.c_str());
    
    try {        
        // Construct service topic using cached world name
        std::string serviceTopic = "/world/" + world_name_ + "/remove";
        
        // Create Entity request message
        gz::msgs::Entity req;
        req.set_name(request->model_name);
        req.set_type(gz::msgs::Entity::MODEL);
        
        // Call service
        gz::transport::Node transportNode;
        gz::msgs::Boolean result;
        bool success = false;
        bool executed = transportNode.Request(serviceTopic, req, 5000, result, success);
        
        if (executed && success) {
            RCLCPP_INFO(node_->get_logger(), "Model '%s' deletion requested successfully", 
                       request->model_name.c_str());
            response->success = true;
            response->status_message = "DeleteModel: deleted";
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to delete model '%s'", request->model_name.c_str());
            response->success = false;
            response->status_message = "Failed to delete model via Gazebo service";
        }
        
    } catch (const std::exception &e) {
        response->success = false;
        response->status_message = "Error deleting model: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "Error in deleteModel: %s", e.what());
    }
}

// Register the plugin
GZ_ADD_PLUGIN(DeepRacerGazeboSystemPlugin,
              gz::sim::System,
              DeepRacerGazeboSystemPlugin::ISystemConfigure,
              DeepRacerGazeboSystemPlugin::ISystemPreUpdate,
              DeepRacerGazeboSystemPlugin::ISystemPostUpdate)

// Add plugin alias for easier SDF reference
GZ_ADD_PLUGIN_ALIAS(DeepRacerGazeboSystemPlugin, 
                    "gz::sim::systems::DeepRacerGazeboSystemPlugin")