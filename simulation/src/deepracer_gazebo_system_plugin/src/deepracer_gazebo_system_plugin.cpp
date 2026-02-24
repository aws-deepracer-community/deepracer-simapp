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

#include <deepracer_gazebo_system_plugin/deepracer_gazebo_system_plugin.h>

namespace gazebo
{
    DeepRacerGazeboSystemPlugin::DeepRacerGazeboSystemPlugin() : SystemPlugin()
    {
        m_pluginLoaded = false;
        m_shouldStop = false;
        m_isWorldCreated = false;
    }

    DeepRacerGazeboSystemPlugin::~DeepRacerGazeboSystemPlugin()
    {
        ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: Destructor");

        m_sigintConnection.reset();
        m_loadDeepRacerGazeboPluginEvent.reset();
        // Do nothing if plugin is not loaded
        if (!m_pluginLoaded)
        {
            ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: Destructor skipped because never loaded");
            return;
        }
        m_gazeboCallbackQueueThread->join();
        m_asyncRosSpin->stop();
        m_gazeboNode->Fini();
        // Shutdown the ROS node
        m_rosNode->shutdown();
        ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: ROS Node Handle Shutdown");
        ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: Unloaded");
    }

    void DeepRacerGazeboSystemPlugin::Load(int argc, char **argv)
    {
        ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: Load");

        m_sigintConnection = gazebo::event::Events::ConnectSigInt(boost::bind(&DeepRacerGazeboSystemPlugin::processSigInt, this));

        // below needs the world to be created first
        m_loadDeepRacerGazeboPluginEvent = gazebo::event::Events::ConnectWorldCreated(boost::bind(&DeepRacerGazeboSystemPlugin::loadDeepRacerGazeboSystemPlugin,
                                                                                                  this, _1));
        m_pluginLoaded = true;
        ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: Finished loading.");
    }

    void DeepRacerGazeboSystemPlugin::processSigInt()
    {
        ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: SigInt event received");
        m_shouldStop = true;
    }

    void DeepRacerGazeboSystemPlugin::loadDeepRacerGazeboSystemPlugin(std::string world_name)
    {
        m_lock.lock();
        if (m_isWorldCreated)
        {
            m_lock.unlock();
            return;
        }

        // set flag to true and load this plugin
        m_isWorldCreated = true;
        m_lock.unlock();

        // check if the ros master is available - required
        // gazebo_ros_api plugin will init the ros,
        // so deepracer gazebo plugin can just wait for ros master to be alive.
        while (!ros::master::check())
        {
            ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: No ROS master - start roscore to continue...");
            // wait 0.5 second
            usleep(500 * 1000); // can't use ROS Time here b/c node handle is not yet initialized

            if (m_shouldStop)
            {
                ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: Canceled loading by SigInt event");
                return;
            }
        }
        m_rosNode.reset(new ros::NodeHandle("~"));

        m_asyncRosSpin.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
        m_asyncRosSpin->start();

        m_world = gazebo::physics::get_world(world_name);
        if (!m_world)
        {
            ROS_FATAL_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: cannot load plugin, physics::get_world() fails to return world");
            return;
        }

        // Create the node
        m_gazeboNode = transport::NodePtr(new transport::Node());
        m_gazeboNode->Init(world_name);

        // Init Gazebo Publisher
        m_visualPub = m_gazeboNode->Advertise<msgs::Visual>("~/visual");

        advertiseServices();
        m_gazeboCallbackQueueThread.reset(new boost::thread(&DeepRacerGazeboSystemPlugin::update, this));
        ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "DeepRacerGazeboSystemPlugin: Init Complete");
    }

    void DeepRacerGazeboSystemPlugin::advertiseServices()
    {
        // advertise services on the custom queue

        ros::AdvertiseServiceOptions getLightNamesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::GetLightNames>(
                "get_light_names",
                boost::bind(&DeepRacerGazeboSystemPlugin::getLightNamesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_getLightNamesSrv = m_rosNode->advertiseService(getLightNamesASO);

        ros::AdvertiseServiceOptions getVisualNamesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::GetVisualNames>(
                "get_visual_names",
                boost::bind(&DeepRacerGazeboSystemPlugin::getVisualNamesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_getVisualNamesSrv = m_rosNode->advertiseService(getVisualNamesASO);

        ros::AdvertiseServiceOptions getVisualASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::GetVisual>(
                "get_visual",
                boost::bind(&DeepRacerGazeboSystemPlugin::getVisualCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_getVisualSrv = m_rosNode->advertiseService(getVisualASO);

        ros::AdvertiseServiceOptions getVisualsASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::GetVisuals>(
                "get_visuals",
                boost::bind(&DeepRacerGazeboSystemPlugin::getVisualsCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_getVisualsSrv = m_rosNode->advertiseService(getVisualsASO);

        ros::AdvertiseServiceOptions setVisualColorASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualColor>(
                "set_visual_color",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualColorCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualColorSrv = m_rosNode->advertiseService(setVisualColorASO);

        ros::AdvertiseServiceOptions setVisualColorsASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualColors>(
                "set_visual_colors",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualColorsCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualColorsSrv = m_rosNode->advertiseService(setVisualColorsASO);

        ros::AdvertiseServiceOptions setVisualTransparencyASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualTransparency>(
                "set_visual_transparency",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualTransparencyCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualTransparencySrv = m_rosNode->advertiseService(setVisualTransparencyASO);

        ros::AdvertiseServiceOptions setVisualTransparenciesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualTransparencies>(
                "set_visual_transparencies",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualTransparenciesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualTransparenciesSrv = m_rosNode->advertiseService(setVisualTransparenciesASO);

        ros::AdvertiseServiceOptions setVisualVisibleASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualVisible>(
                "set_visual_visible",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualVisibleCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualVisibleSrv = m_rosNode->advertiseService(setVisualVisibleASO);

        ros::AdvertiseServiceOptions setVisualVisiblesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualVisibles>(
                "set_visual_visibles",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualVisiblesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualVisiblesSrv = m_rosNode->advertiseService(setVisualVisiblesASO);

        ros::AdvertiseServiceOptions setVisualPoseASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualPose>(
                "set_visual_pose",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualPoseCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualPoseSrv = m_rosNode->advertiseService(setVisualPoseASO);

        ros::AdvertiseServiceOptions setVisualPosesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualPoses>(
                "set_visual_poses",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualPosesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualPosesSrv = m_rosNode->advertiseService(setVisualPosesASO);

        ros::AdvertiseServiceOptions setVisualMeshASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualMesh>(
                "set_visual_mesh",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualMeshCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualMeshSrv = m_rosNode->advertiseService(setVisualMeshASO);

        ros::AdvertiseServiceOptions setVisualMeshesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetVisualMeshes>(
                "set_visual_meshes",
                boost::bind(&DeepRacerGazeboSystemPlugin::setVisualMeshesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setVisualMeshesSrv = m_rosNode->advertiseService(setVisualMeshesASO);

        ros::AdvertiseServiceOptions getModelStatesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::GetModelStates>(
                "get_model_states",
                boost::bind(&DeepRacerGazeboSystemPlugin::getModelStatesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_getModelStatesSrv = m_rosNode->advertiseService(getModelStatesASO);

        ros::AdvertiseServiceOptions setModelStatesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetModelStates>(
                "set_model_states",
                boost::bind(&DeepRacerGazeboSystemPlugin::setModelStatesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setModelStatesSrv = m_rosNode->advertiseService(setModelStatesASO);

        ros::AdvertiseServiceOptions getLinkStatesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::GetLinkStates>(
                "get_link_states",
                boost::bind(&DeepRacerGazeboSystemPlugin::getLinkStatesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_getLinkStatesSrv = m_rosNode->advertiseService(getLinkStatesASO);

        ros::AdvertiseServiceOptions setLinkStatesASO =
            ros::AdvertiseServiceOptions::create<deepracer_msgs::SetLinkStates>(
                "set_link_states",
                boost::bind(&DeepRacerGazeboSystemPlugin::setLinkStatesCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_setLinkStatesSrv = m_rosNode->advertiseService(setLinkStatesASO);

        ros::AdvertiseServiceOptions pausePhysicsASO =
            ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                "pause_physics_dr",
                boost::bind(&DeepRacerGazeboSystemPlugin::pausePhysicsCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_pausePhysicsSrv = m_rosNode->advertiseService(pausePhysicsASO);

        ros::AdvertiseServiceOptions unpausePhysicsASO =
            ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                "unpause_physics_dr",
                boost::bind(&DeepRacerGazeboSystemPlugin::unpausePhysicsCallback, this, _1, _2),
                ros::VoidPtr(), &m_rosCallbackQueue);
        m_unpausePhysicsSrv = m_rosNode->advertiseService(unpausePhysicsASO);
    }

    void DeepRacerGazeboSystemPlugin::update()
    {
        static const double timeout = 0.001;
        while (m_rosNode->ok())
        {
            m_rosCallbackQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    void DeepRacerGazeboSystemPlugin::set(msgs::Color *_msg, const std_msgs::ColorRGBA &c)
    {
        _msg->set_r(c.r);
        _msg->set_g(c.g);
        _msg->set_b(c.b);
        _msg->set_a(c.a);
    }

    void DeepRacerGazeboSystemPlugin::set(std_msgs::ColorRGBA &std_c, const msgs::Color &c)
    {
        std_c.r = c.r();
        std_c.g = c.g();
        std_c.b = c.b();
        std_c.a = c.a();
    }

    std_msgs::ColorRGBA DeepRacerGazeboSystemPlugin::defaultColorRGBA()
    {
        std_msgs::ColorRGBA std_c;
        std_c.r = 0.0f;
        std_c.g = 0.0f;
        std_c.b = 0.0f;
        std_c.a = 1.0f;
        return std_c;
    }

    void DeepRacerGazeboSystemPlugin::set(msgs::Vector3d *_msg, const geometry_msgs::Point &p)
    {
        _msg->set_x(p.x);
        _msg->set_y(p.y);
        _msg->set_z(p.z);
    }

    void DeepRacerGazeboSystemPlugin::set(geometry_msgs::Point &geo_p, const msgs::Vector3d &v)
    {
        geo_p.x = v.x();
        geo_p.y = v.y();
        geo_p.z = v.z();
    }

    void DeepRacerGazeboSystemPlugin::set(msgs::Quaternion *_msg, const geometry_msgs::Quaternion &q)
    {
        _msg->set_x(q.x);
        _msg->set_y(q.y);
        _msg->set_z(q.z);
        _msg->set_w(q.w);
    }

    void DeepRacerGazeboSystemPlugin::set(geometry_msgs::Quaternion &geo_q, const msgs::Quaternion &q)
    {
        geo_q.x = q.x();
        geo_q.y = q.y();
        geo_q.z = q.z();
        geo_q.w = q.w();
    }

    void DeepRacerGazeboSystemPlugin::set(msgs::Pose *_msg, const geometry_msgs::Pose &p)
    {
        set(_msg->mutable_position(), p.position);
        set(_msg->mutable_orientation(), p.orientation);
    }

    void DeepRacerGazeboSystemPlugin::set(geometry_msgs::Pose &geo_p, const msgs::Pose &p)
    {
        set(geo_p.position, p.position());
        set(geo_p.orientation, p.orientation());
    }

    void DeepRacerGazeboSystemPlugin::set(msgs::Vector3d *_msg, const geometry_msgs::Vector3 &p)
    {
        _msg->set_x(p.x);
        _msg->set_y(p.y);
        _msg->set_z(p.z);
    }

    void DeepRacerGazeboSystemPlugin::set(geometry_msgs::Vector3 &geo_v, const msgs::Vector3d &v)
    {
        geo_v.x = v.x();
        geo_v.y = v.y();
        geo_v.z = v.z();
    }

    void DeepRacerGazeboSystemPlugin::publishVisualMsg(physics::LinkPtr &link, const msgs::Visual &visualMsg, bool block)
    {
        msgs::Visual newVisualMsg = visualMsg;
        newVisualMsg.set_name(link->GetScopedName());
        // publish visual message
        m_visualPub->Publish(newVisualMsg, block);
        if (block)
        {
            while (m_visualPub->GetOutgoingCount() > 0)
            {
                m_visualPub->SendMessage();
            }
        }

        updateVisualMsg(link, visualMsg);
    }

    void DeepRacerGazeboSystemPlugin::updateVisualMsg(physics::LinkPtr &link, const msgs::Visual &visualMsg)
    {
        physics::Link::Visuals_M &visuals = link->visuals;

        physics::Link::Visuals_M::iterator iter;
        for (iter = visuals.begin(); iter != visuals.end(); ++iter)
        {
            if (iter->second.name() == visualMsg.name())
            {
                iter->second = visualMsg;
                break;
            }
        }
    }

    bool DeepRacerGazeboSystemPlugin::getLightNamesCallback(deepracer_msgs::GetLightNames::Request &req,
                                                            deepracer_msgs::GetLightNames::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        std::vector<std::string> lightNames;

        physics::Light_V lights = m_world->Lights();
        for (physics::Light_V::const_iterator lightIter = lights.begin();
             lightIter != lights.end();
             ++lightIter)
        {
            lightNames.push_back(boost::dynamic_pointer_cast<physics::Base>(*lightIter)->GetName());
        }
        res.light_names = lightNames;
        res.success = true;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::getVisualNamesCallback(deepracer_msgs::GetVisualNames::Request &req,
                                                             deepracer_msgs::GetVisualNames::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        std::vector<std::string> visualNames;
        std::vector<std::string> linkNames;
        for (std::vector<std::string>::const_iterator linkNameIter = req.link_names.begin();
             linkNameIter != req.link_names.end();
             ++linkNameIter)
        {
            physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(*linkNameIter));
            if (!link)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "GetVisual: link [%s] does not exist", (*linkNameIter).c_str());
                res.status_message = "GetVisualNames: link does not exist";
                res.success = false;
                return true;
            }
            for (physics::Link::Visuals_M::const_iterator visualIter = link->visuals.begin();
                 visualIter != link->visuals.end();
                 ++visualIter)
            {
                visualNames.push_back(visualIter->second.name());
                linkNames.push_back(*linkNameIter);
            }
        }
        res.visual_names = visualNames;
        res.link_names = linkNames;
        res.success = true;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::getVisualCallback(deepracer_msgs::GetVisual::Request &req,
                                                        deepracer_msgs::GetVisual::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        // gets the visual message
        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(req.link_name));

        if (!link)
        {
            ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "GetVisual: link [%s] does not exist", req.link_name.c_str());
            res.status_message = "GetVisual: link does not exist";
            res.success = false;
            return true;
        }
        msgs::Visual visualMsg = link->GetVisualMessage(req.visual_name);

        res.ambient = defaultColorRGBA();
        res.diffuse = defaultColorRGBA();
        res.specular = defaultColorRGBA();
        res.emissive = defaultColorRGBA();

        // Return default material if there is no material for the visual
        if (visualMsg.has_material())
        {
            const msgs::Material &materialMsg = visualMsg.material();

            if (materialMsg.has_ambient())
            {
                set(res.ambient, materialMsg.ambient());
            }

            if (materialMsg.has_diffuse())
            {
                set(res.diffuse, materialMsg.diffuse());
            }

            if (materialMsg.has_specular())
            {
                set(res.specular, materialMsg.specular());
            }

            if (materialMsg.has_emissive())
            {
                set(res.emissive, materialMsg.emissive());
            }
        }
        res.geometry_type = 1;
        res.mesh_geom_scale.x = 1.0;
        res.mesh_geom_scale.y = 1.0;
        res.mesh_geom_scale.z = 1.0;
        if (visualMsg.has_geometry())
        {
            const msgs::Geometry &geometryMsg = visualMsg.geometry();
            if (geometryMsg.has_mesh())
            {
                const msgs::MeshGeom &meshMsg = geometryMsg.mesh();
                res.mesh_geom_filename = meshMsg.filename();

                if (meshMsg.has_scale())
                {
                    set(res.mesh_geom_scale, meshMsg.scale());
                }
            }
            if (geometryMsg.has_type())
            {
                res.geometry_type = geometryMsg.has_type();
            }
        }

        if (visualMsg.has_pose())
        {
            set(res.pose, visualMsg.pose());
        }

        res.transparency = visualMsg.has_transparency() ? visualMsg.transparency() : 0.0;
        res.visible = visualMsg.has_visible() ? visualMsg.visible() : true;

        res.success = true;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::getVisualsCallback(deepracer_msgs::GetVisuals::Request &req,
                                                         deepracer_msgs::GetVisuals::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        if (req.link_names.size() != req.visual_names.size())
        {
            res.status_message = "GetVisuals: link_names, visual_names must be same size!";
            res.success = false;
            return true;
        }

        std::vector<std::string> visualNames;
        std::vector<std::string> linkNames;
        std::vector<std::string> filenames;

        std::vector<std_msgs::ColorRGBA> ambients;
        std::vector<std_msgs::ColorRGBA> diffuses;
        std::vector<std_msgs::ColorRGBA> speculars;
        std::vector<std_msgs::ColorRGBA> emissives;

        std::vector<double> transparencies;
        std::vector<int8_t> visibles;
        std::vector<unsigned short> geometry_types;
        std::vector<geometry_msgs::Pose> poses;
        std::vector<geometry_msgs::Vector3> mesh_geom_scales;
        std::vector<int8_t> statusList;
        std::vector<std::string> messages;

        std::vector<std::string>::const_iterator linkNameIter = req.link_names.begin();
        std::vector<std::string>::const_iterator visualNameIter = req.visual_names.begin();
        for (; linkNameIter != req.link_names.end();
             ++linkNameIter, ++visualNameIter)
        {
            // gets the visual message
            physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(*linkNameIter));
            std_msgs::ColorRGBA ambient, diffuse, specular, emissive;
            ambient = defaultColorRGBA();
            diffuse = defaultColorRGBA();
            specular = defaultColorRGBA();
            emissive = defaultColorRGBA();

            std::string filename = "";
            unsigned short geometry_type = 1;
            geometry_msgs::Vector3 mesh_geom_scale;
            mesh_geom_scale.x = 1.0;
            mesh_geom_scale.y = 1.0;
            mesh_geom_scale.z = 1.0;

            geometry_msgs::Pose pose;

            double transparency = 0.0;
            bool visible = false;

            bool status = true;
            std::string message = "";

            if (!link)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "GetVisuals: link [%s] does not exist", (*linkNameIter).c_str());
                message = "GetVisuals: link does not exist";
                status = false;
            }
            else
            {
                msgs::Visual visualMsg = link->GetVisualMessage(*visualNameIter);

                // Use default material if there is no material for the visual
                // Otherwise, retrieve the visual material.
                if (visualMsg.has_material())
                {

                    const msgs::Material &materialMsg = visualMsg.material();

                    if (materialMsg.has_ambient())
                    {
                        set(ambient, materialMsg.ambient());
                    }

                    if (materialMsg.has_diffuse())
                    {
                        set(diffuse, materialMsg.diffuse());
                    }

                    if (materialMsg.has_specular())
                    {
                        set(specular, materialMsg.specular());
                    }

                    if (materialMsg.has_emissive())
                    {
                        set(emissive, materialMsg.emissive());
                    }
                }

                if (visualMsg.has_geometry())
                {
                    const msgs::Geometry &geometryMsg = visualMsg.geometry();
                    if (geometryMsg.has_mesh())
                    {
                        const msgs::MeshGeom &meshMsg = geometryMsg.mesh();
                        filename = meshMsg.filename();

                        if (meshMsg.has_scale())
                        {
                            set(mesh_geom_scale, meshMsg.scale());
                        }
                    }
                    if (geometryMsg.has_type())
                    {
                        geometry_type = (unsigned short)geometryMsg.type();
                    }
                }

                if (visualMsg.has_pose())
                {
                    set(pose, visualMsg.pose());
                }

                transparency = visualMsg.has_transparency() ? visualMsg.transparency() : 0.0;
                visible = visualMsg.has_visible() ? visualMsg.visible() : true;
            }

            visualNames.push_back(*visualNameIter);
            linkNames.push_back(*linkNameIter);
            ambients.push_back(ambient);
            diffuses.push_back(diffuse);
            speculars.push_back(specular);
            emissives.push_back(emissive);
            transparencies.push_back(transparency);
            visibles.push_back(visible);
            geometry_types.push_back(geometry_type);
            poses.push_back(pose);
            filenames.push_back(filename);
            statusList.push_back(status);
            messages.push_back(message);
            mesh_geom_scales.push_back(mesh_geom_scale);
        }
        res.visual_names = visualNames;
        res.link_names = linkNames;
        res.ambients = ambients;
        res.diffuses = diffuses;
        res.speculars = speculars;
        res.emissives = emissives;
        res.transparencies = transparencies;
        res.visibles = visibles;
        res.geometry_types = geometry_types;
        res.mesh_geom_filenames = filenames;
        res.poses = poses;
        res.mesh_geom_scales = mesh_geom_scales;
        res.success = true;
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualColorCallback(deepracer_msgs::SetVisualColor::Request &req,
                                                             deepracer_msgs::SetVisualColor::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(req.link_name));

        if (!link)
        {
            ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualColor: link [%s] does not exist", req.link_name.c_str());
            res.status_message = "SetVisualColor: link does not exist";
            res.success = false;
            return true;
        }
        msgs::Visual visualMsg = link->GetVisualMessage(req.visual_name);

        // Set colors to material object
        msgs::Material *materialMsg = visualMsg.mutable_material();

        set(materialMsg->mutable_ambient(), req.ambient);
        set(materialMsg->mutable_diffuse(), req.diffuse);
        set(materialMsg->mutable_specular(), req.specular);
        set(materialMsg->mutable_emissive(), req.emissive);

        // publish visual message
        publishVisualMsg(link, visualMsg, req.block);

        res.success = true;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualColorsCallback(deepracer_msgs::SetVisualColors::Request &req,
                                                              deepracer_msgs::SetVisualColors::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        if (req.link_names.size() != req.visual_names.size() ||
            req.link_names.size() != req.ambients.size() ||
            req.link_names.size() != req.diffuses.size() ||
            req.link_names.size() != req.speculars.size() ||
            req.link_names.size() != req.emissives.size())
        {
            res.status_message = "SetVisualColors: link_names, visual_names, ambients, diffuses, speculars, emissives must be same size!";
            res.success = false;
            return true;
        }
        std::vector<std::string>::const_iterator linkNameIter = req.link_names.begin();
        std::vector<std::string>::const_iterator visualNameIter = req.visual_names.begin();
        std::vector<std_msgs::ColorRGBA>::const_iterator ambientIter = req.ambients.begin();
        std::vector<std_msgs::ColorRGBA>::const_iterator diffuseIter = req.diffuses.begin();
        std::vector<std_msgs::ColorRGBA>::const_iterator specularIter = req.speculars.begin();
        std::vector<std_msgs::ColorRGBA>::const_iterator emissiveIter = req.emissives.begin();

        std::vector<int8_t> statusList;
        std::vector<std::string> messages;
        for (; linkNameIter != req.link_names.end();
             ++linkNameIter, ++visualNameIter, ++ambientIter, ++diffuseIter, ++specularIter, ++emissiveIter)
        {
            physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(*linkNameIter));

            if (!link)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualColors: link [%s] does not exist", (*linkNameIter).c_str());
                statusList.push_back(false);
                messages.push_back("SetVisualColors: link does not exist");
                continue;
            }
            msgs::Visual visualMsg = link->GetVisualMessage(*visualNameIter);

            // Set colors to material object
            msgs::Material *materialMsg = visualMsg.mutable_material();
            set(materialMsg->mutable_ambient(), *ambientIter);
            set(materialMsg->mutable_diffuse(), *diffuseIter);
            set(materialMsg->mutable_specular(), *specularIter);
            set(materialMsg->mutable_emissive(), *emissiveIter);

            // publish visual message
            publishVisualMsg(link, visualMsg, req.block);
            statusList.push_back(true);
            messages.push_back("");
        }
        res.success = true;
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualTransparencyCallback(deepracer_msgs::SetVisualTransparency::Request &req,
                                                                    deepracer_msgs::SetVisualTransparency::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(req.link_name));

        if (!link)
        {
            ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualTransparency: link [%s] does not exist", req.link_name.c_str());
            res.status_message = "SetVisualTransparency: link does not exist";
            res.success = false;
            return true;
        }
        msgs::Visual visualMsg = link->GetVisualMessage(req.visual_name);

        visualMsg.set_transparency(req.transparency);

        // publish visual message
        publishVisualMsg(link, visualMsg, req.block);

        res.success = true;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualTransparenciesCallback(deepracer_msgs::SetVisualTransparencies::Request &req,
                                                                      deepracer_msgs::SetVisualTransparencies::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        if (req.link_names.size() != req.visual_names.size() ||
            req.link_names.size() != req.transparencies.size())
        {
            res.status_message = "SetVisualTransparencies: link_names, visual_names, transparencies must be same size!";
            res.success = false;
            return true;
        }
        std::vector<std::string>::const_iterator linkNameIter = req.link_names.begin();
        std::vector<std::string>::const_iterator visualNameIter = req.visual_names.begin();
        std::vector<double>::const_iterator transparencyIter = req.transparencies.begin();

        std::vector<int8_t> statusList;
        std::vector<std::string> messages;
        for (; linkNameIter != req.link_names.end();
             ++linkNameIter, ++visualNameIter, ++transparencyIter)
        {
            physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(*linkNameIter));

            if (!link)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualTransparencies: link [%s] does not exist", (*linkNameIter).c_str());
                statusList.push_back(false);
                messages.push_back("SetVisualTransparencies: link does not exist");
                continue;
            }
            msgs::Visual visualMsg = link->GetVisualMessage(*visualNameIter);
            visualMsg.set_transparency(*transparencyIter);

            // publish visual message
            publishVisualMsg(link, visualMsg, req.block);
            statusList.push_back(true);
            messages.push_back("");
        }
        res.success = true;
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualVisibleCallback(deepracer_msgs::SetVisualVisible::Request &req,
                                                               deepracer_msgs::SetVisualVisible::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(req.link_name));

        if (!link)
        {
            ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualVisible: link [%s] does not exist", req.link_name.c_str());
            res.status_message = "SetVisualVisible: link does not exist";
            res.success = false;
            return true;
        }
        msgs::Visual visualMsg = link->GetVisualMessage(req.visual_name);

        visualMsg.set_visible((bool)req.visible);

        // publish visual message
        publishVisualMsg(link, visualMsg, req.block);

        res.success = true;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualVisiblesCallback(deepracer_msgs::SetVisualVisibles::Request &req,
                                                                deepracer_msgs::SetVisualVisibles::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        if (req.link_names.size() != req.visual_names.size() ||
            req.link_names.size() != req.visibles.size())
        {
            res.status_message = "SetVisualVisibles: link_names, visual_names, visibles must be same size!";
            res.success = false;
            return true;
        }
        std::vector<std::string>::const_iterator linkNameIter = req.link_names.begin();
        std::vector<std::string>::const_iterator visualNameIter = req.visual_names.begin();
        std::vector<signed char>::const_iterator visibleIter = req.visibles.begin();

        std::vector<int8_t> statusList;
        std::vector<std::string> messages;
        for (; linkNameIter != req.link_names.end();
             ++linkNameIter, ++visualNameIter, ++visibleIter)
        {
            physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(*linkNameIter));

            if (!link)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualVisibles: link [%s] does not exist", (*linkNameIter).c_str());
                statusList.push_back(false);
                messages.push_back("SetVisualVisibles: link does not exist");
                continue;
            }
            msgs::Visual visualMsg = link->GetVisualMessage(*visualNameIter);
            visualMsg.set_visible((bool)*visibleIter);

            // publish visual message
            publishVisualMsg(link, visualMsg, req.block);
            statusList.push_back(true);
            messages.push_back("");
        }
        res.success = true;
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualPoseCallback(deepracer_msgs::SetVisualPose::Request &req,
                                                            deepracer_msgs::SetVisualPose::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(req.link_name));

        if (!link)
        {
            ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualPose: link [%s] does not exist", req.link_name.c_str());
            res.status_message = "SetVisualPose: link does not exist";
            res.success = false;
            return true;
        }
        msgs::Visual visualMsg = link->GetVisualMessage(req.visual_name);

        set(visualMsg.mutable_pose(), req.pose);

        // publish visual message
        publishVisualMsg(link, visualMsg, req.block);

        res.success = true;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualPosesCallback(deepracer_msgs::SetVisualPoses::Request &req,
                                                             deepracer_msgs::SetVisualPoses::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        if (req.link_names.size() != req.visual_names.size() ||
            req.link_names.size() != req.poses.size())
        {
            res.status_message = "SetVisualPoses: link_names, visual_names, poses must be same size!";
            res.success = false;
            return true;
        }
        std::vector<std::string>::const_iterator linkNameIter = req.link_names.begin();
        std::vector<std::string>::const_iterator visualNameIter = req.visual_names.begin();
        std::vector<geometry_msgs::Pose>::const_iterator poseIter = req.poses.begin();

        std::vector<int8_t> statusList;
        std::vector<std::string> messages;
        for (; linkNameIter != req.link_names.end();
             ++linkNameIter, ++visualNameIter, ++poseIter)
        {
            physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(*linkNameIter));

            if (!link)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualPoses: link [%s] does not exist", (*linkNameIter).c_str());
                statusList.push_back(false);
                messages.push_back("SetVisualPoses: link does not exist");
                continue;
            }
            msgs::Visual visualMsg = link->GetVisualMessage(*visualNameIter);

            set(visualMsg.mutable_pose(), *poseIter);

            // publish visual message
            publishVisualMsg(link, visualMsg, req.block);
            statusList.push_back(true);
            messages.push_back("");
        }
        res.success = true;
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualMeshCallback(deepracer_msgs::SetVisualMesh::Request &req,
                                                            deepracer_msgs::SetVisualMesh::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);

        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(req.link_name));

        if (!link)
        {
            ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualMesh: link [%s] does not exist", req.link_name.c_str());
            res.status_message = "SetVisualMesh: link does not exist";
            res.success = false;
            return true;
        }
        msgs::Visual visualMsg = link->GetVisualMessage(req.visual_name);

        msgs::Geometry *geometryMsg = visualMsg.mutable_geometry();
        geometryMsg->set_type(msgs::Geometry::MESH);

        msgs::MeshGeom *meshGeomMsg = geometryMsg->mutable_mesh();
        std::string geometryName = common::find_file(req.filename);
        meshGeomMsg->set_filename(geometryName);
        set(meshGeomMsg->mutable_scale(), req.scale);

        // publish visual message
        publishVisualMsg(link, visualMsg, req.block);

        res.success = true;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setVisualMeshesCallback(deepracer_msgs::SetVisualMeshes::Request &req,
                                                              deepracer_msgs::SetVisualMeshes::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        if (req.link_names.size() != req.visual_names.size() ||
            req.link_names.size() != req.filenames.size() ||
            req.link_names.size() != req.scales.size())
        {
            res.status_message = "SetVisualMeshes: link_names, visual_names, filenames, and scales must be same size!";
            res.success = false;
            return true;
        }
        std::vector<std::string>::const_iterator linkNameIter = req.link_names.begin();
        std::vector<std::string>::const_iterator visualNameIter = req.visual_names.begin();
        std::vector<std::string>::const_iterator filenameIter = req.filenames.begin();
        std::vector<geometry_msgs::Vector3>::const_iterator scaleIter = req.scales.begin();

        std::vector<int8_t> statusList;
        std::vector<std::string> messages;

        for (; linkNameIter != req.link_names.end();
             ++linkNameIter, ++visualNameIter, ++filenameIter, ++scaleIter)
        {
            physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(m_world->EntityByName(*linkNameIter));

            if (!link)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "SetVisualMeshes: link [%s] does not exist", (*linkNameIter).c_str());
                statusList.push_back(false);
                messages.push_back("SetVisualMeshes: link does not exist");
                continue;
            }
            msgs::Visual visualMsg = link->GetVisualMessage(*visualNameIter);

            msgs::Geometry *geometryMsg = visualMsg.mutable_geometry();
            geometryMsg->set_type(msgs::Geometry::MESH);

            msgs::MeshGeom *meshGeomMsg = geometryMsg->mutable_mesh();
            std::string geometryName = common::find_file(*filenameIter);
            meshGeomMsg->set_filename(geometryName);
            set(meshGeomMsg->mutable_scale(), *scaleIter);

            // publish visual message
            publishVisualMsg(link, visualMsg, req.block);
            statusList.push_back(true);
            messages.push_back("");
        }
        res.success = true;
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::getModelStatesCallback(deepracer_msgs::GetModelStates::Request &req,
                                                             deepracer_msgs::GetModelStates::Response &res)
    {
        if (req.model_names.size() != req.relative_entity_names.size())
        {
            res.status_message = "GetModelStates: model_names, relative_entity_names must be same size!";
            res.success = false;
            return true;
        }

        std::vector<std::string>::const_iterator modelNameIter = req.model_names.begin();
        std::vector<std::string>::const_iterator relativeEntityNameIter = req.relative_entity_names.begin();

        std::vector<gazebo_msgs::ModelState> modelStates;
        std::vector<int8_t> statusList;
        std::vector<std::string> messages;

        for (; modelNameIter != req.model_names.end();
             ++modelNameIter, ++relativeEntityNameIter)
        {
            gazebo_msgs::ModelState modelState;
            modelState.model_name = *modelNameIter;
            modelState.reference_frame = *relativeEntityNameIter;

            gazebo::physics::ModelPtr model = m_world->ModelByName(*modelNameIter);
            gazebo::physics::EntityPtr frame = m_world->EntityByName(*relativeEntityNameIter);

            if (!model)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "GetModelStates: model [%s] does not exist", (*modelNameIter).c_str());
                modelStates.push_back(modelState);
                statusList.push_back(false);
                messages.push_back("GetModelStates: model does not exist");
                continue;
            }
            else
            {
                ignition::math::Pose3d model_pose = model->WorldPose();
                ignition::math::Vector3d model_linear_vel = model->WorldLinearVel();
                ignition::math::Vector3d model_angular_vel = model->WorldAngularVel();

                ignition::math::Vector3d model_pos = model_pose.Pos();
                ignition::math::Quaterniond model_rot = model_pose.Rot();

                if (frame)
                {
                    // convert to relative pose, rates
                    ignition::math::Pose3d frame_pose = frame->WorldPose();
                    ignition::math::Vector3d frame_vpos = frame->WorldLinearVel();  // get velocity in gazebo frame
                    ignition::math::Vector3d frame_veul = frame->WorldAngularVel(); // get velocity in gazebo frame

                    ignition::math::Pose3d model_rel_pose = model_pose - frame_pose;
                    model_pos = model_rel_pose.Pos();
                    model_rot = model_rel_pose.Rot();

                    model_linear_vel = frame_pose.Rot().RotateVectorReverse(model_linear_vel - frame_vpos);
                    model_angular_vel = frame_pose.Rot().RotateVectorReverse(model_angular_vel - frame_veul);
                }
                /// @todo: FIXME map is really wrong, need to use tf here somehow
                else if ((*relativeEntityNameIter) == "" || (*relativeEntityNameIter) == "world" || (*relativeEntityNameIter) == "map" || (*relativeEntityNameIter) == "/map")
                {
                    ROS_DEBUG_NAMED("deepracer_gazebo_system_plugin", "GetModelStates: relative_entity_name is empty/world/map, using inertial frame");
                }
                else
                {
                    modelStates.push_back(modelState);
                    statusList.push_back(false);
                    messages.push_back("GetModelStates: reference relative_entity_name not found, did you forget to scope the body by model name?");
                    continue;
                }

                // fill in response

                modelState.pose.position.x = model_pos.X();
                modelState.pose.position.y = model_pos.Y();
                modelState.pose.position.z = model_pos.Z();
                modelState.pose.orientation.w = model_rot.W();
                modelState.pose.orientation.x = model_rot.X();
                modelState.pose.orientation.y = model_rot.Y();
                modelState.pose.orientation.z = model_rot.Z();

                modelState.twist.linear.x = model_linear_vel.X();
                modelState.twist.linear.y = model_linear_vel.Y();
                modelState.twist.linear.z = model_linear_vel.Z();
                modelState.twist.angular.x = model_angular_vel.X();
                modelState.twist.angular.y = model_angular_vel.Y();
                modelState.twist.angular.z = model_angular_vel.Z();
                modelStates.push_back(modelState);
                statusList.push_back(true);
                messages.push_back("");
            }
        }
        res.model_states = modelStates;
        res.success = true;
        res.status_message = "GetModelStates: got properties";
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setModelStatesCallback(deepracer_msgs::SetModelStates::Request &req,
                                                             deepracer_msgs::SetModelStates::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        std::vector<gazebo_msgs::ModelState>::const_iterator modelStateIter = req.model_states.begin();

        std::vector<int8_t> statusList;
        std::vector<std::string> messages;

        for (; modelStateIter != req.model_states.end();
             ++modelStateIter)
        {
            gazebo_msgs::ModelState model_state = *modelStateIter;
            ignition::math::Vector3d target_pos(model_state.pose.position.x,
                                                model_state.pose.position.y,
                                                model_state.pose.position.z);
            ignition::math::Quaterniond target_rot(model_state.pose.orientation.w,
                                                   model_state.pose.orientation.x,
                                                   model_state.pose.orientation.y,
                                                   model_state.pose.orientation.z);
            target_rot.Normalize(); // eliminates invalid rotation (0, 0, 0, 0)
            ignition::math::Pose3d target_pose(target_pos, target_rot);
            ignition::math::Vector3d target_pos_dot(model_state.twist.linear.x, model_state.twist.linear.y, model_state.twist.linear.z);
            ignition::math::Vector3d target_rot_dot(model_state.twist.angular.x, model_state.twist.angular.y, model_state.twist.angular.z);

            gazebo::physics::ModelPtr model = m_world->ModelByName(model_state.model_name);

            if (!model)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "Updating ModelState: model [%s] does not exist",
                                model_state.model_name.c_str());
                statusList.push_back(false);
                messages.push_back("SetModelStates: model does not exist");
                continue;
            }
            else
            {
                gazebo::physics::EntityPtr relative_entity = m_world->EntityByName(model_state.reference_frame);

                if (relative_entity)
                {
                    ignition::math::Pose3d frame_pose = relative_entity->WorldPose(); // - myBody->GetCoMPose();

                    target_pose = target_pose + frame_pose;

                    // Velocities should be commanded in the requested reference
                    // frame, so we need to translate them to the world frame
                    target_pos_dot = frame_pose.Rot().RotateVector(target_pos_dot);
                    target_rot_dot = frame_pose.Rot().RotateVector(target_rot_dot);
                }
                /// @todo: FIXME map is really wrong, need to use tf here somehow
                else if (model_state.reference_frame == "" || model_state.reference_frame == "world" || model_state.reference_frame == "map" || model_state.reference_frame == "/map")
                {
                    ROS_DEBUG_NAMED("deepracer_gazebo_system_plugin", "Updating ModelState: reference frame is empty/world/map, usig inertial frame");
                }
                else
                {
                    ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "Updating ModelState: for model[%s], specified reference frame entity [%s] does not exist",
                                    model_state.model_name.c_str(), model_state.reference_frame.c_str());
                    statusList.push_back(false);
                    messages.push_back("SetModelStates: specified reference frame entity does not exist");
                    continue;
                }

                bool is_paused = m_world->IsPaused();
                if (!is_paused)
                    m_world->SetPaused(true);
                model->SetWorldPose(target_pose);
                m_world->SetPaused(is_paused);

                // set model velocity
                model->SetLinearVel(target_pos_dot);
                model->SetAngularVel(target_rot_dot);
                statusList.push_back(true);
                messages.push_back("");
            }
        }
        res.success = true;
        res.status_message = "SetModelStates: set model state done";
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::getLinkStatesCallback(deepracer_msgs::GetLinkStates::Request &req,
                                                            deepracer_msgs::GetLinkStates::Response &res)
    {

        if (req.link_names.size() != req.reference_frames.size())
        {
            res.status_message = "GetLinkStates: link_names, reference_frames must be same size!";
            res.success = false;
            return true;
        }
        std::vector<std::string>::const_iterator linkNameIter = req.link_names.begin();
        std::vector<std::string>::const_iterator referenceFrameIter = req.reference_frames.begin();

        std::vector<gazebo_msgs::LinkState> linkStates;
        std::vector<int8_t> statusList;
        std::vector<std::string> messages;

        for (; linkNameIter != req.link_names.end();
             ++linkNameIter, ++referenceFrameIter)
        {
            gazebo_msgs::LinkState linkState;

            linkState.link_name = *linkNameIter;
            linkState.reference_frame = *referenceFrameIter;

            gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(m_world->EntityByName(*linkNameIter));
            gazebo::physics::EntityPtr frame = m_world->EntityByName(*referenceFrameIter);

            if (!body)
            {
                linkStates.push_back(linkState);
                statusList.push_back(false);
                messages.push_back("GetLinkStates: link not found, did you forget to scope the link by model name?");
                continue;
            }

            // get body pose
            // Get inertial rates
            ignition::math::Pose3d body_pose = body->WorldPose();
            ignition::math::Vector3d body_vpos = body->WorldLinearVel();  // get velocity in gazebo frame
            ignition::math::Vector3d body_veul = body->WorldAngularVel(); // get velocity in gazebo frame

            if (frame)
            {
                // convert to relative pose, rates
                ignition::math::Pose3d frame_pose = frame->WorldPose();
                ignition::math::Vector3d frame_vpos = frame->WorldLinearVel();  // get velocity in gazebo frame
                ignition::math::Vector3d frame_veul = frame->WorldAngularVel(); // get velocity in gazebo frame

                body_pose = body_pose - frame_pose;

                body_vpos = frame_pose.Rot().RotateVectorReverse(body_vpos - frame_vpos);
                body_veul = frame_pose.Rot().RotateVectorReverse(body_veul - frame_veul);
            }
            /// @todo: FIXME map is really wrong, need to use tf here somehow
            else if ((*referenceFrameIter) == "" || (*referenceFrameIter) == "world" || (*referenceFrameIter) == "map" || (*referenceFrameIter) == "/map")
            {
                ROS_DEBUG_NAMED("deepracer_gazebo_system_plugin", "GetLinkStates: reference_frame is empty/world/map, using inertial frame");
            }
            else
            {
                linkStates.push_back(linkState);
                statusList.push_back(false);
                messages.push_back("GetLinkStates: reference reference_frame not found, did you forget to scope the link by model name?");
                continue;
            }

            linkState.pose.position.x = body_pose.Pos().X();
            linkState.pose.position.y = body_pose.Pos().Y();
            linkState.pose.position.z = body_pose.Pos().Z();
            linkState.pose.orientation.x = body_pose.Rot().X();
            linkState.pose.orientation.y = body_pose.Rot().Y();
            linkState.pose.orientation.z = body_pose.Rot().Z();
            linkState.pose.orientation.w = body_pose.Rot().W();
            linkState.twist.linear.x = body_vpos.X();
            linkState.twist.linear.y = body_vpos.Y();
            linkState.twist.linear.z = body_vpos.Z();
            linkState.twist.angular.x = body_veul.X();
            linkState.twist.angular.y = body_veul.Y();
            linkState.twist.angular.z = body_veul.Z();

            linkStates.push_back(linkState);
            statusList.push_back(true);
            messages.push_back("");
        }
        res.link_states = linkStates;
        res.success = true;
        res.status_message = "GetLinkStates: got state";
        res.status = statusList;
        res.messages = messages;

        return true;
    }

    bool DeepRacerGazeboSystemPlugin::setLinkStatesCallback(deepracer_msgs::SetLinkStates::Request &req,
                                                            deepracer_msgs::SetLinkStates::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        std::vector<gazebo_msgs::LinkState>::const_iterator linkStateIter = req.link_states.begin();

        std::vector<int8_t> statusList;
        std::vector<std::string> messages;

        for (; linkStateIter != req.link_states.end();
             ++linkStateIter)
        {
            gazebo_msgs::LinkState link_state = *linkStateIter;

            gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(m_world->EntityByName(link_state.link_name));
            gazebo::physics::LinkPtr frame = boost::dynamic_pointer_cast<gazebo::physics::Link>(m_world->EntityByName(link_state.reference_frame));

            if (!body)
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "Updating LinkState: link [%s] does not exist",
                                link_state.link_name.c_str());
                statusList.push_back(false);
                messages.push_back("SetLinkStates: link does not exist");
                continue;
            }

            /// @todo: FIXME map is really wrong, unless using tf here somehow
            // get reference frame (body/model(link)) pose and
            // transform target pose to absolute world frame
            ignition::math::Vector3d target_pos(link_state.pose.position.x, link_state.pose.position.y, link_state.pose.position.z);
            ignition::math::Quaterniond target_rot(link_state.pose.orientation.w, link_state.pose.orientation.x,
                                                   link_state.pose.orientation.y, link_state.pose.orientation.z);
            ignition::math::Pose3d target_pose(target_pos, target_rot);
            ignition::math::Vector3d target_linear_vel(link_state.twist.linear.x, link_state.twist.linear.y, link_state.twist.linear.z);
            ignition::math::Vector3d target_angular_vel(link_state.twist.angular.x, link_state.twist.angular.y, link_state.twist.angular.z);

            if (frame)
            {
                ignition::math::Pose3d frame_pose = frame->WorldPose(); // - myBody->GetCoMPose();
                ignition::math::Vector3d frame_linear_vel = frame->WorldLinearVel();
                ignition::math::Vector3d frame_angular_vel = frame->WorldAngularVel();

                ignition::math::Vector3d frame_pos = frame_pose.Pos();
                ignition::math::Quaterniond frame_rot = frame_pose.Rot();

                target_pose = target_pose + frame_pose;

                target_linear_vel -= frame_linear_vel;
                target_angular_vel -= frame_angular_vel;
            }
            else if (link_state.reference_frame == "" || link_state.reference_frame == "world" || link_state.reference_frame == "map" || link_state.reference_frame == "/map")
            {
                ROS_INFO_NAMED("deepracer_gazebo_system_plugin", "Updating LinkState: reference_frame is empty/world/map, using inertial frame");
            }
            else
            {
                ROS_ERROR_NAMED("deepracer_gazebo_system_plugin", "Updating LinkState: reference_frame is not a valid entity name");
                statusList.push_back(false);
                messages.push_back("SetLinkStates: reference_frame is not a valid entity name");
                continue;
            }

            bool is_paused = m_world->IsPaused();
            if (!is_paused)
                m_world->SetPaused(true);
            body->SetWorldPose(target_pose);
            m_world->SetPaused(is_paused);

            // set body velocity to desired twist
            body->SetLinearVel(target_linear_vel);
            body->SetAngularVel(target_angular_vel);
            statusList.push_back(true);
            messages.push_back("");
        }

        res.success = true;
        res.status_message = "SetLinkStates: success";
        res.status = statusList;
        res.messages = messages;
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::pausePhysicsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        m_world->SetPaused(true);
        return true;
    }

    bool DeepRacerGazeboSystemPlugin::unpausePhysicsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        std::lock_guard<std::mutex> lock(m_lock);
        m_world->SetPaused(false);
        return true;
    }

    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(DeepRacerGazeboSystemPlugin)
}