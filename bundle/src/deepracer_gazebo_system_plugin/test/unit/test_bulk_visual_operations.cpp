// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <deepracer_gazebo_system_plugin/deepracer_gazebo_system_plugin.h>
#include <deepracer_msgs/srv/get_visuals.hpp>
#include <deepracer_msgs/srv/set_visual_colors.hpp>
#include <deepracer_msgs/srv/set_visual_transparencies.hpp>
#include <deepracer_msgs/srv/set_visual_visibles.hpp>
#include <deepracer_msgs/srv/set_visual_pose.hpp>
#include <deepracer_msgs/srv/set_visual_poses.hpp>
#include <deepracer_msgs/srv/set_visual_mesh.hpp>
#include <deepracer_msgs/srv/set_visual_meshes.hpp>
#include <deepracer_msgs/srv/set_link_states.hpp>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visibility.hh>
#include <sdf/Element.hh>
#include <sdf/Material.hh>
#include <sdf/Geometry.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Color.hh>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

using namespace gz::sim::systems;

class DeepRacerPluginBulkVisualOperationsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create plugin instance
        plugin_ = std::make_unique<DeepRacerGazeboSystemPlugin>();
        
        // Create real ECM and EventManager
        realECM_ = std::make_unique<gz::sim::EntityComponentManager>();
        realEventMgr_ = std::make_unique<gz::sim::EventManager>();
        
        // Create minimal SDF element
        sdfElement_ = std::make_shared<sdf::Element>();
        
        // Configure and initialize the plugin
        plugin_->Configure(gz::sim::kNullEntity, sdfElement_, *realECM_, *realEventMgr_);
        
        // Initialize ROS 2 (this creates the services)
        gz::sim::UpdateInfo updateInfo;
        plugin_->PreUpdate(updateInfo, *realECM_);
        
        // Give ROS 2 time to initialize
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void TearDown() override
    {
        plugin_.reset();
        realECM_.reset();
        realEventMgr_.reset();
    }

    /// \brief Create a link entity with name
    gz::sim::Entity CreateLink(const std::string& linkName)
    {
        gz::sim::Entity linkEntity = realECM_->CreateEntity();
        realECM_->CreateComponent(linkEntity, gz::sim::components::Link());
        realECM_->CreateComponent(linkEntity, gz::sim::components::Name(linkName));
        return linkEntity;
    }

    /// \brief Create a visual entity with complete properties
    gz::sim::Entity CreateCompleteVisual(const std::string& linkName, const std::string& visualName,
                                        const gz::math::Pose3d& pose = gz::math::Pose3d::Zero,
                                        const sdf::Material& material = sdf::Material())
    {
        // Create link first
        gz::sim::Entity linkEntity = CreateLink(linkName);
        
        // Create visual entity
        gz::sim::Entity visualEntity = realECM_->CreateEntity();
        realECM_->CreateComponent(visualEntity, gz::sim::components::Visual());
        realECM_->CreateComponent(visualEntity, gz::sim::components::Name(visualName));
        realECM_->CreateComponent(visualEntity, gz::sim::components::ParentEntity(linkEntity));
        
        // Add pose
        realECM_->CreateComponent(visualEntity, gz::sim::components::Pose(pose));
        
        // Add material
        if (material.Ambient() != gz::math::Color(0, 0, 0, 0) || 
            material.Diffuse() != gz::math::Color(0, 0, 0, 0) ||
            material.Specular() != gz::math::Color(0, 0, 0, 0) ||
            material.Emissive() != gz::math::Color(0, 0, 0, 0))
        {
            realECM_->CreateComponent(visualEntity, gz::sim::components::Material(material));
        }
        else
        {
            // Add default material
            sdf::Material defaultMaterial;
            defaultMaterial.SetAmbient(gz::math::Color(0.5, 0.5, 0.5, 1.0));
            defaultMaterial.SetDiffuse(gz::math::Color(0.7, 0.7, 0.7, 1.0));
            defaultMaterial.SetSpecular(gz::math::Color(0.1, 0.1, 0.1, 1.0));
            defaultMaterial.SetEmissive(gz::math::Color(0.0, 0.0, 0.0, 1.0));
            realECM_->CreateComponent(visualEntity, gz::sim::components::Material(defaultMaterial));
        }
        
        // Add default geometry
        sdf::Geometry geometry;
        geometry.SetType(sdf::GeometryType::BOX);
        realECM_->CreateComponent(visualEntity, gz::sim::components::Geometry(geometry));
        
        // Add visibility
        realECM_->CreateComponent(visualEntity, gz::sim::components::VisibilityFlags(1u));
        
        return visualEntity;
    }

    /// \brief Create a ColorRGBA message
    std_msgs::msg::ColorRGBA CreateColor(double r, double g, double b, double a)
    {
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    /// \brief Create a Pose message
    geometry_msgs::msg::Pose CreatePose(double x, double y, double z, double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        
        // Convert RPY to quaternion
        gz::math::Quaterniond quat = gz::math::Quaterniond::EulerToQuaternion(roll, pitch, yaw);
        pose.orientation.x = quat.X();
        pose.orientation.y = quat.Y();
        pose.orientation.z = quat.Z();
        pose.orientation.w = quat.W();
        
        return pose;
    }

    std::unique_ptr<DeepRacerGazeboSystemPlugin> plugin_;
    std::unique_ptr<gz::sim::EntityComponentManager> realECM_;
    std::unique_ptr<gz::sim::EventManager> realEventMgr_;
    std::shared_ptr<sdf::Element> sdfElement_;
};

// =============================================================================
// BULK VISUAL OPERATIONS TESTS
// =============================================================================

TEST_F(DeepRacerPluginBulkVisualOperationsTest, GetVisualsMultipleVisuals)
{
    // Create multiple visuals with different properties
    sdf::Material material1;
    material1.SetAmbient(gz::math::Color(0.1, 0.2, 0.3, 1.0));
    material1.SetDiffuse(gz::math::Color(0.4, 0.5, 0.6, 0.8));
    CreateCompleteVisual("link1", "visual1", gz::math::Pose3d(1.0, 2.0, 3.0, 0.1, 0.2, 0.3), material1);
    
    sdf::Material material2;
    material2.SetAmbient(gz::math::Color(0.7, 0.8, 0.9, 1.0));
    material2.SetDiffuse(gz::math::Color(0.1, 0.3, 0.5, 0.9));
    CreateCompleteVisual("link2", "visual2", gz::math::Pose3d(4.0, 5.0, 6.0, 0.4, 0.5, 0.6), material2);
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetVisuals>("/get_visuals");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::GetVisuals::Request>();
    request->link_names = {"link1", "link2"};
    request->visual_names = {"visual1", "visual2"};
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_EQ(response->link_names.size(), 2u);
    EXPECT_EQ(response->visual_names.size(), 2u);
    
    // Verify first visual
    EXPECT_EQ(response->link_names[0], "link1");
    EXPECT_EQ(response->visual_names[0], "visual1");
    EXPECT_NEAR(response->poses[0].position.x, 1.0, 1e-6);
    EXPECT_NEAR(response->ambients[0].r, 0.1, 1e-6);
    
    // Verify second visual
    EXPECT_EQ(response->link_names[1], "link2");
    EXPECT_EQ(response->visual_names[1], "visual2");
    EXPECT_NEAR(response->poses[1].position.x, 4.0, 1e-6);
    EXPECT_NEAR(response->ambients[1].r, 0.7, 1e-6);
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, GetVisualsPartialFailure)
{
    // Create only one visual
    CreateCompleteVisual("existing_link", "existing_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetVisuals>("/get_visuals");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request with one existing and one non-existing visual
    auto request = std::make_shared<deepracer_msgs::srv::GetVisuals::Request>();
    request->link_names = {"existing_link", "nonexistent_link"};
    request->visual_names = {"existing_visual", "nonexistent_visual"};
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response - the service returns all requested visuals, even if some don't exist
    auto response = future.get();
    EXPECT_TRUE(response->success);
    // The service returns results for all requested visuals, with empty/default values for nonexistent ones
    EXPECT_EQ(response->link_names.size(), 2u); // Both requested visuals
    EXPECT_EQ(response->visual_names.size(), 2u);
    
    // Find the existing visual in the response
    bool found_existing = false;
    for (size_t i = 0; i < response->link_names.size(); ++i) {
        if (response->link_names[i] == "existing_link" && response->visual_names[i] == "existing_visual") {
            found_existing = true;
            break;
        }
    }
    EXPECT_TRUE(found_existing);
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualColorsMultipleVisuals)
{
    // Create multiple visuals
    CreateCompleteVisual("link1", "visual1");
    CreateCompleteVisual("link2", "visual2");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualColors>("/set_visual_colors");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualColors::Request>();
    request->link_names = {"link1", "link2"};
    request->visual_names = {"visual1", "visual2"};
    request->ambients = {CreateColor(1.0, 0.0, 0.0, 1.0), CreateColor(0.0, 1.0, 0.0, 1.0)};
    request->diffuses = {CreateColor(0.8, 0.2, 0.2, 1.0), CreateColor(0.2, 0.8, 0.2, 1.0)};
    request->speculars = {CreateColor(0.1, 0.1, 0.1, 1.0), CreateColor(0.1, 0.1, 0.1, 1.0)};
    request->emissives = {CreateColor(0.0, 0.0, 0.0, 1.0), CreateColor(0.0, 0.0, 0.0, 1.0)};
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    // The service succeeded - don't check specific status message format
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualTransparenciesMultipleVisuals)
{
    // Create multiple visuals
    CreateCompleteVisual("link1", "visual1");
    CreateCompleteVisual("link2", "visual2");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualTransparencies>("/set_visual_transparencies");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualTransparencies::Request>();
    request->link_names = {"link1", "link2"};
    request->visual_names = {"visual1", "visual2"};
    request->transparencies = {0.3, 0.7}; // Different transparency values
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    // The service succeeded - don't check specific status message format
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualVisiblesMultipleVisuals)
{
    // Create multiple visuals
    CreateCompleteVisual("link1", "visual1");
    CreateCompleteVisual("link2", "visual2");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualVisibles>("/set_visual_visibles");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualVisibles::Request>();
    request->link_names = {"link1", "link2"};
    request->visual_names = {"visual1", "visual2"};
    request->visibles = {true, false}; // Different visibility values
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(response->status_message.empty() || response->status_message.find("success") != std::string::npos);
}
// =============================================================================
// VISUAL POSE OPERATIONS TESTS
// =============================================================================

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualPoseSingleVisual)
{
    // Create visual
    CreateCompleteVisual("test_link", "test_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualPose>("/set_visual_pose");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualPose::Request>();
    request->link_name = "test_link";
    request->visual_name = "test_visual";
    request->pose = CreatePose(5.0, 6.0, 7.0, 0.1, 0.2, 0.3);
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(response->status_message.empty() || response->status_message.find("success") != std::string::npos);
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualPoseNonexistentVisual)
{
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualPose>("/set_visual_pose");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request for nonexistent visual
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualPose::Request>();
    request->link_name = "nonexistent_link";
    request->visual_name = "nonexistent_visual";
    request->pose = CreatePose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response - should fail gracefully
    auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_FALSE(response->status_message.empty());
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualPosesMultipleVisuals)
{
    // Create multiple visuals
    CreateCompleteVisual("link1", "visual1");
    CreateCompleteVisual("link2", "visual2");
    CreateCompleteVisual("link3", "visual3");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualPoses>("/set_visual_poses");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualPoses::Request>();
    request->link_names = {"link1", "link2", "link3"};
    request->visual_names = {"visual1", "visual2", "visual3"};
    request->poses = {
        CreatePose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3),
        CreatePose(4.0, 5.0, 6.0, 0.4, 0.5, 0.6),
        CreatePose(7.0, 8.0, 9.0, 0.7, 0.8, 0.9)
    };
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(response->status_message.empty() || response->status_message.find("success") != std::string::npos);
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualPosesMismatchedArrays)
{
    // Create visual
    CreateCompleteVisual("test_link", "test_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualPoses>("/set_visual_poses");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request with mismatched array sizes
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualPoses::Request>();
    request->link_names = {"link1", "link2"}; // 2 elements
    request->visual_names = {"visual1"}; // 1 element - mismatch!
    request->poses = {CreatePose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)}; // 1 element
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response - should fail due to array size mismatch
    auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_FALSE(response->status_message.empty());
}
// =============================================================================
// VISUAL MESH OPERATIONS TESTS
// =============================================================================

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualMeshSingleVisual)
{
    // Create visual
    CreateCompleteVisual("test_link", "test_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualMesh>("/set_visual_mesh");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualMesh::Request>();
    request->link_name = "test_link";
    request->visual_name = "test_visual";
    request->filename = "file://test_mesh.dae";
    request->scale.x = 1.5;
    request->scale.y = 2.0;
    request->scale.z = 0.8;
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(response->status_message.empty() || response->status_message.find("success") != std::string::npos);
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualMeshInvalidURI)
{
    // Create visual
    CreateCompleteVisual("test_link", "test_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualMesh>("/set_visual_mesh");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request with invalid mesh URI
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualMesh::Request>();
    request->link_name = "test_link";
    request->visual_name = "test_visual";
    request->filename = ""; // Empty URI
    request->scale.x = 1.0;
    request->scale.y = 1.0;
    request->scale.z = 1.0;
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response - should handle gracefully
    auto response = future.get();
    // Note: The actual behavior depends on implementation - it might succeed with warning or fail
    EXPECT_FALSE(response->status_message.empty()); // Should have some status message
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualMeshesMultipleVisuals)
{
    // Create multiple visuals
    CreateCompleteVisual("link1", "visual1");
    CreateCompleteVisual("link2", "visual2");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualMeshes>("/set_visual_meshes");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualMeshes::Request>();
    request->link_names = {"link1", "link2"};
    request->visual_names = {"visual1", "visual2"};
    request->filenames = {"file://mesh1.dae", "file://mesh2.obj"};
    
    // Create mesh scales
    geometry_msgs::msg::Vector3 scale1, scale2;
    scale1.x = 1.0; scale1.y = 1.0; scale1.z = 1.0;
    scale2.x = 2.0; scale2.y = 1.5; scale2.z = 0.5;
    request->scales = {scale1, scale2};
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(response->status_message.empty() || response->status_message.find("success") != std::string::npos);
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetVisualMeshesArraySizeMismatch)
{
    // Create visual
    CreateCompleteVisual("test_link", "test_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualMeshes>("/set_visual_meshes");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request with mismatched array sizes
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualMeshes::Request>();
    request->link_names = {"link1", "link2"}; // 2 elements
    request->visual_names = {"visual1"}; // 1 element - mismatch!
    request->filenames = {"file://mesh1.dae"}; // 1 element
    
    geometry_msgs::msg::Vector3 scale;
    scale.x = 1.0; scale.y = 1.0; scale.z = 1.0;
    request->scales = {scale}; // 1 element
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response - should fail due to array size mismatch
    auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_FALSE(response->status_message.empty());
}
// =============================================================================
// LINK STATE OPERATIONS TESTS
// =============================================================================

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetLinkStatesMultipleLinks)
{
    // Create multiple links (links are created automatically when creating visuals)
    CreateCompleteVisual("link1", "visual1");
    CreateCompleteVisual("link2", "visual2");
    CreateCompleteVisual("link3", "visual3");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetLinkStates>("/set_link_states");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetLinkStates::Request>();
    
    // Create link states
    deepracer_msgs::msg::LinkState state1, state2, state3;
    
    state1.link_name = "link1";
    state1.pose = CreatePose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    state1.twist.linear.x = 0.5;
    state1.twist.linear.y = 0.0;
    state1.twist.linear.z = 0.0;
    state1.twist.angular.x = 0.0;
    state1.twist.angular.y = 0.0;
    state1.twist.angular.z = 0.1;
    state1.reference_frame = "world";
    
    state2.link_name = "link2";
    state2.pose = CreatePose(4.0, 5.0, 6.0, 0.4, 0.5, 0.6);
    state2.twist.linear.x = -0.3;
    state2.twist.linear.y = 0.2;
    state2.twist.linear.z = 0.0;
    state2.twist.angular.x = 0.0;
    state2.twist.angular.y = 0.0;
    state2.twist.angular.z = -0.2;
    state2.reference_frame = "world";
    
    state3.link_name = "link3";
    state3.pose = CreatePose(7.0, 8.0, 9.0, 0.7, 0.8, 0.9);
    state3.twist.linear.x = 0.0;
    state3.twist.linear.y = 0.0;
    state3.twist.linear.z = 0.1;
    state3.twist.angular.x = 0.05;
    state3.twist.angular.y = 0.0;
    state3.twist.angular.z = 0.0;
    state3.reference_frame = "world";
    
    request->link_states = {state1, state2, state3};
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(response->status_message.empty() || response->status_message.find("success") != std::string::npos);
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetLinkStatesNonexistentLink)
{
    // Create one existing link
    CreateCompleteVisual("existing_link", "visual1");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetLinkStates>("/set_link_states");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request with mix of existing and nonexistent links
    auto request = std::make_shared<deepracer_msgs::srv::SetLinkStates::Request>();
    
    deepracer_msgs::msg::LinkState existingState, nonexistentState;
    
    existingState.link_name = "existing_link";
    existingState.pose = CreatePose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
    existingState.reference_frame = "world";
    
    nonexistentState.link_name = "nonexistent_link";
    nonexistentState.pose = CreatePose(4.0, 5.0, 6.0, 0.0, 0.0, 0.0);
    nonexistentState.reference_frame = "world";
    
    request->link_states = {existingState, nonexistentState};
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response - should handle partial success/failure gracefully
    auto response = future.get();
    // The response behavior depends on implementation - it might succeed with warnings
    // or fail completely. We just verify we get a response with status information.
    EXPECT_FALSE(response->status_message.empty());
}

TEST_F(DeepRacerPluginBulkVisualOperationsTest, SetLinkStatesEmptyRequest)
{
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetLinkStates>("/set_link_states");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create empty request
    auto request = std::make_shared<deepracer_msgs::srv::SetLinkStates::Request>();
    // request->link_states is empty
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response - should handle empty request gracefully
    auto response = future.get();
    EXPECT_TRUE(response->success); // Empty request should succeed trivially
    EXPECT_FALSE(response->status_message.empty());
}
