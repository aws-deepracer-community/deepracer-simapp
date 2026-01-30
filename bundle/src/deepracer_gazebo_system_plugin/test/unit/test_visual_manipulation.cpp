// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <deepracer_gazebo_system_plugin/deepracer_gazebo_system_plugin.h>
#include <deepracer_msgs/srv/get_visual.hpp>
#include <deepracer_msgs/srv/set_visual_color.hpp>
#include <deepracer_msgs/srv/set_visual_transparency.hpp>
#include <deepracer_msgs/srv/set_visual_visible.hpp>

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

class DeepRacerPluginVisualManipulationTest : public ::testing::Test
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

    /// \brief Create a complete visual entity with all components for testing
    /// \param[in] linkName Parent link name
    /// \param[in] visualName Visual name
    /// \param[in] pose Visual pose
    /// \param[in] material Visual material
    /// \return Visual entity ID
    gz::sim::Entity CreateCompleteVisual(const std::string& linkName, 
                                        const std::string& visualName,
                                        const gz::math::Pose3d& pose = gz::math::Pose3d::Zero,
                                        const sdf::Material& material = sdf::Material())
    {
        // Create link entity
        gz::sim::Entity linkEntity = realECM_->CreateEntity();
        realECM_->CreateComponent(linkEntity, gz::sim::components::Link());
        realECM_->CreateComponent(linkEntity, gz::sim::components::Name(linkName));
        
        // Create visual entity
        gz::sim::Entity visualEntity = realECM_->CreateEntity();
        realECM_->CreateComponent(visualEntity, gz::sim::components::Visual());
        realECM_->CreateComponent(visualEntity, gz::sim::components::Name(visualName));
        realECM_->CreateComponent(visualEntity, gz::sim::components::ParentEntity(linkEntity));
        realECM_->CreateComponent(visualEntity, gz::sim::components::Pose(pose));
        
        // Add material if provided, otherwise create default
        if (material.Ambient() != gz::math::Color::White || 
            material.Diffuse() != gz::math::Color::White) {
            realECM_->CreateComponent(visualEntity, gz::sim::components::Material(material));
        } else {
            // Create default material
            sdf::Material defaultMaterial;
            defaultMaterial.SetAmbient(gz::math::Color(0.3, 0.3, 0.3, 1.0));
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

    std::unique_ptr<DeepRacerGazeboSystemPlugin> plugin_;
    std::unique_ptr<gz::sim::EntityComponentManager> realECM_;
    std::unique_ptr<gz::sim::EventManager> realEventMgr_;
    std::shared_ptr<sdf::Element> sdfElement_;
};

TEST_F(DeepRacerPluginVisualManipulationTest, GetVisualWithCompleteProperties)
{
    // Create visual with specific properties
    gz::math::Pose3d testPose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    sdf::Material testMaterial;
    testMaterial.SetAmbient(gz::math::Color(0.1, 0.2, 0.3, 1.0));
    testMaterial.SetDiffuse(gz::math::Color(0.4, 0.5, 0.6, 0.8));
    testMaterial.SetSpecular(gz::math::Color(0.7, 0.8, 0.9, 1.0));
    testMaterial.SetEmissive(gz::math::Color(0.05, 0.15, 0.25, 1.0));
    
    CreateCompleteVisual("test_link", "test_visual", testPose, testMaterial);
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetVisual>("/get_visual");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::GetVisual::Request>();
    request->link_name = "test_link";
    request->visual_name = "test_visual";
    
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
    
    // Verify pose
    EXPECT_NEAR(response->pose.position.x, 1.0, 1e-6);
    EXPECT_NEAR(response->pose.position.y, 2.0, 1e-6);
    EXPECT_NEAR(response->pose.position.z, 3.0, 1e-6);
    
    // Verify colors
    EXPECT_NEAR(response->ambient.r, 0.1, 1e-6);
    EXPECT_NEAR(response->ambient.g, 0.2, 1e-6);
    EXPECT_NEAR(response->ambient.b, 0.3, 1e-6);
    EXPECT_NEAR(response->ambient.a, 1.0, 1e-6);
    
    EXPECT_NEAR(response->diffuse.r, 0.4, 1e-6);
    EXPECT_NEAR(response->diffuse.g, 0.5, 1e-6);
    EXPECT_NEAR(response->diffuse.b, 0.6, 1e-6);
    EXPECT_NEAR(response->diffuse.a, 0.8, 1e-6);
    
    // Verify transparency calculation (1.0 - alpha)
    EXPECT_NEAR(response->transparency, 0.2, 1e-6); // 1.0 - 0.8
    
    // Verify visibility
    EXPECT_TRUE(response->visible);
}

TEST_F(DeepRacerPluginVisualManipulationTest, GetVisualNotFound)
{
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetVisual>("/get_visual");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request for non-existent visual
    auto request = std::make_shared<deepracer_msgs::srv::GetVisual::Request>();
    request->link_name = "nonexistent_link";
    request->visual_name = "nonexistent_visual";
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_TRUE(response->status_message.find("not found") != std::string::npos);
}

TEST_F(DeepRacerPluginVisualManipulationTest, SetVisualColorSuccess)
{
    // Create visual
    CreateCompleteVisual("test_link", "test_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualColor>("/set_visual_color");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request with new colors
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualColor::Request>();
    request->link_name = "test_link";
    request->visual_name = "test_visual";
    request->ambient = CreateColor(1.0, 0.0, 0.0, 1.0);  // Red
    request->diffuse = CreateColor(0.0, 1.0, 0.0, 1.0);  // Green
    request->specular = CreateColor(0.0, 0.0, 1.0, 1.0); // Blue
    request->emissive = CreateColor(0.5, 0.5, 0.5, 1.0); // Gray
    
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
    EXPECT_TRUE(response->status_message.find("updated successfully") != std::string::npos);
    
    // Verify the color was actually changed by calling GetVisual
    auto getClient = node->create_client<deepracer_msgs::srv::GetVisual>("/get_visual");
    ASSERT_TRUE(getClient->wait_for_service(std::chrono::seconds(5)));
    
    auto getRequest = std::make_shared<deepracer_msgs::srv::GetVisual::Request>();
    getRequest->link_name = "test_link";
    getRequest->visual_name = "test_visual";
    
    auto getFuture = getClient->async_send_request(getRequest);
    auto getStatus = executor.spin_until_future_complete(getFuture, std::chrono::seconds(5));
    ASSERT_EQ(getStatus, rclcpp::FutureReturnCode::SUCCESS);
    
    auto getResponse = getFuture.get();
    EXPECT_TRUE(getResponse->success);
    
    // Verify colors were updated
    EXPECT_NEAR(getResponse->ambient.r, 1.0, 1e-6);
    EXPECT_NEAR(getResponse->ambient.g, 0.0, 1e-6);
    EXPECT_NEAR(getResponse->ambient.b, 0.0, 1e-6);
    
    EXPECT_NEAR(getResponse->diffuse.r, 0.0, 1e-6);
    EXPECT_NEAR(getResponse->diffuse.g, 1.0, 1e-6);
    EXPECT_NEAR(getResponse->diffuse.b, 0.0, 1e-6);
}

TEST_F(DeepRacerPluginVisualManipulationTest, SetVisualTransparencySuccess)
{
    // Create visual
    CreateCompleteVisual("test_link", "test_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualTransparency>("/set_visual_transparency");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualTransparency::Request>();
    request->link_name = "test_link";
    request->visual_name = "test_visual";
    request->transparency = 0.5f; // 50% transparent
    
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
    
    // Verify transparency was actually changed by calling GetVisual
    auto getClient = node->create_client<deepracer_msgs::srv::GetVisual>("/get_visual");
    ASSERT_TRUE(getClient->wait_for_service(std::chrono::seconds(5)));
    
    auto getRequest = std::make_shared<deepracer_msgs::srv::GetVisual::Request>();
    getRequest->link_name = "test_link";
    getRequest->visual_name = "test_visual";
    
    auto getFuture = getClient->async_send_request(getRequest);
    auto getStatus = executor.spin_until_future_complete(getFuture, std::chrono::seconds(5));
    ASSERT_EQ(getStatus, rclcpp::FutureReturnCode::SUCCESS);
    
    auto getResponse = getFuture.get();
    EXPECT_TRUE(getResponse->success);
    
    // Verify transparency was updated (transparency = 1.0 - alpha, so alpha should be 0.5)
    EXPECT_NEAR(getResponse->transparency, 0.5, 1e-6);
    EXPECT_NEAR(getResponse->diffuse.a, 0.5, 1e-6);
}

TEST_F(DeepRacerPluginVisualManipulationTest, SetVisualVisibleSuccess)
{
    // Create visual
    CreateCompleteVisual("test_link", "test_visual");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetVisualVisible>("/set_visual_visible");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request to make visual invisible
    auto request = std::make_shared<deepracer_msgs::srv::SetVisualVisible::Request>();
    request->link_name = "test_link";
    request->visual_name = "test_visual";
    request->visible = false;
    
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
    
    // Verify visibility was actually changed by calling GetVisual
    auto getClient = node->create_client<deepracer_msgs::srv::GetVisual>("/get_visual");
    ASSERT_TRUE(getClient->wait_for_service(std::chrono::seconds(5)));
    
    auto getRequest = std::make_shared<deepracer_msgs::srv::GetVisual::Request>();
    getRequest->link_name = "test_link";
    getRequest->visual_name = "test_visual";
    
    auto getFuture = getClient->async_send_request(getRequest);
    auto getStatus = executor.spin_until_future_complete(getFuture, std::chrono::seconds(5));
    ASSERT_EQ(getStatus, rclcpp::FutureReturnCode::SUCCESS);
    
    auto getResponse = getFuture.get();
    EXPECT_TRUE(getResponse->success);
    
    // Verify visibility was updated
    EXPECT_FALSE(getResponse->visible);
}
