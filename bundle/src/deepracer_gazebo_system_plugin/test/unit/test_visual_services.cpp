// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <deepracer_gazebo_system_plugin/deepracer_gazebo_system_plugin.h>
#include <deepracer_msgs/srv/get_light_names.hpp>
#include <deepracer_msgs/srv/get_visual_names.hpp>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace gz::sim::systems;

class DeepRacerPluginVisualServicesTest : public ::testing::Test
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

    /// \brief Add a light entity to the ECM for testing
    /// \param[in] name Light name
    /// \return Entity ID
    gz::sim::Entity AddLightToECM(const std::string& name)
    {
        gz::sim::Entity entity = realECM_->CreateEntity();
        realECM_->CreateComponent(entity, gz::sim::components::Light());
        realECM_->CreateComponent(entity, gz::sim::components::Name(name));
        return entity;
    }

    /// \brief Add a link entity to the ECM for testing
    /// \param[in] name Link name
    /// \return Entity ID
    gz::sim::Entity AddLinkToECM(const std::string& name)
    {
        gz::sim::Entity entity = realECM_->CreateEntity();
        realECM_->CreateComponent(entity, gz::sim::components::Link());
        realECM_->CreateComponent(entity, gz::sim::components::Name(name));
        return entity;
    }

    /// \brief Add a visual entity to the ECM for testing
    /// \param[in] name Visual name
    /// \param[in] parentLink Parent link entity
    /// \return Entity ID
    gz::sim::Entity AddVisualToECM(const std::string& name, gz::sim::Entity parentLink)
    {
        gz::sim::Entity entity = realECM_->CreateEntity();
        realECM_->CreateComponent(entity, gz::sim::components::Visual());
        realECM_->CreateComponent(entity, gz::sim::components::Name(name));
        realECM_->CreateComponent(entity, gz::sim::components::ParentEntity(parentLink));
        return entity;
    }

    /// \brief Helper to check if vector contains string
    bool VectorContains(const std::vector<std::string>& vec, const std::string& str)
    {
        return std::find(vec.begin(), vec.end(), str) != vec.end();
    }

    std::unique_ptr<DeepRacerGazeboSystemPlugin> plugin_;
    std::unique_ptr<gz::sim::EntityComponentManager> realECM_;
    std::unique_ptr<gz::sim::EventManager> realEventMgr_;
    std::shared_ptr<sdf::Element> sdfElement_;
};

TEST_F(DeepRacerPluginVisualServicesTest, GetLightNamesWithNoLights)
{
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetLightNames>("/get_light_names");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::GetLightNames::Request>();
    
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
    EXPECT_EQ(response->light_names.size(), 0);
    EXPECT_TRUE(response->status_message.find("Found 0 lights") != std::string::npos);
}

TEST_F(DeepRacerPluginVisualServicesTest, GetLightNamesWithMultipleLights)
{
    // Add lights to ECM
    AddLightToECM("light1");
    AddLightToECM("light2");
    AddLightToECM("light3");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetLightNames>("/get_light_names");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::GetLightNames::Request>();
    
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
    EXPECT_EQ(response->light_names.size(), 3);
    
    // Check that all expected lights are present
    EXPECT_TRUE(VectorContains(response->light_names, "light1"));
    EXPECT_TRUE(VectorContains(response->light_names, "light2"));
    EXPECT_TRUE(VectorContains(response->light_names, "light3"));
    
    EXPECT_TRUE(response->status_message.find("Found 3 lights") != std::string::npos);
}

TEST_F(DeepRacerPluginVisualServicesTest, GetVisualNamesWithNoVisuals)
{
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetVisualNames>("/get_visual_names");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request (empty means get all)
    auto request = std::make_shared<deepracer_msgs::srv::GetVisualNames::Request>();
    
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
    EXPECT_EQ(response->visual_names.size(), 0);
    EXPECT_EQ(response->link_names.size(), 0);
}

TEST_F(DeepRacerPluginVisualServicesTest, GetVisualNamesWithLinkVisualHierarchy)
{
    // Create link-visual hierarchy
    auto link1 = AddLinkToECM("link1");
    auto link2 = AddLinkToECM("link2");
    
    AddVisualToECM("visual1_1", link1);
    AddVisualToECM("visual1_2", link1);
    AddVisualToECM("visual2_1", link2);
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetVisualNames>("/get_visual_names");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request for specific links
    auto request = std::make_shared<deepracer_msgs::srv::GetVisualNames::Request>();
    request->link_names = {"link1", "link2"};
    
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
    EXPECT_EQ(response->visual_names.size(), 3);
    EXPECT_EQ(response->link_names.size(), 3);
    
    // Check that we got the expected visuals
    EXPECT_TRUE(VectorContains(response->visual_names, "visual1_1"));
    EXPECT_TRUE(VectorContains(response->visual_names, "visual1_2"));
    EXPECT_TRUE(VectorContains(response->visual_names, "visual2_1"));
    
    // Check that visual-to-link mapping is correct
    for (size_t i = 0; i < response->visual_names.size(); ++i) {
        const std::string& visual_name = response->visual_names[i];
        const std::string& link_name = response->link_names[i];
        
        if (visual_name == "visual1_1" || visual_name == "visual1_2") {
            EXPECT_EQ(link_name, "link1");
        } else if (visual_name == "visual2_1") {
            EXPECT_EQ(link_name, "link2");
        }
    }
}
