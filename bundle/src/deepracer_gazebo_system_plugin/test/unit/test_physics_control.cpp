// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <deepracer_gazebo_system_plugin/deepracer_gazebo_system_plugin.h>
#include <std_srvs/srv/empty.hpp>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/PhysicsCmd.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

using namespace gz::sim::systems;

class DeepRacerPluginPhysicsControlTest : public ::testing::Test
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

    /// \brief Create a world entity for physics control testing
    /// \return World entity ID
    gz::sim::Entity CreateWorld()
    {
        gz::sim::Entity entity = realECM_->CreateEntity();
        realECM_->CreateComponent(entity, gz::sim::components::World());
        return entity;
    }

    std::unique_ptr<DeepRacerGazeboSystemPlugin> plugin_;
    std::unique_ptr<gz::sim::EntityComponentManager> realECM_;
    std::unique_ptr<gz::sim::EventManager> realEventMgr_;
    std::shared_ptr<sdf::Element> sdfElement_;
};

TEST_F(DeepRacerPluginPhysicsControlTest, PausePhysicsWithWorld)
{
    // Create world entity
    CreateWorld();
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<std_srvs::srv::Empty>("/pause_physics_dr");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response (Empty service has no response data)
    auto response = future.get();
    EXPECT_TRUE(response != nullptr);
    
    // Verify PhysicsCmd component was created/updated
    // We can't directly access the component from the test, but the service should complete successfully
}

TEST_F(DeepRacerPluginPhysicsControlTest, UnpausePhysicsWithWorld)
{
    // Create world entity
    CreateWorld();
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<std_srvs::srv::Empty>("/unpause_physics_dr");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response (Empty service has no response data)
    auto response = future.get();
    EXPECT_TRUE(response != nullptr);
}

TEST_F(DeepRacerPluginPhysicsControlTest, PausePhysicsWithoutWorld)
{
    // Don't create world entity - test error handling
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<std_srvs::srv::Empty>("/pause_physics_dr");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response - service should still complete (error is logged but not returned)
    auto response = future.get();
    EXPECT_TRUE(response != nullptr);
}

TEST_F(DeepRacerPluginPhysicsControlTest, PauseUnpauseCycle)
{
    // Create world entity
    CreateWorld();
    
    // Create ROS 2 clients
    auto node = rclcpp::Node::make_shared("test_client");
    auto pauseClient = node->create_client<std_srvs::srv::Empty>("/pause_physics_dr");
    auto unpauseClient = node->create_client<std_srvs::srv::Empty>("/unpause_physics_dr");
    
    // Wait for services to be available
    ASSERT_TRUE(pauseClient->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(unpauseClient->wait_for_service(std::chrono::seconds(5)));
    
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    // Test multiple pause/unpause cycles
    for (int i = 0; i < 3; ++i) {
        // Pause physics
        auto pauseRequest = std::make_shared<std_srvs::srv::Empty::Request>();
        auto pauseFuture = pauseClient->async_send_request(pauseRequest);
        auto pauseStatus = executor.spin_until_future_complete(pauseFuture, std::chrono::seconds(5));
        ASSERT_EQ(pauseStatus, rclcpp::FutureReturnCode::SUCCESS);
        
        // Unpause physics
        auto unpauseRequest = std::make_shared<std_srvs::srv::Empty::Request>();
        auto unpauseFuture = unpauseClient->async_send_request(unpauseRequest);
        auto unpauseStatus = executor.spin_until_future_complete(unpauseFuture, std::chrono::seconds(5));
        ASSERT_EQ(unpauseStatus, rclcpp::FutureReturnCode::SUCCESS);
    }
}
