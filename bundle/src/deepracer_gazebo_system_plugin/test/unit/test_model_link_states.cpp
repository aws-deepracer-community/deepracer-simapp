// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <deepracer_gazebo_system_plugin/deepracer_gazebo_system_plugin.h>
#include <deepracer_msgs/srv/get_model_states.hpp>
#include <deepracer_msgs/srv/set_model_states.hpp>
#include <deepracer_msgs/srv/get_link_states.hpp>
#include <deepracer_msgs/srv/set_link_states.hpp>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Element.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

using namespace gz::sim::systems;

class DeepRacerPluginModelLinkStatesTest : public ::testing::Test
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

    /// \brief Create a model entity with pose and velocity
    /// \param[in] name Model name
    /// \param[in] pose Model pose
    /// \param[in] linearVel Linear velocity
    /// \param[in] angularVel Angular velocity
    /// \return Model entity ID
    gz::sim::Entity CreateModel(const std::string& name,
                               const gz::math::Pose3d& pose = gz::math::Pose3d::Zero,
                               const gz::math::Vector3d& linearVel = gz::math::Vector3d::Zero,
                               const gz::math::Vector3d& angularVel = gz::math::Vector3d::Zero)
    {
        gz::sim::Entity entity = realECM_->CreateEntity();
        realECM_->CreateComponent(entity, gz::sim::components::Model());
        realECM_->CreateComponent(entity, gz::sim::components::Name(name));
        realECM_->CreateComponent(entity, gz::sim::components::Pose(pose));
        
        if (linearVel != gz::math::Vector3d::Zero) {
            realECM_->CreateComponent(entity, gz::sim::components::LinearVelocity(linearVel));
        }
        
        if (angularVel != gz::math::Vector3d::Zero) {
            realECM_->CreateComponent(entity, gz::sim::components::AngularVelocity(angularVel));
        }
        
        return entity;
    }

    /// \brief Create a link entity with pose and velocity
    /// \param[in] name Link name
    /// \param[in] parentModel Parent model entity
    /// \param[in] pose Link pose
    /// \param[in] linearVel Linear velocity
    /// \param[in] angularVel Angular velocity
    /// \return Link entity ID
    gz::sim::Entity CreateLink(const std::string& name,
                              gz::sim::Entity parentModel,
                              const gz::math::Pose3d& pose = gz::math::Pose3d::Zero,
                              const gz::math::Vector3d& linearVel = gz::math::Vector3d::Zero,
                              const gz::math::Vector3d& angularVel = gz::math::Vector3d::Zero)
    {
        gz::sim::Entity entity = realECM_->CreateEntity();
        realECM_->CreateComponent(entity, gz::sim::components::Link());
        realECM_->CreateComponent(entity, gz::sim::components::Name(name));
        realECM_->CreateComponent(entity, gz::sim::components::ParentEntity(parentModel));
        realECM_->CreateComponent(entity, gz::sim::components::Pose(pose));
        
        if (linearVel != gz::math::Vector3d::Zero) {
            realECM_->CreateComponent(entity, gz::sim::components::LinearVelocity(linearVel));
        }
        
        if (angularVel != gz::math::Vector3d::Zero) {
            realECM_->CreateComponent(entity, gz::sim::components::AngularVelocity(angularVel));
        }
        
        return entity;
    }

    /// \brief Create a ModelState message
    deepracer_msgs::msg::ModelState CreateModelState(const std::string& name,
                                                     const std::string& referenceFrame,
                                                     const gz::math::Pose3d& pose,
                                                     const gz::math::Vector3d& linearVel = gz::math::Vector3d::Zero,
                                                     const gz::math::Vector3d& angularVel = gz::math::Vector3d::Zero)
    {
        deepracer_msgs::msg::ModelState state;
        state.model_name = name;
        state.reference_frame = referenceFrame;
        
        state.pose.position.x = pose.Pos().X();
        state.pose.position.y = pose.Pos().Y();
        state.pose.position.z = pose.Pos().Z();
        state.pose.orientation.w = pose.Rot().W();
        state.pose.orientation.x = pose.Rot().X();
        state.pose.orientation.y = pose.Rot().Y();
        state.pose.orientation.z = pose.Rot().Z();
        
        state.twist.linear.x = linearVel.X();
        state.twist.linear.y = linearVel.Y();
        state.twist.linear.z = linearVel.Z();
        state.twist.angular.x = angularVel.X();
        state.twist.angular.y = angularVel.Y();
        state.twist.angular.z = angularVel.Z();
        
        return state;
    }

    std::unique_ptr<DeepRacerGazeboSystemPlugin> plugin_;
    std::unique_ptr<gz::sim::EntityComponentManager> realECM_;
    std::unique_ptr<gz::sim::EventManager> realEventMgr_;
    std::shared_ptr<sdf::Element> sdfElement_;
};

TEST_F(DeepRacerPluginModelLinkStatesTest, GetModelStatesWorldFrame)
{
    // Create models with known poses and velocities
    gz::math::Pose3d pose1(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    gz::math::Vector3d linVel1(0.1, 0.2, 0.3);
    gz::math::Vector3d angVel1(0.01, 0.02, 0.03);
    
    gz::math::Pose3d pose2(4.0, 5.0, 6.0, 0.4, 0.5, 0.6);
    gz::math::Vector3d linVel2(0.4, 0.5, 0.6);
    gz::math::Vector3d angVel2(0.04, 0.05, 0.06);
    
    CreateModel("model1", pose1, linVel1, angVel1);
    CreateModel("model2", pose2, linVel2, angVel2);
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetModelStates>("/get_model_states");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::GetModelStates::Request>();
    request->model_names = {"model1", "model2"};
    request->relative_entity_names = {"world", "world"};
    
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
    EXPECT_EQ(response->model_states.size(), 2);
    EXPECT_EQ(response->status.size(), 2);
    EXPECT_EQ(response->status[0], 1); // Success
    EXPECT_EQ(response->status[1], 1); // Success
    
    // Verify model1 state
    EXPECT_EQ(response->model_states[0].model_name, "model1");
    EXPECT_NEAR(response->model_states[0].pose.position.x, 1.0, 1e-6);
    EXPECT_NEAR(response->model_states[0].pose.position.y, 2.0, 1e-6);
    EXPECT_NEAR(response->model_states[0].pose.position.z, 3.0, 1e-6);
    EXPECT_NEAR(response->model_states[0].twist.linear.x, 0.1, 1e-6);
    EXPECT_NEAR(response->model_states[0].twist.linear.y, 0.2, 1e-6);
    EXPECT_NEAR(response->model_states[0].twist.linear.z, 0.3, 1e-6);
    
    // Verify model2 state
    EXPECT_EQ(response->model_states[1].model_name, "model2");
    EXPECT_NEAR(response->model_states[1].pose.position.x, 4.0, 1e-6);
    EXPECT_NEAR(response->model_states[1].pose.position.y, 5.0, 1e-6);
    EXPECT_NEAR(response->model_states[1].pose.position.z, 6.0, 1e-6);
}

TEST_F(DeepRacerPluginModelLinkStatesTest, GetModelStatesArraySizeMismatch)
{
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetModelStates>("/get_model_states");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request with mismatched array sizes
    auto request = std::make_shared<deepracer_msgs::srv::GetModelStates::Request>();
    request->model_names = {"model1", "model2"};
    request->relative_entity_names = {"world"}; // Size mismatch
    
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
    EXPECT_TRUE(response->status_message.find("must be same size") != std::string::npos);
}

TEST_F(DeepRacerPluginModelLinkStatesTest, SetModelStatesWorldFrame)
{
    // Create a model
    CreateModel("test_model");
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetModelStates>("/set_model_states");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request with new state
    auto request = std::make_shared<deepracer_msgs::srv::SetModelStates::Request>();
    gz::math::Pose3d newPose(10.0, 20.0, 30.0, 0.0, 0.0, 0.0);
    gz::math::Vector3d newLinVel(1.0, 2.0, 3.0);
    gz::math::Vector3d newAngVel(0.1, 0.2, 0.3);
    
    auto modelState = CreateModelState("test_model", "world", newPose, newLinVel, newAngVel);
    request->model_states = {modelState};
    
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
    EXPECT_EQ(response->status.size(), 1);
    EXPECT_EQ(response->status[0], 1); // Success
    
    // Verify the state was actually changed by calling GetModelStates
    auto getClient = node->create_client<deepracer_msgs::srv::GetModelStates>("/get_model_states");
    ASSERT_TRUE(getClient->wait_for_service(std::chrono::seconds(5)));
    
    auto getRequest = std::make_shared<deepracer_msgs::srv::GetModelStates::Request>();
    getRequest->model_names = {"test_model"};
    getRequest->relative_entity_names = {"world"};
    
    auto getFuture = getClient->async_send_request(getRequest);
    auto getStatus = executor.spin_until_future_complete(getFuture, std::chrono::seconds(5));
    ASSERT_EQ(getStatus, rclcpp::FutureReturnCode::SUCCESS);
    
    auto getResponse = getFuture.get();
    EXPECT_TRUE(getResponse->success);
    EXPECT_EQ(getResponse->model_states.size(), 1);
    
    // Verify pose was updated
    EXPECT_NEAR(getResponse->model_states[0].pose.position.x, 10.0, 1e-6);
    EXPECT_NEAR(getResponse->model_states[0].pose.position.y, 20.0, 1e-6);
    EXPECT_NEAR(getResponse->model_states[0].pose.position.z, 30.0, 1e-6);
    
    // Verify velocity was updated (if velocity components exist)
    // Note: Velocity components might not be created if they were zero initially
}

TEST_F(DeepRacerPluginModelLinkStatesTest, SetModelStatesNonexistentModel)
{
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::SetModelStates>("/set_model_states");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request for non-existent model
    auto request = std::make_shared<deepracer_msgs::srv::SetModelStates::Request>();
    auto modelState = CreateModelState("nonexistent_model", "world", gz::math::Pose3d::Zero);
    request->model_states = {modelState};
    
    // Call service
    auto future = client->async_send_request(request);
    
    // Spin until response received
    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    
    auto status = executor.spin_until_future_complete(future, std::chrono::seconds(5));
    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    
    // Check response
    auto response = future.get();
    EXPECT_FALSE(response->success); // Service fails when no models are updated
    EXPECT_EQ(response->status.size(), 1);
    EXPECT_EQ(response->status[0], 0); // Individual model fails
    EXPECT_TRUE(response->messages[0].find("does not exist") != std::string::npos);
}

TEST_F(DeepRacerPluginModelLinkStatesTest, GetLinkStatesBasic)
{
    // Create model and link
    auto modelEntity = CreateModel("test_model");
    gz::math::Pose3d linkPose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
    gz::math::Vector3d linkLinVel(0.1, 0.2, 0.3);
    CreateLink("test_link", modelEntity, linkPose, linkLinVel);
    
    // Create ROS 2 client
    auto node = rclcpp::Node::make_shared("test_client");
    auto client = node->create_client<deepracer_msgs::srv::GetLinkStates>("/get_link_states");
    
    // Wait for service to be available
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    // Create request
    auto request = std::make_shared<deepracer_msgs::srv::GetLinkStates::Request>();
    request->link_names = {"test_link"};
    request->reference_frames = {"world"};
    
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
    EXPECT_EQ(response->link_states.size(), 1);
    EXPECT_EQ(response->status.size(), 1);
    EXPECT_EQ(response->status[0], 1); // Success
    
    // Verify link state
    EXPECT_EQ(response->link_states[0].link_name, "test_link");
    EXPECT_NEAR(response->link_states[0].pose.position.x, 1.0, 1e-6);
    EXPECT_NEAR(response->link_states[0].pose.position.y, 2.0, 1e-6);
    EXPECT_NEAR(response->link_states[0].pose.position.z, 3.0, 1e-6);
    EXPECT_NEAR(response->link_states[0].twist.linear.x, 0.1, 1e-6);
    EXPECT_NEAR(response->link_states[0].twist.linear.y, 0.2, 1e-6);
    EXPECT_NEAR(response->link_states[0].twist.linear.z, 0.3, 1e-6);
}
