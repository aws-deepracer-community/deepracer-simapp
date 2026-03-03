// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <deepracer_gazebo_system_plugin/deepracer_gazebo_system_plugin.h>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <sdf/Element.hh>

using namespace gz::sim::systems;

class DeepRacerPluginLifecycleTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create plugin instance
        plugin_ = std::make_unique<DeepRacerGazeboSystemPlugin>();
        
        // Create real ECM and EventManager for basic lifecycle testing
        realECM_ = std::make_unique<gz::sim::EntityComponentManager>();
        realEventMgr_ = std::make_unique<gz::sim::EventManager>();
        
        // Create minimal SDF element
        sdfElement_ = std::make_shared<sdf::Element>();
    }

    void TearDown() override
    {
        plugin_.reset();
        realECM_.reset();
        realEventMgr_.reset();
    }

    std::unique_ptr<DeepRacerGazeboSystemPlugin> plugin_;
    std::unique_ptr<gz::sim::EntityComponentManager> realECM_;
    std::unique_ptr<gz::sim::EventManager> realEventMgr_;
    std::shared_ptr<sdf::Element> sdfElement_;
};

TEST_F(DeepRacerPluginLifecycleTest, ConstructorInitialization)
{
    // Test that plugin can be created without exceptions
    EXPECT_NO_THROW({
        auto testPlugin = std::make_unique<DeepRacerGazeboSystemPlugin>();
    });
}

TEST_F(DeepRacerPluginLifecycleTest, DestructorCleanup)
{
    // Test that plugin can be destroyed without hanging
    auto testPlugin = std::make_unique<DeepRacerGazeboSystemPlugin>();
    
    EXPECT_NO_THROW({
        testPlugin.reset();
    });
}

TEST_F(DeepRacerPluginLifecycleTest, ConfigureWithValidInputs)
{
    EXPECT_NO_THROW({
        plugin_->Configure(
            gz::sim::kNullEntity, 
            sdfElement_, 
            *realECM_, 
            *realEventMgr_);
    });
}

TEST_F(DeepRacerPluginLifecycleTest, ConfigureWithNullSDF)
{
    // Test with null SDF element - should handle gracefully
    EXPECT_NO_THROW({
        plugin_->Configure(
            gz::sim::kNullEntity, 
            nullptr, 
            *realECM_, 
            *realEventMgr_);
    });
}

TEST_F(DeepRacerPluginLifecycleTest, PreUpdateWithoutConfigure)
{
    gz::sim::UpdateInfo updateInfo;
    
    // Should handle gracefully without prior Configure
    EXPECT_NO_THROW({
        plugin_->PreUpdate(updateInfo, *realECM_);
    });
}

TEST_F(DeepRacerPluginLifecycleTest, PreUpdateAfterConfigure)
{
    // Configure plugin first
    plugin_->Configure(gz::sim::kNullEntity, sdfElement_, *realECM_, *realEventMgr_);
    
    gz::sim::UpdateInfo updateInfo;
    
    // First PreUpdate should initialize ROS 2 (may throw due to ROS 2 init)
    // We'll test this doesn't crash the plugin
    EXPECT_NO_THROW({
        plugin_->PreUpdate(updateInfo, *realECM_);
    });
}

TEST_F(DeepRacerPluginLifecycleTest, PostUpdateBasicFunctionality)
{
    gz::sim::UpdateInfo updateInfo;
    
    // Should execute without issues
    EXPECT_NO_THROW({
        plugin_->PostUpdate(updateInfo, *realECM_);
    });
}

TEST_F(DeepRacerPluginLifecycleTest, MultiplePreUpdateCalls)
{
    // Configure plugin first
    plugin_->Configure(gz::sim::kNullEntity, sdfElement_, *realECM_, *realEventMgr_);
    
    gz::sim::UpdateInfo updateInfo;
    
    // Multiple PreUpdate calls should not cause issues
    EXPECT_NO_THROW({
        plugin_->PreUpdate(updateInfo, *realECM_);
        plugin_->PreUpdate(updateInfo, *realECM_);
        plugin_->PreUpdate(updateInfo, *realECM_);
    });
}

TEST_F(DeepRacerPluginLifecycleTest, ConfigurePreUpdatePostUpdateSequence)
{
    gz::sim::UpdateInfo updateInfo;
    
    // Test normal lifecycle sequence
    EXPECT_NO_THROW({
        plugin_->Configure(gz::sim::kNullEntity, sdfElement_, *realECM_, *realEventMgr_);
        plugin_->PreUpdate(updateInfo, *realECM_);
        plugin_->PostUpdate(updateInfo, *realECM_);
    });
}
