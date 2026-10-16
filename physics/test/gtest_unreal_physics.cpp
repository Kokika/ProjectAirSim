// Copyright (C) Microsoft Corporation. 
// Copyright (C) 2025 IAMAI CONSULTING CORP

// MIT License. All rights reserved.

#include <functional>
#include <memory>

#include "core_sim/actor/robot.hpp"
#include "core_sim/simulator.hpp"
#include "gtest/gtest.h"
#include "test_data/physics_test_config.hpp"  // defines physics_test_config
#include "unreal_physics.hpp"

namespace projectairsim = microsoft::projectairsim;

// Custom test subclass to expose protected members for testing
namespace microsoft {
namespace projectairsim {
class TestUnrealPhysicsBody : public UnrealPhysicsBody {
 public:
  // Expose parent constructors for testing
  using UnrealPhysicsBody::UnrealPhysicsBody;
  using UnrealPhysicsBody::external_wrench_;
  using UnrealPhysicsBody::external_wrench_points_;
  using UnrealPhysicsBody::kinematics_;
};
}  // namespace projectairsim
}  // namespace microsoft

// -----------------------------------------------------------------------------
// class UnrealPhysicsBody

TEST(UnrealPhysicsBody, Constructor) {
  // General description:
  // Ensures UnrealPhysicsBody default construction succeeds and returns a valid
  // object instance.
  // Arrange: prepare context for default constructor.
  // Act: allocate UnrealPhysicsBody with std::make_unique.
  auto body = std::make_unique<projectairsim::UnrealPhysicsBody>();
  // Assert: allocation produced a non-null pointer.
  EXPECT_NE(body, nullptr);
}

TEST(UnrealPhysicsBody, InitializeUnrealPhysicsBody) {
  // General description:
  // Verifies InitializeUnrealPhysicsBody collects rotor wrench points from robot
  // actuators into external_wrench_points_ vector.
  // Arrange: load simulator with test robot containing rotors.
  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();
  auto actors = scene.GetActors();

  // Arrange: find robot actor in scene.
  projectairsim::Robot* sim_robot = nullptr;
  for (auto actor_ref : actors) {
    if (actor_ref.get().GetType() == projectairsim::ActorType::kRobot) {
      sim_robot = &dynamic_cast<projectairsim::Robot&>(actor_ref.get());
      break;
    }
  }
  EXPECT_NE(sim_robot, nullptr);

  // Act: construct UnrealPhysicsBody from robot, which triggers
  // InitializeUnrealPhysicsBody.
  auto test_body = std::make_shared<projectairsim::TestUnrealPhysicsBody>();
  *static_cast<projectairsim::UnrealPhysicsBody*>(test_body.get()) =
      projectairsim::UnrealPhysicsBody(*sim_robot);

  // Assert: rotor wrench points are collected.
  EXPECT_GT(test_body->external_wrench_points_.size(), 0);

  // Assert: all stored pointers are valid and non-null.
  for (const auto& wrench_pt : test_body->external_wrench_points_) {
    EXPECT_NE(wrench_pt, nullptr);
  }
}

TEST(UnrealPhysicsBody, CalculateExternalWrench) {
  // General description:
  // Verifies CalculateExternalWrench aggregates wrench from all rotor points
  // and applies world-frame transformation.
  // Arrange: load simulator and create physics body from robot.
  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();
  auto actors = scene.GetActors();

  projectairsim::Robot* sim_robot = nullptr;
  for (auto actor_ref : actors) {
    if (actor_ref.get().GetType() == projectairsim::ActorType::kRobot) {
      sim_robot = &dynamic_cast<projectairsim::Robot&>(actor_ref.get());
      break;
    }
  }
  EXPECT_NE(sim_robot, nullptr);

  // Act: construct body and calculate external wrench.
  auto test_body = std::make_shared<projectairsim::TestUnrealPhysicsBody>(
      *sim_robot);
  test_body->CalculateExternalWrench();

  // Assert: external wrench was computed (even if zero initially).
  const auto& external_wrench = test_body->external_wrench_;
  EXPECT_TRUE(std::isfinite(external_wrench.force.x()));
  EXPECT_TRUE(std::isfinite(external_wrench.force.y()));
  EXPECT_TRUE(std::isfinite(external_wrench.force.z()));
  EXPECT_TRUE(std::isfinite(external_wrench.torque.x()));
  EXPECT_TRUE(std::isfinite(external_wrench.torque.y()));
  EXPECT_TRUE(std::isfinite(external_wrench.torque.z()));
}

TEST(UnrealPhysicsBody, WriteRobotData) {
  // General description:
  // Verifies WriteRobotData updates robot kinematics with supplied data.
  // Arrange: create robot kinematics with known values.
  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();
  auto actors = scene.GetActors();

  projectairsim::Robot* sim_robot = nullptr;
  for (auto actor_ref : actors) {
    if (actor_ref.get().GetType() == projectairsim::ActorType::kRobot) {
      sim_robot = &dynamic_cast<projectairsim::Robot&>(actor_ref.get());
      break;
    }
  }
  EXPECT_NE(sim_robot, nullptr);

  // Arrange: construct physics body and save original kinematics.
  projectairsim::UnrealPhysicsBody body(*sim_robot);
  auto orig_kin = sim_robot->GetKinematics();

  // Arrange: create altered kinematics to write.
  projectairsim::Kinematics new_kin;
  new_kin.pose.position = {1.5, 2.5, 3.5};
  new_kin.pose.orientation =
      projectairsim::Quaternion(1, 0, 0, 0).normalized();
  new_kin.twist.linear = {0.1, 0.2, 0.3};
  new_kin.twist.angular = {0.4, 0.5, 0.6};

  // Act: write new kinematics to robot.
  body.WriteRobotData(new_kin);

  // Assert: robot kinematics updated to match written data.
  auto updated_kin = sim_robot->GetKinematics();
  EXPECT_FLOAT_EQ(updated_kin.pose.position.x(), 1.5);
  EXPECT_FLOAT_EQ(updated_kin.pose.position.y(), 2.5);
  EXPECT_FLOAT_EQ(updated_kin.pose.position.z(), 3.5);
  EXPECT_FLOAT_EQ(updated_kin.twist.linear.x(), 0.1);
  EXPECT_FLOAT_EQ(updated_kin.twist.linear.y(), 0.2);
  EXPECT_FLOAT_EQ(updated_kin.twist.linear.z(), 0.3);
}

TEST(UnrealPhysicsBody, SetCallbackSetExternalWrench) {
  // General description:
  // Verifies callback is invoked during CalculateExternalWrench with the
  // computed wrench.
  // Arrange: create robot and physics body.
  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();
  auto actors = scene.GetActors();

  projectairsim::Robot* sim_robot = nullptr;
  for (auto actor_ref : actors) {
    if (actor_ref.get().GetType() == projectairsim::ActorType::kRobot) {
      sim_robot = &dynamic_cast<projectairsim::Robot&>(actor_ref.get());
      break;
    }
  }
  EXPECT_NE(sim_robot, nullptr);

  // Arrange: set flag and callback that tracks if called.
  bool callback_called = false;
  projectairsim::Wrench received_wrench;
  auto callback = [&](const projectairsim::Wrench& wrench) {
    callback_called = true;
    received_wrench = wrench;
  };

  auto body = std::make_shared<projectairsim::UnrealPhysicsBody>(*sim_robot);
  // Act: register callback.
  body->SetCallbackSetExternalWrench(callback);

  // Act: calculate external wrench which should trigger callback.
  body->CalculateExternalWrench();

  // Assert: callback was invoked.
  EXPECT_TRUE(callback_called);

  // Assert: wrench passed to callback is valid.
  EXPECT_TRUE(std::isfinite(received_wrench.force.x()));
  EXPECT_TRUE(std::isfinite(received_wrench.force.y()));
  EXPECT_TRUE(std::isfinite(received_wrench.force.z()));
  EXPECT_TRUE(std::isfinite(received_wrench.torque.x()));
  EXPECT_TRUE(std::isfinite(received_wrench.torque.y()));
  EXPECT_TRUE(std::isfinite(received_wrench.torque.z()));
}

// -----------------------------------------------------------------------------
// class UnrealPhysicsModel

TEST(UnrealPhysicsModel, Constructor) {
  // General description:
  // Ensures UnrealPhysicsModel default construction succeeds and returns a
  // valid object instance.
  // Arrange: prepare context for default constructor.
  // Act: allocate UnrealPhysicsModel with std::make_unique.
  auto model = std::make_unique<projectairsim::UnrealPhysicsModel>();
  // Assert: allocation produced a non-null pointer.
  EXPECT_NE(model, nullptr);
}

TEST(UnrealPhysicsModel, SetWrenchesOnPhysicsBody) {
  // General description:
  // Verifies SetWrenchesOnPhysicsBody correctly dispatches to
  // UnrealPhysicsBody::CalculateExternalWrench when body is correct type.
  // Arrange: create robot and UnrealPhysicsBody.
  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();
  auto actors = scene.GetActors();

  projectairsim::Robot* sim_robot = nullptr;
  for (auto actor_ref : actors) {
    if (actor_ref.get().GetType() == projectairsim::ActorType::kRobot) {
      sim_robot = &dynamic_cast<projectairsim::Robot&>(actor_ref.get());
      break;
    }
  }
  EXPECT_NE(sim_robot, nullptr);

  // Arrange: create model and body.
  auto model = std::make_shared<projectairsim::UnrealPhysicsModel>();
  auto body = std::make_shared<projectairsim::UnrealPhysicsBody>(*sim_robot);
  std::shared_ptr<projectairsim::BasePhysicsBody> base_body =
      std::static_pointer_cast<projectairsim::BasePhysicsBody>(body);

  // Arrange: set callback to verify CalculateExternalWrench was called.
  bool callback_invoked = false;
  auto callback = [&](const projectairsim::Wrench&) {
    callback_invoked = true;
  };
  body->SetCallbackSetExternalWrench(callback);

  // Act: invoke SetWrenchesOnPhysicsBody on model.
  model->SetWrenchesOnPhysicsBody(base_body);

  // Assert: callback was triggered, indicating wrench was calculated.
  EXPECT_TRUE(callback_invoked);
}
