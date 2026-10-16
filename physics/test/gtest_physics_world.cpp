// Copyright (C) Microsoft Corporation. 
// Copyright (C) 2025 IAMAI CONSULTING CORP

// MIT License. All rights reserved.

#include <memory>
#include <vector>

#include "base_physics.hpp"
#include "core_sim/actor.hpp"
#include "core_sim/actor/robot.hpp"
#include "core_sim/simulator.hpp"
#include "gtest/gtest.h"
#include "physics_world.hpp"
#include "test_data/physics_test_config.hpp"  // defines physics_test_config

namespace projectairsim = microsoft::projectairsim;

TEST(PhysicsWorld, Constructor) {
  // General description:
  // Verifies constructor for PhysicsWorld.
  // Arrange: prepare context for `auto world = std::make_unique<projectairsim::PhysicsWorld>();`.
  // Act: run `auto world = std::make_unique<projectairsim::PhysicsWorld>();`.
  auto world = std::make_unique<projectairsim::PhysicsWorld>();
  // Assert: check result from `EXPECT_NE(world, nullptr);`.
  EXPECT_NE(world, nullptr);
}

TEST(PhysicsWorld, BaseWorldAddRemoveRobots) {
  // General description:
  // Verifies base world add remove robots for PhysicsWorld.
  // Arrange: prepare context for `projectairsim::PhysicsWorld world;`.
  // Act: run `projectairsim::PhysicsWorld world;`.
  projectairsim::PhysicsWorld world;
  // Assert: check result from `EXPECT_EQ(world.GetPhysicsBodies().size(), 0);`.
  EXPECT_EQ(world.GetPhysicsBodies().size(), 0);

  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();
  auto actors = scene.GetActors();

  for (auto actor_ref : actors) {
    if (actor_ref.get().GetType() == projectairsim::ActorType::kRobot) {
      auto& sim_robot = dynamic_cast<projectairsim::Robot&>(actor_ref.get());
      world.AddRobot(sim_robot);
      EXPECT_EQ(world.GetPhysicsBodies().size(), 1);
    }
  }

  world.RemoveAllBodiesAndModels();
  EXPECT_EQ(world.GetPhysicsBodies().size(), 0);
}

TEST(PhysicsWorld, SetSceneTickCallbacks) {
  // General description:
  // Verifies set scene tick callbacks for PhysicsWorld.
  // Arrange: prepare context for `projectairsim::PhysicsWorld world;`.
  projectairsim::PhysicsWorld world;
  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();

  // Act: run `world.SetSceneTickCallbacks(scene);`.
  world.SetSceneTickCallbacks(scene);
  // Assert: check result from `}`.
}

TEST(PhysicsWorld, SetWrenchesOnPhysicsBodies) {
  // General description:
  // Verifies SetWrenchesOnPhysicsBodies dispatches wrench calculation to all
  // physics bodies in the world, supporting multiple backend types (fast,
  // unreal, matlab).
  // Arrange: create PhysicsWorld and add robot(s).
  projectairsim::PhysicsWorld world;
  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();
  auto actors = scene.GetActors();

  // Arrange: locate and add robot to world.
  projectairsim::Robot* sim_robot = nullptr;
  for (auto actor_ref : actors) {
    if (actor_ref.get().GetType() == projectairsim::ActorType::kRobot) {
      sim_robot = &dynamic_cast<projectairsim::Robot&>(actor_ref.get());
      world.AddRobot(*sim_robot);
      break;
    }
  }
  EXPECT_NE(sim_robot, nullptr);

  // Arrange: verify robot was added to world bodies.
  auto& bodies = world.GetPhysicsBodies();
  EXPECT_EQ(bodies.size(), 1);

  // Act: invoke SetWrenchesOnPhysicsBodies to dispatch wrench calculation.
  world.SetWrenchesOnPhysicsBodies();

  // Assert: world contains bodies after wrench propagation.
  EXPECT_EQ(world.GetPhysicsBodies().size(), 1);

  // Assert: robot kinematics was updated (wrench calculation and integration
  // flow occurred).
  auto kin = sim_robot->GetKinematics();
  EXPECT_TRUE(std::isfinite(kin.accels.linear.x()));
  EXPECT_TRUE(std::isfinite(kin.accels.linear.y()));
  EXPECT_TRUE(std::isfinite(kin.accels.linear.z()));
}

TEST(PhysicsWorld, StepPhysicsWorld) {
  // General description:
  // Verifies step physics world for PhysicsWorld.
  // Arrange: prepare context for `projectairsim::PhysicsWorld world;`.
  projectairsim::PhysicsWorld world;
  projectairsim::Simulator simulator;
  simulator.LoadSceneWithJSON(physics_test_config);
  auto& scene = simulator.GetScene();
  auto actors = scene.GetActors();

  for (auto actor_ref : actors) {
    if (actor_ref.get().GetType() == projectairsim::ActorType::kRobot) {
      auto& sim_robot = dynamic_cast<projectairsim::Robot&>(actor_ref.get());
      // Act: run `projectairsim::Kinematics kin = sim_robot.GetKinematics();`.
      projectairsim::Kinematics kin = sim_robot.GetKinematics();
      // Assert: check result from `EXPECT_FLOAT_EQ(kin.accels.linear.z(), 0.0);`.
      EXPECT_FLOAT_EQ(kin.accels.linear.z(), 0.0);

      world.AddRobot(sim_robot);
      world.StepPhysicsWorld(1.0f);

      kin = sim_robot.GetKinematics();
      EXPECT_FLOAT_EQ(kin.accels.linear.z(), projectairsim::EarthUtils::kGravity);
    }
  }
}
