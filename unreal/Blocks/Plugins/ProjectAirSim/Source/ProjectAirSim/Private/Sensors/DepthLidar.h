// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#pragma once

#include <vector>

#include "CoreMinimal.h"
#include "UnrealSensor.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "core_sim/clock.hpp"
#include "core_sim/sensors/lidar.hpp"
#include "core_sim/transforms/transform_utils.hpp"

#include "DepthLidar.generated.h"

// Lidar implementation based on an Unreal depth camera.
// The component simulates horizontal sweeping by accumulating samples per tick
// and uses a depth scene capture to reconstruct returns for each vertical channel.
UCLASS() class UDepthLidar : public UUnrealSensor {
  GENERATED_BODY()

 public:
  explicit UDepthLidar(const FObjectInitializer& ObjectInitializer);

  // Binds the Unreal component to the core lidar object and runs initial setup.
  void Initialize(const microsoft::projectairsim::Lidar& SimLidar);

  // Advances the sweep using simulation time and publishes the ready sweep snapshot.
  void TickComponent(float DeltaTime, ELevelTick TickType,
                     FActorComponentTickFunction* ThisTickFunction) override;

  // Copies lidar parameters and derives per-channel angles and the base resolution.
  void SetupLidarFromSettings(
      const microsoft::projectairsim::LidarSettings& LidarSettings);

 protected:
  void BeginPlay() override;
  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

 private:
  static bool IsAngleInRange(float AngleDeg, float StartAngleDeg,
                             float EndAngleDeg);

  // Applies the configured NED origin to the component's relative Unreal transform.
  void InitializePose();
  // Creates the capture camera and floating-point render target that stores planar depth.
  void SetupDepthCapture();
  // Runs one lidar tick: captures depth, reconstructs rays, and accumulates returns.
  void Simulate(float SimTimeDeltaSec);
  // Adjusts yaw, FOV, and capture size to cover the current horizontal slice.
  void UpdateCaptureConfiguration(float HorizontalFovDeg, float YawDeg,
                                  int32 SliceSamples);
  // Reads the render target back from the render thread into CPU-visible pixels.
  void ReadDepthPixels(TArray<FFloat16Color>& OutDepthPixels) const;
  // Diagnostic helper for inspecting the value ranges returned by the capture.
  void LogDepthPixels(const TArray<FFloat16Color>& DepthPixels,
                      float SliceAngleDeg) const;
  // Appends a valid hit to the buffers accumulated for the current sweep.
  void AppendReturnPoint(const FVector& PointSensorFrame, float AzimuthDeg,
                         float ElevationDeg, int LaserIndex);
  // Appends a no-return sample when the configured output needs to preserve it.
  void AppendNoReturnPoint(float AzimuthDeg, float ElevationDeg, int LaserIndex);

 private:
  microsoft::projectairsim::Lidar Lidar;
  microsoft::projectairsim::LidarSettings Settings;

  // If the horizontal FOV covers 360 degrees, angular range clipping is skipped.
  bool bUseFullHorizontalFov = false;
  // Azimuth cursor where the next partial simulation will begin.
  float CurrentHorizontalAngleDeg = 0.0f;
  // Tracks how many degrees of the current sweep have been accumulated so far.
  float AccumulatedSweepAngleDeg = 0.0f;
  TimeNano LastSimTime = 0;
  mutable TimeNano LastDepthDebugLogSimTime = 0;
  bool bHasPendingLidarMsg = false;

  // Buffers ready to publish on this frame if `Simulate` produced new data.
  std::vector<float> PointCloud;
  std::vector<float> AzimuthElevationRangeCloud;
  std::vector<int> SegmentationCloud;
  std::vector<float> IntensityCloud;
  std::vector<int> LaserIndexCloud;
  // Working buffers that accumulate samples until a full revolution completes.
  std::vector<float> SweepPointCloud;
  std::vector<float> SweepAzimuthElevationRangeCloud;
  std::vector<int> SweepSegmentationCloud;
  std::vector<float> SweepIntensityCloud;
  std::vector<int> SweepLaserIndexCloud;
  // One vertical angle per logical lidar channel, derived from the configured vertical FOV.
  std::vector<float> ChannelVerticalAnglesDeg;

  // Active render target dimensions for the slice currently being captured.
  uint32 CurrentCaptureWidth = 0;
  uint32 CurrentCaptureHeight = 0;

  UPROPERTY() UMaterial* DepthPlanarMaterialStatic;
  UPROPERTY() USceneCaptureComponent2D* DepthCaptureComponent = nullptr;
  UPROPERTY() UTextureRenderTarget2D* DepthRenderTarget = nullptr;
};