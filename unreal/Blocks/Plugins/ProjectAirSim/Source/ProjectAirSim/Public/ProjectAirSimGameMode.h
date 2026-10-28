// Copyright (C) Microsoft Corporation.  
// Copyright (C) 2025 IAMAI CONSULTING CORP
//
// MIT License. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"

//
#include "ProjectAirSimGameMode.generated.h"

// Defined in UnrealSimLoader.h, which pulls in the plugin's private headers.
// Only a pointer is kept here so that other modules can subclass this game
// mode without the plugin's private include paths.
class AUnrealSimLoader;

UCLASS()
class PROJECTAIRSIM_API AProjectAirSimGameMode : public AGameModeBase {
  GENERATED_BODY()

 public:
  explicit AProjectAirSimGameMode(const FObjectInitializer& ObjectInitializer);

  ~AProjectAirSimGameMode() override;

  void StartPlay() override;

  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

 private:
  AUnrealSimLoader* UnrealSimLoader = nullptr;
};
