// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/SceneCaptureCamera.h"
#include "Carla/Sensor/FaultySensor.h"

#include "FaultySceneCaptureCamera.generated.h"

/// A sensor that captures images from the scene.
UCLASS()
class CARLA_API AFaultySceneCaptureCamera : public ASceneCaptureCamera
{
  GENERATED_BODY()

public:
	AFaultySceneCaptureCamera(const FObjectInitializer& ObjectInitializer);
	static FActorDefinition GetSensorDefinition();
	void Set(const FActorDescription& Description) override;
protected:
	void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds) override;
  
private:
	PackageLoss _PackageLoss;
	ShaderError _ShaderError;
};
