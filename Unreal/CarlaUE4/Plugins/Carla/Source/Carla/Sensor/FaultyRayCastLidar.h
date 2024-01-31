// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/FaultyLidarDescription.h"
#include "Carla/Sensor/RayCastLidar.h"
#include "FaultyRayCastLidar.generated.h"

/// A ray-cast based Lidar sensor.
UCLASS()
class CARLA_API  AFaultyRayCastLidar : public ARayCastLidar
{
  GENERATED_BODY()
 

  using FLidarData = carla::sensor::data::LidarData;
  using FDetection = carla::sensor::data::LidarDetection;
  using Distribution = FFaultyLidarDescription::Distribution;
  using VerticalFOV_Type = FFaultyLidarDescription::VerticalFOV_Type;
  using HorizontalFOV_Type = FFaultyLidarDescription::HorizontalFOV_Type;
  using SensorBlockage_ObjectLifeTime = FFaultyLidarDescription::SensorBlockage_ObjectLifeTime;
  using SensorBlockage_BlockageType = FFaultyLidarDescription::SensorBlockage_BlockageType;
  using ScenarioID = FFaultyLidarDescription::ScenarioID;
  using SensorShift_Flag = FFaultyLidarDescription::SensorShift_Flag;
  using SensorShift_TriggerFlag = FFaultyLidarDescription::SensorShift_TriggerFlag;
public:
  static FActorDefinition GetSensorDefinition();

  AFaultyRayCastLidar(const FObjectInitializer &ObjectInitializer);
  virtual void Set(const FActorDescription &Description) override;

  virtual void ComputeAndSaveDetections(const FTransform& SensorTransform) override;
  void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime) override;

protected:
	FFaultyLidarDescription FaultyLidarDescription;
	void CreatePoints();
	FVector CalculateEndLocation(bool CloseRange, VerticalFOV_Type VertFOV, HorizontalFOV_Type HorzFOV);
	void CreateBlockage();
	void DetectNonExisitingPoints();
	void CheckRangeReduction();
	void ShiftDetectionPoints();
	float CreateRandomNumber(Distribution DistType);
	bool HasToLooseCurrentPackage();
	void Destroyed() override;
	void GenerateHexagons();
	void MoveLidar(FRotator rot);
	void ShiftSensor();
	void MoveLidar();
	void EventShift();

	TArray<AActor*> BlockObjects;

	bool MoveOnce = true;

	std::mt19937 gen_weibull;
	std::weibull_distribution<float> weibull;
	std::mt19937 gen_uniform;
	std::uniform_real_distribution<float> uniform;
};
