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
public:
  static FActorDefinition GetSensorDefinition();

  AFaultyRayCastLidar(const FObjectInitializer &ObjectInitializer);
  virtual void Set(const FActorDescription &Description) override;

  virtual void ComputeAndSaveDetections(const FTransform& SensorTransform) override;


  void MoveLidar()
  {
      if (FaultyLidarDescription.Scenario & FFaultyLidarDescription::ScenarioID::RadarCollosionShift)
      {
          auto rot = this->GetActorRotation();
          rot.Yaw += 10;
          this->SetActorRotation(rot);
      }
  }
  void MoveLidar(FRotator rot)
  {
      auto RadarRot = this->GetActorRotation();
      RadarRot += rot;
      this->SetActorRotation(RadarRot);
  }

protected:
	FFaultyLidarDescription FaultyLidarDescription;

};
