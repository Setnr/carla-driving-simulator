#pragma once
// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/RayCastLidar.h"
#include "Carla/Sensor/FaultySensor.h"


#include "FaultyRayCastLidar.generated.h"

/// A ray-cast based Lidar sensor.
UCLASS()
class CARLA_API AFaultyRayCastLidar : public ARayCastLidar
{
    GENERATED_BODY()
public:
    static FActorDefinition GetSensorDefinition();
    void Set(const FActorDescription& Description) override;
protected:
    virtual void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime) override;
private:
    PackageLoss _PackageLoss;
    PackageDelay<carla::sensor::data::LidarData> _PackageDelay;
    ShiftSensor _ShiftSensor;
    CoordinatenPointDataShift<carla::sensor::data::LidarData> _CoordinatenPointDataShift;
    AdditionalPointDataShift<carla::sensor::data::LidarData> _AdditionalPointDataShift;
    RangeReduction _RangeReduction;
    Blockage _Blockage;
    RandomPoints _RandomPoints;
};
