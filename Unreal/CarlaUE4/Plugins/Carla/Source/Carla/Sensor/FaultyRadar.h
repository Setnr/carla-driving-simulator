// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/Radar.h"

#include "Carla/Sensor/FaultySensor.h"

#include "Carla/Actor/ActorDefinition.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/RadarData.h>
#include <compiler/enable-ue4-macros.h>

#include "FaultyRadar.generated.h"

/// A ray-cast based Radar sensor.
UCLASS()
class CARLA_API AFaultyRadar : public ARadar
{
    GENERATED_BODY()
public:
    static FActorDefinition GetSensorDefinition();
    void Set(const FActorDescription& Description) override;
protected:
    virtual void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime) override;
private:
    PackageLoss _PackageLoss;
    PackageDelay<carla::sensor::data::RadarData> _PackageDelay;
    ShiftSensor _ShiftSensor;
    CoordinatenPointDataShift<carla::sensor::data::RadarData> _CoordinatenPointDataShift;
    AdditionalPointDataShift<carla::sensor::data::RadarData> _AdditionalPointDataShift;
    RangeReduction _RangeReduction;
    Blockage _Blockage;
    RandomPoints _RandomPoints;
};


