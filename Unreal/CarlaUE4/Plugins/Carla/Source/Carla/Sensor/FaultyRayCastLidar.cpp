// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/FaultyRayCastLidar.h"

FActorDefinition AFaultyRayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeFaultyLidarDefinition(TEXT("ray_cast"));
}


AFaultyRayCastLidar::AFaultyRayCastLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {
}

void AFaultyRayCastLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetFaultyLidar(ActorDescription, LidarDescription, FaultyLidarDescription);
  Super::Set(LidarDescription);

  FaultyLidarDescription.PackageLoss_Start = FaultyLidarDescription.PackageLoss_StartOffset + GetWorld()->GetTimeSeconds();
  FaultyLidarDescription.ConstantShift_Start = FaultyLidarDescription.ConstantShift_StartOffset + GetWorld()->GetTimeSeconds();
}

void AFaultyRayCastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) 
{
    float time = GetWorld()->GetTimeSeconds();
    if (FaultyLidarDescription.Scenario & FFaultyLidarDescription::ScenarioID::RadarPackageLoss)
    {

        if (time >= FaultyLidarDescription.PackageLoss_Start)
        {
            if (time >= FaultyLidarDescription.PackageLoss_Start + FaultyLidarDescription.PackageLoss_Duration)
            {
                FaultyLidarDescription.PackageLoss_Start += FaultyLidarDescription.PackageLoss_Interval;
                FaultyLidarDescription.PackageLoss_Interval -= FaultyLidarDescription.PackageLoss_IntervallDegradation;
            }
            return;
        }
    }

	Super::ComputeAndSaveDetections(SensorTransform);

    if (FaultyLidarDescription.Scenario & FFaultyLidarDescription::ScenarioID::RadarConstantShift)
    {
        if (time >= FaultyLidarDescription.ConstantShift_Start)
        {
            FaultyLidarDescription.ConstantShift_Start += FaultyLidarDescription.ConstantShift_Interval;
            MoveLidar(FaultyLidarDescription.ConstantShift_Rotation);
        }
    }


    if (FaultyLidarDescription.Scenario & FFaultyLidarDescription::ScenarioID::RadarRandomShift)
    {
        if (FaultyLidarDescription.RandomShift_Time == 0.0f)
        {
            FaultyLidarDescription.RandomShift_Time = FMath::RandRange(FaultyLidarDescription.RandomShift_Start, FaultyLidarDescription.RandomShift_End);
        }
        if (time >= FaultyLidarDescription.RandomShif_StartOffset + FaultyLidarDescription.RandomShift_Time)
        {
            FaultyLidarDescription.RandomShif_StartOffset = time;
            FRotator rotator(0.0f, FMath::FRandRange(-1.5f, 1.5f), 0.0f); //We assume that we will always shift horizontaly
            if (FMath::FRand() < 0.1f) //We give the radar chance to shift verticaly aswell
                rotator.Pitch = FMath::FRandRange(-1.5f, 1.5f);
            MoveLidar(rotator);
            FaultyLidarDescription.RandomShift_Time = FMath::RandRange(FaultyLidarDescription.RandomShift_Start, FaultyLidarDescription.RandomShift_End);
        }
    }
}


