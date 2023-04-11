// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "Carla/Sensor/FaultyRadar.h"

FActorDefinition AFaultyRadar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeFaultyRadarDefinition();
}

AFaultyRadar::AFaultyRadar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
    this->LooseContact_Interval = 15.0f;
    this->LooseContact_Duration = 2.5f;
    this->LooseContact_StartOffset = 15.0f;
}

void AFaultyRadar::AddLooseContactInterval(float Interval)
{
    this->LooseContact_Interval = Interval;
}
void AFaultyRadar::AddLooseContactDuration(float Duration)
{
    this->LooseContact_Duration = Duration;
}
void AFaultyRadar::AddLooseContactStart(float StartTime)
{
    this->LooseContact_StartOffset = StartTime;
}

void AFaultyRadar::AddScenario(int ScenarioID)
{
    if (!(this->Scenario & ScenarioID))
        this->Scenario |= ScenarioID;

}

void AFaultyRadar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
}

void AFaultyRadar::MoveRadar()
{
    if(this->Scenario & ScenarioID::RadarShifted)
    {
        auto rot = this->GetActorRotation();
        rot.Yaw += 10;
        this->SetActorRotation(rot);
    }
}

void AFaultyRadar::BeginPlay()
{
  Super::BeginPlay();
  this->LooseContact_Start = this->LooseContact_StartOffset + GetWorld()->GetTimeSeconds();
  PrevLocation = GetActorLocation();
}
void AFaultyRadar::WriteLineTraces()
{
    if (this->Scenario & ScenarioID::RadarLooseContact)
    {
        float time = GetWorld()->GetTimeSeconds();

        if (time >= this->LooseContact_Start)
        {
            if (time >= this->LooseContact_Start + this->LooseContact_Duration)
            {
                this->LooseContact_Start += this->LooseContact_Interval;
            }
            return;
        }
    }


    for (auto& ray : Rays) {
        if (ray.Hitted) {
            RadarData.WriteDetection({
                ray.RelativeVelocity,
                ray.AzimuthAndElevation.X,
                ray.AzimuthAndElevation.Y,
                ray.Distance
                });
        }
    }
}

