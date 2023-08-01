// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/CollisionSensor.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Actor/ActorRegistry.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameInstance.h"
#include "Carla/Game/CarlaGameModeBase.h"
#include "Carla/Sensor/FaultyRayCastLidar.h"

ACollisionSensor::ACollisionSensor(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = false;
}

FActorDefinition ACollisionSensor::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeGenericSensorDefinition(
      TEXT("other"),
      TEXT("collision"));
}

void ACollisionSensor::SetOwner(AActor *NewOwner)
{
  Super::SetOwner(NewOwner);

  /// @todo Deregister previous owner if there was one.

  if (NewOwner != nullptr)
  {
    NewOwner->OnActorHit.AddDynamic(this, &ACollisionSensor::OnCollisionEvent);
  }
}

void ACollisionSensor::OnCollisionEvent(
    AActor *Actor,
    AActor *OtherActor,
    FVector NormalImpulse,
    const FHitResult &Hit)
{
  if ((Actor != nullptr) && (OtherActor != nullptr))
  {
    const auto &Episode = GetEpisode();
    constexpr float TO_METERS = 1e-2;
    NormalImpulse *= TO_METERS;
    GetDataStream(*this).Send(
        *this,
        Episode.SerializeActor(Actor),
        Episode.SerializeActor(OtherActor),
        carla::geom::Vector3D{NormalImpulse.X, NormalImpulse.Y, NormalImpulse.Z});
    // record the collision event
    if (Episode.GetRecorder()->IsEnabled())
      Episode.GetRecorder()->AddCollision(Actor, OtherActor);

    auto CarlaActor = Episode.FindCarlaActor(Actor);
    auto children = CarlaActor->GetChildren();
    for (auto child : children) {
        auto id = (FCarlaActor::IdType)(child);
        if (Episode.GetActorRegistry().Contains(id)) {
            UCarlaEpisode& ptr = const_cast<UCarlaEpisode&>(Episode);
            auto childActor = ptr.FindCarlaActor(id);
            AActor* childAActor = childActor->GetActor();
            if (childAActor->GetClass() == AFaultyRadar::StaticClass()) 
            {
                AFaultyRadar* radar = reinterpret_cast<AFaultyRadar*>(childAActor);
                radar->SetActorRotation(NormalImpulse.Rotation() * 0.05f);
            }
            if (childAActor->GetClass() == AFaultyRayCastLidar::StaticClass())
            {
                AFaultyRayCastLidar* lidar = reinterpret_cast<AFaultyRayCastLidar*>(childAActor);
                lidar->SetActorRotation(NormalImpulse.Rotation() * 0.05f);
            }
        }
    }
  }
}
