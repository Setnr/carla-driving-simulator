// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/FaultyRayCastLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

FActorDefinition AFaultyRayCastLidar::GetSensorDefinition()
{
    FActorDefinition def =  UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));

    def.Id = def.Id.Replace(TEXT("lidar"), TEXT("faulty_lidar"));
    def.Tags = def.Tags.Replace(TEXT("lidar"), TEXT("faulty_lidar"));

    FActorVariation Scenario;
    Scenario.Id = TEXT("scenario");
    Scenario.Type = EActorAttributeType::Int;
    Scenario.RecommendedValues = { TEXT("0") };
    Scenario.bRestrictToRecommended = false;
    def.Variations.Append({ Scenario });

    def.Variations.Append(PackageLoss::CreateFailureDefinition());
    def.Variations.Append(PackageDelay<carla::sensor::data::LidarData>::CreateFailureDefinition());
    def.Variations.Append(ShiftSensor::CreateFailureDefinition());
    def.Variations.Append(CoordinatenPointDataShift<carla::sensor::data::LidarData>::CreateFailureDefinition());
    def.Variations.Append(AdditionalPointDataShift<carla::sensor::data::LidarData>::CreateFailureDefinition());
    def.Variations.Append(RangeReduction::CreateFailureDefinition());
    def.Variations.Append(Blockage::CreateFailureDefinition());
    def.Variations.Append(RandomPoints::CreateFailureDefinition());

    return def;
}

void AFaultyRayCastLidar::Set(const FActorDescription& ActorDescription)
{
    Super::Set(ActorDescription);
    _PackageLoss.Set(ActorDescription, GetWorld()->GetTimeSeconds());
    _PackageDelay.Set(ActorDescription, GetWorld()->GetTimeSeconds());
    _ShiftSensor.Set(ActorDescription, GetWorld()->GetTimeSeconds());
    _CoordinatenPointDataShift.Set(ActorDescription, GetWorld()->GetTimeSeconds());
    _AdditionalPointDataShift.Set(ActorDescription, GetWorld()->GetTimeSeconds());
    _RangeReduction.Set(ActorDescription, GetWorld()->GetTimeSeconds());
    _Blockage.Set(ActorDescription, GetWorld()->GetTimeSeconds());
    _RandomPoints.Set(ActorDescription, GetWorld()->GetTimeSeconds());
    if (ActorDescription.Variations.Contains("Scenario"))
    {
        SensorFailure::ScenarioID Scenario = static_cast<SensorFailure::ScenarioID>(UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt("Scenario", ActorDescription.Variations, 0.0f));
        switch (Scenario)
        {
        case SensorFailure::ScenarioID::PackageLoss:
            _PackageLoss.UpdateScenario();
            break;
        case SensorFailure::PackageDelay:
            _PackageDelay.UpdateScenario();
            break;
        case SensorFailure::SensorShift:
            _ShiftSensor.UpdateScenario();
            break;
        case SensorFailure::DetectionPointShift:
            _CoordinatenPointDataShift.UpdateScenario();
            break;
        case SensorFailure::VelocityShift:
            _AdditionalPointDataShift.UpdateScenario();
            break;
        case SensorFailure::RangeReduction:
            _RangeReduction.UpdateScenario();
            break;
        case SensorFailure::SensorBlockage:
            _Blockage.UpdateScenario();
            break;
        case SensorFailure::DetectNonExistingPoints:
            _RandomPoints.UpdateScenario();
            break;
        }
    }
}



void AFaultyRayCastLidar::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidar::PostPhysTick);
    float VerticalFOV = abs(Description.LowerFovLimit) < abs(Description.UpperFovLimit) ? abs(Description.UpperFovLimit) : abs(Description.LowerFovLimit);

    if (_Blockage.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _Blockage.CreateBlockage(Description.HorizontalFov, VerticalFOV, Description.Range, this, this->GetActorLocation(), this->GetActorTransform(), GetWorld());

    SimulateLidar(DeltaTime);

    if (_PackageLoss.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        return;

    if (_PackageDelay.IsScenarioActive(GetWorld()->GetTimeSeconds()))
    {
        _PackageDelay.Delay(LidarData);
        return;
    }
    else
    {
        _PackageDelay.Delay(LidarData);
        _PackageDelay.GetDelayedPackage(LidarData);
    }

    if (_ShiftSensor.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _ShiftSensor.UpdateSensor(this);

    if (_CoordinatenPointDataShift.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _CoordinatenPointDataShift.UpdateData(LidarData);

    if (_AdditionalPointDataShift.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _AdditionalPointDataShift.UpdateData(LidarData);
    if (_RangeReduction.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _RangeReduction.UpdateRange(Description.Range);

    if (_RandomPoints.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _RandomPoints.CreatePoints<carla::sensor::data::LidarData>(LidarData, Description.HorizontalFov, VerticalFOV, Description.Range, this->GetActorLocation(), this->GetActorTransform());


    auto DataStream = GetDataStream(*this);
    auto SensorTransform = DataStream.GetSensorTransform();

    {
        TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
        DataStream.SerializeAndSend(*this, LidarData, DataStream.PopBufferFromPool());
    }
    // ROS2
#if defined(WITH_ROS2)
    auto ROS2 = carla::ros2::ROS2::GetInstance();
    if (ROS2->IsEnabled())
    {
        TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
        auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
        AActor* ParentActor = GetAttachParentActor();
        if (ParentActor)
        {
            FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
            ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, LidarData, this);
        }
        else
        {
            ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, SensorTransform, LidarData, this);
        }
    }
#endif


}
