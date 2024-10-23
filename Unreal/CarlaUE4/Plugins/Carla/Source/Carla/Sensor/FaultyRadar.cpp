#include <PxScene.h>

#include "Carla.h"
#include "Carla/Sensor/FaultyRadar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include <compiler/enable-ue4-macros.h>

FActorDefinition AFaultyRadar::GetSensorDefinition()
{
	FActorDefinition def = UActorBlueprintFunctionLibrary::MakeRadarDefinition();

	def.Id = def.Id.Replace(TEXT("radar"), TEXT("faulty_radar"));
	def.Tags = def.Tags.Replace(TEXT("radar"), TEXT("faulty_radar"));

    FActorVariation Scenario;
    Scenario.Id = TEXT("scenario");
    Scenario.Type = EActorAttributeType::Int;
    Scenario.RecommendedValues = { TEXT("0") };
    Scenario.bRestrictToRecommended = false;
    def.Variations.Append({ Scenario });

	def.Variations.Append(PackageLoss::CreateFailureDefinition());
    def.Variations.Append(PackageDelay<carla::sensor::data::RadarData>::CreateFailureDefinition());
    def.Variations.Append(ShiftSensor::CreateFailureDefinition());
    def.Variations.Append(CoordinatenPointDataShift<carla::sensor::data::RadarData>::CreateFailureDefinition());
    def.Variations.Append(AdditionalPointDataShift<carla::sensor::data::RadarData>::CreateFailureDefinition());
    def.Variations.Append(RangeReduction::CreateFailureDefinition());
    def.Variations.Append(Blockage::CreateFailureDefinition());
    def.Variations.Append(RandomPoints::CreateFailureDefinition());

	return def;
}

void AFaultyRadar::Set(const FActorDescription& ActorDescription)
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

void AFaultyRadar::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(AFaultyRadar::PostPhysTick);
    CalculateCurrentVelocity(DeltaTime);

    RadarData.Reset();

    if (_Blockage.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _Blockage.CreateBlockage(HorizontalFOV, VerticalFOV, Range, this, this->GetActorLocation(), this->GetActorTransform(), GetWorld());

    SendLineTraces(DeltaTime);

    if (_PackageLoss.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        return;

    if (_PackageDelay.IsScenarioActive(GetWorld()->GetTimeSeconds()))
    {
        _PackageDelay.Delay(RadarData);
        return;
    }
    else
    {
        _PackageDelay.Delay(RadarData);
        _PackageDelay.GetDelayedPackage(RadarData);
    }

    if (_ShiftSensor.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _ShiftSensor.UpdateSensor(this);

    if (_CoordinatenPointDataShift.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _CoordinatenPointDataShift.UpdateData(RadarData);

    if (_AdditionalPointDataShift.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _AdditionalPointDataShift.UpdateData(RadarData);
    if (_RangeReduction.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _RangeReduction.UpdateRange(Range);

    if (_RandomPoints.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _RandomPoints.CreatePoints<carla::sensor::data::RadarData>(RadarData, HorizontalFOV, VerticalFOV, Range, this->GetActorLocation(), this->GetActorTransform());

    auto DataStream = GetDataStream(*this);

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
            ROS2->ProcessDataFromRadar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, RadarData, this);
        }
        else
        {
            ROS2->ProcessDataFromRadar(DataStream.GetSensorType(), StreamId, DataStream.GetSensorTransform(), RadarData, this);
        }
    }
#endif

    {
        TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
        DataStream.SerializeAndSend(*this, RadarData, DataStream.PopBufferFromPool());
    }
}