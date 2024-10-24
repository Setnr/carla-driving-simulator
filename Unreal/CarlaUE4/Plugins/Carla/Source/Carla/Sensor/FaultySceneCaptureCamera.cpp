// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"

#include "Carla/Sensor/FaultySceneCaptureCamera.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

FActorDefinition AFaultySceneCaptureCamera::GetSensorDefinition()
{
    constexpr bool bEnableModifyingPostProcessEffects = true;
    FActorDefinition def = UActorBlueprintFunctionLibrary::MakeCameraDefinition(TEXT("rgb"),bEnableModifyingPostProcessEffects);

    def.Id = def.Id.Replace(TEXT("rgb"), TEXT("faulty_rgb"));
    def.Tags = def.Tags.Replace(TEXT("rgb"), TEXT("faulty_rgb"));

    FActorVariation Scenario;
    Scenario.Id = TEXT("scenario");
    Scenario.Type = EActorAttributeType::Int;
    Scenario.RecommendedValues = { TEXT("0") };
    Scenario.bRestrictToRecommended = false;
    def.Variations.Append({ Scenario });

    def.Variations.Append(PackageLoss::CreateFailureDefinition());
    def.Variations.Append(ShaderError::CreateFailureDefinition());
    return def;
}

AFaultySceneCaptureCamera::AFaultySceneCaptureCamera(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    AddPostProcessingMaterial(TEXT("Material'/Carla/PostProcessingMaterials/FailureBlackScreen.FailureBlackScreen'"));
    _ShaderError.SetShaderIndex(MaterialsFound.Num()-1);
}
void AFaultySceneCaptureCamera::Set(const FActorDescription& Description) 
{
    Super::Set(Description);
    _PackageLoss.Set(Description, GetWorld()->GetTimeSeconds());
    _ShaderError.Set(Description, GetWorld()->GetTimeSeconds());
    if (Description.Variations.Contains("Scenario"))
    {
        SensorFailure::ScenarioID Scenario = static_cast<SensorFailure::ScenarioID>(UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt("Scenario", Description.Variations, 0.0f));
        switch (Scenario)
        {
        case SensorFailure::ScenarioID::PackageLoss:
            _PackageLoss.UpdateScenario();
            break;
        case SensorFailure::ScenarioID::ShaderError:
            _ShaderError.UpdateScenario();
            break;
        }
    }
}

void AFaultySceneCaptureCamera::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ASceneCaptureCamera::PostPhysTick);
    ENQUEUE_RENDER_COMMAND(MeasureTime)
        (
            [](auto& InRHICmdList)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> Time =
                    std::chrono::high_resolution_clock::now();
                auto Duration = std::chrono::duration_cast<std::chrono::milliseconds>(Time.time_since_epoch());
                uint64_t Milliseconds = Duration.count();
                FString ProfilerText = FString("(Render)Frame: ") + FString::FromInt(FCarlaEngine::GetFrameCounter()) +
                    FString(" Time: ") + FString::FromInt(Milliseconds);
                TRACE_CPUPROFILER_EVENT_SCOPE_TEXT(*ProfilerText);
            }
            );
    if (_ShaderError.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        _ShaderError.UpdateShader(Shaders);
    if (_PackageLoss.IsScenarioActive(GetWorld()->GetTimeSeconds()))
        return;
    FPixelReader::SendPixelsInRenderThread<ASceneCaptureCamera, FColor>(*this);
}