// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/FaultySceneCaptureCamera.h"
#include "Carla/Game/CarlaEngine.h"
#include <chrono>

#include "Runtime/RenderCore/Public/RenderingThread.h"
#include "Materials/MaterialInstanceDynamic.h"
FActorDefinition AFaultySceneCaptureCamera::GetSensorDefinition()
{
    constexpr bool bEnableModifyingPostProcessEffects = true;
    FActorDefinition def = UActorBlueprintFunctionLibrary::MakeCameraDefinition(
        TEXT("faulty_rgb"),
        bEnableModifyingPostProcessEffects);


    auto ScenatioDefinition = FaultySensor::CreateScenarioDefinition();
    auto PackageLossDefinition = FaultySensor::CreatePackageLossDefinition();

    FActorVariation FailureState;
    FailureState.Id = TEXT("CameraFailureState");
    FailureState.Type = EActorAttributeType::Bool;
    FailureState.RecommendedValues = { TEXT("true") };
    FailureState.bRestrictToRecommended = false;


    def.Variations.Append(ScenatioDefinition);
    def.Variations.Append(PackageLossDefinition);
    def.Variations.Append({FailureState});
    UActorBlueprintFunctionLibrary::CheckActorDefinition(def);
    return def;
}

AFaultySceneCaptureCamera::AFaultySceneCaptureCamera(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    col = FLinearColor(0.f, 0.f, 0.f, 0.f);
    this->Loss_Interval = 15.0f;
    this->Loss_Duration = 2.5f;
    this->Loss_Start = FLT_MAX;
    this->Loss_IntervalDegradation = 0.0f;
    this->Loss_DurationDeg = 0.0f;
    AddPostProcessingMaterial(TEXT("Material'/Carla/PostProcessingMaterials/FailureBlackScreen.FailureBlackScreen'"));



}

void AFaultySceneCaptureCamera::BeginPlay()
{
    Super::BeginPlay();
}

void AFaultySceneCaptureCamera::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
    float time = GetWorld()->GetTimeSeconds();
    if (time >= this->Loss_Start)
    {
        bool SwitchedState = false;
        if (!FailInitialized)
        {
            FailInitialized = true;
            SwitchedState = true;

            if(FailureState)
            {
                col.A = 1.f;
            }


        }

        if (time >= this->Loss_Start + this->Loss_Duration)
        {
            this->Loss_Start += this->Loss_Interval;
            this->Loss_Interval -= Loss_IntervalDegradation;
            this->Loss_Duration += Loss_DurationDeg;

            FailInitialized = false;
            SwitchedState = true;

            col.A = 0.0f;

        }
        if (SwitchedState) 
        {
            for (const auto& Shader : Shaders) {
                if (Shader.Name.Contains("FailureBlackScreen"))
                {
                    Shader.PostProcessMaterial->SetVectorParameterValue(FName("MyColor"), col);
                }
            }
        }
        if(!FailureState)
            return;
    }
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
    FPixelReader::SendPixelsInRenderThread<ASceneCaptureCamera, FColor>(*this);
}

void AFaultySceneCaptureCamera::SendGBufferTextures(FGBufferRequest& GBuffer)
{

    SendGBufferTexturesInternal(*this, GBuffer);
}

void AFaultySceneCaptureCamera::Set(const FActorDescription& Description)
{
    Super::Set(Description);
    if (Description.Variations.Contains("PackageLoss_Start"))
        this->Loss_Start = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat("PackageLoss_Start", Description.Variations, 0.0f) + GetWorld()->GetTimeSeconds();
    if (Description.Variations.Contains("PackageLoss_Interval"))
        this->Loss_Interval = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat("PackageLoss_Interval", Description.Variations, 0.0f);
    if (Description.Variations.Contains("PackageLoss_Duration"))
        this->Loss_Duration = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat("PackageLoss_Duration", Description.Variations, 0.0f);
    if (Description.Variations.Contains("PackageLoss_IntervalDegradation"))
        this->Loss_DurationDeg = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat("PackageLoss_IntervalDegradation", Description.Variations, 0.0f);
    if (Description.Variations.Contains("PackageLoss_DurationDegradation"))
        this->Loss_IntervalDegradation = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat("PackageLoss_DurationDegradation", Description.Variations, 0.0f);
    if (Description.Variations.Contains("CameraFailureState"))
        this->FailureState = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToBool("CameraFailureState", Description.Variations,true);


}