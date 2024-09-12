// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/SceneCaptureCamera.h"
#include "Carla/Game/CarlaEngine.h"
#include <chrono>

#include "Runtime/RenderCore/Public/RenderingThread.h"
#include "Materials/MaterialInstanceDynamic.h"

FActorDefinition ASceneCaptureCamera::GetSensorDefinition()
{
    constexpr bool bEnableModifyingPostProcessEffects = true;
    return UActorBlueprintFunctionLibrary::MakeCameraDefinition(
        TEXT("rgb"),
        bEnableModifyingPostProcessEffects);
}

ASceneCaptureCamera::ASceneCaptureCamera(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    AddPostProcessingMaterial(
        TEXT("Material'/Carla/PostProcessingMaterials/PhysicLensDistortion.PhysicLensDistortion'"));
    AddPostProcessingMaterial(TEXT("Material'/Game/NewMaterial.NewMaterial'"));
    /*for (const auto& Shader : Shaders)
    {
        if (Shader.Name.Contains("NewMaterial"))
        {
            FLinearColor col;
            Shader.PostProcessMaterial->GetVectorParameterValue(FName("MyColor"), col);
            ColR = col.R;
            ColG = col.G;
            ColB = col.B;
        }
    }*/
}

void ASceneCaptureCamera::BeginPlay()
{
  Super::BeginPlay();
}

void ASceneCaptureCamera::OnFirstClientConnected()
{
}

void ASceneCaptureCamera::OnLastClientDisconnected()
{
}

void ASceneCaptureCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
}

void ASceneCaptureCamera::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ASceneCaptureCamera::PostPhysTick);
  ENQUEUE_RENDER_COMMAND(MeasureTime)
  (
    [](auto &InRHICmdList)
    {
      std::chrono::time_point<std::chrono::high_resolution_clock> Time = 
          std::chrono::high_resolution_clock::now();
      auto Duration = std::chrono::duration_cast< std::chrono::milliseconds >(Time.time_since_epoch());
      uint64_t Milliseconds = Duration.count();
      FString ProfilerText = FString("(Render)Frame: ") + FString::FromInt(FCarlaEngine::GetFrameCounter()) + 
          FString(" Time: ") + FString::FromInt(Milliseconds);
      TRACE_CPUPROFILER_EVENT_SCOPE_TEXT(*ProfilerText);
    }
  );
  FPixelReader::SendPixelsInRenderThread<ASceneCaptureCamera, FColor>(*this);
}

void ASceneCaptureCamera::SendGBufferTextures(FGBufferRequest& GBuffer)
{
    SendGBufferTexturesInternal(*this, GBuffer);
}
/*
void ASceneCaptureCamera::UpdateOpacity(float opacity)
{
    for (const auto& Shader : Shaders)
    {
        if (Shader.Name.Contains("NewMaterial"))
        {
            Shader.PostProcessMaterial->SetScalarParameterValue(FName("Opacity"), opacity);
        }
    }
}

void ASceneCaptureCamera::UpdateColorR(float Col) 
{
    ColR = Col;
    FLinearColor ColAll(ColR, ColG, ColB);
    for (const auto& Shader : Shaders)
    {
        if (Shader.Name.Contains("NewMaterial"))
        {
            Shader.PostProcessMaterial->SetVectorParameterValue(FName("MyColor"), ColAll);
        }

    }
}

void ASceneCaptureCamera::UpdateColorG(float Col)
{
    ColG = Col;
    FLinearColor ColAll(ColR, ColG, ColB);
    for (const auto& Shader : Shaders)
    {
        if (Shader.Name.Contains("NewMaterial"))
        {
            Shader.PostProcessMaterial->SetVectorParameterValue(FName("MyColor"), ColAll);
        }

    }
}

void ASceneCaptureCamera::UpdateColorB(float Col)
{
    ColB = Col;
    FLinearColor ColAll(ColR, ColG, ColB);
    for (const auto& Shader : Shaders)
    {
        if (Shader.Name.Contains("NewMaterial"))
        {
            Shader.PostProcessMaterial->SetVectorParameterValue(FName("MyColor"), ColAll);
        }

    }
}
*/