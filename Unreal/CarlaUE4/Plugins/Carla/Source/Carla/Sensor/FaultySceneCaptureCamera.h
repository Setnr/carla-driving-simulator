#pragma once
// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/SceneCaptureCamera.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include "FaultySceneCaptureCamera.generated.h"

class FaultySensor
{
public:
    static TArray<FActorVariation> CreateScenarioDefinition()
    {
        FActorVariation Scenario;
        Scenario.Id = TEXT("scenario");
        Scenario.Type = EActorAttributeType::Int;
        Scenario.RecommendedValues = { TEXT("0") };
        Scenario.bRestrictToRecommended = false;

        return { Scenario };
    }
    static TArray<FActorVariation> CreatePackageLossDefinition()
    {
        FActorVariation PackageLoss_Interval;
        PackageLoss_Interval.Id = TEXT("PackageLoss_Interval");
        PackageLoss_Interval.Type = EActorAttributeType::Float;
        PackageLoss_Interval.RecommendedValues = { TEXT("0") };
        PackageLoss_Interval.bRestrictToRecommended = false;

        FActorVariation PackageLoss_Duration;
        PackageLoss_Duration.Id = TEXT("PackageLoss_Duration");
        PackageLoss_Duration.Type = EActorAttributeType::Float;
        PackageLoss_Duration.RecommendedValues = { TEXT("0") };
        PackageLoss_Duration.bRestrictToRecommended = false;

        FActorVariation PackageLoss_Start;
        PackageLoss_Start.Id = TEXT("PackageLoss_Start");
        PackageLoss_Start.Type = EActorAttributeType::Float;
        PackageLoss_Start.RecommendedValues = { TEXT("0") };
        PackageLoss_Start.bRestrictToRecommended = false;

        FActorVariation PackageLoss_IntervalDegradation;
        PackageLoss_IntervalDegradation.Id = TEXT("PackageLoss_IntervalDegradation");
        PackageLoss_IntervalDegradation.Type = EActorAttributeType::Float;
        PackageLoss_IntervalDegradation.RecommendedValues = { TEXT("0") };
        PackageLoss_IntervalDegradation.bRestrictToRecommended = false;

        FActorVariation PackageLoss_DurationDegradation;
        PackageLoss_DurationDegradation.Id = TEXT("PackageLoss_DurationDegradation");
        PackageLoss_DurationDegradation.Type = EActorAttributeType::Float;
        PackageLoss_DurationDegradation.RecommendedValues = { TEXT("0") };
        PackageLoss_DurationDegradation.bRestrictToRecommended = false;


        return { PackageLoss_Interval,PackageLoss_Duration ,PackageLoss_Start ,PackageLoss_IntervalDegradation ,PackageLoss_DurationDegradation };
    }



    /*
    
    ------------- Add Material for Visual Effects for a Camera on a existing Object
    AddPostProcessingMaterial(TEXT("Material'/Game/NewMaterial.NewMaterial'"));

    UE_LOG(LogTemp, Warning, TEXT("Shaders got added"));
    for (const auto& Shader : Shaders) {
        UE_LOG(LogTemp, Warning, TEXT("FOUND MATERIAL %s"), *Shader.Name);
        if (Shader.Name.Contains("NewMaterial"))
        {
            FLinearColor col;
            Shader.PostProcessMaterial->GetVectorParameterValue(FName("MyColor"), col);
            ColR = col.R;
            ColG = col.G;
            ColB = col.B;
        }
    }
    

    -------------   Update Material
    for (const auto& Shader : Shaders)
    {
        UE_LOG(LogTemp, Warning, TEXT("FOUND MATERIAL %s"), *Shader.Name);
        if (Shader.Name.Contains("NewMaterial"))
        {
            UE_LOG(LogTemp, Warning, TEXT("FOUND MATERIAL"));
            FLinearColor col(this->ColR, this->ColG, this->ColB);
            Shader.PostProcessMaterial->SetVectorParameterValue(FName("MyColor"), col);
            Shader.PostProcessMaterial->SetScalarParameterValue(FName("Opacity"), Opacity);
        }
    }

    */
};

/// A sensor that captures images from the scene.
UCLASS()
class CARLA_API AFaultySceneCaptureCamera : public ASceneCaptureCamera
{
	GENERATED_BODY()

public:
	static FActorDefinition GetSensorDefinition();
	AFaultySceneCaptureCamera(const FObjectInitializer& ObjectInitializer);

protected:

	virtual void SendGBufferTextures(FGBufferRequest& GBuffer) override;

    void BeginPlay() override;
	void Set(const FActorDescription& ActorDescription) override;
    void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds) override;
private:
    int Scenario;
    float Loss_Start, Loss_Interval, Loss_Duration, Loss_DurationDeg, Loss_IntervalDegradation;
    FGBufferRequest BackupBuffer;

    FLinearColor col;
    bool FailureState = false; //True = BlackScreen, False = Standbild;
    bool FailInitialized = false;
};


