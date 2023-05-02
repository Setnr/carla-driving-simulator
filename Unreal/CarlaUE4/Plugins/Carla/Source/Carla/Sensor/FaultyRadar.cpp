// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "DrawDebugHelpers.h"

#include "Carla/Sensor/FaultyRadar.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

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
    this->LooseContact_ProgressionRate = 0.0f;

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

void AFaultyRadar::SetProgressionRate(float Rate)
{
    this->LooseContact_ProgressionRate = Rate;
}

void AFaultyRadar::SetConstantShiftRotation(FString string) 
{
    TArray<FString> Tokens;
    const TCHAR* Delims = TEXT(";");
    string.ParseIntoArray(Tokens, Delims, false);
    if (Tokens.Num() != 3) 
    {
        //GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("ConstantShiftArray is broken, couldn´t read 3 values there!"));
        //UE_LOG(LogTemp, Error, TEXT("ConstantShiftArray is broken, couldn´t read 3 values there!"));
        return;
    }
    ConstantShift_Rotation.Yaw = FCString::Atof(*Tokens[0]);
    ConstantShift_Rotation.Pitch = FCString::Atof(*Tokens[1]);
    ConstantShift_Rotation.Roll = FCString::Atof(*Tokens[2]);

}
void AFaultyRadar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetFaultyRadar(ActorDescription, this);
}

void AFaultyRadar::MoveRadar()
{
    if(this->Scenario & ScenarioID::RadarCollosionShift)
    {
        auto rot = this->GetActorRotation();
        rot.Yaw += 10;
        this->SetActorRotation(rot);
    }
}
void AFaultyRadar::MoveRadar(FRotator rot) 
{
    auto RadarRot = this->GetActorRotation();
    RadarRot += rot;
    this->SetActorRotation(RadarRot);
}

void AFaultyRadar::BeginPlay()
{
  Super::BeginPlay();
  LooseContact_Start = LooseContact_StartOffset + GetWorld()->GetTimeSeconds();
  ConstantShift_Start = ConstantShift_StartOffset + GetWorld()->GetTimeSeconds();
  RadarDisturbance_Start = RadarDisturbance_StartOffset + GetWorld()->GetTimeSeconds();
  RadarSpoof_Start = RadarSpoof_StartOffset + GetWorld()->GetTimeSeconds();
  PrevLocation = GetActorLocation();
  return;

/* 
}
void AFaultyRadar::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    */

    FVector SpawnLocation = GetActorLocation() + GetActorForwardVector() * 1.0f;
    for (auto Comp : MeshComponents) {
        Comp->DestroyComponent();
    }
    MeshComponents.Empty();

    for (int32 i = 0; i < 3; i++)
    {

        UProceduralMeshComponent* MeshComponent = NewObject<UProceduralMeshComponent>(this);
        MeshComponent->SetMobility(EComponentMobility::Movable);
        
        auto Radius = FMath::RandRange(75.f, 75.f);
        GenerateHexagonMesh(MeshComponent, Radius);
        FVector ForwardVector = this->GetActorForwardVector();
        FVector Origin = this->GetActorLocation();
        FTransform Transform(FRotator(FMath::FRandRange(-VerticalFOV, VerticalFOV), FMath::FRandRange(-HorizontalFOV, HorizontalFOV), 0));
        auto MoveVec =  Transform.TransformPosition(Origin);

        TArray<FVector> Vertices;
            
        DrawDebugLine(GetWorld(), GetActorForwardVector(), GetActorForwardVector()*50.f, FColor::Red, false, 5000);
        DrawDebugLine(GetWorld(), GetActorForwardVector().GetSafeNormal(), GetActorForwardVector().GetSafeNormal() * 50.f, FColor::Green, false, 5000);

        MeshComponent->SetRelativeLocation((GetActorForwardVector()*2.f)  + MoveVec);
        MeshComponent->SetRelativeRotation(GetActorRotation() + FRotator(-90.0f, 0.0f, 0.0f));
        MeshComponent->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));

        MeshComponent->AttachToComponent(this->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        MeshComponents.Add(MeshComponent);
        MeshComponent->RegisterComponent();
        //MeshComponent->SetVisibility(false);
        UMaterialInterface* Material = MeshComponent->GetMaterial(0);
        UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(Material, MeshComponent);
        DynamicMaterial->SetScalarParameterValue(TEXT("OpacityAmount"), 0.f);
    }
}
void AFaultyRadar::GenerateHexagonMesh(UProceduralMeshComponent* OutMesh, float Radius)
{
    const int32 NumVertices = 7;
    const int32 NumTriangles = 6;

    TArray< FVector > Vertices;
    TArray< int32 > Triangles;
    TArray< FVector > Normals;
    TArray< FVector2D > UVs;
    TArray<FLinearColor> Colors;
    TArray< FProcMeshTangent > Tangents;



    // Generate vertices
    float angle = PI / 3.0f;

    for (int i = 0; i < 6; i++)
    {
        float x = Radius * FMath::Cos(angle * i);
        float y = Radius * FMath::Sin(angle * i);
        Vertices.Add(FVector(x, y, 0));
    }
    for (int i = 0; i < Vertices.Num(); i++)
    {
        Normals.Add(FVector(0.f, 0.f, 1.f));
    }
    UVs.Add(FVector2D(0.f, 0.f));
    UVs.Add(FVector2D(1.f, 0.f));
    UVs.Add(FVector2D(1.f, 1.f));
    UVs.Add(FVector2D(0.f, 1.f));
    UVs.Add(FVector2D(0.f, 1.f));
    UVs.Add(FVector2D(0.f, 1.f));
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    Colors.Add(FLinearColor::Red);
    // Front-facing triangles
    Triangles.Add(0);
    Triangles.Add(1);
    Triangles.Add(2);
    Triangles.Add(0);
    Triangles.Add(2);
    Triangles.Add(3);
    Triangles.Add(0);
    Triangles.Add(3);
    Triangles.Add(4);
    Triangles.Add(0);
    Triangles.Add(4);
    Triangles.Add(5);

    // Create mesh section
    OutMesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UVs, Colors, Tangents, true);
}

void AFaultyRadar::WriteLineTraces()
{
    float time = GetWorld()->GetTimeSeconds();
    if (this->Scenario & ScenarioID::RadarLooseContact)
    {
        if (time >= this->LooseContact_Start)
        {
            if (time >= this->LooseContact_Start + this->LooseContact_Duration)
            {
                this->LooseContact_Start += this->LooseContact_Interval;
                this->LooseContact_Interval -= LooseContact_ProgressionRate;
            }
            return;
        }
    }

    if (this->Scenario & ScenarioID::RadarDisturbance)
    {
        if (time >= this->RadarDisturbance_Start)
        {
            if (time >= this->RadarDisturbance_Start + RadarDisturbance_Duration) 
            {
                RadarDisturbance_Start += RadarDisturbance_Interval;
                RadarDisturbance_Interval -= RadarDisturbance_ProgressionRate;
            }
            DisturbeRadar();
        }
    }
    if (this->Scenario & ScenarioID::RadarSpoofing)
    {
        if (time >= this->RadarSpoof_Start)
        {
            if (time >= this->RadarSpoof_Start + RadarSpoof_Duration)
            {
                RadarSpoof_Start += RadarSpoof_Interval;
                RadarSpoof_Interval -= RadarSpoof_ProgressionRate;
            }
            SpoofRadar();
        }
    }

    for (auto& ray : Rays) {
        if (ray.Hitted) {
            RadarData.WriteDetection({
                ray.RelativeVelocity,
                ray.AzimuthAndElevation.X,
                ray.AzimuthAndElevation.Y,
                ray.Distance,
                ray.label
                });
        }
    }


    //We shift the Radar after all stuff with the Data has finsied
    //So we can use the shifted Radar in the next WorldTick
    if (this->Scenario & ScenarioID::RadarConstantShift)
    {
        if (time >= this->ConstantShift_Start)
        {
            this->ConstantShift_Start += this->ConstantShift_Interval;
            MoveRadar(this->ConstantShift_Rotation);
        }
    }

    if (this->Scenario & ScenarioID::RadarRandomShift)
    {
        if (RandomShift_Time == 0.0f) 
        {
            RandomShift_Time = FMath::RandRange(this->RandomShift_Start, this->RandomShift_End);
        }
        if (time >= this->RandomShif_StartOffset + RandomShift_Time)
        {
            this->RandomShif_StartOffset = time;
            FRotator rotator( 0.0f, FMath::FRandRange(-1.5f, 1.5f), 0.0f); //We assume that we will always shift horizontaly
            if (FMath::FRand() < 0.1f) //We give the radar chance to shift verticaly aswell
                rotator.Pitch = FMath::FRandRange(-1.5f, 1.5f);
            MoveRadar(rotator);
            RandomShift_Time = FMath::RandRange(this->RandomShift_Start, this->RandomShift_End);
        }
    }
}

void AFaultyRadar::DisturbeRadar() 
{
    //ToDo:
    // How to Disturbe exactly?
    for (auto& ray : Rays) 
    {
        if (ray.Hitted) 
        {
            if (FMath::FRand() < 0.1f)
            {
                ray.RelativeVelocity = ray.RelativeVelocity * -1; //Just Disturbe Velocity at current State
            }
        }
    }
}

void AFaultyRadar::SpoofRadar()
{
    //ToDo:
    // How to Disturbe exactly?
    for (auto& ray : Rays)
    {
        if (ray.Hitted)
        {
            if (ray.Distance > RadarSpoof_CutOff) 
            {
                ray.Hitted = false;
            }
        }
    }
}

