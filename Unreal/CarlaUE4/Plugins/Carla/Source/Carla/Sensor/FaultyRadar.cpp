// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "DrawDebugHelpers.h"

#include "Carla/Sensor/FaultyRadar.h"
#include "Carla/Sensor/HexagonActor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

FActorDefinition AFaultyRadar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeFaultyRadarDefinition();
}

AFaultyRadar::AFaultyRadar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
    this->SetVerticalFOV(20);
    this->SetHorizontalFOV(35);

    this->PackageLoss_Interval = 15.0f;
    this->PackageLoss_Duration = 2.5f;
    this->PackageLoss_StartOffset = 15.0f;
    this->PackageLoss_Start = FLT_MAX;
    this->PackageLoss_IntervallDegradation = 0.0f;
    this->PackageLoss_DurationDegradation = 0.0f;

    this->PackageDelay_Start = FLT_MAX;
    this->PackageDelay_DelaySize = 0;
    this->PackageDelay_WriteRingBufferPtr = 0;
    this->PackageDelay_ReadRingBufferPtr = 0;
    this->PackageDelay_RingBufferMaxUseSize = 100;
}


void AFaultyRadar::AddPackageLossInterval(float Interval)
{
    this->PackageLoss_Interval = Interval;
}
void AFaultyRadar::AddPackageLossDuration(float Duration)
{
    this->PackageLoss_Duration = Duration;
}
void AFaultyRadar::AddPackageLossStart(float StartTime)
{
    this->PackageLoss_StartOffset = StartTime;
}

void AFaultyRadar::AddScenario(int ScenarioID)
{
    if (!(this->Scenario & ScenarioID))
        this->Scenario |= ScenarioID;

}

void AFaultyRadar::SetProgressionRate(float Rate)
{
    this->PackageLoss_IntervallDegradation = Rate;
}

void AFaultyRadar::SetConstantShiftRotation(FString string) 
{
    TArray<FString> Tokens;
    const TCHAR* Delims = TEXT(";");
    string.ParseIntoArray(Tokens, Delims, false);
    if (Tokens.Num() != 3) 
    {
        //GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("ConstantShiftArray is broken, couldn�t read 3 values there!"));
        //UE_LOG(LogTemp, Error, TEXT("ConstantShiftArray is broken, couldn�t read 3 values there!"));
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
        rot.Yaw += 20;
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
    PackageLoss_Start = PackageLoss_StartOffset + GetWorld()->GetTimeSeconds();
    ConstantShift_Start = ConstantShift_StartOffset + GetWorld()->GetTimeSeconds();
    RadarDisturbance_Start = RadarDisturbance_StartOffset + GetWorld()->GetTimeSeconds();
    RadarSpoof_Start = RadarSpoof_StartOffset + GetWorld()->GetTimeSeconds();
    PrevLocation = GetActorLocation();
    return;
}
 

void AFaultyRadar::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

}
void AFaultyRadar::GenerateHexagon(int Ammount)
{
    float Range = 1;
    float CalculateHFOV = HorizontalFOV + 4.f;
    float CalculateVFOV = VerticalFOV + 4.f;


    const float MaxRx = FMath::Tan(FMath::DegreesToRadians(CalculateHFOV * 0.5f)) * Range;
    const float MaxRy = FMath::Tan(FMath::DegreesToRadians(CalculateVFOV * 0.5f)) * Range;
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& TransformRotator = ActorTransform.Rotator();
    const FVector& RadarLocation = GetActorLocation();
    float Sin, Cos;

    for (int32 i = 0; i < Ammount; i++)
    {
        const float Radius =RandomEngine->GetUniformFloat();
        const float Angle = RandomEngine->GetUniformFloatInRange(0.0f, carla::geom::Math::Pi2<float>());
        FMath::SinCos(&Sin, &Cos, Angle);
        const FVector EndLocation = RadarLocation + TransformRotator.RotateVector({
              Range,
              MaxRx * Radius * Cos,
              MaxRy * Radius * Sin
            });

        AHexagonActor* Hexagon = GetWorld()->SpawnActor<AHexagonActor>(EndLocation, FRotator(0.f, 0.f, 0.f));
        if (Hexagon != nullptr)
        {
            float Radius = FMath::FRandRange(.001f, .050f);//.0001f, .0050f);
            Hexagon->CreateHexagonMesh(Radius);
            Hexagon->AttachToActor(this,FAttachmentTransformRules(EAttachmentRule::KeepWorld,true));
            Hexagon->SetOwner(this);
            Hexagon->SetActorRotation(this->GetActorRotation() + FRotator(90.f, 0.f, 0.f));
            BlockObjects.Add(Hexagon);
        }else
            UE_LOG(LogTemp, Error, TEXT("Hexagon == nullptr!"));
    }
}

void AFaultyRadar::Destroyed()
{
    // Notify ObjectBs that this object is destroyed
    for (AActor* ObjectB : BlockObjects)
    {
        if (ObjectB != nullptr)
        {
            ObjectB->Destroy();
        }
    }

    Super::Destroyed();
}

void AFaultyRadar::WriteLineTraces()
{
    float time = GetWorld()->GetTimeSeconds();
    if (this->Scenario & ScenarioID::RadarPackageLoss)
    {
        if (time >= this->PackageLoss_Start)
        {
            if (time >= this->PackageLoss_Start + this->PackageLoss_Duration)
            {
                this->PackageLoss_Start += this->PackageLoss_Interval;
                this->PackageLoss_Interval -= PackageLoss_IntervallDegradation;
                this->PackageLoss_Duration += PackageLoss_DurationDegradation;
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
    if (this->Scenario & ScenarioID::RadarInterference)
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
    if(this->ScenarioID & ScenarioID::RadarPackageDelay && time >= this->PackageDelay_Start)
    {
        if(PackageDelay_RingBufferMaxUseSize > RadarDelay_RingBufferSize)
        {
            UE_LOG(LogTemp, Error, TEXT("RingBufferMaxUseSize too Big! Please adjust and rerun the simulation!"));
            return;
        }

        if(time >= PackageDelay_DegradationZeit)
        {
            PackageDelay_DelaySize += PackageDelay_DegradationSize;
            PackageDelay_DegradationZeit += PackageDelay_Interval;
        }
        
        PackageDelay_RingBuffer[PackageDelay_WriteRingBufferPtr] = Rays;
        PackageDelay_WriteRingBufferPtr++;
        if(PackageDelay_WriteRingBufferPtr >= PackageDelay_RingBufferMaxUseSize)
            PackageDelay_WriteRingBufferPtr = 0;

        if(!(PackageDelay_WaitCounter < PackageDelay_DelaySize))
        {
            auto RayList = PackageDelay_RingBuffer[PackageDelay_ReadRingBufferPtr];
            WriteLineTraces(RayList);
            PackageDelay_ReadRingBufferPtr++;
            if(PackageDelay_ReadRingBufferPtr  >= PackageDelay_RingBufferMaxUseSize)
                PackageDelay_ReadRingBufferPtr = 0;
        }
        else
        {
            PackageDelay_WaitCounter++;
        }
    }
    else
    {
        WriteLineTraces(Rays);
    }


    //We shift the Radar after all stuff with the Data has finsied
    //So we can use the shifted Radar in the next WorldTick
    if (this->Scenario & ScenarioID::RadarConstantShift)
    {
        if (time >= this->ConstantShift_Start)
        {
            this->ConstantShift_Start += this->ConstantShift_Interval;
            //MoveRadar(this->ConstantShift_Rotation);
        }
    }
    if (this->Scenario & ScenarioID::RadarBlockage) 
    {
        if (time >= this->Blockage_Start)
        {
            this->Blockage_Start += this->Blockage_Interval;
            GenerateHexagon(this->Blockage_HexagonAmmounts);
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
