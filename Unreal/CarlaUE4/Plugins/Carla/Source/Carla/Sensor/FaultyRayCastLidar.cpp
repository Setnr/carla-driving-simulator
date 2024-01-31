// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/FaultyRayCastLidar.h"
#include "Carla/Sensor/HexagonActor.h"

FActorDefinition AFaultyRayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeFaultyLidarDefinition(TEXT("ray_cast"));
}


AFaultyRayCastLidar::AFaultyRayCastLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {
}

void AFaultyRayCastLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetFaultyLidar(ActorDescription, LidarDescription, FaultyLidarDescription);
  Super::Set(LidarDescription);

  


}
void AFaultyRayCastLidar::EventShift()
{
    if (this->FaultyLidarDescription.Scenario & ScenarioID::SensorShift && this->FaultyLidarDescription.SensorShiftTriggerFlag == SensorShift_TriggerFlag::Collision)
    {
        ShiftSensor();
    }
}
void AFaultyRayCastLidar::MoveLidar()
{
    float time = GetWorld()->GetTimeSeconds();
    if (this->FaultyLidarDescription.Scenario & ScenarioID::SensorShift && this->FaultyLidarDescription.SensorShiftTriggerFlag == SensorShift_TriggerFlag::Timed)
    {
        if (time >= this->FaultyLidarDescription.SensorShift_Start)
        {
            if (time >= this->FaultyLidarDescription.SensorShift_Start + this->FaultyLidarDescription.SensorShift_Duration ||
                this->FaultyLidarDescription.SensorShiftFlag == SensorShift_Flag::JumpingShift)
            {
                this->FaultyLidarDescription.SensorShift_Start += this->FaultyLidarDescription.SensorShift_Interval;
                this->FaultyLidarDescription.SensorShift_Interval -= FaultyLidarDescription.SensorShift_IntervalDegradation;
                this->FaultyLidarDescription.SensorShift_Duration += FaultyLidarDescription.SensorShift_DurationDegradation;
            }

            ShiftSensor();
        }
    }
}
void AFaultyRayCastLidar::ShiftSensor()
{
    auto RadarRot = this->GetActorRotation();
    FRotator rot(this->FaultyLidarDescription.SensorShift_Pitch, this->FaultyLidarDescription.SensorShift_Yaw, this->FaultyLidarDescription.SensorShift_Roll);
    RadarRot += rot;
    this->SetActorRotation(RadarRot);
}
void AFaultyRayCastLidar::MoveLidar(FRotator rot)
{
    auto RadarRot = this->GetActorRotation();
    RadarRot += rot;
    this->SetActorRotation(RadarRot);
}
void AFaultyRayCastLidar::GenerateHexagons()
{
    for (int32 i = 0; i < FaultyLidarDescription.SensorBlockage_AmountOfBlockingObjects; i++)
    {
        const FVector EndLocation = CalculateEndLocation(FaultyLidarDescription.SensorBlockage_Type == SensorBlockage_BlockageType::CloseRange,
            this->FaultyLidarDescription.SensorBlockage_VertFOVFlag, this->FaultyLidarDescription.SensorBlockage_HorFOVFlag);

        AHexagonActor* Hexagon = GetWorld()->SpawnActor<AHexagonActor>(EndLocation, FRotator(0.f, 0.f, 0.f));
        if (Hexagon != nullptr)
        {
            float Radius = FMath::FRandRange(.001f, .050f);//.0001f, .0050f);
            Hexagon->CreateHexagonMesh(Radius);

            FVector TargetLocation = GetActorLocation();

            // Calculate the direction vector from your actor to the target location
            FVector Direction = (TargetLocation - Hexagon->GetActorLocation()).GetSafeNormal();

            // Calculate the new rotation for your actor
            FRotator NewRotation = Direction.Rotation() + FRotator(-90.f, 0.f, 0.f);

            // Apply the new rotation to your actor
            Hexagon->SetActorRotation(NewRotation);

            //Hexagon->SetActorRotation(this->GetActorRotation() + FRotator(90.f, 0.f, 0.f));
            Hexagon->AttachToActor(this, FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
            Hexagon->SetOwner(this);
            float LifeTime = 0.1f;
            if (FaultyLidarDescription.SensorBlockage_LifeTime == SensorBlockage_ObjectLifeTime::Random)
                LifeTime = CreateRandomNumber(Distribution::Linear) * FaultyLidarDescription.SensorBlockage_MaxBlockingObjectLifeTime;
            else
                LifeTime = FaultyLidarDescription.SensorBlockage_BlockingObjectLifeTime;
            Hexagon->SetBlockageParamter(LifeTime, FaultyLidarDescription.SensorBlockage_BlockageDropSpeed);
            if (LifeTime < 0.0f)
                BlockObjects.Add(Hexagon);
        }
        else
            UE_LOG(LogTemp, Error, TEXT("Hexagon == nullptr!"));
    }
}

void AFaultyRayCastLidar::Destroyed()
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
bool AFaultyRayCastLidar::HasToLooseCurrentPackage()
{
    float time = GetWorld()->GetTimeSeconds();
    if (this->FaultyLidarDescription.Scenario & ScenarioID::PackageLoss)
    {
        if (time >= this->FaultyLidarDescription.PackageLoss_Start)
        {
            if (time >= this->FaultyLidarDescription.PackageLoss_Start + this->FaultyLidarDescription.PackageLoss_Duration)
            {
                this->FaultyLidarDescription.PackageLoss_Start += this->FaultyLidarDescription.PackageLoss_Interval;
                this->FaultyLidarDescription.PackageLoss_Interval -= FaultyLidarDescription.PackageLoss_IntervalDegradation;
                this->FaultyLidarDescription.PackageLoss_Duration += FaultyLidarDescription.PackageLoss_DurationDegradation;
            }
            return true;
        }
    }
    return false;
}

float AFaultyRayCastLidar::CreateRandomNumber(Distribution DistType)
{
    float ret = 0.0f;
    switch (DistType)
    {
    case Distribution::Weibull:
    {
        ret = weibull(gen_weibull);
    }
    break;
    case Distribution::Linear:
        ret = uniform(gen_uniform);

        break;
    }

    return ret;
}

void AFaultyRayCastLidar::ShiftDetectionPoints()
{
    float time = GetWorld()->GetTimeSeconds();
    if (this->FaultyLidarDescription.Scenario & ScenarioID::DetectionPointShift)
    {
        if (time >= this->FaultyLidarDescription.DetectionPointShift_Start)
        {
            if (time >= this->FaultyLidarDescription.DetectionPointShift_Start + this->FaultyLidarDescription.DetectionPointShift_Duration)
            {
                this->FaultyLidarDescription.DetectionPointShift_Start += this->FaultyLidarDescription.DetectionPointShift_Interval;
                this->FaultyLidarDescription.DetectionPointShift_Interval -= FaultyLidarDescription.DetectionPointShift_IntervalDegradation;
                this->FaultyLidarDescription.DetectionPointShift_Duration += FaultyLidarDescription.DetectionPointShift_DurationDegradation;
            }
            for (int i = 0; i < LidarData.GetSize(); i++) 
            {
                float x = CreateRandomNumber(FaultyLidarDescription.DetectionPoint_Distribution);
                float y = CreateRandomNumber(FaultyLidarDescription.DetectionPoint_Distribution);
                float z = CreateRandomNumber(FaultyLidarDescription.DetectionPoint_Distribution);
                LidarData.ShiftPoint(i, x, y, z);
            }
        }
    }
}


void AFaultyRayCastLidar::CheckRangeReduction()
{
    float time = GetWorld()->GetTimeSeconds();
    if (this->FaultyLidarDescription.Scenario & ScenarioID::RangeReduction)
    {
        if (time >= this->FaultyLidarDescription.RangeReduction_Start)
        {
            if (time >= this->FaultyLidarDescription.RangeReduction_Start + this->FaultyLidarDescription.RangeReduction_Duration)
            {
                this->FaultyLidarDescription.RangeReduction_Start += this->FaultyLidarDescription.RangeReduction_Interval;
                this->FaultyLidarDescription.RangeReduction_Interval -= FaultyLidarDescription.RangeReduction_IntervalDegradation;
                this->FaultyLidarDescription.RangeReduction_Duration += FaultyLidarDescription.RangeReduction_DurationDegradation;
                this->FaultyLidarDescription.RangeReduction_Active = false;
                this->Description.Range = this->FaultyLidarDescription.RangeReduction_OldRangeValue;
                return;
            }
            if (!this->FaultyLidarDescription.RangeReduction_Active)
            {
                this->FaultyLidarDescription.RangeReduction_OldRangeValue = this->Description.Range;
                this->Description.Range -= this->FaultyLidarDescription.RangeReduction_RangeReductionValue;
                this->FaultyLidarDescription.RangeReduction_Active = true;
            }
        }
    }
}
void AFaultyRayCastLidar::DetectNonExisitingPoints()
{
    float time = GetWorld()->GetTimeSeconds();
    if (this->FaultyLidarDescription.Scenario & ScenarioID::DetectNonExistingPoints)
    {
        if (time >= this->FaultyLidarDescription.DetectNonExistingPoints_Start)
        {
            if (time >= this->FaultyLidarDescription.DetectNonExistingPoints_Start + this->FaultyLidarDescription.DetectNonExistingPoints_Duration)
            {
                this->FaultyLidarDescription.DetectNonExistingPoints_Start += this->FaultyLidarDescription.DetectNonExistingPoints_Interval;
                this->FaultyLidarDescription.DetectNonExistingPoints_Interval -= FaultyLidarDescription.DetectNonExistingPoints_IntervalDegradation;
                this->FaultyLidarDescription.DetectNonExistingPoints_Duration += FaultyLidarDescription.DetectNonExistingPoints_DurationDegradation;
                return;
            }

            CreatePoints();
        }
    }
}
void AFaultyRayCastLidar::CreateBlockage()
{
    float time = GetWorld()->GetTimeSeconds();
    if (this->FaultyLidarDescription.Scenario & ScenarioID::SensorBlockage)
    {
        if (time >= this->FaultyLidarDescription.SensorBlockage_Start)
        {
            this->FaultyLidarDescription.SensorBlockage_Start += this->FaultyLidarDescription.SensorBlockage_Interval;
            this->FaultyLidarDescription.SensorBlockage_Interval -= FaultyLidarDescription.SensorBlockage_IntervalDegradation;

            GenerateHexagons();
        }
    }
}
void AFaultyRayCastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) 
{
    CheckRangeReduction();
	Super::ComputeAndSaveDetections(SensorTransform);
}
void AFaultyRayCastLidar::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaTime) 
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidar::PostPhysTick);

    if (HasToLooseCurrentPackage())
        return;
    float time = GetWorld()->GetTimeSeconds();

    DetectNonExisitingPoints();
    ShiftDetectionPoints();

    CreateBlockage();
    SimulateLidar(DeltaTime);

    if (this->FaultyLidarDescription.Scenario & ScenarioID::PackageDelay)
    {
        if (time >= this->FaultyLidarDescription.PackageDelay_Start)
        {
            if (FaultyLidarDescription.PackageDelay_RingBufferMaxUseSize > RadarDelay_RingBufferSize)
            {
                UE_LOG(LogTemp, Error, TEXT("RingBufferMaxUseSize too Big! Please adjust and rerun the simulation!"));
                return;
            }

            if (time >= FaultyLidarDescription.PackageDelay_DegradationZeit)
            {
                FaultyLidarDescription.PackageDelay_DelaySize += FaultyLidarDescription.PackageDelay_DegradationSize;
                FaultyLidarDescription.PackageDelay_DegradationZeit += FaultyLidarDescription.PackageDelay_Interval;
            }

            /*
            Prepare your eyes for some ugly Code
            I dont want to activate Copy-Consturctor for the following Copy part
            because i am scared some other stuff could break
            I dont have any deeper knowledge why the copy-constructor is deleted by default so i dont want to change it
            */
            FaultyLidarDescription.PackageDelay_RingBuffer[FaultyLidarDescription.PackageDelay_WriteRingBufferPtr].SetHeader(LidarData.GetHeader());
            FaultyLidarDescription.PackageDelay_RingBuffer[FaultyLidarDescription.PackageDelay_WriteRingBufferPtr].SetMaxChannelPoints(LidarData.GetMaxChannelPoints());
            //FaultyLidarDescription.PackageDelay_RingBuffer[FaultyLidarDescription.PackageDelay_WriteRingBufferPtr].SetSerPoints(LidarData.GetSerPoints());
            FaultyLidarDescription.PackageDelay_RingBuffer[FaultyLidarDescription.PackageDelay_WriteRingBufferPtr].SetPoints(LidarData.GetPoints());

            FaultyLidarDescription.PackageDelay_WriteRingBufferPtr++;
            if (FaultyLidarDescription.PackageDelay_WriteRingBufferPtr >= FaultyLidarDescription.PackageDelay_RingBufferMaxUseSize)
                FaultyLidarDescription.PackageDelay_WriteRingBufferPtr = 0;

            if (!(FaultyLidarDescription.PackageDelay_WaitCounter < FaultyLidarDescription.PackageDelay_DelaySize))
            {
                {
                    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
                    auto DataStream = GetDataStream(*this);
                    DataStream.Send(*this, FaultyLidarDescription.PackageDelay_RingBuffer[FaultyLidarDescription.PackageDelay_ReadRingBufferPtr], DataStream.PopBufferFromPool());
                }
            
                FaultyLidarDescription.PackageDelay_ReadRingBufferPtr++;
                if (FaultyLidarDescription.PackageDelay_ReadRingBufferPtr >= FaultyLidarDescription.PackageDelay_RingBufferMaxUseSize)
                    FaultyLidarDescription.PackageDelay_ReadRingBufferPtr = 0;
            }
            else
            {
                FaultyLidarDescription.PackageDelay_WaitCounter++;
            }
            return;
        }
    }

    {
        TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
        auto DataStream = GetDataStream(*this);
        DataStream.Send(*this, LidarData, DataStream.PopBufferFromPool());
    }
}

FVector AFaultyRayCastLidar::CalculateEndLocation(bool CloseRange, VerticalFOV_Type VertFOV, HorizontalFOV_Type HorzFOV)
{
    // Create Random Angle and get Cos & Sin for that Angle
    float SpawnRange = CreateRandomNumber(Distribution::Linear) * this->Description.Range;
    if (CloseRange)
        SpawnRange = 1.0f;
    float VerticalFOV = Description.LowerFovLimit;
    if (VerticalFOV < 0.0f)
        VerticalFOV *= -1.0f;
    if (Description.UpperFovLimit > VerticalFOV)
        VerticalFOV = Description.UpperFovLimit;
    const float MaxRx = FMath::Tan(FMath::DegreesToRadians(Description.HorizontalFov * 0.5f)) * SpawnRange;
    const float MaxRy = FMath::Tan(FMath::DegreesToRadians(VerticalFOV)) * SpawnRange;
    float Angle = carla::geom::Math::Pi2<float>() * CreateRandomNumber(Distribution::Linear);
    float Sin, Cos;
    FMath::SinCos(&Sin, &Cos, Angle);

    //Create a EndPoint within the FOV based on the Random Angle
    if (VertFOV == VerticalFOV_Type::Down && Sin >= 0.0f)
        Sin = Sin * -1;
    if (VertFOV == VerticalFOV_Type::Up && Sin <= 0.0f)
        Sin = Sin * -1;

    if (HorzFOV == HorizontalFOV_Type::Left && Cos >= 0.0f)
        Cos = Cos * -1;
    if (HorzFOV == HorizontalFOV_Type::Right && Cos <= 0.0f)
        Cos = Cos * -1;

    const FVector& RadarLocation = GetActorLocation();
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& TransformRotator = ActorTransform.Rotator();
    FVector Helper = {
       SpawnRange,
       MaxRx * Cos,
       MaxRy * Sin
    };
    FVector EndLocation = RadarLocation + TransformRotator.RotateVector(Helper);


    return EndLocation;

}

void AFaultyRayCastLidar::CreatePoints()
{
    //Init some Basic Values needed for Following Calculations;
    int AddPoints = this->FaultyLidarDescription.DetectNonExistingPoints_AmountDetections; // * ((int)gen_uniform());
    const FVector& RadarLocation = GetActorLocation();
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& TransformRotator = ActorTransform.Rotator();
    const FVector TransformXAxis = ActorTransform.GetUnitAxis(EAxis::X);
    const FVector TransformYAxis = ActorTransform.GetUnitAxis(EAxis::Y);
    const FVector TransformZAxis = ActorTransform.GetUnitAxis(EAxis::Z);
    
    
    for (int i = 0; i < AddPoints; i++)
    {
        auto EndLocation = CalculateEndLocation(false, FaultyLidarDescription.DetectNonExistingPoints_VertFOVFlag, FaultyLidarDescription.DetectNonExistingPoints_HorFOVFlag);
        
        carla::sensor::data::LidarDetection data(EndLocation.X, EndLocation.Y, EndLocation.Z, CreateRandomNumber(Distribution::Linear));
        //Create a Random Distance within the current Range of the Radar
        LidarData.WritePointSync(data);
    }
}

