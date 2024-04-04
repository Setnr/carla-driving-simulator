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
    gen_weibull.seed(RandomEngineSeed);
    gen_uniform.seed(RandomEngineSeed);
    weibull = std::weibull_distribution<float>(1.0, 3.602);
    uniform = std::uniform_real_distribution<float>(0.0, 1.0);

    this->FaultyLidarDescription.World = nullptr;
    this->FaultyLidarDescription.isBlocked = false;
    this->FaultyLidarDescription.Scenario = 0;

    this->FaultyLidarDescription.PackageLoss_Interval = 15.0f;
    this->FaultyLidarDescription.PackageLoss_Duration = 2.5f;
    this->FaultyLidarDescription.PackageLoss_Start = FLT_MAX;
    this->FaultyLidarDescription.PackageLoss_IntervalDegradation = 0.0f;
    this->FaultyLidarDescription.PackageLoss_DurationDegradation = 0.0f;

    this->FaultyLidarDescription.PackageDelay_Start = FLT_MAX;
    this->FaultyLidarDescription.PackageDelay_DegradationZeit = 0.0f;
    this->FaultyLidarDescription.PackageDelay_Interval = 0.0f;
    this->FaultyLidarDescription.PackageDelay_DegradationSize = 0;
    this->FaultyLidarDescription.PackageDelay_DelaySize = 0;
    this->FaultyLidarDescription.PackageDelay_WaitCounter = 0;
    this->FaultyLidarDescription.PackageDelay_WriteRingBufferPtr = 0;
    this->FaultyLidarDescription.PackageDelay_ReadRingBufferPtr = 0;
    this->FaultyLidarDescription.PackageDelay_RingBufferMaxUseSize = 100;

    this->FaultyLidarDescription.DetectionPointShift_Start = FLT_MAX;
    this->FaultyLidarDescription.DetectionPointShift_Interval = 0.0f;
    this->FaultyLidarDescription.DetectionPointShift_Duration = 0.0f;
    this->FaultyLidarDescription.DetectionPointShift_IntervalDegradation = 0.0f;
    this->FaultyLidarDescription.DetectionPointShift_DurationDegradation = 0.0f;
    this->FaultyLidarDescription.DetectionPoint_MaxDepthDisturbance = 0.0f;
    this->FaultyLidarDescription.DetectionPoint_MaxAzimuthDisturbance = 0.0f;
    this->FaultyLidarDescription.DetectionPoint_MaxAltitudeDisturbance = 0.0f;
    this->FaultyLidarDescription.DetectionPoint_Distribution = Distribution::None;

    this->FaultyLidarDescription.VelocityShift_Start = FLT_MAX;
    this->FaultyLidarDescription.VelocityShift_Interval = 0.0f;
    this->FaultyLidarDescription.VelocityShift_Duration = 0.0f;
    this->FaultyLidarDescription.VelocityShift_IntervalDegradation = 0.0f;
    this->FaultyLidarDescription.VelocityShift_DurationDegradation = 0.0f;
    this->FaultyLidarDescription.VelocityShift_MaxVelocityDisturbance = 0.0f;
    this->FaultyLidarDescription.VelocityShift_Distribution = Distribution::None;

    this->FaultyLidarDescription.RangeReduction_Start = FLT_MAX;
    this->FaultyLidarDescription.RangeReduction_Interval = 0.0f;
    this->FaultyLidarDescription.RangeReduction_Duration = 0.0f;
    this->FaultyLidarDescription.RangeReduction_IntervalDegradation = 0.0f;
    this->FaultyLidarDescription.RangeReduction_DurationDegradation = 0.0f;
    this->FaultyLidarDescription.RangeReduction_RangeReductionValue = 0.0f;
    this->FaultyLidarDescription.RangeReduction_OldRangeValue = 0.0f;
    this->FaultyLidarDescription.RangeReduction_Active = false;;

    this->FaultyLidarDescription.DetectNonExistingPoints_Start = FLT_MAX;
    this->FaultyLidarDescription.DetectNonExistingPoints_Interval = 0.0f;
    this->FaultyLidarDescription.DetectNonExistingPoints_Duration = 0.0f;
    this->FaultyLidarDescription.DetectNonExistingPoints_IntervalDegradation = 0.0f;
    this->FaultyLidarDescription.DetectNonExistingPoints_DurationDegradation = 0.0f;
    this->FaultyLidarDescription.DetectNonExistingPoints_AmountDetections = 0;
    this->FaultyLidarDescription.DetectNonExistingPoints_HorFOVFlag = (HorizontalFOV_Type )0xFFFFFFFF; // -1 to make it dead
    this->FaultyLidarDescription.DetectNonExistingPoints_VertFOVFlag = (VerticalFOV_Type )0xFFFFFFFF; // -1 to make it dead

    this->FaultyLidarDescription.SensorShift_Start = FLT_MAX;
    this->FaultyLidarDescription.SensorShift_Interval = 0.0f;
    this->FaultyLidarDescription.SensorShift_Duration = 0.0f;
    this->FaultyLidarDescription.SensorShift_IntervalDegradation = 0.0f;
    this->FaultyLidarDescription.SensorShift_DurationDegradation = 0.0f;
    this->FaultyLidarDescription.SensorShift_Yaw = 0.0f;
    this->FaultyLidarDescription.SensorShift_Roll = 0.0f;
    this->FaultyLidarDescription.SensorShift_Pitch = 0.0f;
    this->FaultyLidarDescription.SensorShiftFlag = SensorShift_Flag::ConstantShift;
    this->FaultyLidarDescription.SensorShiftTriggerFlag = SensorShift_TriggerFlag::Timed;

    this->FaultyLidarDescription.SensorBlockage_Start = FLT_MAX;
    this->FaultyLidarDescription.SensorBlockage_Interval = 0.0f;
    this->FaultyLidarDescription.SensorBlockage_IntervalDegradation = 0.0f;

    this->FaultyLidarDescription.SensorBlockage_AmountOfBlockingObjects = 0;
    this->FaultyLidarDescription.SensorBlockage_Type = SensorBlockage_BlockageType::CloseRange;

    this->FaultyLidarDescription.SensorBlockage_LifeTime = SensorBlockage_ObjectLifeTime::Static;
    this->FaultyLidarDescription.SensorBlockage_BlockingObjectLifeTime = -1.0f;
    this->FaultyLidarDescription.SensorBlockage_MaxBlockingObjectLifeTime = 0.0f;

    this->FaultyLidarDescription.SensorBlockage_BlockageDropSpeed = 0.0f;
    this->FaultyLidarDescription.SensorBlockage_HorFOVFlag = HorizontalFOV_Type::WholeHorFOV;
    this->FaultyLidarDescription.SensorBlockage_VertFOVFlag = VerticalFOV_Type::WholeVerFOV;
}

void AFaultyRayCastLidar::Set(const FActorDescription &ActorDescription)
{
  this->FaultyLidarDescription.World = GetWorld();
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
            if (LifeTime <= 0.0f)
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
        if (ret > 15.0f)
            return CreateRandomNumber(DistType);
        ret = ret / 15.0f;
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

                float x = CreateRandomNumber(FaultyLidarDescription.DetectionPoint_Distribution) * FaultyLidarDescription.DetectionPoint_MaxDepthDisturbance;
                float y = CreateRandomNumber(FaultyLidarDescription.DetectionPoint_Distribution) * FaultyLidarDescription.DetectionPoint_MaxAzimuthDisturbance;
                float z = CreateRandomNumber(FaultyLidarDescription.DetectionPoint_Distribution) * FaultyLidarDescription.DetectionPoint_MaxAltitudeDisturbance;

                if (CreateRandomNumber(Distribution::Linear) < 0.5f)
                    x = x * -1;
                if (CreateRandomNumber(Distribution::Linear) < 0.5f)
                    y = y * -1;
                if (CreateRandomNumber(Distribution::Linear) < 0.5f)
                    z = z * -1;
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
    MoveLidar();
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
    float SpawnRange = CreateRandomNumber(Distribution::Linear) * this->Description.Range;
    if (CloseRange)
        SpawnRange = 1.f;


    float LocHepler = 1.0f;
    if(this->Description.HorizontalFov >= 180.0f)
        if (CreateRandomNumber(Distribution::Linear) >= 0.5f)
            LocHepler = -1.0f;
    float MaxRx = CreateRandomNumber(Distribution::Linear) * SpawnRange * LocHepler;
    if (CreateRandomNumber(Distribution::Linear) >= 0.5f)
        LocHepler = -1.0f;
    float MaxRy = CreateRandomNumber(Distribution::Linear) * SpawnRange * LocHepler; 
    if (CreateRandomNumber(Distribution::Linear) >= 0.5f)
        LocHepler = -1.0f;
    float MaxRz = CreateRandomNumber(Distribution::Linear) * SpawnRange * LocHepler;
   

    //Create a EndPoint within the FOV based on the Random Angle
    if (VertFOV == VerticalFOV_Type::Down && MaxRz >= 0.0f)
        MaxRz = MaxRz * -1;
    if (VertFOV == VerticalFOV_Type::Up && MaxRz <= 0.0f)
        MaxRz = MaxRz * -1;

    if (HorzFOV == HorizontalFOV_Type::Left && MaxRy >= 0.0f)
        MaxRy = MaxRy * -1;
    if (HorzFOV == HorizontalFOV_Type::Right && MaxRy <= 0.0f)
        MaxRy = MaxRy * -1;

    const FVector& RadarLocation = GetActorLocation();
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& TransformRotator = ActorTransform.Rotator();
    if (CreateRandomNumber(Distribution::Linear) >= 0.5f)
        SpawnRange *= -1.0f;
    FVector Helper = {
       MaxRx,
       MaxRy,
       MaxRz
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

