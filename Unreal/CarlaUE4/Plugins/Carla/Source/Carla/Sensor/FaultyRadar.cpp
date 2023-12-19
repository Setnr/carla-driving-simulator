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
    gen_weibull.seed(RandomEngineSeed);
    gen_uniform.seed(RandomEngineSeed);
    weibull = std::weibull_distribution<float>(0.0, 1.0);
    uniform = std::uniform_real_distribution<float>(0.0, 1.0);

    this->PackageLoss_Interval = 15.0f;
    this->PackageLoss_Duration = 2.5f;
    this->PackageLoss_Start = FLT_MAX;
    this->PackageLoss_IntervallDegradation = 0.0f;
    this->PackageLoss_DurationDegradation = 0.0f;

    this->PackageDelay_Start = FLT_MAX;
    this->PackageDelay_DegradationZeit = 0.0f;
    this->PackageDelay_Interval = 0.0f;
    this->PackageDelay_DegradationSize = 0;
    this->PackageDelay_DelaySize = 0;
    this->PackageDelay_WaitCounter = 0;
    this->PackageDelay_WriteRingBufferPtr = 0;
    this->PackageDelay_ReadRingBufferPtr = 0;
    this->PackageDelay_RingBufferMaxUseSize = 100;

    this->DetectionPointShift_Start = FLT_MAX;
	this->DetectionPointShift_Intervall = 0.0f;
	this->DetectionPointShift_Duration= 0.0f;
	this->DetectionPointShift_IntervallDegradation= 0.0f;
	this->DetectionPointShift_DurationDegradation= 0.0f;
	this->DetectionPoint_MaxDepthDisturbance= 0.0f;
	this->DetectionPoint_MaxAzimuthDisturbance= 0.0f;
	this->DetectionPoint_MaxAltitudeDisturbance= 0.0f;
	this->DetectionPoint_Distribution = Distribution::None;

    this->VelocityShift_Start = FLT_MAX;
	this->VelocityShift_Intervall = 0.0f;
	this->VelocityShift_Duration = 0.0f;
	this->VelocityShift_IntervallDegradation = 0.0f;
	this->VelocityShift_DurationDegradation = 0.0f;
	this->VelocityShift_MaxVelocityDisturbance = 0.0f;
	this->VelocityShift_Distribution = Distribution::None;

    this->RangeReduction_Start = FLT_MAX;
	this->RangeReduction_Intervall = 0.0f;
	this->RangeReduction_Duration = 0.0f;
	this->RangeReduction_IntervallDegradation = 0.0f;
	this->RangeReduction_DurationDegradation = 0.0f;
	this->RangeReduction_RangeReductionValue = 0.0f;
	this->RangeReduction_OldRangeValue = 0.0f;
	this->RangeReduction_Active = false;;

    //TestEndPoints();

}
void AFaultyRadar::TestEndPoints()
{
    this->SetVerticalFOV(20);
    this->SetHorizontalFOV(35);
    this->Range = 100;
    FVector RadarLocation = GetActorLocation();
    FVector loc;
    FVector helper;
    float MaxRx = FMath::Tan(FMath::DegreesToRadians(HorizontalFOV * 0.5f)) * Range;
    float MaxRy = FMath::Tan(FMath::DegreesToRadians(VerticalFOV * 0.5f)) * Range;

    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::Left;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::WholeVerFOV;
    int LeftCheck = 0;
    for(int i = 0; i < 150; i++)
    {
        loc = CalculateEndLocation();
        helper = loc - RadarLocation;
        if(helper.X <= RadarLocation.X)
            LeftCheck++;
        
    }

    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::Right;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::WholeVerFOV;
    int RightCheck = 0;
    for(int i = 0; i < 150; i++)
    {
        loc = CalculateEndLocation();
        if(loc.X >= RadarLocation.X)
            RightCheck++;
    }

    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::WholeHorFOV;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::Up;
    int UpCheck = 0;
    for(int i = 0; i < 150; i++)
    {
        loc = CalculateEndLocation();
        if(loc.Y >= RadarLocation.Y)
            UpCheck++;
    }

    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::WholeHorFOV;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::Down;
    int DownCheck = 0;
    for(int i = 0; i < 150; i++)
    {
        loc = CalculateEndLocation();
        if(loc.Y <= RadarLocation.Y)
            DownCheck++;
    }

    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::Right;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::Up;
    int UpRightCheck = 0;
    for(int i = 0; i < 150; i++)
    {
        loc = CalculateEndLocation();
        if(loc.Y >= RadarLocation.Y && loc.X >= RadarLocation.X)
            UpRightCheck++;
    }

    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::Left;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::Up;
    int UpLeftCheck = 0;
    for(int i = 0; i < 150; i++)
    {
        loc = CalculateEndLocation();
        if(loc.Y >= RadarLocation.Y && loc.X <= RadarLocation.X)
            UpLeftCheck++;
    }

    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::Right;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::Down;
    int DownRightCheck = 0;
    for(int i = 0; i < 150; i++)
    {
        loc = CalculateEndLocation();
        if(loc.Y <= RadarLocation.Y && loc.X >= RadarLocation.X)
            DownRightCheck++;
    }

    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::Left;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::Down;
    int DownLeftCheck = 0;
    for(int i = 0; i < 150; i++)
    {
        loc = CalculateEndLocation();
        if(loc.Y <= RadarLocation.Y && loc.X <= RadarLocation.X)
            DownLeftCheck++;
    }

    if(LeftCheck == 150)
        UE_LOG(LogTemp, Warning, TEXT("LeftCheck passed"));
    if(RightCheck == 150)
        UE_LOG(LogTemp, Warning, TEXT("RightCheck passed"));
    if(UpCheck == 150)
        UE_LOG(LogTemp, Warning, TEXT("UpCheck passed"));
    if(DownCheck == 150)
        UE_LOG(LogTemp, Warning, TEXT("DownCheck passed"));

    if(UpRightCheck == 150)
        UE_LOG(LogTemp, Warning, TEXT("UpRightCheck passed"));
    if(UpLeftCheck == 150)
        UE_LOG(LogTemp, Warning, TEXT("UpLeftCheck passed"));
    if(DownRightCheck == 150)
        UE_LOG(LogTemp, Warning, TEXT("DownRightCheck passed"));
    if(DownLeftCheck == 150)
        UE_LOG(LogTemp, Warning, TEXT("DownLeftCheck passed"));
}
void AFaultyRadar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) 
{
    CheckRangeReduction();
    ARadar::PostPhysTick(World,TickType,DeltaTime);
}
void AFaultyRadar::AddScenario(int ScenarioID)
{
    if (!(this->Scenario & ScenarioID))
        this->Scenario |= ScenarioID;

}

void AFaultyRadar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetFaultyRadar(ActorDescription, this);
}

void AFaultyRadar::MoveRadar()
{
    //if(this->Scenario & ScenarioID::RadarCollosionShift)
    {
        auto rot = this->GetActorRotation();
        //rot.Yaw += 20;
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
    PrevLocation = GetActorLocation();
    return;
}

void AFaultyRadar::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

}
void AFaultyRadar::GenerateHexagon(int Ammount)
{
    float Range = 1.0f;
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
bool AFaultyRadar::HasToLooseCurrentPackage()
{    
    float time = GetWorld()->GetTimeSeconds();
    if (this->Scenario & ScenarioID::PackageLoss)
    {
        if (time >= this->PackageLoss_Start)
        {
            if (time >= this->PackageLoss_Start + this->PackageLoss_Duration)
            {
                this->PackageLoss_Start += this->PackageLoss_Interval;
                this->PackageLoss_Interval -= PackageLoss_IntervallDegradation;
                this->PackageLoss_Duration += PackageLoss_DurationDegradation;
            }
            return true;
        }
    }
    return false;
}
float AFaultyRadar::CreateRandomNumber(Distribution DistType)
{
    float ret = 0.0f;
    switch(DistType)
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
void AFaultyRadar::ShiftDetectionPoints()
{
    float time = GetWorld()->GetTimeSeconds();
    if(this->Scenario & ScenarioID::DetectionPointShift)
    {
        if(time >= this->DetectionPointShift_Start)
        {
            if (time >= this->DetectionPointShift_Start + this->DetectionPointShift_Duration)
            {
                this->DetectionPointShift_Start += this->DetectionPointShift_Intervall;
                this->DetectionPointShift_Intervall -= DetectionPointShift_IntervallDegradation;
                this->DetectionPointShift_Duration += DetectionPointShift_DurationDegradation;
            }

            for (auto& ray : Rays)
            {
                if (ray.Hitted)
                {
                    float rand = CreateRandomNumber(DetectionPoint_Distribution);
                    if (CreateRandomNumber(DetectionPoint_Distribution))
                        rand *= -1;
                    ray.Distance += DetectionPoint_MaxDepthDisturbance * rand;

                    rand = CreateRandomNumber(DetectionPoint_Distribution);
                    if (CreateRandomNumber(DetectionPoint_Distribution))
                        rand *= -1;
                    ray.AzimuthAndElevation.X  += DetectionPoint_MaxAzimuthDisturbance * rand;

                    rand = CreateRandomNumber(DetectionPoint_Distribution);
                    if (CreateRandomNumber(DetectionPoint_Distribution))
                        rand *= -1;
                    ray.AzimuthAndElevation.Y += DetectionPoint_MaxAltitudeDisturbance * rand;
                }
            }

        }
    }
}

void AFaultyRadar::ShiftVelocitys()
{
    float time = GetWorld()->GetTimeSeconds();
    if(this->Scenario & ScenarioID::VelocityShift)
    {
        if(time >= this->VelocityShift_Start)
        {
            if (time >= this->VelocityShift_Start + this->VelocityShift_Duration)
            {
                this->VelocityShift_Start += this->VelocityShift_Intervall;
                this->VelocityShift_Intervall -= VelocityShift_IntervallDegradation;
                this->VelocityShift_Duration += VelocityShift_DurationDegradation;
            }

            for (auto& ray : Rays)
            {
                if (ray.Hitted)
                {
                    float rand = CreateRandomNumber(VelocityShift_Distribution);
                    if (CreateRandomNumber(DetectionPoint_Distribution))
                        rand *= -1;
                    ray.RelativeVelocity += VelocityShift_MaxVelocityDisturbance * rand;
                }
            }

        }
    }
}
void AFaultyRadar::CheckRangeReduction()
{
    float time = GetWorld()->GetTimeSeconds();
    if(this->Scenario & ScenarioID::RangeReduction)
    {
        if(time >= this->RangeReduction_Start)
        {
            if (time >= this->RangeReduction_Start + this->RangeReduction_Duration)
            {
                this->RangeReduction_Start += this->RangeReduction_Intervall;
                this->RangeReduction_Intervall -= RangeReduction_IntervallDegradation;
                this->RangeReduction_Duration += RangeReduction_DurationDegradation;    
                this->RangeReduction_Active = false;
                this->Range = this->RangeReduction_OldRangeValue;
                return;
            }
            if(!this->RangeReduction_Active)
            {
                this->RangeReduction_OldRangeValue = this->Range;
                this->Range -= this->RangeReduction_RangeReductionValue;
                this->RangeReduction_Active = true;
            }
        }
    }
}

void AFaultyRadar::DrawRadarBorder() 
{
    const float MaxRx = FMath::Tan(FMath::DegreesToRadians(HorizontalFOV * 0.5f)) * Range;
    const float MaxRy = FMath::Tan(FMath::DegreesToRadians(VerticalFOV * 0.5f)) * Range;
    const FVector& RadarLocation = GetActorLocation();
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& TransformRotator = ActorTransform.Rotator();

    const FVector TransformXAxis = ActorTransform.GetUnitAxis(EAxis::X);
    const FVector TransformYAxis = ActorTransform.GetUnitAxis(EAxis::Y);
    const FVector TransformZAxis = ActorTransform.GetUnitAxis(EAxis::Z);
    for (float XBorder = -1.0f; XBorder <= 1.0f; XBorder += 0.1f) 
    {
        FVector TopHelper = {
           Range,
           MaxRx * XBorder,
           MaxRy
        };
        FVector TopEndLocation = RadarLocation + TransformRotator.RotateVector(TopHelper);
        FVector BotHelper = {
           Range,
           MaxRx * XBorder,
           MaxRy * -1
        };
        FVector BotEndLocation = RadarLocation + TransformRotator.RotateVector(BotHelper);
        RayData data;
        data.Hitted = true;
        //Create a Fake positiv or negativ Velocity
        data.RelativeVelocity = -1.0f;
        //Create a Random Distance within the current Range of the Radar
        data.Distance = Range;
        //Calculated Azimuth and Altitude for the Fake Point 
        data.AzimuthAndElevation = FMath::GetAzimuthAndElevation(
            (TopEndLocation - RadarLocation).GetSafeNormal() * Range,
            TransformXAxis,
            TransformYAxis,
            TransformZAxis
        );
        Rays.push_back(data);

        data.AzimuthAndElevation = FMath::GetAzimuthAndElevation(
            (BotEndLocation - RadarLocation).GetSafeNormal() * Range,
            TransformXAxis,
            TransformYAxis,
            TransformZAxis
        );
        Rays.push_back(data);
    }

    for (float YBorder = -1.0f; YBorder <= 1.0f; YBorder += 0.1f)
    {
        FVector TopHelper = {
           Range,
           MaxRx,
           MaxRy * YBorder
        };
        FVector TopEndLocation = RadarLocation + TransformRotator.RotateVector(TopHelper);
        FVector BotHelper = {
           Range,
           MaxRx * -1,
           MaxRy * YBorder
        };
        FVector BotEndLocation = RadarLocation + TransformRotator.RotateVector(BotHelper);
        RayData data;
        data.Hitted = true;
        //Create a Fake positiv or negativ Velocity
        data.RelativeVelocity = CreateRandomNumber(Distribution::Linear);
        if (CreateRandomNumber(Distribution::Linear) <= 0.5f)
            data.RelativeVelocity *= -1;
        //Create a Random Distance within the current Range of the Radar
        data.Distance = Range;
        //Calculated Azimuth and Altitude for the Fake Point 
        data.AzimuthAndElevation = FMath::GetAzimuthAndElevation(
            (TopEndLocation - RadarLocation).GetSafeNormal() * Range,
            TransformXAxis,
            TransformYAxis,
            TransformZAxis
        );
        Rays.push_back(data);

        data.AzimuthAndElevation = FMath::GetAzimuthAndElevation(
            (BotEndLocation - RadarLocation).GetSafeNormal() * Range,
            TransformXAxis,
            TransformYAxis,
            TransformZAxis
        );
        Rays.push_back(data);
    }
    
}

void AFaultyRadar::DetectNonExisitingPoints()
{
#ifdef DEBUG_NONEXISITING_POINTS
    Rays.clear();
    this->Scenario = ScenarioID::DetectNonExistingPoints;
    DrawRadarBorder();
    DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::WholeHorFOV;
    DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::WholeVerFOV;
    DetectNonExistingPoints_AmmountDetections = 5000;
    DetectNonExistingPoints_Duration = MAX_FLT;
    this->Range = 12.0f;
#endif

    float time = GetWorld()->GetTimeSeconds();
    if(this->Scenario & ScenarioID::DetectNonExistingPoints)
    {
        if(time >= this->DetectNonExistingPoints_Start)
        {
            if (time >= this->DetectNonExistingPoints_Start + this->DetectNonExistingPoints_Duration)
            {
                this->DetectNonExistingPoints_Start += this->DetectNonExistingPoints_Intervall;
                this->DetectNonExistingPoints_Intervall -= DetectNonExistingPoints_IntervallDegradation;
                this->DetectNonExistingPoints_Duration += DetectNonExistingPoints_DurationDegradation; 
                return;
            }

            CreatePoints();
        }
    }
}


void AFaultyRadar::WriteLineTraces()
{
    if(HasToLooseCurrentPackage())
        return;

    float time = GetWorld()->GetTimeSeconds();

    DetectNonExisitingPoints();
    ShiftDetectionPoints();
    ShiftVelocitys();

    if(this->Scenario & ScenarioID::PackageDelay)
    {
        if(time >= this->PackageDelay_Start)
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
                ARadar::WriteLineTraces(RayList);
                PackageDelay_ReadRingBufferPtr++;
                if(PackageDelay_ReadRingBufferPtr  >= PackageDelay_RingBufferMaxUseSize)
                    PackageDelay_ReadRingBufferPtr = 0;
            }
            else
            {
                PackageDelay_WaitCounter++;
            }
        }
    }
    else
    {
        ARadar::WriteLineTraces(Rays);
    }
}

FVector AFaultyRadar::CalculateEndLocation()
{
    // Create Random Angle and get Cos & Sin for that Angle
    const float MaxRx = FMath::Tan(FMath::DegreesToRadians(HorizontalFOV * 0.5f)) * Range;
    const float MaxRy = FMath::Tan(FMath::DegreesToRadians(VerticalFOV * 0.5f)) * Range;
    float Angle = carla::geom::Math::Pi2<float>() * CreateRandomNumber(Distribution::Linear);
    float Sin, Cos;
    FMath::SinCos(&Sin, &Cos, Angle);

    //Create a EndPoint within the FOV based on the Random Angle
    if (DetectNonExistingPoints_VertFOVFlag == VerticalFOV_Type::Down && Sin >= 0.0f )
        Sin = Sin * -1;
    if (DetectNonExistingPoints_VertFOVFlag == VerticalFOV_Type::Up && Sin <= 0.0f)
        Sin = Sin* -1;

    if (DetectNonExistingPoints_HorFOVFlag == HorizontalFOV_Type::Left && Cos >= 0.0f)
        Cos = Cos * -1;
    if (DetectNonExistingPoints_HorFOVFlag == HorizontalFOV_Type::Right && Cos <= 0.0f)
        Cos = Cos * -1;

    const FVector& RadarLocation = GetActorLocation();
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& TransformRotator = ActorTransform.Rotator();
    FVector Helper = {
       Range,
       MaxRx * Cos,
       MaxRy * Sin
    };
    FVector EndLocation = RadarLocation + TransformRotator.RotateVector(Helper);


    return EndLocation;
    
}

void AFaultyRadar::CreatePoints()
{
    //Init some Basic Values needed for Following Calculations;
    int AddPoints = this->DetectNonExistingPoints_AmmountDetections; // * ((int)gen_uniform());
    const FVector& RadarLocation = GetActorLocation();
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& TransformRotator = ActorTransform.Rotator();
    const FVector TransformXAxis = ActorTransform.GetUnitAxis(EAxis::X);
    const FVector TransformYAxis = ActorTransform.GetUnitAxis(EAxis::Y);
    const FVector TransformZAxis = ActorTransform.GetUnitAxis(EAxis::Z);
    RayData data;

    for (int i = 0; i < AddPoints; i++)
    {   
        auto EndLocation = CalculateEndLocation();
        data.Hitted = true;
        //Create a Fake positiv or negativ Velocity
        data.RelativeVelocity = CreateRandomNumber(Distribution::Linear);
        if(CreateRandomNumber(Distribution::Linear) <= 0.5f)
            data.RelativeVelocity *= -1;
        float LocalRange = Range * CreateRandomNumber(Distribution::Linear);
        //Calculated Azimuth and Altitude for the Fake Point 
        data.AzimuthAndElevation = FMath::GetAzimuthAndElevation(
                    (EndLocation - RadarLocation).GetSafeNormal() * LocalRange,
                    TransformXAxis,
                    TransformYAxis,
                    TransformZAxis
                );
        //Create a Random Distance within the current Range of the Radar
        data.Distance = LocalRange;

        Rays.push_back(data);
    }
}
