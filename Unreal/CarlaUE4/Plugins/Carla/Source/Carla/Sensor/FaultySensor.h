#pragma once

#include "Carla.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/ShaderBasedSensor.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "HexagonActor.h"
#include <random>

#define MaxRingBufferSize 128

class SensorFailure 
{
public:
	SensorFailure() : uniform(-1.f, 1.f)
	{

	}
	enum ScenarioID : int
	{
		PackageLoss = 0x1, //							1	y
		PackageDelay = 0x2, //							2	y
		DetectionPointShift = 0x4, //					4	y
		VelocityShift = 0x8, //							8	y
		RangeReduction = 0x10, //						16	y
		DetectNonExistingPoints = 0x20,				//	32	y
		SensorShift = 0x40, // Timed not Collision		64
		SensorBlockage = 0x80,//						128 y
		ShaderError = 0x100
	};
	void UpdateScenario() { ScenarioActive = !ScenarioActive; }
	virtual bool IsScenarioActive(float CurTime)
	{
		if (ScenarioActive)
		{
			if (CurTime >= Start)
			{
				if (CurTime >= Start + Duration)
				{
					Degredation();
				}
				return true;
			}
		}
		return false;
	}
protected:
	float Duration = .0f;
	float Interval = .0f;
	float DurationDegradation = .0f;
	float IntervalDegradation = .0f;

	std::mt19937 gen_uniform;
	std::uniform_real_distribution<float> uniform;


	float Start = .0f;
	bool ScenarioActive = false;
	void Degredation() 
	{
		Start += Interval;
		Interval -= IntervalDegradation;
		Duration -= DurationDegradation;
	}
	static TArray<FActorVariation> CreatePrimaryFailureDefinition(FString FailureType)
	{
		FActorVariation StartTimeVariation;
		StartTimeVariation.Id = FailureType + TEXT("_Start");
		StartTimeVariation.Type = EActorAttributeType::Float;
		StartTimeVariation.RecommendedValues = { TEXT("0") };
		StartTimeVariation.bRestrictToRecommended = false;


		FActorVariation DurationVariation;
		DurationVariation.Id = FailureType + TEXT("_Duration");
		DurationVariation.Type = EActorAttributeType::Float;
		DurationVariation.RecommendedValues = { TEXT("0") };
		DurationVariation.bRestrictToRecommended = false;


		FActorVariation IntervalVariaton;
		IntervalVariaton.Id = FailureType + TEXT("_Interval");
		IntervalVariaton.Type = EActorAttributeType::Float;
		IntervalVariaton.RecommendedValues = { TEXT("0") };
		IntervalVariaton.bRestrictToRecommended = false;


		FActorVariation IntervalDegradationVariation;
		IntervalDegradationVariation.Id = FailureType + TEXT("_IntervalDegradation");
		IntervalDegradationVariation.Type = EActorAttributeType::Float;
		IntervalDegradationVariation.RecommendedValues = { TEXT("0") };
		IntervalDegradationVariation.bRestrictToRecommended = false;


		FActorVariation DurationDegradationVariation;
		DurationDegradationVariation.Id = FailureType + TEXT("_DurationDegradation");
		DurationDegradationVariation.Type = EActorAttributeType::Float;
		DurationDegradationVariation.RecommendedValues = { TEXT("0") };
		DurationDegradationVariation.bRestrictToRecommended = false;


		FActorVariation SeedVariation;
		SeedVariation.Id = FailureType + TEXT("_seed");
		SeedVariation.Type = EActorAttributeType::Int;
		SeedVariation.RecommendedValues = { TEXT("0") };
		SeedVariation.bRestrictToRecommended = false;

		return { StartTimeVariation, DurationVariation, IntervalVariaton,IntervalDegradationVariation, DurationDegradationVariation,SeedVariation };
	}
	void Set(const FActorDescription& ActorDescription, FString FailureType,float CurTime)
	{
		if (ActorDescription.Variations.Contains(FailureType + TEXT("_Start")))
			Start = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_Start"), ActorDescription.Variations, 0.0f) + CurTime;

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_Duration")))
			Duration = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_Duration"), ActorDescription.Variations, 0.0f);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_Interval")))
			Interval = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_Interval"), ActorDescription.Variations, 0.0f);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_IntervalDegradation")))
			IntervalDegradation = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_IntervalDegradation"), ActorDescription.Variations, 0.0f);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_DurationDegradation")))
			DurationDegradation = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_DurationDegradation"), ActorDescription.Variations, 0.0f);


		if (ActorDescription.Variations.Contains(FailureType + TEXT("_seed")))
			gen_uniform.seed(UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(FailureType + TEXT("_seed"), ActorDescription.Variations, 0.0f));
	}

};

class PackageLoss : public SensorFailure
{
public:
	static TArray<FActorVariation> CreateFailureDefinition() 
	{
		TArray<FActorVariation> VariationArray = CreatePrimaryFailureDefinition(FailureType);

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription,float CurTime)
	{
		SensorFailure::Set(ActorDescription, FailureType,CurTime);
	}
private:
	static FString FailureType;
};

template<class  T>
class PackageDelay : public SensorFailure
{
private:
	static FString FailureType;
	int WritePtr = 0;
	int ReadPtr = 0;
	int MaxUseSize = MaxRingBufferSize - 1;
	int DegradationSize = 0;
	int DelaySize = 0;
	int CurSetDelayed = 0;

	T DelayBuffer[MaxRingBufferSize];
public:
	static TArray<FActorVariation> CreateFailureDefinition()
	{
		TArray<FActorVariation> VariationArray = CreatePrimaryFailureDefinition(PackageDelay::FailureType);

		FActorVariation DegradationSizeVariation;
		DegradationSizeVariation.Id = FailureType + TEXT("_DegradationSize");
		DegradationSizeVariation.Type = EActorAttributeType::Int;
		DegradationSizeVariation.RecommendedValues = { TEXT("0") };
		DegradationSizeVariation.bRestrictToRecommended = false;

		FActorVariation DelaySizeVariation;
		DelaySizeVariation.Id = FailureType + TEXT("_DelaySize");
		DelaySizeVariation.Type = EActorAttributeType::Int;
		DelaySizeVariation.RecommendedValues = { TEXT("0") };
		DelaySizeVariation.bRestrictToRecommended = false;

		FActorVariation RingBufferMaxUseSizeVariation;
		RingBufferMaxUseSizeVariation.Id = FailureType + TEXT("_RingBufferMaxUseSize");
		RingBufferMaxUseSizeVariation.Type = EActorAttributeType::Int;
		RingBufferMaxUseSizeVariation.RecommendedValues = { TEXT("0") };
		RingBufferMaxUseSizeVariation.bRestrictToRecommended = false;

		VariationArray.Add(DegradationSizeVariation);
		VariationArray.Add(DelaySizeVariation);
		VariationArray.Add(RingBufferMaxUseSizeVariation);
		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, float CurTime)
	{
		SensorFailure::Set(ActorDescription, PackageDelay::FailureType, CurTime);
		if (ActorDescription.Variations.Contains(FailureType + TEXT("_DegradationSize")))
			DegradationSize = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(FailureType + TEXT("_DegradationSize"), ActorDescription.Variations, 0);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_DelaySize")))
			DelaySize = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(FailureType + TEXT("_DelaySize"), ActorDescription.Variations, 0);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_RingBufferMaxUseSize")))
		{
			MaxUseSize = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(FailureType + TEXT("_RingBufferMaxUseSize"), ActorDescription.Variations, 0);
			if(MaxUseSize > MaxRingBufferSize - 1)
				MaxUseSize = MaxRingBufferSize - 1;
		}
	}

	void Delay(T& package)
	{
		if (!ScenarioActive)
			return;

		DelayBuffer[WritePtr].SetDetections(package.GetDetections());
		WritePtr++;
		if (WritePtr >= MaxRingBufferSize || WritePtr >= MaxUseSize)
			WritePtr = 0;
	}

	void GetDelayedPackage(T& package) 
	{
		if (!ScenarioActive)
			return;
		package.SetDetections(DelayBuffer[ReadPtr].GetDetections());
		ReadPtr++;
		if (ReadPtr >= MaxRingBufferSize || ReadPtr >= MaxUseSize)
			ReadPtr = 0;
	}

	bool IsScenarioActive(float CurTime) override
	{
		if (ScenarioActive)
		{
			if (CurTime >= Start)
			{
				CurSetDelayed++;
				if (CurSetDelayed >= DelaySize)
				{
					CurSetDelayed = 0;
					DelaySize -= DegradationSize;

					Degredation();
				}
				return true;
			}
		}
		return false;
	}
};

class ShiftSensor : public SensorFailure
{
private:
	static FString FailureType;
	float Yaw = .0f;
	float Roll = .0f;
	float Pitch = .0f;
	bool ConstantShiftFlag = false;	
public:
	static TArray<FActorVariation> CreateFailureDefinition()
	{
		TArray<FActorVariation> VariationArray = CreatePrimaryFailureDefinition(FailureType);

		FActorVariation YawVariation;
		YawVariation.Id = FailureType + TEXT("_Yaw");
		YawVariation.Type = EActorAttributeType::Float;
		YawVariation.RecommendedValues = { TEXT("0") };
		YawVariation.bRestrictToRecommended = false;
		VariationArray.Add(YawVariation);

		FActorVariation RollVariation;
		RollVariation.Id = FailureType + TEXT("_Roll");
		RollVariation.Type = EActorAttributeType::Float;
		RollVariation.RecommendedValues = { TEXT("0") };
		RollVariation.bRestrictToRecommended = false;
		VariationArray.Add(RollVariation);

		FActorVariation PitchVariation;
		PitchVariation.Id = FailureType + TEXT("_Pitch");
		PitchVariation.Type = EActorAttributeType::Float;
		PitchVariation.RecommendedValues = { TEXT("0") };
		PitchVariation.bRestrictToRecommended = false;
		VariationArray.Add(PitchVariation);

		FActorVariation ConstantShiftVariation;
		ConstantShiftVariation.Id = FailureType + TEXT("_ConstantShiftFlag");
		ConstantShiftVariation.Type = EActorAttributeType::Bool;
		ConstantShiftVariation.RecommendedValues = { TEXT("False") };
		ConstantShiftVariation.bRestrictToRecommended = false;
		VariationArray.Add(ConstantShiftVariation);

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, float CurTime)
	{
		SensorFailure::Set(ActorDescription, FailureType, CurTime);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_Yaw")))
			Yaw = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_Yaw"), ActorDescription.Variations, 0.f);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_Roll")))
			Roll = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_Roll"), ActorDescription.Variations, 0.f);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_Pitch")))
			Pitch = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_Pitch"), ActorDescription.Variations, 0.f);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_ConstantShiftFlag")))
			ConstantShiftFlag = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToBool(FailureType + TEXT("_ConstantShiftFlag"), ActorDescription.Variations, false);

	}

	void UpdateSensor(ASensor* Sensor) 
	{
		if (!ConstantShiftFlag)
			Degredation();
		auto RadarRot = Sensor->GetActorRotation();
		FRotator rot(Pitch, Yaw, Roll);
		RadarRot += rot;
		Sensor->SetActorRotation(RadarRot);
	}

};

template<class T>
class PointDataShift : public SensorFailure 
{
protected:
	static FString FailureType;

	float MaxShift;
	float PossibilityToShiftPoint;
public:
	static TArray<FActorVariation> CreateFailureDefinition(FString TypeShift)
	{
		TArray<FActorVariation> VariationArray = CreatePrimaryFailureDefinition(TypeShift + TEXT("_") + FailureType);

		FActorVariation MaxShiftVariation;
		MaxShiftVariation.Id = TypeShift + TEXT("_") + FailureType + TEXT("_MaxShift");
		MaxShiftVariation.Type = EActorAttributeType::Float;
		MaxShiftVariation.RecommendedValues = { TEXT("0") };
		MaxShiftVariation.bRestrictToRecommended = false;
		VariationArray.Add(MaxShiftVariation);

		FActorVariation PossibilityToShiftPointtVariation;
		PossibilityToShiftPointtVariation.Id = TypeShift + TEXT("_") + FailureType + TEXT("_PossibilityToShiftPoint");
		PossibilityToShiftPointtVariation.Type = EActorAttributeType::Float;
		PossibilityToShiftPointtVariation.RecommendedValues = { TEXT("0") };
		PossibilityToShiftPointtVariation.bRestrictToRecommended = false;
		VariationArray.Add(PossibilityToShiftPointtVariation);


		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, FString TypeShift, float CurTime)
	{
		SensorFailure::Set(ActorDescription, TypeShift + TEXT("_") + FailureType, CurTime);

		if (ActorDescription.Variations.Contains(TypeShift + TEXT("_") + FailureType + TEXT("_MaxShift")))
			MaxShift = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(TypeShift + TEXT("_") + FailureType + TEXT("_MaxShift"), ActorDescription.Variations, 0.f);
		if (ActorDescription.Variations.Contains(TypeShift + TEXT("_") + FailureType + TEXT("_PossibilityToShiftPoint")))
			PossibilityToShiftPoint = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(TypeShift + TEXT("_") + FailureType + TEXT("_PossibilityToShiftPoint"), ActorDescription.Variations, 0.f);

	}
	virtual void UpdateData(T& data) = 0;
};

template<class T>
class CoordinatenPointDataShift : public PointDataShift<T>
{
private:
	static FString TypeShift;
public:
	static TArray<FActorVariation> CreateFailureDefinition()
	{
		TArray<FActorVariation> VariationArray = PointDataShift::CreateFailureDefinition(TypeShift);

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, float CurTime)
	{
		PointDataShift::Set(ActorDescription, TypeShift, CurTime);

	}
	void UpdateData(T& data) override
	{		
		data.RandomizeCoords(uniform, gen_uniform, PossibilityToShiftPoint,MaxShift);
	}
};

template<class T>
class AdditionalPointDataShift : public PointDataShift<T>
{
private:
	static FString TypeShift;
public:
	static TArray<FActorVariation> CreateFailureDefinition()
	{
		TArray<FActorVariation> VariationArray = PointDataShift::CreateFailureDefinition(TypeShift);

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, float CurTime)
	{
		PointDataShift::Set(ActorDescription, TypeShift, CurTime);

	}
	void UpdateData(T& data) override
	{
		float Changed = data.RandomizeAdditionalData(uniform, gen_uniform, PossibilityToShiftPoint, MaxShift);
		//UE_LOG(LogTemp, Warning, TEXT("Changed %f Points"), Changed);
	}
};

class RangeReduction : public SensorFailure
{
public:
	static TArray<FActorVariation> CreateFailureDefinition()
	{
		TArray<FActorVariation> VariationArray = CreatePrimaryFailureDefinition(FailureType);

		FActorVariation RangeReductionVariation;
		RangeReductionVariation.Id = FailureType + TEXT("_Range");
		RangeReductionVariation.Type = EActorAttributeType::Float;
		RangeReductionVariation.RecommendedValues = { TEXT("0") };
		RangeReductionVariation.bRestrictToRecommended = false;
		VariationArray.Add(RangeReductionVariation);

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, float CurTime)
	{
		SensorFailure::Set(ActorDescription, FailureType, CurTime);

		if (ActorDescription.Variations.Contains(FailureType + TEXT("_Range")))
			RangeReduction = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(FailureType + TEXT("_Range"), ActorDescription.Variations, 0) * 100;
	}
	bool IsScenarioActive(float CurTime) override
	{
		if (ScenarioActive)
		{
			if (CurTime >= Start && !StartCheckd)
			{
				InitFailure = true;
				StartCheckd = true;
				return true;
			}
			if (CurTime >= Start + Duration)
			{
				InitFailure = false;
				StartCheckd = false;
				Degredation();
				return true;
			}
		}
		return false;
	}
	void UpdateRange(float& Range) 
	{
		if (InitFailure)
			Range -= RangeReduction;
		else
			Range += RangeReduction;
	}
private:
	static FString FailureType;
	float RangeReduction;
	bool InitFailure = false;
	bool StartCheckd = false;
};

class AreaEffects : public SensorFailure 
{
public: 
	FVector CalculateEndPoint(float HorFov, float VerFov, float Range, const FVector& ActorLocation, const FTransform& ActorTransform)
	{
		Range *= 1e-2;
		float SpawnRange = uniform(gen_uniform) * Range;

		if (CloseRange)
			SpawnRange = 1.f;
		float VerFovDeg = VerFov * uniform(gen_uniform) * .5f;
		float HorFovDef = HorFov * uniform(gen_uniform) * .5f;
		float VerFovRand = FMath::DegreesToRadians(VerFovDeg);
		float HorFovRand = FMath::DegreesToRadians(HorFovDef);

		float NewHyptohenusis = FMath::Cos(VerFovRand) * SpawnRange;
		float NewY = FMath::Sin(HorFovRand) * NewHyptohenusis;
		float NewX = FMath::Cos(HorFovRand) * NewHyptohenusis;
		float NewZ = FMath::Sin(VerFovRand) * SpawnRange;

		FVector Helper = {	
		   NewX,
		   NewY,
		   NewZ
		};
		const FRotator& TransformRotator = ActorTransform.Rotator();
		FVector EndLocation = ActorLocation + TransformRotator.RotateVector(Helper);


		return EndLocation;
	}

	static TArray<FActorVariation> CreateFailureDefinition(FString Type)
	{
		TArray<FActorVariation> VariationArray = CreatePrimaryFailureDefinition(Type + TEXT("_") + FailureType);

		FActorVariation CloseRangeVariation;
		CloseRangeVariation.Id = Type + TEXT("_") + FailureType + TEXT("_CloseRange");
		CloseRangeVariation.Type = EActorAttributeType::Bool;
		CloseRangeVariation.RecommendedValues = { TEXT("False") };
		CloseRangeVariation.bRestrictToRecommended = false;
		VariationArray.Add(CloseRangeVariation);

		FActorVariation HorFlagVariation;
		HorFlagVariation.Id = Type + TEXT("_") + FailureType + TEXT("_HorizontalFlag");
		HorFlagVariation.Type = EActorAttributeType::Int;
		HorFlagVariation.RecommendedValues = { TEXT("0") };
		HorFlagVariation.bRestrictToRecommended = false;
		VariationArray.Add(HorFlagVariation);

		FActorVariation VertFlagVariation;
		VertFlagVariation.Id = Type + TEXT("_") + FailureType + TEXT("_VerticalFlag");
		VertFlagVariation.Type = EActorAttributeType::Int;
		VertFlagVariation.RecommendedValues = { TEXT("0") };
		VertFlagVariation.bRestrictToRecommended = false;
		VariationArray.Add(VertFlagVariation);

		FActorVariation AmmountVariation;
		AmmountVariation.Id = Type + TEXT("_") + FailureType + TEXT("_Ammount");
		AmmountVariation.Type = EActorAttributeType::Int;
		AmmountVariation.RecommendedValues = { TEXT("0") };
		AmmountVariation.bRestrictToRecommended = false;
		VariationArray.Add(AmmountVariation);

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, FString Type, float CurTime)
	{
		SensorFailure::Set(ActorDescription, Type + TEXT("_") + FailureType, CurTime);

		if (ActorDescription.Variations.Contains(Type + TEXT("_") + FailureType + TEXT("_CloseRange")))
			CloseRange = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToBool(Type + TEXT("_") + FailureType + TEXT("_CloseRange"), ActorDescription.Variations, 0);

		if (ActorDescription.Variations.Contains(Type + TEXT("_") + FailureType + TEXT("_HorizontalFlag")))
			HorFOV = (HorizontalFlag)UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(Type + TEXT("_") + FailureType + TEXT("_HorizontalFlag"), ActorDescription.Variations, 0);

		if (ActorDescription.Variations.Contains(Type + TEXT("_") + FailureType + TEXT("_VerticalFlag")))
			VertFOV = (VerticalFlag)UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(Type + TEXT("_") + FailureType + TEXT("_VerticalFlag"), ActorDescription.Variations, 0);

		if (ActorDescription.Variations.Contains(Type + TEXT("_") + FailureType + TEXT("_Ammount")))
			Ammount = (VerticalFlag)UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(Type + TEXT("_") + FailureType + TEXT("_Ammount"), ActorDescription.Variations, 0);
	}
protected:
	enum HorizontalFlag : int
	{
		Left = 0,
		WholeHorFOV = 1,
		Right = 2
	};
	enum VerticalFlag : int
	{
		Down = 0,
		WholeVerFOV = 1,
		Up = 2
	};
	static FString FailureType;

	HorizontalFlag HorFOV;
	VerticalFlag VertFOV;
	bool CloseRange = false;
	int Ammount;
};

class Blockage : public AreaEffects
{
public:
	~Blockage() 
	{
		for (AActor* ObjectB : BlockObjects)
		{
			if (ObjectB != nullptr)
			{
				ObjectB->Destroy();
			}
		}
	}
	static TArray<FActorVariation> CreateFailureDefinition()
	{
		TArray<FActorVariation> VariationArray = AreaEffects::CreateFailureDefinition(Type);

		FActorVariation RandomObjectLifeTimeVariation;
		RandomObjectLifeTimeVariation.Id = Type + TEXT("_") + FailureType + TEXT("_RandomObjectLifeTime");
		RandomObjectLifeTimeVariation.Type = EActorAttributeType::Bool;
		RandomObjectLifeTimeVariation.RecommendedValues = { TEXT("False") };
		RandomObjectLifeTimeVariation.bRestrictToRecommended = false;
		VariationArray.Add(RandomObjectLifeTimeVariation);

		FActorVariation MaxLifeTimeVariation;
		MaxLifeTimeVariation.Id = Type + TEXT("_") + FailureType + TEXT("_MaxLifeTime");
		MaxLifeTimeVariation.Type = EActorAttributeType::Float;
		MaxLifeTimeVariation.RecommendedValues = { TEXT("0") };
		MaxLifeTimeVariation.bRestrictToRecommended = false;
		VariationArray.Add(MaxLifeTimeVariation);

		FActorVariation LifeTimeVariation;
		LifeTimeVariation.Id = Type + TEXT("_") + FailureType + TEXT("_LifeTime");
		LifeTimeVariation.Type = EActorAttributeType::Float;
		LifeTimeVariation.RecommendedValues = { TEXT("0") };
		LifeTimeVariation.bRestrictToRecommended = false;
		VariationArray.Add(LifeTimeVariation);

		FActorVariation DropSpeedVariation;
		DropSpeedVariation.Id = Type + TEXT("_") + FailureType + TEXT("_DropSpeed");
		DropSpeedVariation.Type = EActorAttributeType::Float;
		DropSpeedVariation.RecommendedValues = { TEXT("0") };
		DropSpeedVariation.bRestrictToRecommended = false;
		VariationArray.Add(DropSpeedVariation);

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, float CurTime)
	{
		AreaEffects::Set(ActorDescription, Type, CurTime);

		if (ActorDescription.Variations.Contains(Type + TEXT("_") + FailureType + TEXT("_RandomObjectLifeTime")))
			RandomObjectLifeTime = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToBool(Type + TEXT("_") + FailureType + TEXT("_RandomObjectLifeTime"), ActorDescription.Variations, 0);

		if (ActorDescription.Variations.Contains(Type + TEXT("_") + FailureType + TEXT("_MaxLifeTime")))
			MaxLifeTime = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(Type + TEXT("_") + FailureType + TEXT("_MaxLifeTime"), ActorDescription.Variations, 0);

		if (ActorDescription.Variations.Contains(Type + TEXT("_") + FailureType + TEXT("_LifeTime")))
			LifeTime = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(Type + TEXT("_") + FailureType + TEXT("_LifeTime"), ActorDescription.Variations, 0);

		if (ActorDescription.Variations.Contains(Type + TEXT("_") + FailureType + TEXT("_DropSpeed")))
			DropSpeed = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(Type + TEXT("_") + FailureType + TEXT("_DropSpeed"), ActorDescription.Variations, 0);
	}
	void CreateBlockage(float HorFov, float VerFov, float Range, ASensor* Sensor, const FVector& ActorLocation, const FTransform& ActorTransform, UWorld* World)
	{
		for (int i = 0; i < Ammount; i++) 
		{
			const FVector EndLocation = CalculateEndPoint(HorFov, VerFov, Range, ActorLocation, ActorTransform); 
			AHexagonActor* Hexagon = World->SpawnActor<AHexagonActor>(EndLocation, FRotator(0.f, 0.f, 0.f));
			if (Hexagon) 
			{
				float Radius = FMath::FRandRange(.001f, .05f);//.0001f, .0050f);
				Hexagon->CreateHexagonMesh(Radius);

				FVector TargetLocation = ActorLocation;

				// Calculate the direction vector from your actor to the target location
				FVector Direction = (TargetLocation - Hexagon->GetActorLocation()).GetSafeNormal();

				// Calculate the new rotation for your actor
				FRotator NewRotation = Direction.Rotation() + FRotator(-90.f, 0.f, 0.f);

				// Apply the new rotation to your actor
				Hexagon->SetActorRotation(NewRotation);

				//Hexagon->SetActorRotation(this->GetActorRotation() + FRotator(90.f, 0.f, 0.f));
				Hexagon->AttachToActor(Sensor, FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
				Hexagon->SetOwner(Sensor);

				float _LifeTime = 0.1f;
				if (RandomObjectLifeTime)
					_LifeTime = uniform(gen_uniform) * MaxLifeTime;
				else
					_LifeTime = LifeTime;
				Hexagon->SetBlockageParamter(_LifeTime, DropSpeed);
				if (_LifeTime <= 0.0f)
					BlockObjects.Add(Hexagon);
			}
			else
				UE_LOG(LogTemp, Error, TEXT("Hexagon == nullptr!"));
		}
	}
protected:
	static FString Type;

private:
	TArray<AActor*> BlockObjects;
	bool RandomObjectLifeTime;
	float MaxLifeTime;
	float LifeTime;
	float DropSpeed;
};

class RandomPoints : public AreaEffects
{
public:
	template<class T> 
	void CreatePoints(T& data, float HorFov, float VerFov, float Range, const FVector& ActorLocation, const FTransform& ActorTransform)
	{
		for (int i = 0; i < Ammount; i++) 
		{
			FVector EndLocation = CalculateEndPoint(HorFov, VerFov, Range, ActorLocation, ActorTransform);
			float AdditionalValue = uniform(gen_uniform);

			const FVector TransformXAxis = ActorTransform.GetUnitAxis(EAxis::X);
			const FVector TransformYAxis = ActorTransform.GetUnitAxis(EAxis::Y);
			const FVector TransformZAxis = ActorTransform.GetUnitAxis(EAxis::Z);
			auto AziEle = FMath::GetAzimuthAndElevation((EndLocation - ActorLocation).GetSafeNormal() * Range,TransformXAxis,TransformYAxis,TransformZAxis);
			float dist = sqrt((EndLocation.X - ActorLocation.X) * (EndLocation.X - ActorLocation.X)
				+ (EndLocation.Y - ActorLocation.Y) * (EndLocation.Y - ActorLocation.Y)
				+ (EndLocation.Z - ActorLocation.Z) * (EndLocation.Z - ActorLocation.Z));
			data.AddPoint(AziEle.X, AziEle.Y, dist, EndLocation.X - ActorLocation.X, EndLocation.Y - ActorLocation.Y, EndLocation.Z - ActorLocation.Z, AdditionalValue);
		}
	}
	static TArray<FActorVariation> CreateFailureDefinition()
	{
		TArray<FActorVariation> VariationArray = AreaEffects::CreateFailureDefinition(Type);

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, float CurTime)
	{
		AreaEffects::Set(ActorDescription, Type, CurTime);
	}
protected:
	static FString Type;

};

class ShaderError : public SensorFailure 
{
public:
	ShaderError() {
		ShaderCol = FLinearColor(0.f, 0.f, 0.f, 0.f);
	}
	static TArray<FActorVariation> CreateFailureDefinition()
	{
		TArray<FActorVariation> VariationArray = CreatePrimaryFailureDefinition( FailureType);
	

		return VariationArray;
	}
	void Set(const FActorDescription& ActorDescription, float CurTime)
	{
		SensorFailure::Set(ActorDescription, FailureType, CurTime);

		
	}
	bool IsScenarioActive(float CurTime) override
	{
		if (ScenarioActive)
		{
			if (CurTime >= Start && !StartCheckd)
			{
				StartCheckd = true;
				return true;
			}
			if (CurTime >= Start + Duration)
			{
				StartCheckd = false;
				Degredation();
				return true;
			}
		}
		return false;
	}
	void UpdateShader(TArray<FSensorShader> Shaders)
	{
		if (StartCheckd)
			ShaderCol.A = 1.f;
		else
			ShaderCol.A = 0.f;
		Shaders[ShaderIndx].PostProcessMaterial->SetVectorParameterValue(FName("MyColor"), ShaderCol);
	}
	void SetShaderIndex(int id) { ShaderIndx = id; }
private:
	static FString FailureType;
	bool StartCheckd = false;
	FLinearColor ShaderCol;
	int ShaderIndx = 1;
};