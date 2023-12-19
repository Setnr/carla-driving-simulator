// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <random>

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"

#include "Carla/Sensor/Radar.h"
#include "FaultyRadar.generated.h"


#define RadarDelay_RingBufferSize 100
#define RandomEngineSeed 9857
//#define DEBUG_NONEXISITING_POINTS //DEFINE FOR SPOOFINGTEST

UCLASS()
class CARLA_API AFaultyRadar : public ARadar
{
  GENERATED_BODY()
public:

  static FActorDefinition GetSensorDefinition();

  AFaultyRadar(const FObjectInitializer &ObjectInitializer);

  void Set(const FActorDescription &Description) override;

  void AddScenario(int SzenarioID);
  float CalculateStartTime(float StartTime){
	return StartTime + GetWorld()->GetTimeSeconds();
	}
	enum Distribution : int
	{
		None = 0x0,
		Weibull = 0x1,
		Linear = 0x2
	};
#pragma region PackageLoss_Function
  void AddPackageLossInterval(float Interval){
	this->PackageLoss_Interval = Interval;}
  void AddPackageLossDuration(float Duration){
	this->PackageLoss_Duration = Duration;}
  void AddPackageLossStart(float StartTime){
	this->PackageLoss_Start = CalculateStartTime(StartTime);}
  void SetProgressionRate(float Rate){
	this->PackageLoss_IntervallDegradation = Rate;}
#pragma endregion

#pragma region PackageDelay_Function
	void SetPackageDelay_Start(float Start){
		PackageDelay_Start = CalculateStartTime(Start);;}
	void SetPackageDelay_Interval(float Interval){
		PackageDelay_Interval = Interval;
		PackageDelay_DegradationZeit = PackageDelay_Start + Interval;}
	void SetPackageDelay_DegradationSize(int Size){
		PackageDelay_DegradationSize = Size;}
	void SetPackageDelay_DelaySize(int Size){
		PackageDelay_DelaySize = Size;}
	void SetPackageDelay_RingBufferMaxUseSize(int Size){
		PackageDelay_RingBufferMaxUseSize = Size;}
#pragma endregion
  
#pragma region DetectionPointShift_Function
	void SetDetectionPointShift_Start(float StartTime){
		this->DetectionPointShift_Start = CalculateStartTime(StartTime);
	}
	void SetDetectionPointShift_Intervall(float Intervall){
		this->DetectionPointShift_Intervall = Intervall;
	}
	void SetDetectionPointShift_Duration(float Duration){
		this->DetectionPointShift_Duration = Duration;
	}
	void SetDetectionPointShift_IntervallDegradation(float IntDegre){
		this->DetectionPointShift_IntervallDegradation = IntDegre;
	}
	void SetDetectionPointShift_DurationDegradation(float DuraDegre){
		this->DetectionPointShift_DurationDegradation = DuraDegre;
	}
	void SetDetectionPoint_MaxDepthDisturbance(float MaxDepth){
		this->DetectionPoint_MaxDepthDisturbance = MaxDepth;
	}
	void SetDetectionPoint_MaxAzimuthDisturbance(float MaxAzi){
		this->DetectionPoint_MaxAzimuthDisturbance = MaxAzi;
	}
	void SetDetectionPoint_MaxAltitudeDisturbance(float MaxAlti){
		this->DetectionPoint_MaxAltitudeDisturbance = MaxAlti;
	}
	void SetDetectionPoint_Distribution(int Dist)
	{	
		this->DetectionPoint_Distribution = (Distribution)Dist;
	}
#pragma endregion
  
#pragma region VelocityShift_Function
	void SetVelocityShift_Start(float StartTime){
		this->VelocityShift_Start = CalculateStartTime(StartTime);
	}
	void SetVelocityShift_Intervall(float Intervall){
		this->VelocityShift_Intervall=Intervall;
	}
	void SetVelocityShift_Duration(float Duration){
		this->VelocityShift_Duration = Duration;
	}
	void SetVelocityShift_IntervallDegradation(float InterDegre){
		this->VelocityShift_IntervallDegradation = InterDegre;
	}
	void SetVelocityShift_DurationDegradation(float DuraDegre){
		this->VelocityShift_DurationDegradation = DuraDegre;
	}
	void SetVelocityShift_MaxVelocityDisturbance(float MaxVel){
		this->VelocityShift_MaxVelocityDisturbance = MaxVel;
	}
	void SetVelocityShift_Distribution(int Dist){
		this->VelocityShift_Distribution = (Distribution) Dist;
	}
#pragma endregion

#pragma region RangeReduction_Function
	void SetRangeReduction_Start(float StartTime){
		this->RangeReduction_Start = CalculateStartTime(StartTime);
	}
	void SetRangeReduction_Intervall(float Intervall){
		this->RangeReduction_Intervall = Intervall;
	}
	void SetRangeReduction_Duration(float Duration){
		this->RangeReduction_Duration = Duration;
	}
	void SetRangeReduction_IntervallDegradation(float IntDeg){
		this->RangeReduction_IntervallDegradation = IntDeg;
	}
	void SetRangeReduction_DurationDegradation(float DurDeg){
		this->RangeReduction_DurationDegradation = DurDeg;
	}
	void SetRangeReduction_RangeReductionValue(float RangeRed){
		this->RangeReduction_RangeReductionValue = RangeRed;
	}
#pragma endregion
  
#pragma region DetectNonExistingPoints_Function

	void SetDetectNonExistingPoints_Start(float StartTime) {
		this->DetectNonExistingPoints_Start = CalculateStartTime(StartTime);
	}
	void SetDetectNonExistingPoints_Intervall(float Intervall) {
		this->DetectNonExistingPoints_Intervall = Intervall;
	}
	void SetDetectNonExistingPoints_Duration(float Duration) {
		this->DetectNonExistingPoints_Duration = Duration;
	}
	void SetDetectNonExistingPoints_IntervallDegradation(float InterDeg) {
		this->DetectNonExistingPoints_IntervallDegradation = InterDeg;
	}
	void SetDetectNonExistingPoints_DurationDegradation(float DuraDeg) {
		this->DetectNonExistingPoints_DurationDegradation = DuraDeg;
	}
	void SetDetectNonExistingPoints_AmmountDetections(int MaxAmmount) {
		this->DetectNonExistingPoints_AmmountDetections = MaxAmmount;
	}
	 
	void SetDetectNonExistingPoints_HorFOVFlag(int HorFov) {
		if (HorFov <= 2 && HorFov >= 0)
			this->DetectNonExistingPoints_HorFOVFlag = (HorizontalFOV_Type)HorFov;
		else
			this->DetectNonExistingPoints_HorFOVFlag = HorizontalFOV_Type::WholeHorFOV;
	}
	void SetDetectNonExistingPoints_VertFOVFlag(int VertFov) {
		if (VertFov <= 2 && VertFov >= 0)
			this->DetectNonExistingPoints_VertFOVFlag = (VerticalFOV_Type)VertFov;
		else
			this->DetectNonExistingPoints_VertFOVFlag = VerticalFOV_Type::WholeVerFOV;
	}
#pragma endregion

#pragma region SensorShift_Function
	void SetSensorShift_Start(float StartTime) {
		this->SensorShift_Start = CalculateStartTime(StartTime);
	}
	void SetSensorShift_Intervall(float Intervall) {
		this->SensorShift_Intervall = Intervall;
	}
	void SetSensorShift_Duration(float Duration) {
		this->SensorShift_Duration = Duration;
	}
	void SetSensorShift_IntervallDegradation(float InterDegre) {
		this->SensorShift_IntervallDegradation = InterDegre;
	}
	void SetSensorShift_DurationDegradation(float DuraDegre) {
		this->SensorShift_DurationDegradation = DuraDegre;
	}
	void SetSensorShift_Yaw(float Yaw) {
		this->SensorShift_Yaw = Yaw;
	}
	void SetSensorShift_Pitch(float Pitch) {
		this->SensorShift_Pitch = Pitch;
	}
	void SetSensorShift_Roll(float Roll) {
		this->SensorShift_Roll = Roll;
	}
	 
	void SetSensorShiftFlag(int Flag) {
		if (Flag >= 0 && Flag <= 1)
			this->SensorShiftFlag = (SensorShift_Flag)Flag;
		else
			this->SensorShiftFlag = SensorShift_Flag::ConstantShift;
	}
	void SetSensorShiftTriggerFlag(int Flag) 
	{
		if (Flag >= 0 && Flag <= 1)
			this->SensorShiftTriggerFlag = (SensorShift_TriggerFlag)Flag;
		else
			this->SensorShiftTriggerFlag = SensorShift_TriggerFlag::Timed;
	}
#pragma endregion

  void EventShift();
  void MoveRadar(FRotator rot);
  void MoveRadar(); // Moves the Radar -> Used For RadarCollosionShift
  void SetDurationDegradation(float DurationDegradation)
  {
	this->PackageLoss_DurationDegradation = DurationDegradation;
  }
protected:
	virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)override;

private:
	
	enum ScenarioID : int
	{
		PackageLoss = 0x1,
		PackageDelay = 0x2,
		DetectionPointShift = 0x4,
		VelocityShift = 0x8,
		RangeReduction = 0x10,
		DetectNonExistingPoints = 0x20,

		SensorShift = 0x40,
		SensorBlockage = 0x80
	};

	int Scenario;
	
	float PackageLoss_Interval;
	float PackageLoss_Duration;
	float PackageLoss_Start;
	float PackageLoss_IntervallDegradation;
	float PackageLoss_DurationDegradation;
	bool HasToLooseCurrentPackage();

	float PackageDelay_Start; // Wann tritt der Fehler zuerst auf
	float PackageDelay_DegradationZeit; //Hilfsvariable um die Intervalle zu tracken
	float PackageDelay_Interval; //In welchem Zeitintervall verschlechter sich der Fehler
	int PackageDelay_DegradationSize; // Wie viele Packete werden nach jedem Intervall neu versetzt
	int PackageDelay_DelaySize; //Wie viele Packete sind versetzt, zu diesem Zeitpunkt in der Sim.
								// Kann initial gesetzt werden um zu sagen wie viele Packete beim ersten Auftreten versetzt werden sollen.
	int PackageDelay_WaitCounter; // Wie viele Packete sind zu dem Zeitpunkt schon versetzt

	std::vector<RayData> PackageDelay_RingBuffer[RadarDelay_RingBufferSize] ;
	int PackageDelay_WriteRingBufferPtr;
	int PackageDelay_ReadRingBufferPtr;
	int PackageDelay_RingBufferMaxUseSize;

	float DetectionPointShift_Start;
	float DetectionPointShift_Intervall;
	float DetectionPointShift_Duration;
	float DetectionPointShift_IntervallDegradation;
	float DetectionPointShift_DurationDegradation;
	float DetectionPoint_MaxDepthDisturbance;
	float DetectionPoint_MaxAzimuthDisturbance;
	float DetectionPoint_MaxAltitudeDisturbance;
	Distribution DetectionPoint_Distribution;
	void ShiftDetectionPoints();

	float VelocityShift_Start;
	float VelocityShift_Intervall;
	float VelocityShift_Duration;
	float VelocityShift_IntervallDegradation;
	float VelocityShift_DurationDegradation;
	float VelocityShift_MaxVelocityDisturbance;
	Distribution VelocityShift_Distribution;
	void ShiftVelocitys();

	float RangeReduction_Start;
	float RangeReduction_Intervall;
	float RangeReduction_Duration;
	float RangeReduction_IntervallDegradation;
	float RangeReduction_DurationDegradation;
	float RangeReduction_RangeReductionValue;
	float RangeReduction_OldRangeValue;
	bool RangeReduction_Active;
	void CheckRangeReduction();

	enum HorizontalFOV_Type : int
	{
		Left = 0,
		WholeHorFOV = 1,
		Right = 2
	};
	enum VerticalFOV_Type : int
	{
		Down = 0,
		WholeVerFOV = 1,
		Up = 2
	};
	float DetectNonExistingPoints_Start;
	float DetectNonExistingPoints_Intervall;
	float DetectNonExistingPoints_Duration;
	float DetectNonExistingPoints_IntervallDegradation;
	float DetectNonExistingPoints_DurationDegradation;
	int DetectNonExistingPoints_AmmountDetections;
	HorizontalFOV_Type DetectNonExistingPoints_HorFOVFlag;
	VerticalFOV_Type DetectNonExistingPoints_VertFOVFlag;

	void DetectNonExisitingPoints();
  	void CreatePoints();
	void TestEndPoints();
	FVector CalculateEndLocation();

	enum SensorShift_Flag : int 
	{
		ConstantShift = 0,
		JumpingShift = 1
	};
	enum SensorShift_TriggerFlag : int 
	{
		Timed = 0,
		Collision = 1
	};

	float SensorShift_Start;
	float SensorShift_Intervall;
	float SensorShift_Duration;
	float SensorShift_IntervallDegradation;
	float SensorShift_DurationDegradation;
	float SensorShift_Yaw;
	float SensorShift_Pitch;
	float SensorShift_Roll;
	SensorShift_Flag SensorShiftFlag;
	SensorShift_TriggerFlag SensorShiftTriggerFlag;

	void ShiftSensor();

  void GenerateHexagon(int Ammount);


  virtual void WriteLineTraces();
  void BeginPlay() override;
  void DrawRadarBorder();


  bool once = true;
  virtual void OnConstruction(const FTransform& Transform) override;
  virtual void Destroyed() override;
  TArray<AActor*> BlockObjects;

  bool MoveOnce = true;
	
	std::mt19937 gen_weibull;
	std::weibull_distribution<float> weibull;
	std::mt19937 gen_uniform;
	std::uniform_real_distribution<float> uniform;
	float CreateRandomNumber(Distribution DistType);
};
