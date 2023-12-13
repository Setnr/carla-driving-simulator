// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"

#include "Carla/Sensor/Radar.h"
#include "FaultyRadar.generated.h"

#define RadarDelay_RingBufferSize 100

UCLASS()
class CARLA_API AFaultyRadar : public ARadar
{
  GENERATED_BODY()
public:

  static FActorDefinition GetSensorDefinition();

  AFaultyRadar(const FObjectInitializer &ObjectInitializer);

  void Set(const FActorDescription &Description) override;

  void AddScenario(int SzenarioID);
  void AddPackageLossInterval(float Interval);
  void AddPackageLossDuration(float Duration);
  void AddPackageLossStart(float StartTime);
  void SetProgressionRate(float Rate);
  void SetConstantShiftRotation(FString string);
  void SetConstantShiftInterval(float Interval) {
	  this->ConstantShift_Interval = Interval;
  }
  void SetConstantShiftStart(float Start) {
	  this->ConstantShift_StartOffset = Start;
  }
  void SetRandomShiftStart(float Start) {
	  this->RandomShift_Start = Start;
  }
  void SetRandomShiftEnd(float End) {
	  this->RandomShift_End = End;
  }
  void SetRandomShiftStartOffset(float Start) {
	  RandomShif_StartOffset = Start;
  }

  void SetRadarDisturbance_Interval(float Interval) {
	  this->RadarDisturbance_Interval = Interval;
  }
  void SetRadarDisturbance_Duration(float Duration) {
	  this->RadarDisturbance_Duration = Duration;
  }
  void SetRadarDisturbance_StartOffset(float Offset) {
	  this->RadarDisturbance_StartOffset = Offset;
  }
  void SetRadarDisturbance_ProgressionRate(float ProgressionRate) {
	  this->RadarDisturbance_ProgressionRate = ProgressionRate;
  }

  void SetRadarSpoof_Interval(float Interval) {
	  this->RadarSpoof_Interval = Interval;
  }
  void SetRadarSpoof_Duration(float Duration) {
	  this->RadarSpoof_Duration = Duration;
  }
  void SetRadarSpoof_StartOffset(float Offset) {
	  this->RadarSpoof_StartOffset = Offset;
  }
  void SetRadarSpoof_ProgressionRate(float ProgressionRate) {
	  this->RadarSpoof_ProgressionRate = ProgressionRate;
  }
  void SetRadarSpoof_CutOff(float CutOff) {
	  this->RadarSpoof_CutOff = CutOff;
  }

  void SetBlockageStart(float Start) {
	  this->Blockage_Start = Start;
  }
  void SetBlockageInterval(float Interval) {
	  this->Blockage_Interval = Interval;
  }
  void SetBlockageHexagonAmmounts(int Ammount) {
	  this->Blockage_HexagonAmmounts = Ammount;
  }
  void MoveRadar(FRotator rot);
  void MoveRadar(); // Moves the Radar -> Used For RadarCollosionShift
  void SetDurationDegradation(float DurationDegradation)
  {
	this->PackageLoss_DurationDegradation = DurationDegradation;
  }
private:
	bool isBlocked;
	enum ScenarioID 
	{
		None = 0x0,
		RadarBlocked = 0x1,
		RadarPackageLoss = 0x2,
		RadarConstantShift = 0x4,
		RadarVibration = 0x8,
		RadarDisturbance = 0x10,
		RadarRandomShift = 0x20,
		RadarCollosionShift = 0x40,
		RadarInterference = 0x80,
		RadarBlockage = 0x100,
		RadarPackageDelay = 0x200
	};

	int Scenario;
	
	float PackageLoss_Interval;
	float PackageLoss_Duration;
	float PackageLoss_StartOffset;
	float PackageLoss_Start;

	float PackageLoss_IntervallDegradation;
	float PackageLoss_DurationDegradation;


	float PackageDelay_Start; // Wann tritt der Fehler zuerst auf
	float PackageDelay_DegradationZeit; //Hilfsvariable um die Intervalle zu tracken
	float PackageDelay_Interval; //In welchem Zeitintervall verschlechter sich der Fehler
	int PackageDelay_DegradationSize; // Wie viele Packete werden nach jedem Intervall neu versetzt
	int PackageDelay_DelaySize; //Wie viele Packete sind versetzt, zu diesem Zeitpunkt in der Sim.
								// Kann initial gesetzt werden um zu sagen wie viele Packete beim ersten Auftreten versetzt werden sollen.
	int PackageDelay_WaitCounter; // Wie viele Packete sind zu dem Zeitpunkt schon versetzt

	std::vector<RayData>[RadarDelay_RingBufferSize] PackageDelay_RingBuffer
	int PackageDelay_WriteRingBufferPtr;
	int PackageDelay_ReadRingBufferPtr
	int PackageDelay_RingBufferMaxUseSize;


	void SetPackageDelay_Start(float Start)
	{
		PackageDelay_Start = Start;
	}
	void SetPackageDelay_Interval(float Interval)
	{
		PackageDelay_Interval = Interval;
		PackageDelay_DegradationZeit = PackageDelay_Start + Interval;
	}
	void SetPackageDelay_DegradationSize(int Size){
		PackageDelay_DegradationSize = Size;
	}
	void SetPackageDelay_DelaySize(int Size){
		PackageDelay_DelaySize = Size;
	}
	void SetPackageDelay_RingBufferMaxUseSize(int Size){
		PackageDelay_RingBufferMaxUseSize = Size;
	}

	FRotator ConstantShift_Rotation;
	float ConstantShift_Interval;
	float ConstantShift_Start;
	float ConstantShift_StartOffset;

	float RandomShift_Start;
	float RandomShift_End;
	float RandomShift_Time = 0.0f;
	float RandomShif_StartOffset;

	float RadarDisturbance_Interval;
	float RadarDisturbance_Duration;
	float RadarDisturbance_StartOffset;
	float RadarDisturbance_Start;
	float RadarDisturbance_ProgressionRate;

	float RadarSpoof_Interval;
	float RadarSpoof_Duration;
	float RadarSpoof_StartOffset;
	float RadarSpoof_Start;
	float RadarSpoof_ProgressionRate;
	float RadarSpoof_CutOff;

	float Blockage_Start;
	float Blockage_Interval;
	int Blockage_HexagonAmmounts;

	void GenerateHexagon(int Ammount);


  virtual void WriteLineTraces();
  void BeginPlay() override;
  
  void DisturbeRadar();
  void SpoofRadar();


  bool once = true;
  virtual void OnConstruction(const FTransform& Transform) override;
  virtual void Destroyed() override;
  TArray<AActor*> BlockObjects;

  bool MoveOnce = true;
};
