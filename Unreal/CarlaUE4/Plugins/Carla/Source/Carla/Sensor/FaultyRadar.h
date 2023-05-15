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



UCLASS()
class CARLA_API AFaultyRadar : public ARadar
{
  GENERATED_BODY()
public:

  static FActorDefinition GetSensorDefinition();

  AFaultyRadar(const FObjectInitializer &ObjectInitializer);

  void Set(const FActorDescription &Description) override;

  void AddScenario(int SzenarioID);
  void AddLooseContactInterval(float Interval);
  void AddLooseContactDuration(float Duration);
  void AddLooseContactStart(float StartTime);
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
  void MoveRadar(FRotator rot);
  void MoveRadar(); // Moves the Radar -> Used For RadarCollosionShift

private:
	bool isBlocked;
	enum ScenarioID 
	{
		RadarBlocked = 0x1,
		RadarLooseContact = 0x2,
		RadarConstantShift = 0x4,
		RadarVibration = 0x8,
		RadarDisturbance = 0x10,
		RadarRandomShift = 0x20,
		RadarCollosionShift = 0x40,
		RadarInterference = 0x80
	};

	int Scenario;
	
	float LooseContact_Interval;
	float LooseContact_Duration;
	float LooseContact_StartOffset;
	float LooseContact_Start;
	
	float LooseContact_ProgressionRate;

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

	void GenerateHexagon(int Ammount);


  virtual void WriteLineTraces();
  void BeginPlay() override;
  
  void DisturbeRadar();
  void SpoofRadar();


  bool once = true;
  virtual void OnConstruction(const FTransform& Transform) override;
  virtual void Destroyed() override;
  TArray<AActor*> BlockObjects;
};
