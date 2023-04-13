// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

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
		RadarVictim = 0x10,
		RadarRandomShift = 0x20,
		RadarCollosionShift = 0x80
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

  virtual void WriteLineTraces();
  void BeginPlay() override;
  
};
