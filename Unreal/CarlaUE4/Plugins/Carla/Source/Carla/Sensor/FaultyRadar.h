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

  void MoveRadar();

private:

	bool isBlocked;
	enum ScenarioID 
	{
		RadarBlocked = 0x1,
		RadarLooseContact = 0x2,
		RadarShifted = 0x4,
		RadarVibration = 0x8,
		RadarVictim = 0x10
	};

	int Scenario;
	
	float LooseContact_Interval;
	float LooseContact_Duration;
	float LooseContact_StartOffset;
	float LooseContact_Start;

  virtual void WriteLineTraces();
  void BeginPlay() override;
  
};
