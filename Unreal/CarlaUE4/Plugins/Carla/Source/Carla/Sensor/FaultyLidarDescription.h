// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <carla/sensor/data/LidarData.h>
#include "Engine/World.h"
#define LidarDelay_RingBufferSize 100
#define RandomEngineSeed 9857
//using FLidarData = carla::sensor::data::LidarData;
//#include "FaultyLidarDescription.generated.h"
struct FFaultyLidarDescription
{
	float CalculateStartTime(float StartTime) {
		return StartTime + World->GetTimeSeconds();
	}
    enum Distribution : int
    {
        None = 0x0,
        Weibull = 0x1,
        Linear = 0x2
    };
	UWorld* World;
  bool isBlocked;
  enum ScenarioID : int
  {
	  PackageLoss = 0x1, //
	  PackageDelay = 0x2, //
	  DetectionPointShift = 0x4, //
	  VelocityShift = 0x8, //
	  RangeReduction = 0x10, //
	  DetectNonExistingPoints = 0x20,//
	  SensorShift = 0x40,
	  SensorBlockage = 0x80
  };

  int Scenario;
  void AddScenario(int ScenarioID)
  {
      if (!(this->Scenario & ScenarioID))
          this->Scenario |= ScenarioID;

  }
  


  float PackageLoss_Interval;
  float PackageLoss_Duration;
  float PackageLoss_Start;
  float PackageLoss_IntervalDegradation;
  float PackageLoss_DurationDegradation;
  
  float PackageDelay_Start; // Wann tritt der Fehler zuerst auf
  float PackageDelay_DegradationZeit; //Hilfsvariable um die Intervale zu tracken
  float PackageDelay_Interval; //In welchem ZeitInterval verschlechter sich der Fehler
  int PackageDelay_DegradationSize; // Wie viele Packete werden nach jedem Interval neu versetzt
  int PackageDelay_DelaySize; //Wie viele Packete sind versetzt, zu diesem Zeitpunkt in der Sim.
							  // Kann initial gesetzt werden um zu sagen wie viele Packete beim ersten Auftreten versetzt werden sollen.
  int PackageDelay_WaitCounter; // Wie viele Packete sind zu dem Zeitpunkt schon versetzt

  carla::sensor::data::LidarData PackageDelay_RingBuffer[LidarDelay_RingBufferSize];
  int PackageDelay_WriteRingBufferPtr;
  int PackageDelay_ReadRingBufferPtr;
  int PackageDelay_RingBufferMaxUseSize;

  float DetectionPointShift_Start;
  float DetectionPointShift_Interval;
  float DetectionPointShift_Duration;
  float DetectionPointShift_IntervalDegradation;
  float DetectionPointShift_DurationDegradation;
  float DetectionPoint_MaxDepthDisturbance;
  float DetectionPoint_MaxAzimuthDisturbance;
  float DetectionPoint_MaxAltitudeDisturbance;
  Distribution DetectionPoint_Distribution;

  float VelocityShift_Start;
  float VelocityShift_Interval;
  float VelocityShift_Duration;
  float VelocityShift_IntervalDegradation;
  float VelocityShift_DurationDegradation;
  float VelocityShift_MaxVelocityDisturbance;
  Distribution VelocityShift_Distribution;

  float RangeReduction_Start;
  float RangeReduction_Interval;
  float RangeReduction_Duration;
  float RangeReduction_IntervalDegradation;
  float RangeReduction_DurationDegradation;
  float RangeReduction_RangeReductionValue;
  float RangeReduction_OldRangeValue;
  bool RangeReduction_Active;

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
  float DetectNonExistingPoints_Interval;
  float DetectNonExistingPoints_Duration;
  float DetectNonExistingPoints_IntervalDegradation;
  float DetectNonExistingPoints_DurationDegradation;
  int DetectNonExistingPoints_AmountDetections;
  HorizontalFOV_Type DetectNonExistingPoints_HorFOVFlag;
  VerticalFOV_Type DetectNonExistingPoints_VertFOVFlag;

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
  float SensorShift_Interval;
  float SensorShift_Duration;
  float SensorShift_IntervalDegradation;
  float SensorShift_DurationDegradation;
  float SensorShift_Yaw;
  float SensorShift_Pitch;
  float SensorShift_Roll;
  SensorShift_Flag SensorShiftFlag;
  SensorShift_TriggerFlag SensorShiftTriggerFlag;

  void ShiftSensor();

  enum SensorBlockage_BlockageType : int
  {
	  CloseRange = 0, //Blockages like Mud/Snow/Ice
	  FullRange = 1 //EveryObject is at a Random point within the FOV for e.g Rain
  };
  enum SensorBlockage_ObjectLifeTime : int
  {
	  Static = 0, //Object has a Fixed LifeTime Negative LifeTime => Permanent LifeTime
	  Random = 1  //Object has a random lifeTime (for e.g melting Ice)
  };
  float SensorBlockage_Start;
  float SensorBlockage_Interval;
  float SensorBlockage_IntervalDegradation;

  int SensorBlockage_AmountOfBlockingObjects;
  SensorBlockage_BlockageType SensorBlockage_Type;

  SensorBlockage_ObjectLifeTime SensorBlockage_LifeTime;
  float SensorBlockage_BlockingObjectLifeTime;
  float SensorBlockage_MaxBlockingObjectLifeTime;

  float SensorBlockage_BlockageDropSpeed; // Speed the Object moves downwoards (Falling Rain or Mud which moves slowly away)

  HorizontalFOV_Type SensorBlockage_HorFOVFlag;
  VerticalFOV_Type SensorBlockage_VertFOVFlag;




#pragma region PackageLoss_Function
  void AddPackageLossInterval(float Interval) {
	  this->PackageLoss_Interval = Interval;
  }
  void AddPackageLossDuration(float Duration) {
	  this->PackageLoss_Duration = Duration;
  }
  void AddPackageLossStart(float StartTime) {
	  this->PackageLoss_Start = CalculateStartTime(StartTime);
  }
  void SetProgressionRate(float Rate) {
	  this->PackageLoss_IntervalDegradation = Rate;
  }
  void SetDurationDegradation(float DurationDegradation)
  {
	  this->PackageLoss_DurationDegradation = DurationDegradation;
  }
#pragma endregion

#pragma region PackageDelay_Function
  void SetPackageDelay_Start(float Start) {
	  PackageDelay_Start = CalculateStartTime(Start);;
  }
  void SetPackageDelay_Interval(float Interval) {
	  PackageDelay_Interval = Interval;
	  PackageDelay_DegradationZeit = PackageDelay_Start + Interval;
  }
  void SetPackageDelay_DegradationSize(int Size) {
	  PackageDelay_DegradationSize = Size;
  }
  void SetPackageDelay_DelaySize(int Size) {
	  PackageDelay_DelaySize = Size;
  }
  void SetPackageDelay_RingBufferMaxUseSize(int Size) {
	  PackageDelay_RingBufferMaxUseSize = Size;
  }
#pragma endregion

#pragma region DetectionPointShift_Function
  void SetDetectionPointShift_Start(float StartTime) {
	  this->DetectionPointShift_Start = CalculateStartTime(StartTime);
  }
  void SetDetectionPointShift_Interval(float Interval) {
	  this->DetectionPointShift_Interval = Interval;
  }
  void SetDetectionPointShift_Duration(float Duration) {
	  this->DetectionPointShift_Duration = Duration;
  }
  void SetDetectionPointShift_IntervalDegradation(float IntDegre) {
	  this->DetectionPointShift_IntervalDegradation = IntDegre;
  }
  void SetDetectionPointShift_DurationDegradation(float DuraDegre) {
	  this->DetectionPointShift_DurationDegradation = DuraDegre;
  }
  void SetDetectionPoint_MaxDepthDisturbance(float MaxDepth) {
	  this->DetectionPoint_MaxDepthDisturbance = MaxDepth;
  }
  void SetDetectionPoint_MaxAzimuthDisturbance(float MaxAzi) {
	  this->DetectionPoint_MaxAzimuthDisturbance = MaxAzi;
  }
  void SetDetectionPoint_MaxAltitudeDisturbance(float MaxAlti) {
	  this->DetectionPoint_MaxAltitudeDisturbance = MaxAlti;
  }
  void SetDetectionPoint_Distribution(int Dist)
  {
	  this->DetectionPoint_Distribution = (Distribution)Dist;
  }
#pragma endregion

#pragma region VelocityShift_Function
  void SetVelocityShift_Start(float StartTime) {
	  this->VelocityShift_Start = CalculateStartTime(StartTime);
  }
  void SetVelocityShift_Interval(float Interval) {
	  this->VelocityShift_Interval = Interval;
  }
  void SetVelocityShift_Duration(float Duration) {
	  this->VelocityShift_Duration = Duration;
  }
  void SetVelocityShift_IntervalDegradation(float InterDegre) {
	  this->VelocityShift_IntervalDegradation = InterDegre;
  }
  void SetVelocityShift_DurationDegradation(float DuraDegre) {
	  this->VelocityShift_DurationDegradation = DuraDegre;
  }
  void SetVelocityShift_MaxVelocityDisturbance(float MaxVel) {
	  this->VelocityShift_MaxVelocityDisturbance = MaxVel;
  }
  void SetVelocityShift_Distribution(int Dist) {
	  this->VelocityShift_Distribution = (Distribution)Dist;
  }
#pragma endregion

#pragma region RangeReduction_Function
  void SetRangeReduction_Start(float StartTime) {
	  this->RangeReduction_Start = CalculateStartTime(StartTime);
  }
  void SetRangeReduction_Interval(float Interval) {
	  this->RangeReduction_Interval = Interval;
  }
  void SetRangeReduction_Duration(float Duration) {
	  this->RangeReduction_Duration = Duration;
  }
  void SetRangeReduction_IntervalDegradation(float IntDeg) {
	  this->RangeReduction_IntervalDegradation = IntDeg;
  }
  void SetRangeReduction_DurationDegradation(float DurDeg) {
	  this->RangeReduction_DurationDegradation = DurDeg;
  }
  void SetRangeReduction_RangeReductionValue(float RangeRed) {
	  this->RangeReduction_RangeReductionValue = RangeRed * 100.f;
  }
#pragma endregion

#pragma region DetectNonExistingPoints_Function

  void SetDetectNonExistingPoints_Start(float StartTime) {
	  this->DetectNonExistingPoints_Start = CalculateStartTime(StartTime);
  }
  void SetDetectNonExistingPoints_Interval(float Interval) {
	  this->DetectNonExistingPoints_Interval = Interval;
  }
  void SetDetectNonExistingPoints_Duration(float Duration) {
	  this->DetectNonExistingPoints_Duration = Duration;
  }
  void SetDetectNonExistingPoints_IntervalDegradation(float InterDeg) {
	  this->DetectNonExistingPoints_IntervalDegradation = InterDeg;
  }
  void SetDetectNonExistingPoints_DurationDegradation(float DuraDeg) {
	  this->DetectNonExistingPoints_DurationDegradation = DuraDeg;
  }
  void SetDetectNonExistingPoints_AmountDetections(int MaxAmount) {
	  this->DetectNonExistingPoints_AmountDetections = MaxAmount;
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
  void SetSensorShift_Interval(float Interval) {
	  this->SensorShift_Interval = Interval;
  }
  void SetSensorShift_Duration(float Duration) {
	  this->SensorShift_Duration = Duration;
  }
  void SetSensorShift_IntervalDegradation(float InterDegre) {
	  this->SensorShift_IntervalDegradation = InterDegre;
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

#pragma region SensorBlockage_Function

  void SetSensorBlockage_Start(float StartTime) {
	  this->SensorBlockage_Start = CalculateStartTime(StartTime);
  }

  void SetSensorBlockage_Interval(float Interval) {
	  this->SensorBlockage_Interval = Interval;
  }

  void SetSensorBlockage_IntervalDegradation(float Degredation) {
	  this->SensorBlockage_IntervalDegradation = Degredation;
  }

  void SetSensorBlockage_AmountOfBlockingObjects(int Amount) {
	  this->SensorBlockage_AmountOfBlockingObjects = Amount;
  }

  void SetSensorBlockage_Type(int Flag) {
	  if (Flag < 0 || Flag > 1)
		  this->SensorBlockage_Type = SensorBlockage_BlockageType::CloseRange;
	  else
		  this->SensorBlockage_Type = (SensorBlockage_BlockageType)Flag;
  }

  void SetSensorBlockage_LifeTime(int Flag)
  {
	  if (Flag < 0 || Flag > 1)
		  this->SensorBlockage_LifeTime = SensorBlockage_ObjectLifeTime::Static;
	  else
		  this->SensorBlockage_LifeTime = (SensorBlockage_ObjectLifeTime)Flag;
  }

  void SetSensorBlockage_BlockingObjectLifeTime(float LifeTime) {
	  this->SensorBlockage_BlockingObjectLifeTime = LifeTime;
  }
  void SetSensorBlockage_MaxBlockingObjectLifeTime(float MaxLifeTime) {
	  this->SensorBlockage_MaxBlockingObjectLifeTime = MaxLifeTime;
  }
  void SetSensorBlockage_BlockageDropSpeed(float DroppSpeed) {
	  this->SensorBlockage_BlockageDropSpeed = DroppSpeed;
  }
  void SetSensorBlockage_HorFOVFlag(int HorFov) {
	  if (HorFov <= 2 && HorFov >= 0)
		  this->SensorBlockage_HorFOVFlag = (HorizontalFOV_Type)HorFov;
	  else
		  this->SensorBlockage_HorFOVFlag = HorizontalFOV_Type::WholeHorFOV;
  }
  void SetSensorBlockage_VertFOVFlag(int VertFov) {
	  if (VertFov <= 2 && VertFov >= 0)
		  this->SensorBlockage_VertFOVFlag = (VerticalFOV_Type)VertFov;
	  else
		  this->SensorBlockage_VertFOVFlag = VerticalFOV_Type::WholeVerFOV;
  }

#pragma endregion

};