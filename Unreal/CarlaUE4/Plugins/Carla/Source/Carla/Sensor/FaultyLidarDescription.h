// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once
#include <carla/sensor/data/LidarData.h>
#define LidarDelay_RingBufferSize 100
#define RandomEngineSeed 9857
//using FLidarData = carla::sensor::data::LidarData;
//#include "FaultyLidarDescription.generated.h"
struct FFaultyLidarDescription
{
    enum Distribution : int
    {
        None = 0x0,
        Weibull = 0x1,
        Linear = 0x2
    };

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





};