// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

//#include "FaultyLidarDescription.generated.h"
struct FFaultyLidarDescription
{
  bool isBlocked;
  enum ScenarioID
  {
	  RadarBlocked = 0x1,
	  RadarPackageLoss = 0x2,
	  RadarConstantShift = 0x4,
	  RadarVibration = 0x8,
	  RadarVictim = 0x10,
	  RadarRandomShift = 0x20,
	  RadarCollosionShift = 0x80,
      RadarBlockage = 0x100
  };

  int Scenario;

  float PackageLoss_Interval;
  float PackageLoss_Duration;
  float PackageLoss_StartOffset;
  float PackageLoss_Start;

  float PackageLoss_IntervallDegradation;

  FRotator ConstantShift_Rotation;
  float ConstantShift_Interval;
  float ConstantShift_Start;
  float ConstantShift_StartOffset;

  float RandomShift_Start;
  float RandomShift_End;
  float RandomShift_Time = 0.0f;
  float RandomShif_StartOffset;

  void AddScenario(int ScenarioID)
  {
      if (!(this->Scenario & ScenarioID))
          this->Scenario |= ScenarioID;

  }
  void SetConstantShiftRotation(FString string)
  {
      TArray<FString> Tokens;
      const TCHAR* Delims = TEXT(";");
      string.ParseIntoArray(Tokens, Delims, false);
      if (Tokens.Num() != 3)
      {
          GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red, TEXT("ConstantShiftArray is broken, could not read 3 values there!"));
          UE_LOG(LogTemp, Error, TEXT("ConstantShiftArray is broken, couldnot read 3 values there!"));
          return;
      }
      ConstantShift_Rotation.Yaw = FCString::Atof(*Tokens[0]);
      ConstantShift_Rotation.Pitch = FCString::Atof(*Tokens[1]);
      ConstantShift_Rotation.Roll = FCString::Atof(*Tokens[2]);

  }
};