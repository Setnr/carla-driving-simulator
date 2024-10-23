#pragma once

#include "Carla/Sensor/FaultySensor.h"

FString PackageLoss::FailureType = TEXT("PackageLoss"); 
template<class  T>
FString PackageDelay<T>::FailureType = TEXT("PackageDelay");
FString ShiftSensor::FailureType = TEXT("ShiftSensor");


template<class  T>
FString PointDataShift<T>::FailureType = TEXT("PointDataShift");
template<class  T>
FString CoordinatenPointDataShift<T>::TypeShift = TEXT("Coordinate");
template<class  T>
FString AdditionalPointDataShift<T>::TypeShift = TEXT("AdditionalData");

FString RangeReduction::FailureType = TEXT("RangeReduction");

FString AreaEffects::FailureType = TEXT("AreaEffects");
FString Blockage::Type = TEXT("Blockage");
FString RandomPoints::Type = TEXT("RandomPoints");