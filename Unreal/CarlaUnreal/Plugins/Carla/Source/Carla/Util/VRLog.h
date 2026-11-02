// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla.h"

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include <util/ue-header-guard-end.h>

/// Appends a line to <Saved>/Logs/CarlaVR.log: the VR diagnostics (boarding, clutch, cleanup), also in Shipping
/// builds, where UE_LOG is compiled out (packaged CARLA: %LOCALAPPDATA%/CarlaUnreal/Saved/Logs)
inline void CarlaVRLogLine(const FString& Message)
{
  const FString Line = FString::Printf(TEXT("[%s] %s\n"), *FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S.%s")), *Message);
  FFileHelper::SaveStringToFile(Line, *(FPaths::ProjectSavedDir() / TEXT("Logs") / TEXT("CarlaVR.log")),
      FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
}

/// UE_LOG(LogCarla, Log, ...) and the same line in CarlaVR.log
#define CARLA_VR_LOG(Format, ...) \
  do \
  { \
    UE_LOG(LogCarla, Log, Format, ##__VA_ARGS__); \
    CarlaVRLogLine(FString::Printf(Format, ##__VA_ARGS__)); \
  } while (0)
