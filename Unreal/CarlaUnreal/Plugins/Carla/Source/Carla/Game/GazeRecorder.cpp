// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/GazeRecorder.h"
#include "Carla.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEngine.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/Tagger.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/ue-header-guard-begin.h>
#include "Camera/PlayerCameraManager.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "EyeTrackerFunctionLibrary.h"
#include "EyeTrackerTypes.h"
#include "Engine/TextRenderActor.h"
#include "Components/TextRenderComponent.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformFileManager.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include <util/ue-header-guard-end.h>

namespace GazeRecorderUtil
{
  /// CARLA's coordinates: Unreal's, in meters
  static FString Vec(const FVector& V, float Scale = 0.01f)
  {
    return FString::Printf(TEXT("%.4f,%.4f,%.4f"), V.X * Scale, V.Y * Scale, V.Z * Scale);
  }

  static FString Csv(FString Text)
  {
    Text.ReplaceInline(TEXT(","), TEXT(" "));
    Text.ReplaceInline(TEXT("\n"), TEXT(" "));
    return Text;
  }

  /// The ego vehicle (role 'hero')
  static AActor* FindHero(UCarlaEpisode* Episode, FCarlaActor** OutView)
  {
    if (Episode == nullptr)
    {
      return nullptr;
    }
    for (const auto& Item : Episode->GetActorRegistry())
    {
      FCarlaActor* View = Item.Value.Get();
      const FActorInfo* Info = View != nullptr ? View->GetActorInfo() : nullptr;
      if (Info == nullptr || View->GetActor() == nullptr)
      {
        continue;
      }
      const FActorAttribute* RoleName = Info->Description.Variations.Find(TEXT("role_name"));
      if (RoleName != nullptr && RoleName->Value == TEXT("hero"))
      {
        *OutView = View;
        return View->GetActor();
      }
    }
    return nullptr;
  }

  /// The actor and everything attached to it (driving seat, VR pawn...), recursively
  static void AddWithAttached(AActor* Actor, FCollisionQueryParams& Params)
  {
    if (Actor == nullptr)
    {
      return;
    }
    Params.AddIgnoredActor(Actor);
    TArray<AActor*> Attached;
    Actor->GetAttachedActors(Attached, true, true);
    for (AActor* Child : Attached)
    {
      Params.AddIgnoredActor(Child);
    }
  }
}

FGazeRecorder::~FGazeRecorder()
{
  Stop();
}

FString FGazeRecorder::Start(const FString& Name)
{
  Stop();
  const FString SafeName = FPaths::MakeValidFileName(Name.IsEmpty() ? FString(TEXT("session")) : Name, TEXT('_'));
  const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("GazeLogs"));
  IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
  PlatformFile.CreateDirectoryTree(*Directory);
  FilePath = FPaths::ConvertRelativePathToFull(FPaths::Combine(Directory,
      FString::Printf(TEXT("%s_%s.csv"), *SafeName, *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")))));
  // Readable by others while recorded (the client checks it gets eye data early in the route)
  File = PlatformFile.OpenWrite(*FilePath, false, true);
  if (File == nullptr)
  {
    UE_LOG(LogCarla, Error, TEXT("Gaze recorder: can't create %s"), *FilePath);
    return FString();
  }
  Samples = 0;
  ValidSamples = 0;
  bSyncPending = true;
  SyncStartTime = -1.0;
  LastFlushTime = FPlatformTime::Seconds();
  WriteLine(TEXT("wall_time,sim_time,frame,eye_tracker_connected,gaze_valid,confidence,"
      "gaze_origin_x,gaze_origin_y,gaze_origin_z,gaze_dir_x,gaze_dir_y,gaze_dir_z,"
      "fixation_x,fixation_y,fixation_z,left_blink,right_blink,left_pupil,right_pupil,"
      "head_x,head_y,head_z,head_pitch,head_yaw,head_roll,"
      "ego_id,ego_x,ego_y,ego_z,ego_yaw,ego_speed_kmh,"
      "hit,hit_x,hit_y,hit_z,hit_distance,hit_actor_id,hit_actor_type,hit_semantic,hit_object,sync,"
      "automation_state,takeover_cause,driver_throttle,driver_brake,driver_steer"));
  UE_LOG(LogCarla, Log, TEXT("Gaze recorder: recording to %s (eye tracker %s)"), *FilePath,
      UEyeTrackerFunctionLibrary::IsEyeTrackerConnected() ? TEXT("connected") : TEXT("NOT connected"));
  return FilePath;
}

void FGazeRecorder::Stop()
{
  DestroySyncActors();
  bSyncPending = false;
  if (File == nullptr)
  {
    return;
  }
  File->Flush();
  delete File;
  File = nullptr;
  UE_LOG(LogCarla, Log, TEXT("Gaze recorder: %lld samples (%lld with a valid gaze) written to %s"),
      Samples, ValidSamples, *FilePath);
}

void FGazeRecorder::DestroySyncActors()
{
  for (TWeakObjectPtr<ATextRenderActor>& Actor : SyncActors)
  {
    if (Actor.IsValid())
    {
      Actor->Destroy();
    }
  }
  SyncActors.Empty();
}

bool FGazeRecorder::UpdateSyncCue(UWorld* World)
{
  if (!bSyncPending)
  {
    return false;
  }
  const double Now = FPlatformTime::Seconds();
  if (SyncStartTime < 0.0)
  {
    SyncStartTime = Now;
  }
  const double Elapsed = Now - SyncStartTime;
  if (Elapsed > SyncDuration)
  {
    DestroySyncActors();
    bSyncPending = false;
    return false;
  }
  APlayerCameraManager* Camera = UGameplayStatics::GetPlayerCameraManager(World, 0);
  if (Camera == nullptr)
  {
    return true;
  }
  if (SyncActors.Num() == 0)
  {
    const TCHAR* Texts[] = { TEXT("SYNC"), TEXT("+"), TEXT("+") };
    for (int32 i = 0; i < 3; ++i)
    {
      FActorSpawnParameters SpawnParams;
      SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
      ATextRenderActor* Actor = World->SpawnActor<ATextRenderActor>(FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
      if (Actor != nullptr)
      {
        UTextRenderComponent* Text = Actor->GetTextRender();
        Text->SetText(FText::FromString(Texts[i]));
        Text->SetHorizontalAlignment(EHTA_Center);
        Text->SetVerticalAlignment(EVRTA_TextCenter);
        Text->SetWorldSize(i == 0 ? SyncSize : SyncMarkerSize);
        Text->SetTextRenderColor(FColor(255, 0, 255));
        Text->SetCastShadow(false);
        Actor->SetActorEnableCollision(false);
      }
      SyncActors.Add(Actor);
    }
  }

  // First half: "SYNC" straight ahead. Second half: the "+" at -/+ SyncMarkerAngle (in the view's frame)
  const bool bMarkersPhase = Elapsed > SyncDuration * 0.5;
  const FRotator ViewRotation = Camera->GetCameraRotation();
  const FVector ViewLocation = Camera->GetCameraLocation();
  const float Angles[] = { 0.0f, -SyncMarkerAngle, SyncMarkerAngle };
  for (int32 i = 0; i < SyncActors.Num(); ++i)
  {
    ATextRenderActor* Actor = SyncActors[i].Get();
    if (Actor == nullptr)
    {
      continue;
    }
    Actor->SetActorHiddenInGame(bMarkersPhase == (i == 0));
    const FVector Direction = ViewRotation.RotateVector(FRotator(0.0f, Angles[i], 0.0f).Vector());
    // Facing the view (a text render is read from its +X side)
    Actor->SetActorLocationAndRotation(ViewLocation + Direction * SyncDistance,
        FRotationMatrix::MakeFromXZ(-Direction, ViewRotation.RotateVector(FVector::UpVector)).Rotator());
  }
  return true;
}

void FGazeRecorder::WriteLine(const FString& Line)
{
  if (File == nullptr)
  {
    return;
  }
  const FTCHARToUTF8 Utf8(*(Line + TEXT("\n")));
  File->Write(reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
}

void FGazeRecorder::Tick(UWorld* World, UCarlaEpisode* Episode)
{
  using namespace GazeRecorderUtil;
  if (File == nullptr || World == nullptr)
  {
    return;
  }

  // Sync cue, the first SyncDuration seconds of the recording
  const bool bSync = UpdateSyncCue(World);

  // Gaze (world space)
  FEyeTrackerGazeData Gaze;
  const bool bConnected = UEyeTrackerFunctionLibrary::IsEyeTrackerConnected();
  const bool bValid = UEyeTrackerFunctionLibrary::GetGazeData(Gaze) && !Gaze.GazeDirection.IsNearlyZero();

  // Head (the player's view, i.e. the headset)
  FVector HeadLocation = FVector::ZeroVector;
  FRotator HeadRotation = FRotator::ZeroRotator;
  if (APlayerCameraManager* Camera = UGameplayStatics::GetPlayerCameraManager(World, 0))
  {
    HeadLocation = Camera->GetCameraLocation();
    HeadRotation = Camera->GetCameraRotation();
  }

  // Ego vehicle
  FCarlaActor* HeroView = nullptr;
  AActor* Hero = FindHero(Episode, &HeroView);
  FString EgoColumns = TEXT("-1,0,0,0,0,0");
  if (Hero != nullptr)
  {
    const FVector Location = Hero->GetActorLocation();
    EgoColumns = FString::Printf(TEXT("%u,%s,%.2f,%.2f"), HeroView->GetActorId(), *Vec(Location),
        Hero->GetActorRotation().Yaw, Hero->GetVelocity().Size() * 0.036f);
  }

  // Who drives (the server's own takeover state, see ACarlaWheeledVehicle), and the driver's raw inputs
  FString DriverColumns = TEXT(",,,,");
  if (const ACarlaWheeledVehicle* Vehicle = Cast<ACarlaWheeledVehicle>(Hero))
  {
    DriverColumns = FString::Printf(TEXT("%s,%s,%.3f,%.3f,%.3f"),
        Vehicle->IsDriverTakeover() ? TEXT("driver") : TEXT("engaged"),
        Vehicle->IsDriverTakeover() ? *Csv(Vehicle->GetDriverTakeoverCause()) : TEXT(""),
        Vehicle->GetDriverThrottle(), Vehicle->GetDriverBrake(), Vehicle->GetDriverSteer());
  }

  // What the gaze hits: the first object along it, outside the ego vehicle
  FString HitColumns = TEXT("0,0,0,0,0,-1,,,");
  if (bValid)
  {
    FCollisionQueryParams Params(SCENE_QUERY_STAT(GazeTrace), true);
    AddWithAttached(Hero, Params);
    if (APlayerController* PlayerController = UGameplayStatics::GetPlayerController(World, 0))
    {
      AddWithAttached(PlayerController->GetPawn(), Params);
    }
    FHitResult Hit;
    const FVector End = Gaze.GazeOrigin + Gaze.GazeDirection.GetSafeNormal() * MaxGazeDistance;
    if (World->LineTraceSingleByChannel(Hit, Gaze.GazeOrigin, End, ECC_Visibility, Params))
    {
      AActor* HitActor = Hit.GetActor();
      int64 HitId = -1;
      FString HitType;
      if (HitActor != nullptr && Episode != nullptr)
      {
        if (FCarlaActor* View = Episode->FindCarlaActor(HitActor))
        {
          HitId = View->GetActorId();
          HitType = View->GetActorInfo() != nullptr ? View->GetActorInfo()->Description.Id : FString();
        }
      }
      FString Semantic;
      if (const UPrimitiveComponent* Component = Hit.GetComponent())
      {
        Semantic = ATagger::GetTagAsString(ATagger::GetTagOfTaggedComponent(*Component));
      }
      HitColumns = FString::Printf(TEXT("1,%s,%.3f,%lld,%s,%s,%s"), *Vec(Hit.ImpactPoint), Hit.Distance * 0.01f,
          HitId, *Csv(HitType), *Csv(Semantic), *Csv(HitActor != nullptr ? HitActor->GetName() : FString()));
    }
  }

  const FString Line = FString::Printf(
      TEXT("%s,%.4f,%llu,%d,%d,%.3f,%s,%s,%s,%d,%d,%.3f,%.3f,%s,%.2f,%.2f,%.2f,%s,%s,%d,%s"),
      *FDateTime::Now().ToIso8601(), World->GetTimeSeconds(), (unsigned long long)FCarlaEngine::GetFrameCounter(),
      bConnected ? 1 : 0, bValid ? 1 : 0, Gaze.ConfidenceValue,
      *Vec(Gaze.GazeOrigin), *Vec(Gaze.GazeDirection, 1.0f), *Vec(Gaze.FixationPoint),
      Gaze.bIsLeftEyeBlink ? 1 : 0, Gaze.bIsRightEyeBlink ? 1 : 0, Gaze.LeftPupilDiameter, Gaze.RightPupilDiameter,
      *Vec(HeadLocation), HeadRotation.Pitch, HeadRotation.Yaw, HeadRotation.Roll,
      *EgoColumns, *HitColumns, bSync ? 1 : 0, *DriverColumns);
  WriteLine(Line);
  ++Samples;
  if (bValid)
  {
    ++ValidSamples;
  }

  // Flushed every second: a crash loses at most that
  const double Now = FPlatformTime::Seconds();
  if (Now - LastFlushTime > 1.0)
  {
    File->Flush();
    LastFlushTime = Now;
  }
}
