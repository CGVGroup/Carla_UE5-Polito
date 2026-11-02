// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/CarlaActorFactory.h"
#include "Carla/Dashboard/Dashboard.h"
#include "Carla/Game/GazeRecorder.h"
#include "Carla/Game/PathRibbon.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameInstance.h"
#include "Carla/Game/TaggerDelegate.h"
#include "Carla/OpenDrive/OpenDrive.h"
#include "Carla/Recorder/CarlaRecorder.h"
#include "Carla/Sensor/SceneCaptureSensor.h"
#include "Carla/Settings/CarlaSettingsDelegate.h"
#include "Carla/Traffic/TrafficLightManager.h"
#include "Carla/Util/ObjectRegister.h"
#include "Carla/Weather/Weather.h"

#include <util/disable-ue4-macros.h>
#include <carla/rpc/Texture.h>
#include <carla/rpc/MaterialParameter.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "Components/SplineMeshComponent.h"
#include "MapGen/LargeMapManager.h"
#include <util/ue-header-guard-end.h>

#include <optional>

#include "CarlaGameModeBase.generated.h"

class USceneComponent;
class UStaticMesh;
class UMaterialInterface;
class USplineMeshComponent;

/// Look of the motion prediction arrows of the vehicles other than the ego (see DashboardUpdateArrows)
struct FDashboardArrowLook
{
  TWeakObjectPtr<UStaticMesh> Mesh;
  TWeakObjectPtr<UMaterialInterface> Material;
  TEnumAsByte<ESplineMeshAxis::Type> Axis = ESplineMeshAxis::X;
  FVector2D Scale = FVector2D(1.0, 1.0);
  double Section = 100.0;                  // [cm]
  FVector Offset = FVector(0.0, 0.0, 5.0); // [cm] from the front of the vehicle, at the bottom
  bool bFromEgo = false;
};

/// A motion prediction arrow: an actor attached to its vehicle, its spline mesh sections
struct FDashboardArrow
{
  TWeakObjectPtr<AActor> Actor;
  TArray<TWeakObjectPtr<USplineMeshComponent>> Segments;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnEpisodeInitialisedDelegate, UCarlaEpisode*, InitialisedEpisode);

/// Base class for the CARLA Game Mode.
UCLASS(HideCategories=(ActorTick))
class CARLA_API ACarlaGameModeBase : public AGameModeBase
{
  GENERATED_BODY()

public:

  ACarlaGameModeBase(const FObjectInitializer& ObjectInitializer);

  const UCarlaEpisode &GetCarlaEpisode() const
  {
    check(Episode != nullptr);
    return *Episode;
  }

  const std::optional<carla::road::Map>& GetMap() const {
    return Map;
  }

  carla::rpc::Actor GetDashboard() const {
    // my solution- discarded  return <carla::rpc::Actor>DashboardComponent;
      return Episode->SerializeActor(DashboardComponent);
  }

  const FString GetFullMapPath() const;

  // get path relative to Content folder
  const FString GetRelativeMapPath() const;

  UFUNCTION(Exec, Category = "CARLA Game Mode")
  void DebugShowSignals(bool enable);

  UFUNCTION(BlueprintCallable, Category = "CARLA Game Mode")
  ATrafficLightManager* GetTrafficLightManager();

  UFUNCTION(Category = "Carla Game Mode", BlueprintCallable)
  const TArray<FTransform>& GetSpawnPointsTransforms() const{
    return SpawnPointsTransforms;
  }

  UFUNCTION(Category = "Carla Game Mode", BlueprintCallable, CallInEditor, Exec)
  TArray<FBoundingBox> GetAllBBsOfLevel(uint8 TagQueried = 0xFF) const;

  UFUNCTION(Category = "Carla Game Mode", BlueprintCallable, CallInEditor, Exec)
  TArray<FEnvironmentObject> GetEnvironmentObjects(uint8 QueriedTag = 0xFF) const
  {
    return ObjectRegister->GetEnvironmentObjects(QueriedTag);
  }

  void EnableEnvironmentObjects(const TSet<uint64>& EnvObjectIds, bool Enable);

  void EnableOverlapEvents();

  void CheckForEmptyMeshes();

  UFUNCTION(Category = "Carla Game Mode", BlueprintCallable, CallInEditor, Exec)
  void LoadMapLayer(int32 MapLayers);

  UFUNCTION(Category = "Carla Game Mode", BlueprintCallable, CallInEditor, Exec)
  void UnLoadMapLayer(int32 MapLayers);

  UFUNCTION(Category = "Carla Game Mode")
  ULevel* GetULevelFromName(FString LevelName);

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void OnLoadStreamLevel();

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void OnUnloadStreamLevel();

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void SetDashboard(ADashboard* dash);

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void DashboardSetControlValues(float speed_value, float steering_value);

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void DashboardSetTypeString(FString type);

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void DashboardAddVectorPair(FVector P, FVector R);

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void DashboardRemoveIDObstacle(int Id);

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void DashboardUpdateObstacle(int ID, const FString& Type, bool isDanger, FVector Vector1, FVector Vector2, float obstacleSpeed, float obstacleSteer);

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void DashboardUpdate();

  UFUNCTION(BlueprintCallable, Category = "Carla Game Mode")
  void DashboardTriggerPathSetEvent();

  ALargeMapManager* GetLMManager() const {
    return LMManager;
  }

  AActor* FindActorByName(const FString& ActorName);

  UTexture2D* CreateUETexture(const carla::rpc::TextureColor& Texture);
  UTexture2D* CreateUETexture(const carla::rpc::TextureFloatColor& Texture);

  void ApplyTextureToActor(
      AActor* Actor,
      UTexture2D* Texture,
      const carla::rpc::MaterialParameter& TextureParam);

  [[deprecated("This command no longer makes sense since the apply_textures API relies on IDs.")]]
  TArray<FString> GetNamesOfAllActors();
  
  FOnEpisodeInitialisedDelegate OnEpisodeInitialisedDelegate;

protected:

  void InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage) override;

  void RestartPlayer(AController *NewPlayer) override;

  void BeginPlay() override;

  void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  void Tick(float DeltaSeconds) override;

private:

  void SpawnActorFactories();

  void StoreSpawnPoints();

  void GenerateSpawnPoints();

  void ParseOpenDrive();

  void RegisterEnvironmentObjects();

  void ConvertMapLayerMaskToMapNames(int32 MapLayer, TArray<FName>& OutLevelNames);

  void OnEpisodeSettingsChanged(const FEpisodeSettings &Settings);

  UPROPERTY()
  ADashboard* DashboardComponent = nullptr;

  /// Shows the dashboard's text in the VR vehicles (their 'Write_Text' / 'Write_XAI' event)
  void DashboardWriteVehicleText(const FString& Text);

  /// Hides (or shows back) the AR-HUD driving aids spawned by the dashboard blueprints
  /// (virtual carpets, motion prediction arrows, navigation waypoints)
  void DashboardApplyDrivingAidsVisibility();

  /// Visualization mode set by the client through the dashboard's type string "mode:<name>":
  /// in modes other than "full", the driving aids are hidden
  bool bDashboardHideDrivingAids = false;

  /// Hides the actor if it is a driving aid (true when it is one)
  bool DashboardHideIfDrivingAid(AActor* Actor);

  /// Driving aids spawned while they are hidden are hidden too, as they spawn (instead of searching
  /// the world for them regularly, which made hitches)
  void DashboardOnActorSpawned(AActor* Actor);

  /// Handle of DashboardOnActorSpawned in the world's spawn handlers (valid while aids are hidden)
  FDelegateHandle DashboardActorSpawnedHandle;

  /// Driving aids hidden by DashboardApplyDrivingAidsVisibility, shown back in the "full" mode
  TArray<TWeakObjectPtr<AActor>> DashboardHiddenDrivingAids;

  /// Keeps the dashboard's 3D text in front of the player's view (called every tick)
  void DashboardPlaceTextActor();

  /// Tells the dashboard blueprint that the ego vehicle spawned (its 'E_VR_Vehicle_Spawn' event), so
  /// it creates the AR-HUD elements drawing the global path. Once per ego vehicle
  void DashboardNotifyVehicleSpawn();

  /// Ego vehicle the dashboard blueprint was last told about
  TWeakObjectPtr<AActor> DashboardNotifiedVehicle;

  /// Driver's gaze recording (VR eye tracking), started / stopped by the client through the dashboard's
  /// type string: "gaze:start:<name>", "gaze:stop"
  FGazeRecorder GazeRecorder;

  /// Global path of the route on the road ahead of the ego ("ribbon:" commands, see FPathRibbon)
  FPathRibbon PathRibbon;

  /// Destroys the vehicles of previous runs, the ones the client can't see included (spawned by VR
  /// blueprints), and the VR blueprints' actors they left behind; the player's VR pawn is kept and detached
  /// ("cleanup:vehicles" dashboard command)
  void DestroyPreviousVehicles();

  /// Makes the VR game mode's blueprint function Set_Player_Onboard go through EgoOnlyThunk (true), or back to
  /// the blueprint (false, the dashboard's E_VR_Vehicle_Spawn too)
  void HookSetPlayerOnboard(bool bHook);

  /// Makes the dashboard blueprint's E_VR_Vehicle_Spawn go through EgoOnlyThunk (once the dashboard is known)
  void HookDashboardVehicleSpawn();

  /// Class of the dashboard whose E_VR_Vehicle_Spawn was hooked
  TWeakObjectPtr<UClass> DashboardHookedClass;

  /// A hooked blueprint function (Set_Player_Onboard, E_VR_Vehicle_Spawn), only run when the vehicle it is
  /// about is the ego (see AllowPlayerOnboard)
  DECLARE_FUNCTION(EgoOnlyThunk);

  /// Finds the dashboard blueprint in the world if the VR blueprints did not give it (SetDashboard), or it was
  /// destroyed; logs (CarlaVR.log) when it changes
  void DashboardEnsureComponent();

  /// Dashboard last logged by DashboardEnsureComponent, and when it last searched the world
  TWeakObjectPtr<AActor> DashboardLogged;
  double DashboardLastSearchTime = 0.0;

  /// Obstacles whose first box was logged
  TSet<int32> DashboardLoggedObstacles;

  /// Bounding boxes drawn by the server (BP_BB_Car / Truck / Pedestrian, one per obstacle of the dashboard's
  /// list) when the dashboard blueprint draws none: checked every few seconds (see DashboardUpdateBoxes)
  void DashboardUpdateBoxes();
  void DashboardDestroyServerBoxes();
  TMap<FString, TWeakObjectPtr<AActor>> DashboardServerBoxes;
  bool bDashboardServerBoxes = false;
  double DashboardBoxCheckTime = 0.0;
  double DashboardFirstObstacleTime = 0.0;
  int32 DashboardBoxChecksLogged = 0;

  /// The AR-HUD materials (bounding boxes, virtual carpet, arrows) are only drawn inside the volume of the
  /// material parameter collection MPC_VRFrustum, set every tick by the BP_HUDFrustum of a BP_XAI_vivace:
  /// with an ego without one, it was the volume of a traffic vivace (or none). The traffic vehicles' ones are
  /// stopped and, when the ego has none, the volume is set to the whole world (every tick)
  void DashboardUpdateHUDFrustum();
  TWeakObjectPtr<class UMaterialParameterCollection> HUDFrustumCollection;
  TWeakObjectPtr<AActor> HUDFrustumEgo;
  double HUDFrustumScanTime = 0.0;
  bool bHUDFrustumLogged = false;

  /// Thicker bounding box lines: each box (BP_BB_*) gets copies of its line meshes, slightly larger and
  /// smaller (DashboardBoxLineOffsets, cm out / in), same material, kept in sync every tick (danger color,
  /// visibility). The boxes spawned are noted by DashboardOnBoxSpawned, thickened on the next tick
  void DashboardOnBoxSpawned(AActor* Actor);
  void DashboardUpdateThickBoxes();
  FDelegateHandle DashboardBoxSpawnedHandle;
  TArray<TWeakObjectPtr<AActor>> DashboardBoxesToThicken;
  TArray<TWeakObjectPtr<AActor>> DashboardThickBoxes;

  /// Motion prediction of the vehicles other than the ego ('full' mode), within DashboardArrowRange of it:
  /// the same arrow as the ego's own (BP_VC_Short: a chain of spline meshes), along the path each one
  /// follows in the next DashboardArrowHorizon seconds at its current speed and steering, updated every tick
  /// (BP_VC_Short itself only works as a child of a VR vehicle, and only the ego is one). Its look (mesh,
  /// material, scale, section length, position from the front) is copied from the ego's arrow
  void DashboardUpdateArrows();
  void DashboardDestroyArrows();
  bool DashboardCaptureArrowLook(class ACarlaWheeledVehicle* Hero);
  FDashboardArrowLook DashboardArrowLook;
  TWeakObjectPtr<AActor> DashboardArrowLookHero;
  TMap<TWeakObjectPtr<AActor>, FDashboardArrow> DashboardArrows;
  double DashboardArrowScanTime = 0.0;
  bool bDashboardArrowsLogged = false;
  float DashboardArrowRange = 10000.0f;     // [cm]
  float DashboardArrowHorizon = 2.0f;       // [s]
  float DashboardArrowMaxLength = 3000.0f;  // [cm]

  /// The driver's position in the vehicle, as adjusted (IA_MovePawn_X/Y/Z, IA_RotatePawn: the VR pawn moved
  /// in the vehicle), kept from one vehicle to the next: recorded relatively to the vehicle while on board,
  /// applied to each new vehicle the player is taken on board, once its boarding settled (every tick)
  void KeepPlayerSeatAdjustment();

  /// Where the server's frame time goes, logged every FrameTimingPeriod seconds (CarlaVR.log, "Frame
  /// timing"): frame interval, game thread, render thread, RHI thread, GPU, and this game mode's own
  /// per-frame work (dashboard, arrows, gaze)
  void LogFrameTiming(double GameModeSeconds);
  double FrameTimingStart = 0.0;
  double FrameTimingLast = 0.0;
  int32 FrameTimingFrames = 0;
  double FrameTimingGame = 0.0, FrameTimingRender = 0.0, FrameTimingRHI = 0.0, FrameTimingGPU = 0.0;
  double FrameTimingGameMode = 0.0, FrameTimingMaxFrame = 0.0;
  float FrameTimingPeriod = 10.0f;  // [s]
  TWeakObjectPtr<APawn> SeatPawn;
  TWeakObjectPtr<AActor> SeatVehicle;
  double SeatScanTime = 0.0;
  double SeatBoardTime = 0.0;
  bool bSeatApplied = false;
  /// [s] after the player was taken on board a vehicle before the position is applied (the boarding
  /// blueprint places the pawn on the seat meanwhile)
  float SeatSettleDelay = 2.0f;

  /// Whether Set_Player_Onboard may take the player on board Vehicle: the ego once registered (else the
  /// first vehicle, while the player is on board none)
  bool AllowPlayerOnboard(class ACarlaWheeledVehicle* Vehicle);

  /// Puts the player back on board the ego vehicle when another VR vehicle took it on board as it spawned
  /// (every tick)
  void KeepPlayerInEgoVehicle();

  /// Where the player's pawn is attached in the ego (fallback of KeepPlayerInEgoVehicle), and when the ego
  /// was last asked to take the player on board
  TWeakObjectPtr<USceneComponent> EgoSeatComponent;
  FName EgoSeatSocket;
  FTransform EgoSeatRelative;
  double LastEgoBoardingTime = 0.0;

  /// 3D text showing the dashboard's text in front of the player's view (none when it is empty)
  TWeakObjectPtr<class ATextRenderActor> DashboardTextActor;

  /// Distance of the 3D text in front of the view, how far below its center (set by the client with a
  /// "[height=<cm>]" prefix on the text: Drop = -height), and its height [cm]
  float DashboardTextDistance = 300.0f;
  float DashboardTextDrop = -30.0f;  // negative: above the center of the view
  float DashboardTextSize = 12.0f;

  UPROPERTY()
  UCarlaGameInstance *GameInstance = nullptr;

  UPROPERTY()
  UTaggerDelegate *TaggerDelegate = nullptr;

  UPROPERTY()
  UCarlaSettingsDelegate *CarlaSettingsDelegate = nullptr;

  UPROPERTY()
  UCarlaEpisode *Episode = nullptr;

  UPROPERTY()
  ACarlaRecorder *Recorder = nullptr;

  UPROPERTY()
  UObjectRegister* ObjectRegister = nullptr;

  /// The class of Weather to spawn.
  UPROPERTY(Category = "CARLA Game Mode", EditAnywhere)
  TSubclassOf<AWeather> WeatherClass;

  /// List of actor spawners that will be used to define and spawn the actors
  /// available in game.
  UPROPERTY(Category = "CARLA Game Mode", EditAnywhere)
  TSet<TSubclassOf<ACarlaActorFactory>> ActorFactories;

  UPROPERTY()
  TArray<FTransform> SpawnPointsTransforms;

  UPROPERTY()
  TArray<ACarlaActorFactory *> ActorFactoryInstances;

  UPROPERTY()
  ATrafficLightManager* TrafficLightManager = nullptr;

  ALargeMapManager* LMManager = nullptr;

  FDelegateHandle OnEpisodeSettingsChangeHandle;

  std::optional<carla::road::Map> Map;

  int PendingLevelsToLoad = 0;
  int PendingLevelsToUnLoad = 0;

  bool ReadyToRegisterObjects = false;

  // We keep a global uuid to allow the load/unload layer methods to be called
  // in the same tick
  int32 LatentInfoUUID = 0;

};
