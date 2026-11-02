// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/CarlaGameModeBase.h"
#include "Carla.h"
#include "Carla/Game/Tagger.h"
#include "Carla/Game/CarlaHUD.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Util/VRLog.h"
#include "Carla/Game/CarlaStaticDelegates.h"
#include "Carla/Lights/CarlaLight.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleSpawnPoint.h"
#include "Carla/Util/BoundingBoxCalculator.h"

#include <util/disable-ue4-macros.h>
#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/element/RoadInfoSignal.h"
#include <carla/rpc/EnvironmentObject.h>
#include <carla/rpc/WeatherParameters.h>
#include <carla/rpc/MapLayer.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "Engine/DecalActor.h"
#include "Engine/LevelStreaming.h"
#include "Engine/LocalPlayer.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/Info.h"
#include "GameFramework/HUD.h"
#include "GameFramework/PlayerController.h"
#include "Engine/TextRenderActor.h"
#include "Components/TextRenderComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialParameterCollection.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "Components/SceneComponent.h"
#include "HAL/PlatformTime.h"
#include "Camera/PlayerCameraManager.h"
#include "Kismet/GameplayStatics.h"
#include "Async/ParallelFor.h"
#include "DynamicRHI.h"
#include "RenderTimer.h"
#include "Misc/ScopeExit.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include <util/ue-header-guard-end.h>

namespace cr = carla::road;
namespace crp = carla::rpc;
namespace cre = carla::road::element;

ACarlaGameModeBase::ACarlaGameModeBase(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PrePhysics;
  // In VR the world is paused between the client's ticks (see FCarlaEngine::WaitForTickCue):
  // the gaze is still sampled on those frames
  PrimaryActorTick.bTickEvenWhenPaused = true;
  bAllowTickBeforeBeginPlay = false;

  Episode = CreateDefaultSubobject<UCarlaEpisode>(TEXT("Episode"));

  Recorder = CreateDefaultSubobject<ACarlaRecorder>(TEXT("Recorder"));

  ObjectRegister = CreateDefaultSubobject<UObjectRegister>(TEXT("ObjectRegister"));

  // HUD
  HUDClass = ACarlaHUD::StaticClass();

  TaggerDelegate = CreateDefaultSubobject<UTaggerDelegate>(TEXT("TaggerDelegate"));
  CarlaSettingsDelegate = CreateDefaultSubobject<UCarlaSettingsDelegate>(TEXT("CarlaSettingsDelegate"));
}

const FString ACarlaGameModeBase::GetRelativeMapPath() const
{
  UWorld* World = GetWorld();
  TSoftObjectPtr<UWorld> AssetPtr (World);
  FString Path = FPaths::GetPath(AssetPtr.GetLongPackageName());
  Path.RemoveFromStart("/Game/");
  return Path;
}

const FString ACarlaGameModeBase::GetFullMapPath() const
{
  FString Path = GetRelativeMapPath();
  return FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir()) + Path;
}

void ACarlaGameModeBase::InitGame(
    const FString &MapName,
    const FString &Options,
    FString &ErrorMessage)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaGameModeBase::InitGame);
  Super::InitGame(MapName, Options, ErrorMessage);

  UWorld* World = GetWorld();
  check(World != nullptr);
  FString InMapName(MapName);

  checkf(
      Episode != nullptr,
      TEXT("Missing episode, can't continue without an episode!"));

  AActor* LMManagerActor =
      UGameplayStatics::GetActorOfClass(GetWorld(), ALargeMapManager::StaticClass());
  LMManager = Cast<ALargeMapManager>(LMManagerActor);
  if (LMManager) {
    if (LMManager->GetNumTiles() == 0)
    {
      LMManager->GenerateLargeMap();
    }
    InMapName = LMManager->LargeMapName;
  }

#if WITH_EDITOR
    {
      // When playing in editor the map name gets an extra prefix, here we
      // remove it.
      FString CorrectedMapName = InMapName;
      constexpr auto PIEPrefix = TEXT("UEDPIE_0_");
      CorrectedMapName.RemoveFromStart(PIEPrefix);
      UE_LOG(LogCarla, Log, TEXT("Corrected map name from %s to %s"), *InMapName, *CorrectedMapName);
      Episode->MapName = CorrectedMapName;
    }
#else
  Episode->MapName = InMapName;
#endif // WITH_EDITOR

  GameInstance = Cast<UCarlaGameInstance>(GetGameInstance());
  checkf(
      GameInstance != nullptr,
      TEXT("GameInstance is not a UCarlaGameInstance, did you forget to set "
           "it in the project settings?"));

  if (TaggerDelegate != nullptr) {
    TaggerDelegate->RegisterSpawnHandler(World);
  } else {
    UE_LOG(LogCarla, Error, TEXT("Missing TaggerDelegate!"));
  }

  if(CarlaSettingsDelegate != nullptr) {
    CarlaSettingsDelegate->ApplyQualityLevelPostRestart();
    CarlaSettingsDelegate->RegisterSpawnHandler(World);
  } else {
    UE_LOG(LogCarla, Error, TEXT("Missing CarlaSettingsDelegate!"));
  }

  AActor* WeatherActor =
      UGameplayStatics::GetActorOfClass(GetWorld(), AWeather::StaticClass());
  if (WeatherActor != nullptr) {
    UE_LOG(LogCarla, Log, TEXT("Existing weather actor. Doing nothing then!"));
    Episode->Weather = static_cast<AWeather*>(WeatherActor);
  }
  else if (WeatherClass != nullptr) {
    Episode->Weather = World->SpawnActor<AWeather>(WeatherClass);
  } else {
    UE_LOG(LogCarla, Error, TEXT("Missing weather class!"));
  }

  GameInstance->NotifyInitGame();

  OnEpisodeSettingsChangeHandle = FCarlaStaticDelegates::OnEpisodeSettingsChange.AddUObject(
        this,
        &ACarlaGameModeBase::OnEpisodeSettingsChanged);

  SpawnActorFactories();

  // make connection between Episode and Recorder
  Recorder->SetEpisode(Episode);
  Episode->SetRecorder(Recorder);

  ParseOpenDrive();

  if(Map.has_value())
  {
    StoreSpawnPoints();
  }
}

void ACarlaGameModeBase::RestartPlayer(AController *NewPlayer)
{
  if (CarlaSettingsDelegate != nullptr)
  {
    CarlaSettingsDelegate->ApplyQualityLevelPreRestart();
  }

  Super::RestartPlayer(NewPlayer);
}

void ACarlaGameModeBase::BeginPlay()
{
  Super::BeginPlay();

  UWorld* World = GetWorld();
  check(World != nullptr);

  HookSetPlayerOnboard(true);
  DashboardBoxSpawnedHandle = World->AddOnActorSpawnedHandler(
      FOnActorSpawned::FDelegate::CreateUObject(this, &ACarlaGameModeBase::DashboardOnBoxSpawned));

  LoadMapLayer(GameInstance->GetCurrentMapLayer());
  ReadyToRegisterObjects = true;

  ATagger::TagActorsInLevel(*World, true);
  TaggerDelegate->SetSemanticSegmentationEnabled();

  // HACK: fix transparency see-through issues
  // The problem: transparent objects are visible through walls.
  // This is due to a weird interaction between the SkyAtmosphere component,
  // the shadows of a directional light (the sun)
  // and the custom depth set to 3 used for semantic segmentation
  // The solution: Spawn a Decal.
  // It just works!
  World->SpawnActor<ADecalActor>(
      FVector(0,0,-1000000), FRotator(0,0,0), FActorSpawnParameters());

  ATrafficLightManager* Manager = GetTrafficLightManager();
  Manager->InitializeTrafficLights();

  Episode->InitializeAtBeginPlay();
  GameInstance->NotifyBeginEpisode(*Episode);
  OnEpisodeInitialisedDelegate.Broadcast(Episode);

  if (Episode->Weather != nullptr)
  {
    Episode->Weather->ApplyWeather(carla::rpc::WeatherParameters::Default);
  }

  /// @todo Recorder should not tick here, FCarlaEngine should do it.
  // check if replayer is waiting to autostart
  if (Recorder)
  {
    Recorder->GetReplayer()->CheckPlayAfterMapLoaded();
  }

  if(ReadyToRegisterObjects && PendingLevelsToLoad == 0)
  {
    RegisterEnvironmentObjects();
  }

  if (LMManager) {
    LMManager->RegisterInitialObjects();
    LMManager->ConsiderSpectatorAsEgo(Episode->GetSettings().SpectatorAsEgo);
  }

  // Manually run begin play on lights as it may not run on sublevels
  TArray<AActor*> FoundActors;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), FoundActors);
  for(AActor* Actor : FoundActors)
  {
    TArray<UCarlaLight*> Lights;
    Actor->GetComponents(Lights, false);
    for(UCarlaLight* Light : Lights)
    {
      Light->RegisterLight();
    }
  }
  EnableOverlapEvents();
}

TArray<FString> ACarlaGameModeBase::GetNamesOfAllActors()
{
  TArray<FString> Names;
  TArray<AActor*> Actors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), Actors);
  for (AActor* Actor : Actors)
  {
    TArray<UStaticMeshComponent*> StaticMeshes;
    Actor->GetComponents(StaticMeshes);
    if (StaticMeshes.Num())
    {
      Names.Add(Actor->GetName());
    }
  }
  return Names;
}

AActor* ACarlaGameModeBase::FindActorByName(const FString& ActorName)
{
  TArray<AActor*> Actors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), Actors);
  for (AActor* Actor : Actors)
  {
    if(Actor->GetName() == ActorName)
    {
      return Actor;
      break;
    }
  }
  return nullptr;
}

UTexture2D* ACarlaGameModeBase::CreateUETexture(const carla::rpc::TextureColor& Texture)
{
  FlushRenderingCommands();
  TArray<FColor> Colors;
  for (uint32_t y = 0; y < Texture.GetHeight(); y++)
  {
    for (uint32_t x = 0; x < Texture.GetWidth(); x++)
    {
      auto& Color = Texture.At(x,y);
      Colors.Add(FColor(Color.r, Color.g, Color.b, Color.a));
    }
  }
  UTexture2D* UETexture = UTexture2D::CreateTransient(Texture.GetWidth(), Texture.GetHeight(), EPixelFormat::PF_B8G8R8A8);
  FTexture2DMipMap& Mip = UETexture->GetPlatformData()->Mips[0];
  void* Data = Mip.BulkData.Lock( LOCK_READ_WRITE );
  FMemory::Memcpy( Data,
      &Colors[0],
      Texture.GetWidth()*Texture.GetHeight()*sizeof(FColor));
  Mip.BulkData.Unlock();
  UETexture->UpdateResource();
  return UETexture;
}

UTexture2D* ACarlaGameModeBase::CreateUETexture(const carla::rpc::TextureFloatColor& Texture)
{
  FlushRenderingCommands();
  TArray<FFloat16Color> Colors;
  for (uint32_t y = 0; y < Texture.GetHeight(); y++)
  {
    for (uint32_t x = 0; x < Texture.GetWidth(); x++)
    {
      auto& Color = Texture.At(x,y);
      Colors.Add(FLinearColor(Color.r, Color.g, Color.b, Color.a));
    }
  }
  UTexture2D* UETexture = UTexture2D::CreateTransient(Texture.GetWidth(), Texture.GetHeight(), EPixelFormat::PF_FloatRGBA);
  FTexture2DMipMap& Mip = UETexture->GetPlatformData()->Mips[0];
  void* Data = Mip.BulkData.Lock( LOCK_READ_WRITE );
  FMemory::Memcpy( Data,
      &Colors[0],
      Texture.GetWidth()*Texture.GetHeight()*sizeof(FFloat16Color));
  Mip.BulkData.Unlock();
  UETexture->UpdateResource();
  return UETexture;
}

void ACarlaGameModeBase::ApplyTextureToActor(
    AActor* Actor,
    UTexture2D* Texture,
    const carla::rpc::MaterialParameter& TextureParam)
{
  namespace cr = carla::rpc;
  TArray<UStaticMeshComponent*> StaticMeshes;
  Actor->GetComponents(StaticMeshes);
  for (UStaticMeshComponent* Mesh : StaticMeshes)
  {
    for (int i = 0; i < Mesh->GetNumMaterials(); ++i)
    {
      UMaterialInterface* OriginalMaterial = Mesh->GetMaterial(i);
      UMaterialInstanceDynamic* DynamicMaterial = Cast<UMaterialInstanceDynamic>(OriginalMaterial);
      if(!DynamicMaterial)
      {
        DynamicMaterial = UMaterialInstanceDynamic::Create(OriginalMaterial, NULL);
        Mesh->SetMaterial(i, DynamicMaterial);
      }

      switch(TextureParam)
      {
        case cr::MaterialParameter::Tex_Diffuse:
          DynamicMaterial->SetTextureParameterValue("BaseColor", Texture);
          DynamicMaterial->SetTextureParameterValue("Difuse", Texture);
          DynamicMaterial->SetTextureParameterValue("Difuse 2", Texture);
          DynamicMaterial->SetTextureParameterValue("Difuse 3", Texture);
          DynamicMaterial->SetTextureParameterValue("Difuse 4", Texture);
          break;
        case cr::MaterialParameter::Tex_Normal:
          DynamicMaterial->SetTextureParameterValue("Normal", Texture);
          DynamicMaterial->SetTextureParameterValue("Normal 2", Texture);
          DynamicMaterial->SetTextureParameterValue("Normal 3", Texture);
          DynamicMaterial->SetTextureParameterValue("Normal 4", Texture);
          break;
        case cr::MaterialParameter::Tex_Emissive:
          DynamicMaterial->SetTextureParameterValue("Emissive", Texture);
          break;
        case cr::MaterialParameter::Tex_Ao_Roughness_Metallic_Emissive:
          DynamicMaterial->SetTextureParameterValue("AO / Roughness / Metallic / Emissive", Texture);
          DynamicMaterial->SetTextureParameterValue("ORMH", Texture);
          DynamicMaterial->SetTextureParameterValue("ORMH 2", Texture);
          DynamicMaterial->SetTextureParameterValue("ORMH 3", Texture);
          DynamicMaterial->SetTextureParameterValue("ORMH 4", Texture);
          break;
      }
    }
  }
}

namespace VRBoarding
{
  /// The ego vehicle (role_name "hero")
  static AActor* FindHero(UCarlaEpisode* Episode)
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
        return View->GetActor();
      }
    }
    return nullptr;
  }

  /// The vehicle the player is in: the one its pawn is attached to (through the driving seat), or the
  /// one it possesses
  static ACarlaWheeledVehicle* VehicleOfPlayer(APawn* Pawn)
  {
    if (ACarlaWheeledVehicle* Possessed = Cast<ACarlaWheeledVehicle>(Pawn))
    {
      return Possessed;
    }
    AActor* Actor = Pawn != nullptr ? Pawn->GetAttachParentActor() : nullptr;
    for (int32 Depth = 0; Actor != nullptr && Depth < 6; ++Depth)
    {
      if (ACarlaWheeledVehicle* Vehicle = Cast<ACarlaWheeledVehicle>(Actor))
      {
        return Vehicle;
      }
      AActor* Parent = Actor->GetParentActor();  // a child actor (the driving seat) -> its vehicle
      Actor = Parent != nullptr ? Parent : Actor->GetAttachParentActor();
    }
    return nullptr;
  }

  /// The VR vehicles' blueprint (BP_Master_VehiclesVR and children)
  static bool IsVRVehicle(const AActor* Actor)
  {
    for (const UClass* Class = Actor != nullptr ? Actor->GetClass() : nullptr; Class != nullptr; Class = Class->GetSuperClass())
    {
      if (Class->GetName().StartsWith(TEXT("BP_Master_VehiclesVR")))
      {
        return true;
      }
    }
    return false;
  }

  /// Calls Object's blueprint function / event, its object parameters filled with the first candidate of
  /// their class (the others left at their defaults). False when it has none, or a parameter can't be filled
  static bool CallWithObjects(UObject* Object, const FName& FunctionName, const TArray<UObject*>& Candidates)
  {
    UFunction* Function = Object != nullptr ? Object->FindFunction(FunctionName) : nullptr;
    if (Function == nullptr)
    {
      return false;
    }
    uint8* Params = static_cast<uint8*>(FMemory_Alloca(FMath::Max<int32>(1, Function->ParmsSize)));
    FMemory::Memzero(Params, Function->ParmsSize);
    for (TFieldIterator<FProperty> It(Function); It && It->HasAnyPropertyFlags(CPF_Parm); ++It)
    {
      It->InitializeValue_InContainer(Params);
    }
    bool bFilled = true;
    for (TFieldIterator<FProperty> It(Function); It && It->HasAnyPropertyFlags(CPF_Parm); ++It)
    {
      if (It->HasAnyPropertyFlags(CPF_ReturnParm) ||
          (It->HasAnyPropertyFlags(CPF_OutParm) && !It->HasAnyPropertyFlags(CPF_ReferenceParm)))
      {
        continue;
      }
      if (FObjectPropertyBase* ObjectParam = CastField<FObjectPropertyBase>(*It))
      {
        UObject* Value = nullptr;
        for (UObject* Candidate : Candidates)
        {
          if (Candidate != nullptr && Candidate->IsA(ObjectParam->PropertyClass))
          {
            Value = Candidate;
            break;
          }
        }
        if (Value == nullptr)
        {
          bFilled = false;
          break;
        }
        ObjectParam->SetObjectPropertyValue_InContainer(Params, Value);
      }
    }
    if (bFilled)
    {
      Object->ProcessEvent(Function, Params);
    }
    for (TFieldIterator<FProperty> It(Function); It && It->HasAnyPropertyFlags(CPF_Parm); ++It)
    {
      It->DestroyValue_InContainer(Params);
    }
    return bFilled;
  }
}

namespace VRBoarding
{
  /// The blueprint functions going through ACarlaGameModeBase::EgoOnlyThunk, and their native function before
  /// (UObject::ProcessInternal, running their script: not exported by CoreUObject, kept from the functions)
  static TMap<TWeakObjectPtr<UFunction>, FNativeFuncPtr> EgoOnlyFunctions;

  /// The vehicle a call is about: its caller, or one of its object parameters (the vehicle, or its driving seat
  /// child actor)
  static ACarlaWheeledVehicle* VehicleOfCall(UObject* Caller, UFunction* Function, uint8* Params)
  {
    auto VehicleOf = [](UObject* Object) -> ACarlaWheeledVehicle*
    {
      AActor* Actor = Cast<AActor>(Object);
      for (int32 Depth = 0; Actor != nullptr && Depth < 6; ++Depth)
      {
        if (ACarlaWheeledVehicle* Vehicle = Cast<ACarlaWheeledVehicle>(Actor))
        {
          return Vehicle;
        }
        if (Actor->IsA<APawn>() || Actor->IsA<AController>())
        {
          return nullptr;  // the player's pawn / controller, not a vehicle part
        }
        AActor* Parent = Actor->GetParentActor();
        Actor = Parent != nullptr ? Parent : Actor->GetAttachParentActor();
      }
      return nullptr;
    };
    if (ACarlaWheeledVehicle* Vehicle = VehicleOf(Caller))
    {
      return Vehicle;
    }
    for (TFieldIterator<FProperty> It(Function); It && It->HasAnyPropertyFlags(CPF_Parm); ++It)
    {
      if (FObjectPropertyBase* ObjectParam = CastField<FObjectPropertyBase>(*It))
      {
        if (ACarlaWheeledVehicle* Vehicle = VehicleOf(ObjectParam->GetObjectPropertyValue_InContainer(Params)))
        {
          return Vehicle;
        }
      }
    }
    return nullptr;
  }

  /// Makes Class's blueprint function go through EgoOnlyThunk (bHook), or back to its script
  static void HookEgoOnly(UClass* Class, const FName& FunctionName, bool bHook, FNativeFuncPtr Thunk)
  {
    UFunction* Function = Class != nullptr ? Class->FindFunctionByName(FunctionName) : nullptr;
    if (Function == nullptr || Function->Script.Num() == 0)
    {
      return;  // not this blueprint
    }
    const FNativeFuncPtr* Original = EgoOnlyFunctions.Find(Function);
    if (!bHook)
    {
      if (Original != nullptr && Function->GetNativeFunc() == Thunk)
      {
        Function->FunctionFlags &= ~FUNC_Native;
        Function->SetNativeFunc(*Original);
      }
      EgoOnlyFunctions.Remove(Function);
      return;
    }
    if (Function->GetNativeFunc() == Thunk)
    {
      return;  // hooked already (the class outlives the level)
    }
    for (TFieldIterator<FProperty> It(Function); It && It->HasAnyPropertyFlags(CPF_Parm); ++It)
    {
      if (It->HasAnyPropertyFlags(CPF_OutParm | CPF_ReturnParm))
      {
        CARLA_VR_LOG(TEXT("VR: %s.%s has output parameters, not filtered"), *Class->GetName(), *FunctionName.ToString());
        return;
      }
    }
    if (Function->HasAnyFunctionFlags(FUNC_Native) || Function->GetNativeFunc() == nullptr)
    {
      return;
    }
    EgoOnlyFunctions.Add(Function, Function->GetNativeFunc());
    Function->FunctionFlags |= FUNC_Native;
    Function->SetNativeFunc(Thunk);
    CARLA_VR_LOG(TEXT("VR: %s.%s filtered, only run for the ego"), *Class->GetName(), *FunctionName.ToString());
  }
}

void ACarlaGameModeBase::HookSetPlayerOnboard(bool bHook)
{
    // Every VR vehicle's blueprint calls the VR game mode's Set_Player_Onboard as it begins play: with VR
    // vehicles in the traffic ('full' mode, for their prediction arrow), the player jumped into each new one
    // (with a camera fade). The blueprint function is made to go through EgoOnlyThunk, which only lets it run
    // for the ego (the rest of the vehicles' blueprints run as usual)
    VRBoarding::HookEgoOnly(GetClass(), FName(TEXT("Set_Player_Onboard")), bHook, &ACarlaGameModeBase::EgoOnlyThunk);
    if (!bHook && DashboardHookedClass.IsValid())
    {
        VRBoarding::HookEgoOnly(DashboardHookedClass.Get(), FName(TEXT("E_VR_Vehicle_Spawn")), false, &ACarlaGameModeBase::EgoOnlyThunk);
        DashboardHookedClass.Reset();
    }
}

void ACarlaGameModeBase::HookDashboardVehicleSpawn()
{
    // Some VR vehicles (BP_XAI_vivace) tell the dashboard blueprint they spawned (E_VR_Vehicle_Spawn), which
    // then takes them for the ego vehicle: with VR vehicles in the traffic, only the ego's call is run (see
    // DashboardNotifyVehicleSpawn)
    if (!IsValid(DashboardComponent) || DashboardHookedClass.Get() == DashboardComponent->GetClass())
    {
        return;
    }
    DashboardHookedClass = DashboardComponent->GetClass();
    VRBoarding::HookEgoOnly(DashboardComponent->GetClass(), FName(TEXT("E_VR_Vehicle_Spawn")), true, &ACarlaGameModeBase::EgoOnlyThunk);
}

bool ACarlaGameModeBase::AllowPlayerOnboard(ACarlaWheeledVehicle* Vehicle)
{
    if (Vehicle == nullptr)
    {
        return true;
    }
    // The ego, once registered (by the time the blueprints board, after their Delay)
    if (AActor* Hero = VRBoarding::FindHero(Episode))
    {
        return Vehicle == Hero;
    }
    // Else the first vehicle: not while the player is on board another one
    UWorld* World = GetWorld();
    APlayerController* PlayerController = World != nullptr ? World->GetFirstPlayerController() : nullptr;
    ACarlaWheeledVehicle* Current = VRBoarding::VehicleOfPlayer(PlayerController != nullptr ? PlayerController->GetPawn() : nullptr);
    return !IsValid(Current) || Current->IsActorBeingDestroyed() || Current == Vehicle;
}

DEFINE_FUNCTION(ACarlaGameModeBase::EgoOnlyThunk)
{
    // UFunction::Invoke sets the function being called, in the caller's frame (called from a blueprint) or in
    // the function's own one (called through ProcessEvent)
    UFunction* Function = Stack.CurrentNativeFunction;
    const FNativeFuncPtr* Original = Function != nullptr ? VRBoarding::EgoOnlyFunctions.Find(Function) : nullptr;
    if (Original == nullptr)
    {
        return;
    }
    UWorld* World = Context != nullptr ? Context->GetWorld() : nullptr;
    ACarlaGameModeBase* GameMode = World != nullptr ? Cast<ACarlaGameModeBase>(World->GetAuthGameMode()) : nullptr;

    if (Stack.Node == Function)
    {
        // Called through ProcessEvent (this frame is the function's own, its parameters in Locals): the
        // blueprint function runs as it would have
        ACarlaWheeledVehicle* Vehicle = VRBoarding::VehicleOfCall(nullptr, Function, Stack.Locals);
        if (GameMode == nullptr || GameMode->AllowPlayerOnboard(Vehicle))
        {
            if (GameMode != nullptr && Vehicle != nullptr && Function->GetFName() == FName(TEXT("E_VR_Vehicle_Spawn")))
            {
                GameMode->DashboardNotifiedVehicle = Vehicle;  // the dashboard knows the ego already
            }
            (**Original)(Context, Stack, RESULT_PARAM);
        }
        else
        {
            CARLA_VR_LOG(TEXT("VR: %s not run for %s (not the ego)"), *Function->GetName(), *GetNameSafe(Vehicle));
        }
        return;
    }

    // Called from a blueprint (this frame is the caller's): the parameters are read from its code, then the
    // function is called through ProcessEvent (above) if it is allowed
    uint8* Params = static_cast<uint8*>(FMemory_Alloca_Aligned(FMath::Max<int32>(1, Function->ParmsSize), Function->GetMinAlignment()));
    FMemory::Memzero(Params, Function->ParmsSize);
    for (TFieldIterator<FProperty> It(Function); It && It->HasAnyPropertyFlags(CPF_Parm); ++It)
    {
        It->InitializeValue_InContainer(Params);
    }
    for (TFieldIterator<FProperty> It(Function); It && It->HasAnyPropertyFlags(CPF_Parm); ++It)
    {
        if (FBoolProperty* BoolParam = CastField<FBoolProperty>(*It))
        {
            uint32 Value = 0;  // the code gives bools as 0 / 1
            Stack.StepCompiledIn(&Value, FBoolProperty::StaticClass());
            BoolParam->SetPropertyValue_InContainer(Params, Value != 0);
        }
        else
        {
            Stack.StepCompiledIn(It->ContainerPtrToValuePtr<void>(Params), It->GetClass());
        }
    }
    P_FINISH;

    ACarlaWheeledVehicle* Vehicle = VRBoarding::VehicleOfCall(Stack.Object, Function, Params);
    if (GameMode == nullptr || GameMode->AllowPlayerOnboard(Vehicle))
    {
        Context->ProcessEvent(Function, Params);
    }
    else
    {
        CARLA_VR_LOG(TEXT("VR: %s not run for %s (not the ego)"), *Function->GetName(), *GetNameSafe(Vehicle));
    }
    for (TFieldIterator<FProperty> It(Function); It && It->HasAnyPropertyFlags(CPF_Parm); ++It)
    {
        It->DestroyValue_InContainer(Params);
    }
}

void ACarlaGameModeBase::KeepPlayerInEgoVehicle()
{
    // Every VR vehicle puts the player on board as it spawns (its BeginPlay calls the VR game mode's
    // Set_Player_Onboard): with other VR vehicles in the traffic ('full' mode), the view jumped into the
    // last one spawned, and the wheel / pedals drove it. The player is put back on board the ego, the same
    // way (else attached back where it was in the ego)
    UWorld* World = GetWorld();
    APlayerController* PlayerController = World != nullptr ? World->GetFirstPlayerController() : nullptr;
    APawn* Pawn = PlayerController != nullptr ? PlayerController->GetPawn() : nullptr;
    AActor* Hero = VRBoarding::FindHero(Episode);
    if (Pawn == nullptr || Hero == nullptr || !VRBoarding::IsVRVehicle(Hero))
    {
        return;
    }
    ACarlaWheeledVehicle* Vehicle = VRBoarding::VehicleOfPlayer(Pawn);
    USceneComponent* PawnRoot = Pawn->GetRootComponent();
    if (Vehicle == Hero)
    {
        // Where the player sits in the ego, to put it back there if needed
        if (PawnRoot != nullptr && PawnRoot->GetAttachParent() != nullptr)
        {
            EgoSeatComponent = PawnRoot->GetAttachParent();
            EgoSeatSocket = PawnRoot->GetAttachSocketName();
            EgoSeatRelative = PawnRoot->GetRelativeTransform();
        }
        return;
    }
    if (Vehicle == nullptr)
    {
        return;  // not on board any vehicle (e.g. the VR menu, or the ego not boarded yet)
    }
    const double Now = FPlatformTime::Seconds();
    if (Now - LastEgoBoardingTime < 1.0)
    {
        return;  // asked already, let it act
    }
    LastEgoBoardingTime = Now;

    TArray<UObject*> Candidates;
    Candidates.Add(Hero);
    TArray<AActor*> HeroChildren;
    Hero->GetAllChildActors(HeroChildren, true);
    Candidates.Append(HeroChildren);
    Candidates.Add(PlayerController);
    Candidates.Add(Pawn);
    const bool bBoarded = VRBoarding::CallWithObjects(this, FName(TEXT("Set_Player_Onboard")), Candidates);
    CARLA_VR_LOG(TEXT("VR: the player was taken on board %s, put back on board the ego %s (%s)"),
        *Vehicle->GetName(), *Hero->GetName(), bBoarded ? TEXT("Set_Player_Onboard") : TEXT("attached back"));
    if (!bBoarded && PawnRoot != nullptr && EgoSeatComponent.IsValid())
    {
        PawnRoot->AttachToComponent(EgoSeatComponent.Get(), FAttachmentTransformRules::KeepRelativeTransform, EgoSeatSocket);
        PawnRoot->SetRelativeTransform(EgoSeatRelative);
    }
    // The wheel / pedals drove that vehicle meanwhile: if it took them for a driver takeover, it gives the
    // control back to the traffic manager
    Vehicle->ReleaseDriverTakeover();
}

void ACarlaGameModeBase::DestroyPreviousVehicles()
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }

    // The player's VR pawn (and what it holds) is kept: the pawns of the VR plugin that are not vehicles
    TArray<AActor*> Kept;
    for (TActorIterator<APawn> It(World); It; ++It)
    {
        if (!It->IsA<ACarlaWheeledVehicle>() && It->GetClass()->GetPathName().StartsWith(TEXT("/CarlaVR_Vehicles/")))
        {
            Kept.Add(*It);
        }
    }
    auto IsKept = [&Kept](const AActor* Actor)
    {
        for (const AActor* KeptActor : Kept)
        {
            if (Actor == KeptActor || Actor->IsAttachedTo(KeptActor))
            {
                return true;
            }
        }
        return false;
    };
    for (AActor* KeptActor : Kept)
    {
        if (KeptActor->GetAttachParentActor() != nullptr)
        {
            KeptActor->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        }
    }

    // All the vehicles (the ones of the client's previous runs are registered in the episode, the ones a VR
    // blueprint spawned are not, the client can't see them). Their attached actors go with them (see
    // ACarlaWheeledVehicle::EndPlay)
    int32 Vehicles = 0;
    for (TActorIterator<ACarlaWheeledVehicle> It(World); It; ++It)
    {
        ACarlaWheeledVehicle* Vehicle = *It;
        if (!IsValid(Vehicle) || Vehicle->IsActorBeingDestroyed() || Vehicle->HasAnyFlags(RF_WasLoaded))
        {
            continue;  // placed in the level (parked cars)
        }
        if (Episode == nullptr || !Episode->DestroyActor(Vehicle))
        {
            Vehicle->Destroy();
        }
        ++Vehicles;
    }

    // The parts of the VR vehicles' interior left behind, detached (driving seat with its steering wheel and
    // pedals, mirrors). Only those: the dashboard's own elements (bounding boxes, AR-HUD...) are kept
    int32 Others = 0;
    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!IsValid(Actor) || Actor->IsActorBeingDestroyed() || Actor->HasAnyFlags(RF_WasLoaded) ||
            Actor->IsA<AController>() || Actor->IsA<AInfo>() || Actor->IsA<AHUD>() ||
            Actor->IsA<APlayerCameraManager>() || Actor == DashboardComponent || IsKept(Actor) ||
            Actor->GetAttachParentActor() != nullptr || Actor->GetParentActor() != nullptr ||
            !Actor->GetClass()->GetPathName().StartsWith(TEXT("/CarlaVR_Vehicles/")))
        {
            continue;
        }
        bool bInteriorPart = false;
        for (const UClass* Class = Actor->GetClass(); Class != nullptr && !bInteriorPart; Class = Class->GetSuperClass())
        {
            const FString ClassName = Class->GetName();
            bInteriorPart = ClassName.Contains(TEXT("DrivingSeat")) || ClassName.Contains(TEXT("Mirrors")) ||
                ClassName.Contains(TEXT("Steering"));
        }
        if (!bInteriorPart)
        {
            continue;
        }
        Actor->Destroy();
        ++Others;
    }
    DashboardNotifiedVehicle.Reset();
    // A new route: no box nor arrow of the previous one (the client sends its obstacles again)
    DashboardDestroyServerBoxes();
    DashboardDestroyArrows();
    if (IsValid(DashboardComponent))
    {
        DashboardComponent->ObstacleList.Empty();
    }
    CARLA_VR_LOG(TEXT("Previous runs cleaned up: %d vehicles, %d other actors destroyed"), Vehicles, Others);
}

void ACarlaGameModeBase::Tick(float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaGameModeBase::Tick);
  const double TickStart = FPlatformTime::Seconds();
  ON_SCOPE_EXIT
  {
    LogFrameTiming(FPlatformTime::Seconds() - TickStart);
  };
  KeepPlayerInEgoVehicle();
  DashboardUpdateHUDFrustum();
  DashboardUpdateThickBoxes();
  DashboardUpdateArrows();
  KeepPlayerSeatAdjustment();
  if (GetWorld()->IsPaused())
  {
    // Frame only rendered, the simulation does not advance: only the gaze is sampled. P (give the
    // control back) is checked by the vehicle on simulation frames only: a press on this frame would be lost
    GazeRecorder.Tick(GetWorld(), Episode);
    APlayerController* PlayerController = GetWorld()->GetFirstPlayerController();
    if (PlayerController != nullptr && PlayerController->WasInputKeyJustPressed(EKeys::P))
    {
      for (TActorIterator<ACarlaWheeledVehicle> It(GetWorld()); It; ++It)
      {
        It->ReleaseDriverTakeover();
      }
    }
    return;
  }
  Super::Tick(DeltaSeconds);

  /// @todo Recorder should not tick here, FCarlaEngine should do it.
  if (Recorder)
  {
    Recorder->Tick(DeltaSeconds);
  }

  DashboardPlaceTextActor();
  GazeRecorder.Tick(GetWorld(), Episode);
}

void ACarlaGameModeBase::LogFrameTiming(double GameModeSeconds)
{
    const double Now = FPlatformTime::Seconds();
    if (FrameTimingStart == 0.0)
    {
        FrameTimingStart = FrameTimingLast = Now;
        return;
    }
    FrameTimingMaxFrame = FMath::Max(FrameTimingMaxFrame, 1000.0 * (Now - FrameTimingLast));
    FrameTimingLast = Now;
    ++FrameTimingFrames;
    // The engine's own measures of the last frame (as 'stat unit' shows them, also in Shipping builds)
    FrameTimingGame += FPlatformTime::ToMilliseconds(GGameThreadTime);
    FrameTimingRender += FPlatformTime::ToMilliseconds(GRenderThreadTime);
    FrameTimingRHI += FPlatformTime::ToMilliseconds(GRHIThreadTime);
    FrameTimingGPU += FPlatformTime::ToMilliseconds(RHIGetGPUFrameCycles(0));
    FrameTimingGameMode += 1000.0 * GameModeSeconds;
    if (Now - FrameTimingStart < FrameTimingPeriod)
    {
        return;
    }
    const double N = FrameTimingFrames;
    const double Frame = 1000.0 * (Now - FrameTimingStart) / N;
    const double Game = FrameTimingGame / N, Render = FrameTimingRender / N, RHI = FrameTimingRHI / N,
        GPU = FrameTimingGPU / N;
    // The slowest of them sets the frame time (they run in parallel, one frame apart)
    const TCHAR* Limit = TEXT("GPU");
    double Slowest = GPU;
    if (Game > Slowest) { Slowest = Game; Limit = TEXT("game thread"); }
    if (Render > Slowest) { Slowest = Render; Limit = TEXT("render thread"); }
    if (RHI > Slowest) { Slowest = RHI; Limit = TEXT("RHI thread"); }
    CARLA_VR_LOG(TEXT("Frame timing: %.1f ms per frame (%.1f fps, longest %.0f ms): game thread %.1f (of which game mode %.1f), render thread %.1f, RHI thread %.1f, GPU %.1f -> limited by the %s"),
        Frame, 1000.0 / Frame, FrameTimingMaxFrame, Game, FrameTimingGameMode / N, Render, RHI, GPU, Limit);
    FrameTimingStart = Now;
    FrameTimingFrames = 0;
    FrameTimingGame = FrameTimingRender = FrameTimingRHI = FrameTimingGPU = FrameTimingGameMode = 0.0;
    FrameTimingMaxFrame = 0.0;
}

void ACarlaGameModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  HookSetPlayerOnboard(false);
  GazeRecorder.Stop();
  if (DashboardActorSpawnedHandle.IsValid() && GetWorld() != nullptr)
  {
    GetWorld()->RemoveOnActorSpawnedHandler(DashboardActorSpawnedHandle);
    DashboardActorSpawnedHandle.Reset();
  }
  if (DashboardBoxSpawnedHandle.IsValid() && GetWorld() != nullptr)
  {
    GetWorld()->RemoveOnActorSpawnedHandler(DashboardBoxSpawnedHandle);
    DashboardBoxSpawnedHandle.Reset();
  }
  DashboardDestroyArrows();
  FCarlaStaticDelegates::OnEpisodeSettingsChange.Remove(OnEpisodeSettingsChangeHandle);

  Episode->EndPlay();
  GameInstance->NotifyEndEpisode();

  Super::EndPlay(EndPlayReason);

  if ((CarlaSettingsDelegate != nullptr) && (EndPlayReason != EEndPlayReason::EndPlayInEditor))
  {
    CarlaSettingsDelegate->Reset();
  }
}

void ACarlaGameModeBase::SpawnActorFactories()
{
  auto *World = GetWorld();
  check(World != nullptr);

  for (auto &FactoryClass : ActorFactories)
  {
    if (FactoryClass != nullptr)
    {
      auto *Factory = World->SpawnActor<ACarlaActorFactory>(FactoryClass);
      if (Factory != nullptr)
      {
        Episode->RegisterActorFactory(*Factory);
        ActorFactoryInstances.Add(Factory);
      }
      else
      {
        UE_LOG(LogCarla, Error, TEXT("Failed to spawn actor spawner"));
      }
    }
  }
}

void ACarlaGameModeBase::StoreSpawnPoints()
{
  for (TActorIterator<AVehicleSpawnPoint> It(GetWorld()); It; ++It)
  {
    SpawnPointsTransforms.Add(It->GetActorTransform());
  }

  if(SpawnPointsTransforms.Num() == 0)
  {
    GenerateSpawnPoints();
  }

  UE_LOG(LogCarla, Log, TEXT("There are %d SpawnPoints in the map"), SpawnPointsTransforms.Num());
}

void ACarlaGameModeBase::GenerateSpawnPoints()
{
  UE_LOG(LogCarla, Log, TEXT("Generating SpawnPoints ..."));
  std::vector<std::pair<carla::road::element::Waypoint, carla::road::element::Waypoint>> Topology = Map->GenerateTopology();
  UWorld* World = GetWorld();
  for(auto& Pair : Topology)
  {
    carla::geom::Transform CarlaTransform = Map->ComputeTransform(Pair.first);
    FTransform Transform(CarlaTransform);
    Transform.AddToTranslation(FVector(0.f, 0.f, 50.0f));
    SpawnPointsTransforms.Add(Transform);
  }
}

void ACarlaGameModeBase::ParseOpenDrive()
{
  std::string opendrive_xml = carla::rpc::FromLongFString(UOpenDrive::GetXODR(GetWorld()));
  Map = carla::opendrive::OpenDriveParser::Load(opendrive_xml);
  if (!Map.has_value()) {
    UE_LOG(LogCarla, Error, TEXT("Invalid Map"));
  }
  else
  {
    Episode->MapGeoReference = Map->GetGeoReference();
  }
}

ATrafficLightManager* ACarlaGameModeBase::GetTrafficLightManager()
{
  if (!TrafficLightManager)
  {
    UWorld* World = GetWorld();
    AActor* TrafficLightManagerActor = UGameplayStatics::GetActorOfClass(World, ATrafficLightManager::StaticClass());
    if(TrafficLightManagerActor == nullptr)
    {
      FActorSpawnParameters SpawnParams;
      SpawnParams.OverrideLevel = GetULevelFromName("TrafficLights");
      TrafficLightManager = World->SpawnActor<ATrafficLightManager>(SpawnParams);
    }
    else
    {
      TrafficLightManager = Cast<ATrafficLightManager>(TrafficLightManagerActor);
    }
  }
  return TrafficLightManager;
}

void ACarlaGameModeBase::CheckForEmptyMeshes()
{
  TArray<AActor*> WorldActors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AStaticMeshActor::StaticClass(), WorldActors);

  for (AActor *Actor : WorldActors)
  {
    AStaticMeshActor *MeshActor = CastChecked<AStaticMeshActor>(Actor);
    if (MeshActor->GetStaticMeshComponent()->GetStaticMesh() == NULL)
    {
      UE_LOG(LogTemp, Error, TEXT("The object : %s has no mesh"), *MeshActor->GetFullName());
    }
  }
}

void ACarlaGameModeBase::EnableOverlapEvents()
{
  TArray<AActor*> WorldActors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AStaticMeshActor::StaticClass(), WorldActors);

  for(AActor *Actor : WorldActors)
  {
    AStaticMeshActor *MeshActor = CastChecked<AStaticMeshActor>(Actor);
    if(MeshActor->GetStaticMeshComponent()->GetStaticMesh() != NULL)
    {
      auto MeshTag = ATagger::GetTagOfTaggedComponent(*MeshActor->GetStaticMeshComponent());
      namespace crp = carla::rpc;
      if (MeshTag != crp::CityObjectLabel::Roads && 
          MeshTag != crp::CityObjectLabel::Sidewalks && 
          MeshTag != crp::CityObjectLabel::RoadLines && 
          MeshTag != crp::CityObjectLabel::Ground &&
          MeshTag != crp::CityObjectLabel::Terrain &&
          MeshActor->GetStaticMeshComponent()->GetGenerateOverlapEvents() == false)
      {
        MeshActor->GetStaticMeshComponent()->SetGenerateOverlapEvents(true);
      }
    }
  }
}

void ACarlaGameModeBase::DebugShowSignals(bool enable)
{

  auto World = GetWorld();
  check(World != nullptr);

  if(!Map)
  {
    return;
  }

  if(!enable)
  {
    UKismetSystemLibrary::FlushDebugStrings(World);
    UKismetSystemLibrary::FlushPersistentDebugLines(World);
    return;
  }

  //const std::unordered_map<carla::road::SignId, std::unique_ptr<carla::road::Signal>>
  const auto& Signals = Map->GetSignals();
  const auto& Controllers = Map->GetControllers();

  for(const auto& Signal : Signals) {
    const auto& ODSignal = Signal.second;
    const FTransform Transform = ODSignal->GetTransform();
    const FVector Location = Transform.GetLocation();
    const FQuat Rotation = Transform.GetRotation();
    const FVector Up = Rotation.GetUpVector();
    DrawDebugSphere(
      World,
      Location,
      50.0f,
      10,
      FColor(0, 255, 0),
      true
    );
  }

  TArray<const cre::RoadInfoSignal*> References;
  auto waypoints = Map->GenerateWaypointsOnRoadEntries();
  std::unordered_set<cr::RoadId> ExploredRoads;
  for (auto & waypoint : waypoints)
  {
    // Check if we already explored this road
    if (ExploredRoads.count(waypoint.road_id) > 0)
    {
      continue;
    }
    ExploredRoads.insert(waypoint.road_id);

    // Multiple times for same road (performance impact, not in behavior)
    auto SignalReferences = Map->GetLane(waypoint).
        GetRoad()->GetInfos<cre::RoadInfoSignal>();
    for (auto *SignalReference : SignalReferences)
    {
      References.Add(SignalReference);
    }
  }
  for (auto& Reference : References)
  {
    auto RoadId = Reference->GetRoadId();
    const auto* SignalReference = Reference;
    const FTransform SignalTransform = SignalReference->GetSignal()->GetTransform();
    for(auto &validity : SignalReference->GetValidities())
    {
      for(auto lane : carla::geom::Math::GenerateRange(validity._from_lane, validity._to_lane))
      {
        if(lane == 0)
          continue;

        auto signal_waypoint = Map->GetWaypoint(
            RoadId, lane, SignalReference->GetS()).value();

        if(Map->GetLane(signal_waypoint).GetType() != cr::Lane::LaneType::Driving)
          continue;

        FTransform ReferenceTransform = Map->ComputeTransform(signal_waypoint);

        DrawDebugSphere(
            World,
            ReferenceTransform.GetLocation(),
            50.0f,
            10,
            FColor(0, 0, 255),
            true
        );

        DrawDebugLine(
            World,
            ReferenceTransform.GetLocation(),
            SignalTransform.GetLocation(),
            FColor(0, 0, 255),
            true
        );
      }
    }
  }

}

TArray<FBoundingBox> ACarlaGameModeBase::GetAllBBsOfLevel(uint8 TagQueried) const
{
  UWorld* World = GetWorld();

  // Get all actors of the level
  TArray<AActor*> FoundActors;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), FoundActors);

  TArray<FBoundingBox> BoundingBoxes;
  BoundingBoxes = UBoundingBoxCalculator::GetBoundingBoxOfActors(FoundActors, TagQueried);

  return BoundingBoxes;
}

void ACarlaGameModeBase::RegisterEnvironmentObjects()
{
  // Get all actors of the level
  TArray<AActor*> FoundActors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AActor::StaticClass(), FoundActors);
  ObjectRegister->RegisterObjects(FoundActors);
}

void ACarlaGameModeBase::EnableEnvironmentObjects(
  const TSet<uint64>& EnvObjectIds,
  bool Enable)
{
  ObjectRegister->EnableEnvironmentObjects(EnvObjectIds, Enable);
}

void ACarlaGameModeBase::LoadMapLayer(int32 MapLayers)
{
  const UWorld* World = GetWorld();
  UGameplayStatics::FlushLevelStreaming(World);

  TArray<FName> LevelsToLoad;
  ConvertMapLayerMaskToMapNames(MapLayers, LevelsToLoad);

  FLatentActionInfo LatentInfo;
  LatentInfo.CallbackTarget = this;
  LatentInfo.ExecutionFunction = "OnLoadStreamLevel";
  LatentInfo.Linkage = 0;
  LatentInfo.UUID = LatentInfoUUID;

  PendingLevelsToLoad = LevelsToLoad.Num();

  for(FName& LevelName : LevelsToLoad)
  {
    LatentInfoUUID++;
    UGameplayStatics::LoadStreamLevel(World, LevelName, true, true, LatentInfo);
    LatentInfo.UUID = LatentInfoUUID;
    UGameplayStatics::FlushLevelStreaming(World);
  }
}

void ACarlaGameModeBase::UnLoadMapLayer(int32 MapLayers)
{
  const UWorld* World = GetWorld();

  TArray<FName> LevelsToUnLoad;
  ConvertMapLayerMaskToMapNames(MapLayers, LevelsToUnLoad);

  FLatentActionInfo LatentInfo;
  LatentInfo.CallbackTarget = this;
  LatentInfo.ExecutionFunction = "OnUnloadStreamLevel";
  LatentInfo.UUID = LatentInfoUUID;
  LatentInfo.Linkage = 0;

  PendingLevelsToUnLoad = LevelsToUnLoad.Num();

  for(FName& LevelName : LevelsToUnLoad)
  {
    LatentInfoUUID++;
    UGameplayStatics::UnloadStreamLevel(World, LevelName, LatentInfo, false);
    LatentInfo.UUID = LatentInfoUUID;
    UGameplayStatics::FlushLevelStreaming(World);
  }

}

void ACarlaGameModeBase::ConvertMapLayerMaskToMapNames(int32 MapLayer, TArray<FName>& OutLevelNames)
{
  UWorld* World = GetWorld();
  const TArray <ULevelStreaming*> Levels = World->GetStreamingLevels();
  TArray<FString> LayersToLoad;

  // Get all the requested layers
  int32 LayerMask = 1;
  int32 AllLayersMask = static_cast<crp::MapLayerType>(crp::MapLayer::All);

  while(LayerMask > 0)
  {
    // Convert enum to FString
    FString LayerName = UTF8_TO_TCHAR(MapLayerToString(static_cast<crp::MapLayer>(LayerMask)).c_str());
    bool included = static_cast<crp::MapLayerType>(MapLayer) & LayerMask;
    if(included)
    {
      LayersToLoad.Emplace(LayerName);
    }
    LayerMask = (LayerMask << 1) & AllLayersMask;
  }

  // Get all the requested level maps
  for(ULevelStreaming* Level : Levels)
  {
    TArray<FString> StringArray;
    FString FullSubMapName = Level->GetWorldAssetPackageFName().ToString();
    // Discard full path, we just need the umap name
    FullSubMapName.ParseIntoArray(StringArray, TEXT("/"), false);
    FString SubMapName = StringArray[StringArray.Num() - 1];
    for(FString LayerName : LayersToLoad)
    {
      if(SubMapName.Contains(LayerName))
      {
        OutLevelNames.Emplace(FName(*SubMapName));
        break;
      }
    }
  }

}

ULevel* ACarlaGameModeBase::GetULevelFromName(FString LevelName)
{
  ULevel* OutLevel = nullptr;
  UWorld* World = GetWorld();
  const TArray <ULevelStreaming*> Levels = World->GetStreamingLevels();

  for(ULevelStreaming* Level : Levels)
  {
    FString FullSubMapName = Level->GetWorldAssetPackageFName().ToString();
    if(FullSubMapName.Contains(LevelName))
    {
      OutLevel = Level->GetLoadedLevel();
      if(!OutLevel)
      {
        UE_LOG(LogCarla, Warning, TEXT("%s has not been loaded"), *LevelName);
      }
      break;
    }
  }

  return OutLevel;
}

void ACarlaGameModeBase::OnLoadStreamLevel()
{
  PendingLevelsToLoad--;

  // Register new actors and tag them
  if(ReadyToRegisterObjects && PendingLevelsToLoad == 0)
  {
    RegisterEnvironmentObjects();
    ATagger::TagActorsInLevel(*GetWorld(), true);
  }
}

void ACarlaGameModeBase::OnUnloadStreamLevel()
{
  PendingLevelsToUnLoad--;
  // Update stored registered objects (discarding the deleted objects)
  if(ReadyToRegisterObjects && PendingLevelsToUnLoad == 0)
  {
    RegisterEnvironmentObjects();
  }
}

void ACarlaGameModeBase::SetDashboard(ADashboard* dash) {
    CARLA_VR_LOG(TEXT("Dashboard: set to %s (was %s)"), *GetNameSafe(dash), *GetNameSafe(DashboardComponent));
    DashboardComponent = dash;
    DashboardLogged = dash;
    HookDashboardVehicleSpawn();
}

void ACarlaGameModeBase::DashboardEnsureComponent()
{
    if (!IsValid(DashboardComponent) || DashboardComponent->IsActorBeingDestroyed())
    {
        DashboardComponent = nullptr;
        const double Now = FPlatformTime::Seconds();
        if (Now - DashboardLastSearchTime < 1.0)
        {
            return;
        }
        DashboardLastSearchTime = Now;
        for (TActorIterator<ADashboard> It(GetWorld()); It; ++It)
        {
            if (IsValid(*It) && !It->IsActorBeingDestroyed())
            {
                DashboardComponent = *It;
                break;
            }
        }
    }
    if (DashboardLogged.Get() != DashboardComponent)
    {
        CARLA_VR_LOG(TEXT("Dashboard: %s (%s)"), *GetNameSafe(DashboardComponent),
            DashboardComponent != nullptr ? TEXT("found in the world") : TEXT("none: the bounding boxes can't be drawn"));
        DashboardLogged = DashboardComponent;
    }
    HookDashboardVehicleSpawn();
    // The dashboard draws the boxes only once it knows its vehicle
    DashboardNotifyVehicleSpawn();
}

void ACarlaGameModeBase::OnEpisodeSettingsChanged(const FEpisodeSettings &Settings)
{
  CarlaSettingsDelegate->SetAllActorsDrawDistance(GetWorld(), Settings.MaxCullingDistance);
}

void ACarlaGameModeBase::DashboardSetControlValues(float speed_value, float steering_value)
{
  if(DashboardComponent)
  {
    DashboardComponent->SetControlValues(speed_value, steering_value);
  }
}

void ACarlaGameModeBase::DashboardSetTypeString(FString type)
{
    // "mode:<name>" selects the client's visualization mode: the AR-HUD driving aids (virtual
    // carpets, motion prediction arrows following the vehicle's steering and speed, navigation
    // waypoints) are only shown in the "full" mode
    // "ribbon:x,y,z,width;..." / "ribbon_material:<path>": global path on the road (see FPathRibbon)
    static const FString RibbonMaterialPrefix = TEXT("ribbon_material:");
    static const FString RibbonPrefix = TEXT("ribbon:");
    if (type.StartsWith(RibbonMaterialPrefix))
    {
        PathRibbon.SetMaterial(type.RightChop(RibbonMaterialPrefix.Len()));
        return;
    }
    if (type.StartsWith(RibbonPrefix))
    {
        PathRibbon.SetFromCommand(GetWorld(), type.RightChop(RibbonPrefix.Len()));
        return;
    }

    // "gaze:start:<name>" / "gaze:stop": recording of the driver's gaze (see FGazeRecorder)
    static const FString GazeStartPrefix = TEXT("gaze:start:");
    if (type.StartsWith(GazeStartPrefix))
    {
        GazeRecorder.Start(type.RightChop(GazeStartPrefix.Len()).TrimStartAndEnd());
        return;
    }
    if (type == TEXT("gaze:stop"))
    {
        GazeRecorder.Stop();
        return;
    }

    // "cleanup:vehicles": the vehicles of previous runs and what the VR blueprints spawned with them are
    // destroyed (sent by the client before a route starts)
    if (type == TEXT("cleanup:vehicles"))
    {
        DestroyPreviousVehicles();
        return;
    }

    static const FString ModePrefix = TEXT("mode:");
    if (type.StartsWith(ModePrefix))
    {
        const FString Mode = type.RightChop(ModePrefix.Len()).TrimStartAndEnd().ToLower();
        if (bDashboardHideDrivingAids != (Mode != TEXT("full")) || DashboardLogged.Get() != DashboardComponent)
        {
            DashboardEnsureComponent();
            CARLA_VR_LOG(TEXT("Dashboard: mode %s (dashboard %s)"), *Mode, *GetNameSafe(DashboardComponent));
        }
        bDashboardHideDrivingAids = (Mode != TEXT("full"));
        if (Mode != TEXT("full"))
        {
            PathRibbon.Clear();
            DashboardDestroyServerBoxes();
            DashboardDestroyArrows();
        }
        // The client sends the mode when a route starts: the global path of the previous route is
        // dropped (add_vector_pair_dashboard only appends to it), the new one is sent next
        if (DashboardComponent)
        {
            DashboardComponent->VectorPairList.Empty();
        }
        DashboardApplyDrivingAidsVisibility();
        return;
    }

    if(DashboardComponent)
    {
        DashboardComponent->SetTypeString(type);
    }
    DashboardWriteVehicleText(type);
}

bool ACarlaGameModeBase::DashboardHideIfDrivingAid(AActor* Actor)
{
    if (Actor == nullptr || Actor->IsHidden())
    {
        return false;
    }
    static const TCHAR* DrivingAidClassNames[] = { TEXT("VirtualCarpet"), TEXT("BP_VC_"), TEXT("Nav_Waypoint") };
    const FString ClassName = Actor->GetClass()->GetName();
    for (const TCHAR* AidClassName : DrivingAidClassNames)
    {
        if (ClassName.Contains(AidClassName))
        {
            Actor->SetActorHiddenInGame(true);
            DashboardHiddenDrivingAids.Add(Actor);
            return true;
        }
    }
    return false;
}

void ACarlaGameModeBase::DashboardOnActorSpawned(AActor* Actor)
{
    if (bDashboardHideDrivingAids)
    {
        DashboardHideIfDrivingAid(Actor);
    }
}

void ACarlaGameModeBase::DashboardApplyDrivingAidsVisibility()
{
    UWorld* World = GetWorld();
    if (bDashboardHideDrivingAids && World != nullptr && !DashboardActorSpawnedHandle.IsValid())
    {
        DashboardActorSpawnedHandle = World->AddOnActorSpawnedHandler(
            FOnActorSpawned::FDelegate::CreateUObject(this, &ACarlaGameModeBase::DashboardOnActorSpawned));
    }
    else if (!bDashboardHideDrivingAids && World != nullptr && DashboardActorSpawnedHandle.IsValid())
    {
        World->RemoveOnActorSpawnedHandler(DashboardActorSpawnedHandle);
        DashboardActorSpawnedHandle.Reset();
    }

    if (!bDashboardHideDrivingAids)
    {
        // Show back only what was hidden here (some aids are hidden by their own blueprints)
        for (const TWeakObjectPtr<AActor>& Hidden : DashboardHiddenDrivingAids)
        {
            if (Hidden.IsValid())
            {
                Hidden->SetActorHiddenInGame(false);
            }
        }
        DashboardHiddenDrivingAids.Empty();
        return;
    }

    // Once at the mode change: the ones spawned later are hidden by DashboardOnActorSpawned
    for (TActorIterator<AActor> It(GetWorld()); It; ++It)
    {
        DashboardHideIfDrivingAid(*It);
    }
}

void ACarlaGameModeBase::DashboardPlaceTextActor()
{
    if (!DashboardTextActor.IsValid())
    {
        return;
    }
    APlayerCameraManager* Camera = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0);
    if (Camera == nullptr)
    {
        return;
    }
    // In front of the view, a bit below its center, facing it (a text render is read from its +X side)
    const FRotator ViewRotation = Camera->GetCameraRotation();
    const FRotator Facing(0.0f, ViewRotation.Yaw, 0.0f);
    const FVector Location = Camera->GetCameraLocation()
        + Facing.Vector() * DashboardTextDistance
        - FVector::UpVector * DashboardTextDrop;
    DashboardTextActor->SetActorLocationAndRotation(Location, FRotator(0.0f, ViewRotation.Yaw + 180.0f, 0.0f));
}

namespace DashboardText
{
  /// Warm amber: stands out against the blue sky (its opposite hue) and the grey road (the former cyan
  /// merged into the sky)
  static const FColor Color(255, 190, 40);

  /// Calls Object's function FunctionName if it takes a single text / string parameter. False otherwise
  static bool CallWithText(UObject* Object, const FName& FunctionName, const FString& Text)
  {
    UFunction* Function = Object->FindFunction(FunctionName);
    if (Function == nullptr || Function->NumParms != 1)
    {
      return false;
    }
    FProperty* Param = nullptr;
    for (TFieldIterator<FProperty> ParamIt(Function); ParamIt && ParamIt->HasAnyPropertyFlags(CPF_Parm); ++ParamIt)
    {
      Param = *ParamIt;
    }
    FTextProperty* TextParam = CastField<FTextProperty>(Param);
    FStrProperty* StrParam = CastField<FStrProperty>(Param);
    if (TextParam == nullptr && StrParam == nullptr)
    {
      return false;
    }
    uint8* Params = static_cast<uint8*>(FMemory_Alloca(Function->ParmsSize));
    FMemory::Memzero(Params, Function->ParmsSize);
    Param->InitializeValue_InContainer(Params);
    if (TextParam != nullptr)
    {
      TextParam->SetPropertyValue_InContainer(Params, FText::FromString(Text));
    }
    else
    {
      StrParam->SetPropertyValue_InContainer(Params, Text);
    }
    Object->ProcessEvent(Function, Params);
    Param->DestroyValue_InContainer(Params);
    return true;
  }
}

void ACarlaGameModeBase::DashboardWriteVehicleText(const FString& RawText)
{
    // Optional placement prefix "[height=<cm>]": height of the 3D text above the center of the view
    // (negative: below it). It is kept for the next texts
    FString type = RawText;
    static const FString HeightPrefix = TEXT("[height=");
    if (type.StartsWith(HeightPrefix))
    {
        int32 End = INDEX_NONE;
        if (type.FindChar(TEXT(']'), End))
        {
            const FString Value = type.Mid(HeightPrefix.Len(), End - HeightPrefix.Len()).TrimStartAndEnd();
            if (Value.IsNumeric())
            {
                DashboardTextDrop = -FCString::Atof(*Value);
            }
            type = type.RightChop(End + 1).TrimStart();
        }
    }

    // Shown as a 3D text in front of the player's view (visible in VR, and in packaged builds where
    // on-screen debug messages and logs are disabled). An empty text removes it
    if (type.IsEmpty())
    {
        if (DashboardTextActor.IsValid())
        {
            DashboardTextActor->Destroy();
        }
        DashboardTextActor = nullptr;
    }
    else
    {
        if (!DashboardTextActor.IsValid())
        {
            FActorSpawnParameters SpawnParams;
            SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            ATextRenderActor* TextActor = GetWorld()->SpawnActor<ATextRenderActor>(FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
            if (TextActor != nullptr)
            {
                UTextRenderComponent* TextRender = TextActor->GetTextRender();
                TextRender->SetHorizontalAlignment(EHTA_Center);
                TextRender->SetVerticalAlignment(EVRTA_TextCenter);
                TextRender->SetWorldSize(DashboardTextSize);
                TextRender->SetTextRenderColor(DashboardText::Color);
                TextRender->SetCastShadow(false);
                TextActor->SetActorEnableCollision(false);
                DashboardTextActor = TextActor;
            }
        }
        if (DashboardTextActor.IsValid())
        {
            DashboardTextActor->GetTextRender()->SetText(FText::FromString(type));
            DashboardPlaceTextActor();
        }
    }

    // Also printed on screen (like the blueprints' Print String, only in development builds),
    // replacing the previous text, and written to the log. An empty text clears it
    static const uint64 OnScreenTextKey = 0xDA5B0A2D;
    if (GEngine != nullptr)
    {
        if (type.IsEmpty())
        {
            GEngine->RemoveOnScreenDebugMessage(OnScreenTextKey);
        }
        else
        {
            GEngine->AddOnScreenDebugMessage(OnScreenTextKey, 1.0e6f, DashboardText::Color, type, false, FVector2D(2.0f, 2.0f));
        }
    }
    if (!type.IsEmpty())
    {
        UE_LOG(LogCarla, Log, TEXT("Dashboard text: %s"), *type);
    }

    // (No search of the WB_Text_XAI widgets here any more: it went through every UObject of the
    // engine, a hitch at each text change that showed as judder in VR, and never displayed the text.)

    // Vehicles with a 'Write_Text' / 'Write_XAI' event taking the text (e.g. other VR vehicles)
    static const FName WriteTextFunctions[] = { FName(TEXT("Write_Text")), FName(TEXT("Write_XAI")) };
    for (TActorIterator<APawn> It(GetWorld()); It; ++It)
    {
        for (const FName& FunctionName : WriteTextFunctions)
        {
            if (DashboardText::CallWithText(*It, FunctionName, type))
            {
                break;  // one event per vehicle
            }
        }
    }
}

void ACarlaGameModeBase::DashboardAddVectorPair(FVector P, FVector R)
{
    if(DashboardComponent)
    {
		DashboardComponent->AddVectorPair(P, R);
    }
}

void ACarlaGameModeBase::DashboardRemoveIDObstacle(int Id)
{
    if(DashboardComponent)
    {
        DashboardComponent->RemoveIDObstacle(Id);
    }
}

void ACarlaGameModeBase::DashboardUpdateObstacle(int ID, const FString& Type, bool isDanger, FVector Vector1, FVector Vector2, float obstacleSpeed, float obstacleSteer)
{
    DashboardEnsureComponent();
    if (DashboardComponent && DashboardLoggedObstacles.Num() < 20 && !DashboardLoggedObstacles.Contains(ID))
    {
        DashboardLoggedObstacles.Add(ID);
        CARLA_VR_LOG(TEXT("Dashboard: first box of obstacle %d (%s) sent to %s at (%.0f, %.0f, %.0f) cm"), ID, *Type,
            *DashboardComponent->GetName(), Vector1.X, Vector1.Y, Vector1.Z);
    }
    if(DashboardComponent)
    {
        DashboardComponent->UpdateObstacle(ID, Type, isDanger, Vector1, Vector2, obstacleSpeed, obstacleSteer);
    }
}

void ACarlaGameModeBase::DashboardUpdate()
{
    if(DashboardComponent)
    {
        DashboardComponent->UpdateDashboard();
    }
    DashboardUpdateBoxes();
}

namespace DashboardBoxes
{
  /// The dashboard blueprint's box of an obstacle type ('car', 'truck', 'pedestrian'), nullptr for the others
  static UClass* BoxClass(const FString& Type)
  {
    static TMap<FString, TWeakObjectPtr<UClass>> Classes;
    const FString Key = Type.ToLower();
    const TCHAR* Name = Key == TEXT("car") ? TEXT("BP_BB_Car") : Key == TEXT("truck") ? TEXT("BP_BB_Truck") :
        Key == TEXT("pedestrian") ? TEXT("BP_BB_Pedestrian") : nullptr;
    if (Name == nullptr)
    {
      return nullptr;
    }
    TWeakObjectPtr<UClass>& Class = Classes.FindOrAdd(Key);
    if (!Class.IsValid())
    {
      Class = LoadClass<AActor>(nullptr, *FString::Printf(TEXT("/CarlaVR_Vehicles/ARHUD_TestCase/BoundingBox/%s.%s_C"), Name, Name));
      if (!Class.IsValid())
      {
        CARLA_VR_LOG(TEXT("Dashboard: box class %s not found"), Name);
      }
    }
    return Class.Get();
  }

  static bool IsBox(const AActor* Actor)
  {
    return Actor->GetClass()->GetName().StartsWith(TEXT("BP_BB_"));
  }

  /// Red (danger) or yellow box: the box blueprint's flag, and its material
  static void SetDanger(AActor* Box, bool bDanger, bool bForce)
  {
    static const FName DangerTag(TEXT("CarlaBoxDanger"));
    if (!bForce && Box->Tags.Contains(DangerTag) == bDanger)
    {
      return;
    }
    if (bDanger)
    {
      Box->Tags.AddUnique(DangerTag);
    }
    else
    {
      Box->Tags.Remove(DangerTag);
    }
    for (const TCHAR* Name : { TEXT("isDang"), TEXT("isDanger"), TEXT("isDangerous") })
    {
      if (FBoolProperty* Flag = FindFProperty<FBoolProperty>(Box->GetClass(), Name))
      {
        Flag->SetPropertyValue_InContainer(Box, bDanger);
      }
    }
    UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, bDanger ?
        TEXT("/CarlaVR_Vehicles/ARHUD_TestCase/HUD_Materials/M_HUD_LightRed.M_HUD_LightRed") :
        TEXT("/CarlaVR_Vehicles/ARHUD_TestCase/HUD_Materials/M_HUD_LightYellow.M_HUD_LightYellow"));
    if (Material == nullptr)
    {
      return;
    }
    TArray<UStaticMeshComponent*> Meshes;
    Box->GetComponents<UStaticMeshComponent>(Meshes);
    for (UStaticMeshComponent* Mesh : Meshes)
    {
      for (int32 Slot = 0; Slot < Mesh->GetNumMaterials(); ++Slot)
      {
        Mesh->SetMaterial(Slot, Material);
      }
    }
  }
}

void ACarlaGameModeBase::DashboardUpdateBoxes()
{
    // The dashboard blueprint should spawn a box (BP_BB_*) for each obstacle of its list. When there are
    // obstacles and no visible box in the world for a few seconds, the server draws them itself, with the
    // same box blueprints: one per obstacle, following its position / heading, red when it is a danger
    UWorld* World = GetWorld();
    if (World == nullptr || !IsValid(DashboardComponent) || bDashboardHideDrivingAids)
    {
        return;
    }
    const TMap<FString, FObstacle>& Obstacles = DashboardComponent->ObstacleList;
    const double Now = FPlatformTime::Seconds();
    if (Obstacles.Num() == 0)
    {
        DashboardFirstObstacleTime = 0.0;
        return;
    }
    if (DashboardFirstObstacleTime == 0.0)
    {
        DashboardFirstObstacleTime = Now;
    }

    if (!bDashboardServerBoxes && Now - DashboardBoxCheckTime >= 3.0)
    {
        DashboardBoxCheckTime = Now;
        int32 Boxes = 0;
        int32 Visible = 0;
        FString Example;
        for (TActorIterator<AActor> It(World); It; ++It)
        {
            if (!IsValid(*It) || !DashboardBoxes::IsBox(*It))
            {
                continue;
            }
            ++Boxes;
            Visible += It->IsHidden() ? 0 : 1;
            if (Example.IsEmpty())
            {
                Example = FString::Printf(TEXT(", e.g. %s at %s%s"), *It->GetName(), *It->GetActorLocation().ToString(),
                    It->IsHidden() ? TEXT(" (hidden)") : TEXT(""));
            }
        }
        if (DashboardBoxChecksLogged < 5)
        {
            ++DashboardBoxChecksLogged;
            CARLA_VR_LOG(TEXT("Dashboard: %d obstacles listed, %d boxes in the world (%d visible)%s"), Obstacles.Num(),
                Boxes, Visible, *Example);
        }
        if (Visible == 0 && Now - DashboardFirstObstacleTime >= 3.0)
        {
            bDashboardServerBoxes = true;
            CARLA_VR_LOG(TEXT("Dashboard: the dashboard blueprint draws no boxes, the server draws them"));
        }
    }
    if (!bDashboardServerBoxes)
    {
        return;
    }

    for (const TPair<FString, FObstacle>& Item : Obstacles)
    {
        const FObstacle& Obstacle = Item.Value;
        UClass* Class = DashboardBoxes::BoxClass(Obstacle.Type);
        TWeakObjectPtr<AActor>& Box = DashboardServerBoxes.FindOrAdd(Item.Key);
        if (Box.IsValid() && Box->GetClass() != Class)
        {
            Box->Destroy();
            Box.Reset();
        }
        if (Class == nullptr)
        {
            continue;
        }
        // The client sends the position in cm (box center) and the heading as (0, 0, yaw)
        const FTransform Transform(FRotator(Obstacle.Rotation.Y, Obstacle.Rotation.Z, Obstacle.Rotation.X), Obstacle.Position);
        if (!Box.IsValid())
        {
            AActor* NewBox = World->SpawnActorDeferred<AActor>(Class, Transform, nullptr, nullptr,
                ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
            if (NewBox == nullptr)
            {
                continue;
            }
            DashboardBoxes::SetDanger(NewBox, Obstacle.isDangerous, true);
            NewBox->FinishSpawning(Transform);
            DashboardBoxes::SetDanger(NewBox, Obstacle.isDangerous, true);  // after its BeginPlay too
            Box = NewBox;
            continue;
        }
        DashboardBoxes::SetDanger(Box.Get(), Obstacle.isDangerous, false);
        if (!Box->GetActorTransform().Equals(Transform, 0.1))
        {
            Box->SetActorTransform(Transform);
        }
    }
}

void ACarlaGameModeBase::DashboardUpdateHUDFrustum()
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    if (!HUDFrustumCollection.IsValid())
    {
        if (HUDFrustumScanTime < 0.0)
        {
            return;  // not in this build
        }
        HUDFrustumCollection = LoadObject<UMaterialParameterCollection>(nullptr,
            TEXT("/CarlaVR_Vehicles/ARHUD_TestCase/HUD_Materials/MPC_VRFrustum.MPC_VRFrustum"));
        if (!HUDFrustumCollection.IsValid())
        {
            HUDFrustumScanTime = -1.0;
            CARLA_VR_LOG(TEXT("Dashboard: MPC_VRFrustum not found, the AR-HUD volume is left as it is"));
            return;
        }
    }

    // Once a second: the BP_HUDFrustum actors (a vivace's HUD volume). The ego's one is kept, the others stop
    // ticking (they would move the volume to their vehicle)
    const double Now = FPlatformTime::Seconds();
    if (Now - HUDFrustumScanTime >= 1.0)
    {
        HUDFrustumScanTime = Now;
        AActor* Hero = VRBoarding::FindHero(Episode);
        AActor* EgoFrustum = nullptr;
        int32 Stopped = 0;
        for (TActorIterator<AActor> It(World); It; ++It)
        {
            AActor* Frustum = *It;
            if (!IsValid(Frustum) || !Frustum->GetClass()->GetName().StartsWith(TEXT("BP_HUDFrustum")))
            {
                continue;
            }
            bool bEgos = false;
            for (AActor* Parent = Frustum; Parent != nullptr && Hero != nullptr; )
            {
                if (Parent == Hero)
                {
                    bEgos = true;
                    break;
                }
                AActor* Next = Parent->GetParentActor();
                Parent = Next != nullptr ? Next : Parent->GetAttachParentActor();
            }
            if (bEgos)
            {
                EgoFrustum = Frustum;
                Frustum->SetActorTickEnabled(true);
            }
            else if (Frustum->IsActorTickEnabled())
            {
                Frustum->SetActorTickEnabled(false);
                ++Stopped;
            }
        }
        if (EgoFrustum != HUDFrustumEgo.Get() || Stopped > 0 || !bHUDFrustumLogged)
        {
            CARLA_VR_LOG(TEXT("Dashboard: AR-HUD volume %s (%d traffic HUD volumes stopped)"),
                EgoFrustum != nullptr ? *FString::Printf(TEXT("of the ego's %s"), *EgoFrustum->GetName()) :
                TEXT("set to the whole world (the ego has none)"), Stopped);
            bHUDFrustumLogged = true;
        }
        HUDFrustumEgo = EgoFrustum;
    }
    if (HUDFrustumEgo.IsValid())
    {
        return;  // the ego's BP_HUDFrustum sets it
    }

    // The whole world: centered on the origin, world axes, bounds of 1000 km
    UMaterialParameterCollectionInstance* Instance = World->GetParameterCollectionInstance(HUDFrustumCollection.Get());
    if (Instance == nullptr)
    {
        return;
    }
    static const float WorldBounds = 1.0e8f;  // [cm]
    Instance->SetVectorParameterValue(FName(TEXT("Location")), FLinearColor(0.0f, 0.0f, 0.0f, 0.0f));
    Instance->SetVectorParameterValue(FName(TEXT("X")), FLinearColor(1.0f, 0.0f, 0.0f, 0.0f));
    Instance->SetVectorParameterValue(FName(TEXT("Y")), FLinearColor(0.0f, 1.0f, 0.0f, 0.0f));
    Instance->SetVectorParameterValue(FName(TEXT("Z")), FLinearColor(0.0f, 0.0f, 1.0f, 0.0f));
    Instance->SetVectorParameterValue(FName(TEXT("Bounds")), FLinearColor(WorldBounds, WorldBounds, WorldBounds, 0.0f));
}

namespace DashboardBoxes
{
  /// The copies of a box's lines: ComponentTags
  static const FName ThickLineTag(TEXT("CarlaThickLine"));

  /// How far out (> 0) / in (< 0) the copies of the lines are [cm]: the lines get ~4 cm wider
  static const float LineOffsets[] = { -2.0f, -1.0f, 1.0f, 2.0f };

  /// A copy's offset, kept in its ComponentTags
  static const FString OffsetTagPrefix(TEXT("CarlaThickOffset="));
  static FName OffsetTag(float Offset)
  {
    return FName(*FString::Printf(TEXT("%s%.2f"), *OffsetTagPrefix, Offset));
  }
  static float OffsetOf(const UActorComponent* Copy)
  {
    for (const FName& Tag : Copy->ComponentTags)
    {
      const FString Text = Tag.ToString();
      if (Text.StartsWith(OffsetTagPrefix))
      {
        return FCString::Atof(*Text.RightChop(OffsetTagPrefix.Len()));
      }
    }
    return 0.0f;
  }

  /// Scale (relative to the line mesh) of a copy offset by Offset cm, at the mesh's current world scale
  static FVector CopyScale(const UStaticMeshComponent* Original, float Offset)
  {
    const UStaticMesh* Mesh = Original->GetStaticMesh();
    const FVector Extent = Mesh != nullptr ? Mesh->GetBoundingBox().GetExtent() : FVector::ZeroVector;
    const FVector WorldScale = Original->GetComponentScale().GetAbs();
    FVector Scale = FVector::OneVector;
    for (int32 Axis = 0; Axis < 3; ++Axis)
    {
      if (Extent[Axis] > 0.1 && WorldScale[Axis] > 1.0e-4)
      {
        Scale[Axis] = FMath::Max(0.5, 1.0 + (Offset / WorldScale[Axis]) / Extent[Axis]);
      }
    }
    return Scale;
  }

  /// Places a copy around the center of the line mesh, scaled for its offset
  static void PlaceCopy(UStaticMeshComponent* Copy, const UStaticMeshComponent* Original, float Offset)
  {
    const UStaticMesh* Mesh = Original->GetStaticMesh();
    const FVector Center = Mesh != nullptr ? Mesh->GetBoundingBox().GetCenter() : FVector::ZeroVector;
    const FVector Scale = CopyScale(Original, Offset);
    const FVector Location = Center - Scale * Center;
    if (!Copy->GetRelativeScale3D().Equals(Scale, 1.0e-4) || !Copy->GetRelativeLocation().Equals(Location, 0.01))
    {
      Copy->SetRelativeLocationAndRotation(Location, FRotator::ZeroRotator);
      Copy->SetRelativeScale3D(Scale);
    }
  }
}

void ACarlaGameModeBase::DashboardOnBoxSpawned(AActor* Actor)
{
    // Thickened on the next tick: a deferred spawn (the dashboard blueprint's) has no components yet
    if (Actor != nullptr && DashboardBoxes::IsBox(Actor))
    {
        DashboardBoxesToThicken.Add(Actor);
    }
}

void ACarlaGameModeBase::DashboardUpdateThickBoxes()
{
    for (const TWeakObjectPtr<AActor>& Pending : DashboardBoxesToThicken)
    {
        AActor* Box = Pending.Get();
        if (!IsValid(Box) || Box->IsActorBeingDestroyed())
        {
            continue;
        }
        TArray<UStaticMeshComponent*> Lines;
        Box->GetComponents<UStaticMeshComponent>(Lines);
        int32 Copies = 0;
        for (UStaticMeshComponent* Original : Lines)
        {
            if (Original == nullptr || Original->ComponentHasTag(DashboardBoxes::ThickLineTag) ||
                Original->GetStaticMesh() == nullptr)
            {
                continue;
            }
            for (float Offset : DashboardBoxes::LineOffsets)
            {
                UStaticMeshComponent* Copy = NewObject<UStaticMeshComponent>(Box);
                Copy->ComponentTags.Add(DashboardBoxes::ThickLineTag);
                Copy->ComponentTags.Add(DashboardBoxes::OffsetTag(Offset));
                Copy->SetMobility(EComponentMobility::Movable);
                Copy->SetStaticMesh(Original->GetStaticMesh());
                Copy->SetCollisionEnabled(ECollisionEnabled::NoCollision);
                Copy->SetCastShadow(false);
                Copy->SetupAttachment(Original);
                Copy->RegisterComponent();
                Box->AddInstanceComponent(Copy);
                DashboardBoxes::PlaceCopy(Copy, Original, Offset);
                for (int32 Slot = 0; Slot < Original->GetNumMaterials(); ++Slot)
                {
                    Copy->SetMaterial(Slot, Original->GetMaterial(Slot));
                }
                Copy->SetVisibility(Original->IsVisible());
                ++Copies;
            }
        }
        if (Copies > 0)
        {
            DashboardThickBoxes.Add(Box);
        }
    }
    DashboardBoxesToThicken.Reset();

    // Every tick: the copies follow their line mesh (material set by the box blueprint for the danger color,
    // visibility, scale changes)
    for (int32 Index = DashboardThickBoxes.Num() - 1; Index >= 0; --Index)
    {
        AActor* Box = DashboardThickBoxes[Index].Get();
        if (!IsValid(Box) || Box->IsActorBeingDestroyed())
        {
            DashboardThickBoxes.RemoveAtSwap(Index);
            continue;
        }
        TArray<UStaticMeshComponent*> Lines;
        Box->GetComponents<UStaticMeshComponent>(Lines);
        for (UStaticMeshComponent* Copy : Lines)
        {
            if (Copy == nullptr || !Copy->ComponentHasTag(DashboardBoxes::ThickLineTag))
            {
                continue;
            }
            const UStaticMeshComponent* Original = Cast<UStaticMeshComponent>(Copy->GetAttachParent());
            if (Original == nullptr)
            {
                continue;
            }
            DashboardBoxes::PlaceCopy(Copy, Original, DashboardBoxes::OffsetOf(Copy));
            for (int32 Slot = 0; Slot < Original->GetNumMaterials(); ++Slot)
            {
                UMaterialInterface* Material = Original->GetMaterial(Slot);
                if (Copy->GetMaterial(Slot) != Material)
                {
                    Copy->SetMaterial(Slot, Material);
                }
            }
            if (Copy->IsVisible() != Original->IsVisible())
            {
                Copy->SetVisibility(Original->IsVisible());
            }
        }
    }
}

namespace DashboardArrowsUtil
{
  /// The VR vehicles' own motion prediction arrow (BP_VC_Short): a chain of spline meshes of this mesh. Its
  /// look (mesh, material, scale, section length, position) is copied from the ego's arrow when it has one,
  /// this mesh at its own size until then
  static const TCHAR* Mesh = TEXT("/CarlaVR_Vehicles/ARHUD_TestCase/VirtualCarpet/VirtalCarpet_motionPrediction_Yellow.VirtalCarpet_motionPrediction_Yellow");

  /// Front of the vehicle at the bottom of its bounding box, vehicle coordinates [cm]
  static FVector FrontBottom(const ACarlaWheeledVehicle* Vehicle)
  {
    const FVector Extent = Vehicle->GetVehicleBoundingBoxExtent();
    return Vehicle->GetVehicleBoundingBoxTransform().GetLocation() + FVector(Extent.X, 0.0, -Extent.Z);
  }

  /// Point and direction at S [cm] along a circle arc of this curvature starting along +X (straight: 0)
  static void Arc(double Curvature, double S, FVector& Point, FVector& Direction)
  {
    if (FMath::Abs(Curvature) < 1.0e-6)
    {
      Point = FVector(S, 0.0, 0.0);
      Direction = FVector(1.0, 0.0, 0.0);
      return;
    }
    Point = FVector(FMath::Sin(Curvature * S) / Curvature, (1.0 - FMath::Cos(Curvature * S)) / Curvature, 0.0);
    Direction = FVector(FMath::Cos(Curvature * S), FMath::Sin(Curvature * S), 0.0);
  }
}

bool ACarlaGameModeBase::DashboardCaptureArrowLook(ACarlaWheeledVehicle* Hero)
{
    // The ego's own arrow: the spline meshes of a BP_VC_Short attached to it, the one nearest to its front
    // giving the start, the section length and the scale
    TArray<AActor*> Attached;
    Hero->GetAttachedActors(Attached, true, true);
    const FVector HeroFront = Hero->GetActorTransform().TransformPosition(DashboardArrowsUtil::FrontBottom(Hero));
    USplineMeshComponent* First = nullptr;
    double FirstDistance = TNumericLimits<double>::Max();
    for (AActor* Actor : Attached)
    {
        if (!IsValid(Actor) || !Actor->GetClass()->GetName().StartsWith(TEXT("BP_VC_Short")))
        {
            continue;
        }
        TArray<USplineMeshComponent*> Segments;
        Actor->GetComponents<USplineMeshComponent>(Segments);
        for (USplineMeshComponent* Segment : Segments)
        {
            if (!IsValid(Segment) || Segment->GetStaticMesh() == nullptr || !Segment->IsVisible())
            {
                continue;
            }
            const FVector Start = Segment->GetComponentTransform().TransformPosition(Segment->GetStartPosition());
            const double Distance = FVector::Dist(Start, HeroFront);
            if (Distance < FirstDistance)
            {
                FirstDistance = Distance;
                First = Segment;
            }
        }
    }
    if (First == nullptr)
    {
        return false;
    }
    const FTransform& SegmentTransform = First->GetComponentTransform();
    const FVector Start = SegmentTransform.TransformPosition(First->GetStartPosition());
    const FVector End = SegmentTransform.TransformPosition(First->GetEndPosition());
    const double Section = FVector::Dist(Start, End);
    if (Section < 20.0)
    {
        return false;  // the ego's arrow folded (the ego stopped): its look is taken when it drives
    }
    FVector Offset = Hero->GetActorTransform().InverseTransformPosition(Start) - DashboardArrowsUtil::FrontBottom(Hero);
    if (Offset.Size() > 800.0)
    {
        Offset = FVector(0.0, 0.0, Offset.Z);  // not at the front (some other placement): at the front
    }
    DashboardArrowLook.Mesh = First->GetStaticMesh();
    DashboardArrowLook.Material = First->GetMaterial(0);
    DashboardArrowLook.Axis = First->ForwardAxis;
    DashboardArrowLook.Scale = First->GetStartScale() *
        FVector2D(SegmentTransform.GetScale3D().Y, SegmentTransform.GetScale3D().Z);
    DashboardArrowLook.Section = Section;
    DashboardArrowLook.Offset = Offset;
    DashboardArrowLook.bFromEgo = true;
    CARLA_VR_LOG(TEXT("Dashboard: motion prediction arrows look as the ego's (%s, %s, section %.0f cm, scale %s, %s from the front)"),
        *GetNameSafe(DashboardArrowLook.Mesh.Get()), *GetNameSafe(DashboardArrowLook.Material.Get()), Section,
        *DashboardArrowLook.Scale.ToString(), *Offset.ToString());
    return true;
}

void ACarlaGameModeBase::DashboardUpdateArrows()
{
    UWorld* World = GetWorld();
    ACarlaWheeledVehicle* Hero = nullptr;
    if (World != nullptr && !bDashboardHideDrivingAids && IsValid(DashboardComponent))
    {
        Hero = Cast<ACarlaWheeledVehicle>(VRBoarding::FindHero(Episode));
    }
    if (Hero == nullptr)
    {
        if (DashboardArrows.Num() > 0)
        {
            DashboardDestroyArrows();
        }
        return;
    }

    // Twice a second: the look (the ego's arrow's, once found: the arrows made until then are remade), and
    // which vehicles get one (all but the ego, within DashboardArrowRange of it)
    const double Now = FPlatformTime::Seconds();
    if (Now - DashboardArrowScanTime >= 0.5)
    {
        DashboardArrowScanTime = Now;
        if (DashboardArrowLookHero.Get() != Hero)
        {
            DashboardArrowLookHero = Hero;
            DashboardArrowLook = FDashboardArrowLook();
        }
        if (!DashboardArrowLook.bFromEgo && DashboardCaptureArrowLook(Hero))
        {
            DashboardDestroyArrows();
        }
        if (!DashboardArrowLook.Mesh.IsValid())
        {
            UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, DashboardArrowsUtil::Mesh);
            if (Mesh == nullptr)
            {
                if (!bDashboardArrowsLogged)
                {
                    bDashboardArrowsLogged = true;
                    CARLA_VR_LOG(TEXT("Dashboard: motion prediction mesh %s not found"), DashboardArrowsUtil::Mesh);
                }
                return;
            }
            DashboardArrowLook.Mesh = Mesh;
            DashboardArrowLook.Material = Mesh->GetMaterial(0);
            DashboardArrowLook.Section = FMath::Max(50.0, Mesh->GetBoundingBox().GetSize().X);
        }

        const FVector HeroLocation = Hero->GetActorLocation();
        int32 Added = 0;
        for (TActorIterator<ACarlaWheeledVehicle> It(World); It; ++It)
        {
            ACarlaWheeledVehicle* Vehicle = *It;
            if (!IsValid(Vehicle) || Vehicle == Hero || Vehicle->IsActorBeingDestroyed() || Vehicle->HasAnyFlags(RF_WasLoaded))
            {
                continue;
            }
            const bool bInRange = FVector::Dist(Vehicle->GetActorLocation(), HeroLocation) <= DashboardArrowRange;
            FDashboardArrow* Existing = DashboardArrows.Find(Vehicle);
            if (!bInRange)
            {
                if (Existing != nullptr)
                {
                    if (Existing->Actor.IsValid())
                    {
                        Existing->Actor->Destroy();
                    }
                    DashboardArrows.Remove(Vehicle);
                }
                continue;
            }
            if (Existing == nullptr)
            {
                // An actor following the vehicle (attached to it), its segments in the vehicle's coordinates
                FActorSpawnParameters SpawnParams;
                SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                AActor* Actor = World->SpawnActor<AActor>(AActor::StaticClass(), Vehicle->GetActorTransform(), SpawnParams);
                if (Actor == nullptr)
                {
                    continue;
                }
                USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("MotionPredictionRoot"));
                Root->SetMobility(EComponentMobility::Movable);
                Actor->SetRootComponent(Root);
                Root->RegisterComponent();
                Actor->SetActorEnableCollision(false);
                Actor->AttachToActor(Vehicle, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
                FDashboardArrow Arrow;
                Arrow.Actor = Actor;
                DashboardArrows.Add(Vehicle, Arrow);
                ++Added;
            }
        }
        if (Added > 0 && !bDashboardArrowsLogged)
        {
            bDashboardArrowsLogged = true;
            CARLA_VR_LOG(TEXT("Dashboard: motion prediction drawn on %d vehicles (%s, %s)"), DashboardArrows.Num(),
                *GetNameSafe(DashboardArrowLook.Mesh.Get()), DashboardArrowLook.bFromEgo ? TEXT("as the ego's") : TEXT("default look"));
        }
    }
    if (!DashboardArrowLook.Mesh.IsValid())
    {
        return;
    }

    // Every tick: each arrow along its vehicle's predicted path (the next DashboardArrowHorizon seconds at its
    // current speed and steering, bicycle model: a circle arc of curvature tan(wheel angle) / wheelbase), in
    // sections of the ego's arrow's length; those of the vehicles gone are removed
    for (auto It = DashboardArrows.CreateIterator(); It; ++It)
    {
        ACarlaWheeledVehicle* Vehicle = Cast<ACarlaWheeledVehicle>(It.Key().Get());
        FDashboardArrow& Arrow = It.Value();
        if (!IsValid(Vehicle) || Vehicle->IsActorBeingDestroyed() || !Arrow.Actor.IsValid())
        {
            if (Arrow.Actor.IsValid())
            {
                Arrow.Actor->Destroy();
            }
            It.RemoveCurrent();
            continue;
        }
        const double Length = FMath::Min(static_cast<double>(Vehicle->GetVehicleForwardSpeed()) * DashboardArrowHorizon,
            static_cast<double>(DashboardArrowMaxLength));
        const int32 Count = Length < 150.0 ? 0 : FMath::Max(1, FMath::RoundToInt(Length / DashboardArrowLook.Section));
        while (Arrow.Segments.Num() < Count)
        {
            USplineMeshComponent* Segment = NewObject<USplineMeshComponent>(Arrow.Actor.Get());
            Segment->SetMobility(EComponentMobility::Movable);
            Segment->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            Segment->SetCastShadow(false);
            Segment->SetStaticMesh(DashboardArrowLook.Mesh.Get());
            if (DashboardArrowLook.Material.IsValid())
            {
                Segment->SetMaterial(0, DashboardArrowLook.Material.Get());
            }
            Segment->SetForwardAxis(DashboardArrowLook.Axis, false);
            Segment->SetStartScale(DashboardArrowLook.Scale, false);
            Segment->SetEndScale(DashboardArrowLook.Scale, false);
            Segment->SetupAttachment(Arrow.Actor->GetRootComponent());
            Segment->RegisterComponent();
            Arrow.Segments.Add(Segment);
        }
        const FVector Start = DashboardArrowsUtil::FrontBottom(Vehicle) + DashboardArrowLook.Offset;
        const FVector Extent = Vehicle->GetVehicleBoundingBoxExtent();
        const double Wheelbase = FMath::Max(150.0, 1.2 * Extent.X);
        const double WheelAngle = FMath::DegreesToRadians(FMath::Clamp(Vehicle->GetVehicleControl().Steer, -1.0f, 1.0f) *
            FMath::Clamp(Vehicle->GetMaximumSteerAngle(), 0.0f, 70.0f));
        const double Curvature = FMath::Tan(WheelAngle) / Wheelbase;  // > 0: to the right (+Y)
        const double Section = Count > 0 ? Length / Count : 0.0;
        for (int32 i = 0; i < Arrow.Segments.Num(); ++i)
        {
            USplineMeshComponent* Segment = Arrow.Segments[i].Get();
            if (Segment == nullptr)
            {
                continue;
            }
            if (i >= Count)
            {
                Segment->SetVisibility(false);  // (almost) stopped: no arrow
                continue;
            }
            FVector P0, D0, P1, D1;
            DashboardArrowsUtil::Arc(Curvature, Section * i, P0, D0);
            DashboardArrowsUtil::Arc(Curvature, Section * (i + 1), P1, D1);
            Segment->SetStartAndEnd(Start + P0, D0 * Section, Start + P1, D1 * Section, true);
            Segment->SetVisibility(true);
        }
    }
}

namespace SeatAdjustment
{
  /// The driver's position relatively to the vehicle, for the whole CARLA session (kept across the levels
  /// loaded, the game mode is not)
  static FTransform Relative;
  static bool bRecorded = false;
}

void ACarlaGameModeBase::KeepPlayerSeatAdjustment()
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    const double Now = FPlatformTime::Seconds();

    // The VR pawn (the player's controller may possess the vehicle itself): the VR plugin's pawn that isn't
    // a vehicle, on board one
    APawn* Pawn = SeatPawn.Get();
    if ((!IsValid(Pawn) || VRBoarding::VehicleOfPlayer(Pawn) == nullptr) && Now - SeatScanTime >= 1.0)
    {
        SeatScanTime = Now;
        Pawn = nullptr;
        for (TActorIterator<APawn> It(World); It; ++It)
        {
            if (IsValid(*It) && !It->IsA<ACarlaWheeledVehicle>() &&
                It->GetClass()->GetPathName().StartsWith(TEXT("/CarlaVR_Vehicles/")) &&
                VRBoarding::VehicleOfPlayer(*It) != nullptr)
            {
                Pawn = *It;
                break;
            }
        }
        SeatPawn = Pawn;
    }
    ACarlaWheeledVehicle* Vehicle = IsValid(Pawn) ? VRBoarding::VehicleOfPlayer(Pawn) : nullptr;
    if (Vehicle == nullptr || Vehicle->IsActorBeingDestroyed())
    {
        SeatVehicle.Reset();  // between two vehicles (cleanup of the previous runs): nothing recorded
        return;
    }
    if (Vehicle != SeatVehicle.Get())
    {
        // Taken on board a new vehicle: its boarding settles first
        SeatVehicle = Vehicle;
        SeatBoardTime = Now;
        bSeatApplied = false;
        return;
    }
    if (!bSeatApplied)
    {
        if (Now - SeatBoardTime < SeatSettleDelay)
        {
            return;
        }
        bSeatApplied = true;
        if (SeatAdjustment::bRecorded)
        {
            FTransform Transform = SeatAdjustment::Relative * Vehicle->GetActorTransform();
            Transform.SetScale3D(Pawn->GetActorScale3D());
            Pawn->SetActorTransform(Transform, false, nullptr, ETeleportType::TeleportPhysics);
            CARLA_VR_LOG(TEXT("VR: driver's position applied in %s (%s from the vehicle, yaw %.1f)"), *Vehicle->GetName(),
                *SeatAdjustment::Relative.GetLocation().ToString(), SeatAdjustment::Relative.Rotator().Yaw);
        }
        return;
    }
    // On board, settled: the position (as the driver adjusts it) is recorded
    const FTransform Relative = Pawn->GetActorTransform().GetRelativeTransform(Vehicle->GetActorTransform());
    SeatAdjustment::Relative = Relative;
    SeatAdjustment::bRecorded = true;
}

void ACarlaGameModeBase::DashboardDestroyArrows()
{
    for (const TPair<TWeakObjectPtr<AActor>, FDashboardArrow>& Item : DashboardArrows)
    {
        if (Item.Value.Actor.IsValid())
        {
            Item.Value.Actor->Destroy();
        }
    }
    DashboardArrows.Empty();
}

void ACarlaGameModeBase::DashboardDestroyServerBoxes()
{
    for (const TPair<FString, TWeakObjectPtr<AActor>>& Item : DashboardServerBoxes)
    {
        if (Item.Value.IsValid())
        {
            Item.Value->Destroy();
        }
    }
    DashboardServerBoxes.Empty();
    bDashboardServerBoxes = false;
    DashboardFirstObstacleTime = 0.0;
    DashboardBoxCheckTime = 0.0;
    DashboardBoxChecksLogged = 0;
}

void ACarlaGameModeBase::DashboardTriggerPathSetEvent()
{
    DashboardEnsureComponent();  // and the dashboard told about the ego (DashboardNotifyVehicleSpawn)
    if(DashboardComponent)
    {
        DashboardComponent->TriggerPathSetEvent();
    }
}

void ACarlaGameModeBase::DashboardNotifyVehicleSpawn()
{
    // The dashboard blueprint (BP_DashboardManager) only creates the AR-HUD driving aids drawing the
    // global path (virtual carpet...) and the bounding boxes when told a VR vehicle spawned, through its
    // 'E_VR_Vehicle_Spawn' event (it keeps that vehicle). Only BP_XAI_vivace does it, not the Mercedes VR
    // ego (with traffic VR vehicles it used to be one of theirs, now filtered by EgoOnlyThunk): it is done
    // here for the ego ('hero') vehicle spawned by the client, once per vehicle
    if (DashboardComponent == nullptr || Episode == nullptr)
    {
        return;
    }
    if (DashboardNotifiedVehicle.IsValid() && !DashboardNotifiedVehicle->IsActorBeingDestroyed())
    {
        return;  // done already (by the ego's blueprint, or here)
    }
    AActor* Hero = VRBoarding::FindHero(Episode);
    if (Hero == nullptr)
    {
        return;
    }
    DashboardNotifiedVehicle = Hero;

    // The dashboard blueprint also looks for its VR vehicle by itself when it begins play (GetActorOfClass):
    // the one the player sits in at the start, destroyed since by the cleanup of previous runs (or a
    // previous route's ego). It doesn't draw the boxes without it: its vehicle variables are set to the ego
    // (the blueprint's own variables of a vehicle / pawn type, not AActor's Owner / Instigator)
    for (TFieldIterator<FObjectPropertyBase> It(DashboardComponent->GetClass(), EFieldIteratorFlags::ExcludeSuper); It; ++It)
    {
        UClass* PropertyClass = It->PropertyClass;
        if (PropertyClass == nullptr || !PropertyClass->IsChildOf(APawn::StaticClass()))
        {
            continue;
        }
        UObject* Previous = It->GetObjectPropertyValue_InContainer(DashboardComponent);
        if (!Hero->IsA(PropertyClass))
        {
            CARLA_VR_LOG(TEXT("Dashboard: %s's %s (%s) left to %s: the ego %s is no %s"), *DashboardComponent->GetName(),
                *It->GetName(), *PropertyClass->GetName(), *GetNameSafe(Previous), *Hero->GetClass()->GetName(),
                *PropertyClass->GetName());
            continue;
        }
        It->SetObjectPropertyValue_InContainer(DashboardComponent, Hero);
        CARLA_VR_LOG(TEXT("Dashboard: %s's %s set to the ego %s (was %s%s)"), *DashboardComponent->GetName(),
            *It->GetName(), *Hero->GetName(), *GetNameSafe(Previous),
            IsValid(Previous) ? TEXT("") : TEXT(", destroyed"));
    }

    UFunction* Function = DashboardComponent->FindFunction(FName(TEXT("E_VR_Vehicle_Spawn")));
    if (Function == nullptr)
    {
        CARLA_VR_LOG(TEXT("Dashboard: %s has no E_VR_Vehicle_Spawn event"), *DashboardComponent->GetName());
        return;
    }
    uint8* Params = static_cast<uint8*>(FMemory_Alloca(FMath::Max(1, (int32)Function->ParmsSize)));
    FMemory::Memzero(Params, FMath::Max(1, (int32)Function->ParmsSize));
    for (TFieldIterator<FProperty> ParamIt(Function); ParamIt && ParamIt->HasAnyPropertyFlags(CPF_Parm); ++ParamIt)
    {
        ParamIt->InitializeValue_InContainer(Params);
        // An actor / vehicle parameter: the ego
        FObjectPropertyBase* ObjectParam = CastField<FObjectPropertyBase>(*ParamIt);
        if (ObjectParam != nullptr && Hero->IsA(ObjectParam->PropertyClass))
        {
            ObjectParam->SetObjectPropertyValue_InContainer(Params, Hero);
        }
        else if (ObjectParam != nullptr)
        {
            CARLA_VR_LOG(TEXT("Dashboard: the ego %s is no %s (E_VR_Vehicle_Spawn's %s)"), *Hero->GetClass()->GetName(),
                *GetNameSafe(ObjectParam->PropertyClass), *ParamIt->GetName());
        }
    }
    CARLA_VR_LOG(TEXT("Dashboard: E_VR_Vehicle_Spawn for the ego %s"), *Hero->GetName());
    DashboardComponent->ProcessEvent(Function, Params);
    for (TFieldIterator<FProperty> ParamIt(Function); ParamIt && ParamIt->HasAnyPropertyFlags(CPF_Parm); ++ParamIt)
    {
        ParamIt->DestroyValue_InContainer(Params);
    }
}