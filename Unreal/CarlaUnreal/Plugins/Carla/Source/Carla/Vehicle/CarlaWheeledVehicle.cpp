// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include "Carla.h"
#include "Carla/Game/CarlaHUD.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Util/VRLog.h"
#include "Carla/Trigger/FrictionTrigger.h"
#include "Carla/Util/ActorAttacher.h"
#include "Carla/Util/EmptyActor.h"
#include "Carla/Util/BoundingBoxCalculator.h"
#include "Carla/Vegetation/VegetationManager.h"

#include <util/ue-header-guard-begin.h>
#include "Components/BoxComponent.h"
#include "Engine/CollisionProfile.h"
#include "MovementComponents/DefaultMovementComponent.h"
#include "Rendering/SkeletalMeshRenderData.h"
#include "VehicleAnimationInstance.h"
#include "UObject/UObjectGlobals.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "PhysicsEngine/PhysicsObjectExternalInterface.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include <util/ue-header-guard-end.h>

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

void ACarlaWheeledVehicle::PrintROS2Message(const char* Message)
{
  FString ROSMessage = Message;
  UE_LOG(LogCarla, Warning, TEXT("ROS2 Message received: %s"), *ROSMessage);
}

ACarlaWheeledVehicle::ACarlaWheeledVehicle(const FObjectInitializer& ObjectInitializer) :
  Super(ObjectInitializer)
{
  VehicleBounds = CreateDefaultSubobject<UBoxComponent>(TEXT("VehicleBounds"));
  VehicleBounds->SetupAttachment(RootComponent);
  VehicleBounds->SetHiddenInGame(true);
  VehicleBounds->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);


  VelocityControl = CreateDefaultSubobject<UVehicleVelocityControl>(TEXT("VelocityControl"));
  VelocityControl->Deactivate();

  GetChaosWheeledVehicleMovementComponent()->bReverseAsBrake = false;
  BaseMovementComponent = CreateDefaultSubobject<UBaseCarlaMovementComponent>(TEXT("BaseMovementComponent"));

}

ACarlaWheeledVehicle::~ACarlaWheeledVehicle() {}

namespace VehicleVRBoarding
{
  static bool IsVRVehicle(const AActor* Actor)
  {
    for (const UClass* Class = Actor ? Actor->GetClass() : nullptr; Class != nullptr; Class = Class->GetSuperClass())
    {
      if (Class->GetName().StartsWith(TEXT("BP_Master_VehiclesVR")))
      {
        return true;
      }
    }
    return false;
  }
}

void ACarlaWheeledVehicle::BeginPlay()
{
  Super::BeginPlay();

  // The VR vehicles' text screen (BP_Master_VehiclesVR's WB_text widget, on the right of the wheel) is
  // lowered so it hides less of the road
  if (VehicleVRBoarding::IsVRVehicle(this))
  {
    TArray<USceneComponent*> SceneComponents;
    GetComponents(SceneComponents);
    for (USceneComponent* Component : SceneComponents)
    {
      if (Component != nullptr && Component->GetName().StartsWith(TEXT("WB_text")))
      {
        Component->AddWorldOffset(-GetActorUpVector() * VRTextScreenDrop);
        CARLA_VR_LOG(TEXT("%s: %s lowered by %.0f cm"), *GetName(), *Component->GetName(), VRTextScreenDrop);
      }
    }
  }

  UDefaultMovementComponent::CreateDefaultMovementComponent(this);

  // Get constraint components and their initial transforms
  FTransform ActorInverseTransform = GetActorTransform().Inverse();
  ConstraintsComponents.Empty();
  DoorComponentsTransform.Empty();
  ConstraintDoor.Empty();
  for (FName& ComponentName : ConstraintComponentNames)
  {
    UPhysicsConstraintComponent* ConstraintComponent =
      Cast<UPhysicsConstraintComponent>(GetDefaultSubobjectByName(ComponentName));
    if (ConstraintComponent)
    {
      UPrimitiveComponent* DoorComponent = Cast<UPrimitiveComponent>(
        GetDefaultSubobjectByName(ConstraintComponent->ComponentName1.ComponentName));
      if (DoorComponent)
      {
        UE_LOG(LogCarla, Log, TEXT("Door name: %s"), *(DoorComponent->GetName()));
        FTransform ComponentWorldTransform = DoorComponent->GetComponentTransform();
        FTransform RelativeTransform = ComponentWorldTransform * ActorInverseTransform;
        DoorComponentsTransform.Add(DoorComponent, RelativeTransform);
        ConstraintDoor.Add(ConstraintComponent, DoorComponent);
        ConstraintsComponents.Add(ConstraintComponent);
        ConstraintComponent->TermComponentConstraint();
      }
      else
      {
        UE_LOG(LogCarla, Error, TEXT("Missing component for constraint: %s"), *(ConstraintComponent->GetName()));
      }
    }
  }
  ResetConstraints();

  // get collision disable constraints (used to prevent doors from colliding with each other)
  CollisionDisableConstraints.Empty();
  TArray<UPhysicsConstraintComponent*> Constraints;
  GetComponents(Constraints);
  for (UPhysicsConstraintComponent* Constraint : Constraints)
  {
    if (!ConstraintsComponents.Contains(Constraint))
    {
      UPrimitiveComponent* CollisionDisabledComponent1 = Cast<UPrimitiveComponent>(
        GetDefaultSubobjectByName(Constraint->ComponentName1.ComponentName));
      UPrimitiveComponent* CollisionDisabledComponent2 = Cast<UPrimitiveComponent>(
        GetDefaultSubobjectByName(Constraint->ComponentName2.ComponentName));
      if (CollisionDisabledComponent1)
      {
        CollisionDisableConstraints.Add(CollisionDisabledComponent1, Constraint);
      }
      if (CollisionDisabledComponent2)
      {
        CollisionDisableConstraints.Add(CollisionDisabledComponent2, Constraint);
      }
    }
  }

  float FrictionScale = 3.5f;

  UChaosWheeledVehicleMovementComponent* MovementComponent = GetChaosWheeledVehicleMovementComponent();

  if (MovementComponent)
  {
    check(MovementComponent != nullptr);

    // Setup Tire Configs with default value. This is needed to avoid getting
    // friction values of previously created TireConfigs for the same vehicle
    // blueprint.
    TArray<float> OriginalFrictions;
    OriginalFrictions.Init(FrictionScale, MovementComponent->Wheels.Num());
    SetWheelsFrictionScale(OriginalFrictions);

    // Check if it overlaps with a Friction trigger, if so, update the friction
    // scale.
    TArray<AActor*> OverlapActors;
    GetOverlappingActors(OverlapActors, AFrictionTrigger::StaticClass());
    for (const auto& Actor : OverlapActors)
    {
      AFrictionTrigger* FrictionTrigger = Cast<AFrictionTrigger>(Actor);
      if (FrictionTrigger)
      {
        FrictionScale = FrictionTrigger->Friction;
      }
    }

    // Set the friction scale to Wheel CDO and update wheel setups
    TArray<FChaosWheelSetup> NewWheelSetups = MovementComponent->WheelSetups;
    for (const FChaosWheelSetup& WheelSetup : NewWheelSetups)
    {
      UChaosVehicleWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();
      check(Wheel != nullptr);
    }

    MovementComponent->WheelSetups = NewWheelSetups;

    LastPhysicsControl = GetVehiclePhysicsControl();

    // Update physics in the Ackermann Controller
    AckermannController.UpdateVehiclePhysics(this);
  }
  AddReferenceToManager();
}

void ACarlaWheeledVehicle::TickActor(float DeltaTime, enum ELevelTick TickType, FActorTickFunction& ThisTickFunction){
  Super::TickActor(DeltaTime, TickType, ThisTickFunction);

  FPoseSnapshot pose;
  GetMesh()->SnapshotPose(pose);
  for(FTransform &transform : pose.LocalTransforms)
  {
    transform *= GetMesh()->GetComponentTransform();
  }

  WorldTransformedPose = pose;

}

bool ACarlaWheeledVehicle::IsInVehicleRange(const FVector& Location) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::IsInVehicleRange);

  return FoliageBoundingBox.IsInside(Location);
}

void ACarlaWheeledVehicle::UpdateDetectionBox()
{
  const FTransform GlobalTransform = GetActorTransform();
  const FVector Vec{ DetectionSize, DetectionSize, DetectionSize };
  FBox Box = FBox(-Vec, Vec);
  const FTransform NonScaledTransform(GlobalTransform.GetRotation(), GlobalTransform.GetLocation(), { 1.0f, 1.0f, 1.0f });
  FoliageBoundingBox = Box.TransformBy(NonScaledTransform);
}

const TArray<int32> ACarlaWheeledVehicle::GetFoliageInstancesCloseToVehicle(const UInstancedStaticMeshComponent* Component) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::GetFoliageInstancesCloseToVehicle);
  return Component->GetInstancesOverlappingBox(FoliageBoundingBox);
}

FBox ACarlaWheeledVehicle::GetDetectionBox() const
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ACarlaWheeledVehicle::GetDetectionBox);
  return FoliageBoundingBox;
}

float ACarlaWheeledVehicle::GetDetectionSize() const
{
  return DetectionSize;
}

void ACarlaWheeledVehicle::DrawFoliageBoundingBox() const
{
  const FVector& Center = FoliageBoundingBox.GetCenter();
  const FVector& Extent = FoliageBoundingBox.GetExtent();
  const FQuat& Rotation = GetActorQuat();
  DrawDebugBox(GetWorld(), Center, Extent, Rotation, FColor::Magenta, false, 0.0f, 0, 5.0f);
}

FBoxSphereBounds ACarlaWheeledVehicle::GetBoxSphereBounds() const
{
  ALargeMapManager* LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld());
  if (LargeMap)
  {
    FTransform GlobalTransform = LargeMap->LocalToGlobalTransform(GetActorTransform());
    return VehicleBounds->CalcBounds(GlobalTransform);
  }
  return VehicleBounds->CalcBounds(GetActorTransform());
}

void ACarlaWheeledVehicle::AdjustVehicleBounds()
{
  FBoundingBox BoundingBox = UBoundingBoxCalculator::GetVehicleBoundingBox(this);

  const FTransform& CompToWorldTransform = RootComponent->GetComponentTransform();
  const FRotator Rotation = CompToWorldTransform.GetRotation().Rotator();
  const FVector Translation = CompToWorldTransform.GetLocation();
  const FVector Scale = CompToWorldTransform.GetScale3D();

  // Invert BB origin to local space
  BoundingBox.Origin -= Translation;
  BoundingBox.Origin = Rotation.UnrotateVector(BoundingBox.Origin);
  BoundingBox.Origin /= Scale;

  // Prepare Box Collisions
  FTransform Transform;
  Transform.SetTranslation(BoundingBox.Origin);
  VehicleBounds->SetRelativeTransform(Transform);
  VehicleBounds->SetBoxExtent(BoundingBox.Extent);
}

// =============================================================================
// -- Get functions ------------------------------------------------------------
// =============================================================================

float ACarlaWheeledVehicle::GetVehicleForwardSpeed() const
{
  return BaseMovementComponent->GetVehicleForwardSpeed();
}

FVector ACarlaWheeledVehicle::GetVehicleOrientation() const
{
  return GetVehicleTransform().GetRotation().GetForwardVector();
}

int32 ACarlaWheeledVehicle::GetVehicleCurrentGear() const
{
  return BaseMovementComponent->GetVehicleCurrentGear();
}

FTransform ACarlaWheeledVehicle::GetVehicleBoundingBoxTransform() const
{
  return VehicleBounds->GetRelativeTransform();
}

FVector ACarlaWheeledVehicle::GetVehicleBoundingBoxExtent() const
{
  return VehicleBounds->GetScaledBoxExtent();
}

float ACarlaWheeledVehicle::GetMaximumSteerAngle() const
{
  UChaosWheeledVehicleMovementComponent* MovementComponent = GetChaosWheeledVehicleMovementComponent();
  const auto& Wheels = MovementComponent->WheelSetups;
  check(Wheels.Num() > 0);
  const UChaosVehicleWheel* FrontWheel =
    Cast<UChaosVehicleWheel>(Wheels[0].WheelClass->GetDefaultObject());
  check(FrontWheel != nullptr);
  return FrontWheel->MaxSteerAngle;
}

// =============================================================================
// -- Set functions ------------------------------------------------------------
// =============================================================================

void ACarlaWheeledVehicle::FlushVehicleControl()
{
  UpdateDriverTakeover();
  if (bDriverTakeover)
  {
    // The driver drives: the movement component keeps the inputs the vehicle's blueprint set from the
    // wheel and pedals, the client / autopilot control is dropped
    UChaosWheeledVehicleMovementComponent* MovementComponent = GetChaosWheeledVehicleMovementComponent();
    if (MovementComponent != nullptr)
    {
      LastAppliedControl.Gear = MovementComponent->GetTargetGear();
      LastAppliedControl.bReverse = bClutchReverse;
      LastAppliedControl.Throttle = MovementComponent->GetThrottleInput();
      LastAppliedControl.Steer = MovementComponent->GetSteeringInput();
      LastAppliedControl.Brake = MovementComponent->GetBrakeInput();
      LastAppliedControl.bHandBrake = MovementComponent->GetHandbrakeInput();
    }
    InputControl.Priority = EVehicleInputPriority::INVALID;
    return;
  }

  if (bAckermannControlActive) {
    AckermannController.UpdateVehicleState(this);
    AckermannController.RunLoop(InputControl.Control);
  }
  BaseMovementComponent->ProcessControl(InputControl.Control);
  InputControl.Control.bReverse = InputControl.Control.Gear < 0;
  LastAppliedControl = InputControl.Control;
  InputControl.Priority = EVehicleInputPriority::INVALID;
}

namespace DriverTakeover
{
  /// The first takeover flag of the vehicle's blueprint that is on (BP_Master_VehiclesVR), NAME_None if none
  static FName BlueprintFlag(const AActor* Vehicle)
  {
    static const FName Flags[] = {
      FName(TEXT("overrideAutopilot")), FName(TEXT("Steering_override")),
      FName(TEXT("Throttle_override")), FName(TEXT("Brake_override")) };
    for (const FName& Flag : Flags)
    {
      const FBoolProperty* Property = FindFProperty<FBoolProperty>(Vehicle->GetClass(), Flag);
      if (Property != nullptr && Property->GetPropertyValue_InContainer(Vehicle))
      {
        return Flag;
      }
    }
    return NAME_None;
  }

  /// Turns off the takeover flags of the vehicle's blueprint
  static void ClearBlueprintFlags(AActor* Vehicle)
  {
    static const FName Flags[] = {
      FName(TEXT("overrideAutopilot")), FName(TEXT("Steering_override")),
      FName(TEXT("Throttle_override")), FName(TEXT("Brake_override")) };
    for (const FName& Flag : Flags)
    {
      if (FBoolProperty* Property = FindFProperty<FBoolProperty>(Vehicle->GetClass(), Flag))
      {
        Property->SetPropertyValue_InContainer(Vehicle, false);
      }
    }
  }
}

void ACarlaWheeledVehicle::UpdateDriverTakeover()
{
  UChaosWheeledVehicleMovementComponent* MovementComponent = GetChaosWheeledVehicleMovementComponent();
  if (MovementComponent == nullptr)
  {
    return;
  }

  // The driver's inputs this frame (logged by the gaze recorder)
  DriverThrottle = MovementComponent->GetThrottleInput();
  DriverBrake = MovementComponent->GetBrakeInput();
  DriverSteer = MovementComponent->GetSteeringInput();

  // The clutch: reverse gear while it is held and the driver drives
  UpdateClutchReverse();

  // P gives the control back to the client / autopilot
  if (bDriverTakeover)
  {
    APlayerController* PlayerController = GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr;
    if (PlayerController != nullptr && PlayerController->WasInputKeyJustPressed(EKeys::P))
    {
      ReleaseDriverTakeover();
    }
    return;
  }

  // Pedals as set by the driver: what the movement component holds besides what was written there
  // from the client / autopilot control last time
  const float Throttle = MovementComponent->GetThrottleInput();
  const float Brake = MovementComponent->GetBrakeInput();
  const bool bThrottlePressed = Throttle > DriverPedalThreshold
      && FMath::Abs(Throttle - LastAppliedControl.Throttle) > DriverPedalThreshold;
  const bool bBrakePressed = Brake > DriverPedalThreshold
      && FMath::Abs(Brake - LastAppliedControl.Brake) > DriverPedalThreshold;
  const bool bPedalPressed = bThrottlePressed || bBrakePressed;
  if (bWaitPedalsReleased)
  {
    if (Throttle <= DriverPedalThreshold && Brake <= DriverPedalThreshold)
    {
      bWaitPedalsReleased = false;
    }
  }

  // Right after P, the blueprint's flags are kept off (its own P handling may toggle them)
  if (ReleaseFlagsClearTicks > 0)
  {
    --ReleaseFlagsClearTicks;
    DriverTakeover::ClearBlueprintFlags(this);
  }

  // The blueprint's own takeover detection (e.g. turning the wheel away from the autopilot's steering)
  const FName BlueprintFlagOn = DriverTakeover::BlueprintFlag(this);
  const bool bBlueprintFlag = BlueprintFlagOn != NAME_None;
  const bool bBlueprintTakeover = bBlueprintFlag && !bLastBlueprintTakeoverFlag;
  bLastBlueprintTakeoverFlag = bBlueprintFlag;

  if ((bPedalPressed && !bWaitPedalsReleased) || bBlueprintTakeover)
  {
    bDriverTakeover = true;
    DriverTakeoverCause = (bPedalPressed && !bWaitPedalsReleased)
        ? FString(bBrakePressed ? TEXT("brake") : TEXT("throttle"))
        : BlueprintFlagOn.ToString();
    // The autopilot's throttle (and brake) must not stay applied: from now on they only come from the
    // driver's pedals. The pedal the driver is pressing, if any, keeps its value
    if (!bThrottlePressed)
    {
      MovementComponent->SetThrottleInput(0.0f);
      LastAppliedControl.Throttle = 0.0f;
    }
    if (!bBrakePressed)
    {
      MovementComponent->SetBrakeInput(0.0f);
      LastAppliedControl.Brake = 0.0f;
    }
    CARLA_VR_LOG(TEXT("%s: the driver takes over the control (%s)"), *GetName(),
        bPedalPressed ? TEXT("pedal") : TEXT("steering wheel"));
  }
}

bool ACarlaWheeledVehicle::FClutchAxis::Update(float Value, float DeltaSeconds, bool bLearnRest)
{
  if (!bSeen)
  {
    bSeen = true;
    Rest = StableValue = Value;
  }
  // The rest value is learnt while the autopilot drives (the left foot is off the pedal), once the value
  // has stayed the same for a second: depending on the device, the pedal rests at 0, -1 or 1
  if (FMath::Abs(Value - StableValue) > 0.02f)
  {
    StableValue = Value;
    StableSeconds = 0.0f;
  }
  else
  {
    StableSeconds += DeltaSeconds;
  }
  if (bLearnRest && StableSeconds > 1.0f)
  {
    Rest = StableValue;
  }
  // Pressed past half of its travel (1 when resting at 0, 2 when resting at -1 or 1)
  const float Travel = FMath::Abs(Rest) > 0.5f ? 2.0f : 1.0f;
  if (FMath::Abs(Value - Rest) > 0.2f * Travel)
  {
    bMoved = true;
  }
  return FMath::Abs(Value - Rest) > 0.5f * Travel;
}

void ACarlaWheeledVehicle::UpdateClutchReverse()
{
  UChaosWheeledVehicleMovementComponent* MovementComponent = GetChaosWheeledVehicleMovementComponent();
  UWorld* World = GetWorld();
  if (MovementComponent == nullptr || World == nullptr)
  {
    return;
  }
  const float DeltaSeconds = World->GetDeltaSeconds();

  // The clutch axis of the pedals (the input mapping IMC_CarlaVR maps it to IA_Clutch), read directly, and the
  // blueprint's ClutchValue (if its controller forwards IA_Clutch to the vehicle)
  bool bKeyPressed = false;
  float RawClutch = 0.0f;
  APlayerController* PlayerController = World->GetFirstPlayerController();
  if (PlayerController != nullptr)
  {
    static const FKey ClutchKey(TEXT("Joystick_Thrustmaster_T300RS_Racing_Wheel_Axis_3"));
    RawClutch = PlayerController->GetInputAnalogKeyState(ClutchKey);
    bKeyPressed = ClutchKeyAxis.Update(RawClutch, DeltaSeconds, !bDriverTakeover);
  }
  bool bBlueprintPressed = false;
  float BlueprintClutch = 0.0f;
  if (const FNumericProperty* ClutchProperty = FindFProperty<FNumericProperty>(GetClass(), TEXT("ClutchValue")))
  {
    BlueprintClutch = static_cast<float>(
        ClutchProperty->GetFloatingPointPropertyValue(ClutchProperty->ContainerPtrToValuePtr<void>(this)));
    bBlueprintPressed = ClutchBlueprintAxis.Update(BlueprintClutch, DeltaSeconds, !bDriverTakeover);
  }
  // The pedal's own axis once it moved: the blueprint's ClutchValue is only set while the input is triggered,
  // so after a release it keeps its last value (the clutch looked held for good)
  const bool bClutchPressed = ClutchKeyAxis.bMoved ? bKeyPressed : bBlueprintPressed;

  const bool bClutchJustPressed = bClutchPressed && !bClutchWasPressed;
  bClutchWasPressed = bClutchPressed;

  // Only while the driver drives (the autopilot sets its own gear)
  if (!bDriverTakeover)
  {
    return;
  }
  // Each press of the clutch switches between forward and reverse
  if (bClutchJustPressed)
  {
    bClutchReverse = !bClutchReverse;
    CARLA_VR_LOG(TEXT("%s: clutch pressed (pedal axis %.2f, blueprint ClutchValue %.2f), %s gear"), *GetName(),
        RawClutch, BlueprintClutch, bClutchReverse ? TEXT("reverse") : TEXT("forward"));
  }

  // The gear is checked every tick: SetTargetGear does nothing when asked for the gear the movement component
  // last reported (a tick late), and the blueprint may shift on its own. The throttle drives backwards in
  // reverse gear only when it is not 'reverse as brake' (which shifts back to first gear on the throttle)
  MovementComponent->bReverseAsBrake = false;
  const int32 TargetGear = MovementComponent->GetTargetGear();
  const int32 CurrentGear = MovementComponent->GetCurrentGear();
  if (bClutchReverse && (TargetGear >= 0 || CurrentGear >= 0))
  {
    MovementComponent->SetTargetGear(-1, true);
  }
  else if (!bClutchReverse && (TargetGear < 0 || CurrentGear < 0))
  {
    MovementComponent->SetTargetGear(1, true);
  }
}

void ACarlaWheeledVehicle::ReleaseDriverTakeover()
{
  if (!bDriverTakeover)
  {
    return;
  }
  if (bClutchReverse)
  {
    bClutchReverse = false;
    if (UChaosWheeledVehicleMovementComponent* MovementComponent = GetChaosWheeledVehicleMovementComponent())
    {
      MovementComponent->SetTargetGear(1, true);
    }
  }
  bDriverTakeover = false;
  bWaitPedalsReleased = true;
  DriverTakeover::ClearBlueprintFlags(this);
  ReleaseFlagsClearTicks = 10;
  bLastBlueprintTakeoverFlag = true;  // a flag has to turn off before turning on takes over again
  CARLA_VR_LOG(TEXT("%s: control given back to the autopilot"), *GetName());
}

void ACarlaWheeledVehicle::SetThrottleInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Throttle = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetSteeringInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Steer = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetBrakeInput(const float Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.Brake = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetReverse(const bool Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.bReverse = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

void ACarlaWheeledVehicle::SetHandbrakeInput(const bool Value)
{
  FVehicleControl Control = InputControl.Control;
  Control.bHandBrake = Value;
  ApplyVehicleControl(Control, EVehicleInputPriority::User);
}

TArray<float> ACarlaWheeledVehicle::GetWheelsFrictionScale()
{
  TArray<float> WheelsFrictionScale;
  UChaosWheeledVehicleMovementComponent* Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement)
  {
    check(Movement != nullptr);

    for (auto& Wheel : Movement->Wheels)
    {
      WheelsFrictionScale.Add(Wheel->FrictionForceMultiplier);
    }
  }

  return WheelsFrictionScale;
}

void ACarlaWheeledVehicle::SetWheelsFrictionScale(TArray<float>& WheelsFrictionScale)
{
  UChaosWheeledVehicleMovementComponent* Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement)
  {
    check(Movement != nullptr);
    check(Movement->Wheels.Num() == WheelsFrictionScale.Num());

    for (int32 i = 0; i < Movement->Wheels.Num(); ++i)
    {
      Movement->Wheels[i]->FrictionForceMultiplier = WheelsFrictionScale[i];
    }
  }
}

FVehiclePhysicsControl ACarlaWheeledVehicle::GetVehiclePhysicsControl() const
{
  FVehiclePhysicsControl PhysicsControl = { };
  auto VehicleMovComponentPtr = GetChaosWheeledVehicleMovementComponent();
  check(VehicleMovComponentPtr != nullptr);
  auto& VehicleMovComponent = *VehicleMovComponentPtr;
  auto& EngineSetup = VehicleMovComponent.EngineSetup;
  auto RCurve = EngineSetup.TorqueCurve.GetRichCurve();
  check(RCurve != nullptr);
  PhysicsControl.TorqueCurve = *RCurve;
  PhysicsControl.MaxTorque = EngineSetup.MaxTorque;
  PhysicsControl.MaxRPM = EngineSetup.MaxRPM;
  PhysicsControl.IdleRPM = EngineSetup.EngineIdleRPM;
  PhysicsControl.BrakeEffect = EngineSetup.EngineBrakeEffect;
  PhysicsControl.RevUpMOI = EngineSetup.EngineRevUpMOI;
  PhysicsControl.RevDownRate = EngineSetup.EngineRevDownRate;
  auto& DifferentialSetup = VehicleMovComponent.DifferentialSetup;
  PhysicsControl.DifferentialType = (uint8_t)DifferentialSetup.DifferentialType;
  PhysicsControl.FrontRearSplit = DifferentialSetup.FrontRearSplit;
  auto& TransmissionSetup = VehicleMovComponent.TransmissionSetup;
  PhysicsControl.bUseAutomaticGears = TransmissionSetup.bUseAutomaticGears;
  PhysicsControl.GearChangeTime = TransmissionSetup.GearChangeTime;
  PhysicsControl.FinalRatio = TransmissionSetup.FinalRatio;
  PhysicsControl.ForwardGearRatios = TransmissionSetup.ForwardGearRatios;
  PhysicsControl.ReverseGearRatios = TransmissionSetup.ReverseGearRatios;
  PhysicsControl.ChangeUpRPM = TransmissionSetup.ChangeUpRPM;
  PhysicsControl.ChangeDownRPM = TransmissionSetup.ChangeDownRPM;
  PhysicsControl.TransmissionEfficiency = TransmissionSetup.TransmissionEfficiency;
  PhysicsControl.Mass = VehicleMovComponent.Mass;
  PhysicsControl.DragCoefficient = VehicleMovComponent.DragCoefficient;
  PhysicsControl.CenterOfMass = GetCenterOfMass(VehicleMovComponent);
  PhysicsControl.ChassisWidth = VehicleMovComponent.ChassisWidth;
  PhysicsControl.ChassisHeight = VehicleMovComponent.ChassisHeight;
  PhysicsControl.DownforceCoefficient = VehicleMovComponent.DownforceCoefficient;
  PhysicsControl.DragArea = VehicleMovComponent.DragArea;
  PhysicsControl.InertiaTensorScale = VehicleMovComponent.InertiaTensorScale;
  PhysicsControl.SleepThreshold = VehicleMovComponent.SleepThreshold;
  PhysicsControl.SleepSlopeLimit = VehicleMovComponent.SleepSlopeLimit;
  RCurve = VehicleMovComponent.SteeringSetup.SteeringCurve.GetRichCurve();
  check(RCurve != nullptr);
  PhysicsControl.SteeringCurve = *RCurve;
  PhysicsControl.UseSweepWheelCollision = false;
  // Wheels Setup
  PhysicsControl.Wheels.SetNum(VehicleMovComponent.WheelSetups.Num());
  for (int32 i = 0; i < PhysicsControl.Wheels.Num(); ++i)
  {
    auto& In = *VehicleMovComponent.Wheels[i];
    auto& Out = PhysicsControl.Wheels[i];
    Out.AxleType = In.AxleType;
    Out.Offset = In.Offset;
    Out.WheelRadius = In.WheelRadius;
    Out.WheelWidth = In.WheelWidth;
    Out.WheelMass = In.WheelMass;
    Out.CorneringStiffness = In.CorneringStiffness;
    Out.FrictionForceMultiplier = In.FrictionForceMultiplier;
    Out.SideSlipModifier = In.SideSlipModifier;
    Out.SlipThreshold = In.SlipThreshold;
    Out.SkidThreshold = In.SkidThreshold;
    Out.MaxSteerAngle = In.MaxSteerAngle;
    Out.bAffectedBySteering = In.bAffectedBySteering;
    Out.bAffectedByBrake = In.bAffectedByBrake;
    Out.bAffectedByHandbrake = In.bAffectedByHandbrake;
    Out.bAffectedByEngine = In.bAffectedByEngine;
    Out.bABSEnabled = In.bABSEnabled;
    Out.bTractionControlEnabled = In.bTractionControlEnabled;
    Out.MaxWheelspinRotation = In.MaxWheelspinRotation;
    Out.ExternalTorqueCombineMethod = In.ExternalTorqueCombineMethod;
    RCurve = In.LateralSlipGraph.GetRichCurve();
    check(RCurve != nullptr);
    Out.LateralSlipGraph = *RCurve;
    Out.SuspensionAxis = In.SuspensionAxis;
    Out.SuspensionForceOffset = In.SuspensionForceOffset;
    Out.SuspensionMaxRaise = In.SuspensionMaxRaise;
    Out.SuspensionMaxDrop = In.SuspensionMaxDrop;
    Out.SuspensionDampingRatio = In.SuspensionDampingRatio;
    Out.WheelLoadRatio = In.WheelLoadRatio;
    Out.SpringRate = In.SpringRate;
    Out.SpringPreload = In.SpringPreload;
    Out.SuspensionSmoothing = In.SuspensionSmoothing;
    Out.RollbarScaling = In.RollbarScaling;
    Out.SweepShape = In.SweepShape;
    Out.SweepType = In.SweepType;
    Out.MaxBrakeTorque = In.MaxBrakeTorque;
    Out.MaxHandBrakeTorque = In.MaxHandBrakeTorque;
    Out.WheelIndex = In.WheelIndex;
    Out.Location = In.Location;
    Out.OldLocation = In.OldLocation;
    Out.Velocity = In.Velocity;
    // If some of the wheels use Spherecast, it will be used for everyone
    // if we call ApplyPhisicsControl() as it has no sense to have it only for one.
    if (In.SweepShape == ESweepShape::Spherecast)
      PhysicsControl.UseSweepWheelCollision = true;
  }
  return PhysicsControl;
}

FVector ACarlaWheeledVehicle::GetCenterOfMass(UChaosWheeledVehicleMovementComponent& VehicleMovComponent) const
{
  if (VehicleMovComponent.bEnableCenterOfMassOverride)
  {
    return VehicleMovComponent.CenterOfMassOverride;
  }
  else {
    auto PrimitiveComponentPtr = Cast<UPrimitiveComponent>(VehicleMovComponent.UpdatedComponent);
    check(PrimitiveComponentPtr != nullptr);
    auto& PrimitiveComponent = *PrimitiveComponentPtr;
    return PrimitiveComponent.BodyInstance.COMNudge;
  }
}

FVehicleLightState ACarlaWheeledVehicle::GetVehicleLightState() const
{
  return InputControl.LightState;
}

void ACarlaWheeledVehicle::RestoreVehiclePhysicsControl()
{
  ApplyVehiclePhysicsControl(LastPhysicsControl);
}

void ACarlaWheeledVehicle::ApplyVehiclePhysicsControl(
  const FVehiclePhysicsControl& PhysicsControl)
{
  LastPhysicsControl = PhysicsControl;
  auto VehicleMovComponentPtr = GetChaosWheeledVehicleMovementComponent();
  check(VehicleMovComponentPtr != nullptr);
  auto& VehicleMovComponent = *VehicleMovComponentPtr;
  auto& EngineSetup = VehicleMovComponent.EngineSetup;
  EngineSetup.TorqueCurve.EditorCurveData = PhysicsControl.TorqueCurve;
  EngineSetup.MaxTorque = PhysicsControl.MaxTorque;
  EngineSetup.MaxRPM = PhysicsControl.MaxRPM;
  EngineSetup.EngineIdleRPM = PhysicsControl.IdleRPM;
  EngineSetup.EngineBrakeEffect = PhysicsControl.BrakeEffect;
  EngineSetup.EngineRevUpMOI = PhysicsControl.RevUpMOI;
  EngineSetup.EngineRevDownRate = PhysicsControl.RevDownRate;
  auto& DifferentialSetup = VehicleMovComponent.DifferentialSetup;
  DifferentialSetup.DifferentialType = (EVehicleDifferential)PhysicsControl.DifferentialType;
  DifferentialSetup.FrontRearSplit = PhysicsControl.FrontRearSplit;
  auto& TransmissionSetup = VehicleMovComponent.TransmissionSetup;
  TransmissionSetup.bUseAutomaticGears = PhysicsControl.bUseAutomaticGears;
  TransmissionSetup.GearChangeTime = PhysicsControl.GearChangeTime;
  TransmissionSetup.FinalRatio = PhysicsControl.FinalRatio;
  TransmissionSetup.ForwardGearRatios = PhysicsControl.ForwardGearRatios;
  TransmissionSetup.ReverseGearRatios = PhysicsControl.ReverseGearRatios;
  TransmissionSetup.ChangeUpRPM = PhysicsControl.ChangeUpRPM;
  TransmissionSetup.ChangeDownRPM = PhysicsControl.ChangeDownRPM;
  TransmissionSetup.TransmissionEfficiency = PhysicsControl.TransmissionEfficiency;
  VehicleMovComponent.Mass = PhysicsControl.Mass;
  VehicleMovComponent.DragCoefficient = PhysicsControl.DragCoefficient;
  SetCenterOfMass(VehicleMovComponent, PhysicsControl);
  VehicleMovComponent.ChassisWidth = PhysicsControl.ChassisWidth;
  VehicleMovComponent.ChassisHeight = PhysicsControl.ChassisHeight;
  VehicleMovComponent.DownforceCoefficient = PhysicsControl.DownforceCoefficient;
  VehicleMovComponent.DragArea = PhysicsControl.DragArea;
  VehicleMovComponent.InertiaTensorScale = PhysicsControl.InertiaTensorScale;
  VehicleMovComponent.SleepThreshold = PhysicsControl.SleepThreshold;
  VehicleMovComponent.SleepSlopeLimit = PhysicsControl.SleepSlopeLimit;
  auto& SteeringSetup = VehicleMovComponent.SteeringSetup;
  SteeringSetup.SteeringCurve.EditorCurveData = PhysicsControl.SteeringCurve;

  // Wheels Setup
  auto WheelCount = PhysicsControl.Wheels.Num();

  // @TODO: What about bikes or NW vehicles?
  if (WheelCount != 4)
  {
    UE_LOG(LogCarla, Error, TEXT("Number of WheelPhysicsControl is not 4."));
    return;
  }

  for (auto& WheelSetup : VehicleMovComponent.WheelSetups)
    check(WheelSetup.WheelClass != nullptr);

  for (int32 i = 0; i < WheelCount; ++i)
  {
    auto& In = PhysicsControl.Wheels[i];
    auto OutPtr = VehicleMovComponent.Wheels[i];
    check(OutPtr != nullptr);
    auto& Out = *OutPtr;

    Out.AxleType = In.AxleType;
    Out.Offset = In.Offset;
    VehicleMovComponent.SetWheelRadius(i, In.WheelRadius);
    Out.WheelWidth = In.WheelWidth;
    Out.WheelMass = In.WheelMass;
    Out.CorneringStiffness = In.CorneringStiffness;
    VehicleMovComponent.SetWheelFrictionMultiplier(i, In.FrictionForceMultiplier);
    Out.SideSlipModifier = In.SideSlipModifier;
    Out.SlipThreshold = In.SlipThreshold;
    Out.SkidThreshold = In.SkidThreshold;
    VehicleMovComponent.SetWheelMaxSteerAngle(i, In.MaxSteerAngle);
    Out.bAffectedBySteering = In.bAffectedBySteering;
    Out.bAffectedByBrake = In.bAffectedByBrake;
    Out.bAffectedByHandbrake = In.bAffectedByHandbrake;
    Out.bAffectedByEngine = In.bAffectedByEngine;
    VehicleMovComponent.SetABSEnabled(i, In.bABSEnabled);
    VehicleMovComponent.SetTractionControlEnabled(i, In.bTractionControlEnabled);
    Out.MaxWheelspinRotation = In.MaxWheelspinRotation;
    Out.ExternalTorqueCombineMethod = In.ExternalTorqueCombineMethod;
    Out.LateralSlipGraph.EditorCurveData = In.LateralSlipGraph;
    Out.SuspensionAxis = In.SuspensionAxis;
    Out.SuspensionForceOffset = In.SuspensionForceOffset;
    Out.SuspensionMaxRaise = In.SuspensionMaxRaise;
    Out.SuspensionMaxDrop = In.SuspensionMaxDrop;
    Out.SuspensionDampingRatio = In.SuspensionDampingRatio;
    Out.WheelLoadRatio = In.WheelLoadRatio;
    Out.SpringRate = In.SpringRate;
    Out.SpringPreload = In.SpringPreload;
    Out.SuspensionSmoothing = In.SuspensionSmoothing;
    Out.RollbarScaling = In.RollbarScaling;
    Out.SweepShape = In.SweepShape;
    Out.SweepType = In.SweepType;
    VehicleMovComponent.SetWheelMaxBrakeTorque(i, In.MaxBrakeTorque);
    VehicleMovComponent.SetWheelHandbrakeTorque(i, In.MaxHandBrakeTorque);
    Out.WheelIndex = In.WheelIndex;
    Out.Location = In.Location;
    Out.OldLocation = In.OldLocation;
    Out.Velocity = In.Velocity;

    Out.SweepShape =
      PhysicsControl.UseSweepWheelCollision ?
      ESweepShape::Spherecast :
      ESweepShape::Raycast;
  }

  VehicleMovComponent.RecreatePhysicsState();
  ResetConstraints();

  auto* Recorder = UCarlaStatics::GetRecorder(GetWorld());
  if (Recorder && Recorder->IsEnabled())
  {
    Recorder->AddPhysicsControl(*this);
  }

  // Update physics in the Ackermann Controller
  AckermannController.UpdateVehiclePhysics(this);
}

void ACarlaWheeledVehicle::SetCenterOfMass(UChaosWheeledVehicleMovementComponent& VehicleMovComponent, const FVehiclePhysicsControl& PhysicsControl)
{
  if (VehicleMovComponent.bEnableCenterOfMassOverride)
  {
    VehicleMovComponent.CenterOfMassOverride = PhysicsControl.CenterOfMass;
  }
  else {
    auto PrimitiveComponentPtr = Cast<UPrimitiveComponent>(VehicleMovComponent.UpdatedComponent);
    check(PrimitiveComponentPtr != nullptr);
    auto& PrimitiveComponent = *PrimitiveComponentPtr;
    PrimitiveComponent.BodyInstance.COMNudge = PhysicsControl.CenterOfMass;
  }
}

void ACarlaWheeledVehicle::ApplyVehicleControl(const FVehicleControl& Control, EVehicleInputPriority Priority)
{
  if (bAckermannControlActive) {
    AckermannController.Reset();
  }
  bAckermannControlActive = false;

  if (InputControl.Priority <= Priority)
  {
    InputControl.Control = Control;
    InputControl.Priority = Priority;
  }
}

void ACarlaWheeledVehicle::ApplyVehicleAckermannControl(const FVehicleAckermannControl& AckermannControl, EVehicleInputPriority Priority)
{
  bAckermannControlActive = true;
  LastAppliedAckermannControl = AckermannControl;
  AckermannController.SetTargetPoint(AckermannControl);
}

void ACarlaWheeledVehicle::ActivateVelocityControl(const FVector& Velocity)
{
  VelocityControl->Activate(Velocity);
}

void ACarlaWheeledVehicle::DeactivateVelocityControl()
{
  VelocityControl->Deactivate();
}

void ACarlaWheeledVehicle::ShowDebugTelemetry(bool Enabled)
{
  if (GetWorld()->GetFirstPlayerController())
  {
    ACarlaHUD* hud = Cast<ACarlaHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
    if (hud) {

    // Set/Unset the car movement component in HUD to show the temetry
    if (Enabled) {
      hud->AddDebugVehicleForTelemetry(GetChaosWheeledVehicleMovementComponent());
    }
    else{
      if (hud->DebugVehicle == GetChaosWheeledVehicleMovementComponent()) {
      hud->AddDebugVehicleForTelemetry(nullptr);
      //GetChaosWheeledVehicleMovementComponent()->StopTelemetry();
      }
    }

    }
    else {
    UE_LOG(LogCarla, Warning, TEXT("ACarlaWheeledVehicle::ShowDebugTelemetry:: Cannot find HUD for debug info"));
    }
  }
}

void ACarlaWheeledVehicle::SetVehicleLightState(const FVehicleLightState& LightState)
{
  if (LightState.Position != InputControl.LightState.Position ||
    LightState.LowBeam != InputControl.LightState.LowBeam ||
    LightState.HighBeam != InputControl.LightState.HighBeam ||
    LightState.Brake != InputControl.LightState.Brake ||
    LightState.RightBlinker != InputControl.LightState.RightBlinker ||
    LightState.LeftBlinker != InputControl.LightState.LeftBlinker ||
    LightState.Reverse != InputControl.LightState.Reverse ||
    LightState.Fog != InputControl.LightState.Fog ||
    LightState.Interior != InputControl.LightState.Interior ||
    LightState.Special1 != InputControl.LightState.Special1 ||
    LightState.Special2 != InputControl.LightState.Special2)
  {
    InputControl.LightState = LightState;
    RefreshLightState(LightState);
  }
}

void ACarlaWheeledVehicle::SetFailureState(const carla::rpc::VehicleFailureState& InFailureState)
{
  FailureState = InFailureState;
}

void ACarlaWheeledVehicle::SetCarlaMovementComponent(UBaseCarlaMovementComponent* NewBaseMovementComponent)
{
  if (BaseMovementComponent)
  {
    BaseMovementComponent->DestroyComponent();
  }
  BaseMovementComponent = NewBaseMovementComponent;
}

void ACarlaWheeledVehicle::SetWheelSteerDirection(EVehicleWheelLocation WheelLocation, float AngleInDeg)
{
  if (bPhysicsEnabled == false)
  {
    check((uint8)WheelLocation >= 0)
      UVehicleAnimationInstance* VehicleAnim = Cast<UVehicleAnimationInstance>(GetMesh()->GetAnimInstance());
    check(VehicleAnim != nullptr)
      // ToDo We need to investigate about this
      //VehicleAnim->GetWheelAnimData()SetWheelRotYaw((uint8)WheelLocation, AngleInDeg);
  }
  else
  {
    UE_LOG(LogTemp, Warning, TEXT("Cannot set wheel steer direction. Physics are enabled."))
  }
}

float ACarlaWheeledVehicle::GetWheelSteerAngle(EVehicleWheelLocation WheelLocation) {

#if 0 // @CARLAUE5     // ToDo We need to investigate about this
  check((uint8)WheelLocation >= 0)
    UVehicleAnimationInstance* VehicleAnim = Cast<UVehicleAnimationInstance>(GetMesh()->GetAnimInstance());
  check(VehicleAnim != nullptr)
    check(VehicleAnim->GetWheeledVehicleMovementComponent() != nullptr)

    if (bPhysicsEnabled == true)
    {
      return VehicleAnim->GetWheeledVehicleMovementComponent()->Wheels[(uint8)WheelLocation]->GetSteerAngle();
    }
    else
    {
      return VehicleAnim->GetWheelRotAngle((uint8)WheelLocation);
    }
#else
  return 0.0F;
#endif
}

void ACarlaWheeledVehicle::SetSimulatePhysics(bool enabled) {
  if (!GetCarlaMovementComponent<UDefaultMovementComponent>())
  {
    return;
  }

  UChaosWheeledVehicleMovementComponent* Movement = GetChaosWheeledVehicleMovementComponent();
  if (Movement)
  {
    check(Movement != nullptr);

    if (bPhysicsEnabled == enabled)
      return;

    SetActorEnableCollision(true);
    UPrimitiveComponent* RootPrimitive =
      Cast<UPrimitiveComponent>(GetRootComponent());
    RootPrimitive->SetSimulatePhysics(enabled);
    RootPrimitive->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

    UVehicleAnimationInstance* VehicleAnim = Cast<UVehicleAnimationInstance>(GetMesh()->GetAnimInstance());
    check(VehicleAnim != nullptr)

      if (enabled)
      {
        Movement->RecreatePhysicsState();
        //VehicleAnim->ResetWheelCustomRotations();
      }
      else
      {
        Movement->DestroyPhysicsState();
      }
    bPhysicsEnabled = enabled;

    ResetConstraints();
  }
}

void ACarlaWheeledVehicle::ResetConstraints()
{
  for (int i = 0; i < ConstraintsComponents.Num(); i++)
  {
    OpenDoorPhys(EVehicleDoor(i));
  }
  for (int i = 0; i < ConstraintsComponents.Num(); i++)
  {
    CloseDoorPhys(EVehicleDoor(i));
  }
}

FVector ACarlaWheeledVehicle::GetVelocity() const
{
  return BaseMovementComponent->GetVelocity();
}

void ACarlaWheeledVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  //ShowDebugTelemetry(false);
  if (EndPlayReason == EEndPlayReason::Destroyed)
  {
    // The parts of the VR vehicles' interior attached to the vehicle (driving seat with its steering wheel
    // and pedals, mirrors) are destroyed with it, instead of staying where it was. Everything else is only
    // detached: the player's pawn, CARLA's own actors (sensors, left to their client), and the dashboard
    // blueprint's elements (bounding boxes, prediction arrows: BP_DashboardManager reuses them)
    UWorld* World = GetWorld();
    APlayerController* PlayerController = World != nullptr ? World->GetFirstPlayerController() : nullptr;
    APawn* PlayerPawn = PlayerController != nullptr ? PlayerController->GetPawn() : nullptr;
    UCarlaEpisode* Episode = UCarlaStatics::GetCurrentEpisode(World);
    TArray<AActor*> AttachedActors;
    GetAttachedActors(AttachedActors, true, true);
    for (AActor* Attached : AttachedActors)
    {
      if (!IsValid(Attached) || Attached->IsActorBeingDestroyed())
      {
        continue;
      }
      if (PlayerPawn != nullptr && (Attached == PlayerPawn || Attached->IsAttachedTo(PlayerPawn)))
      {
        if (Attached == PlayerPawn)
        {
          PlayerPawn->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
        }
        continue;
      }
      if (Attached->GetParentActor() != nullptr ||
          (Episode != nullptr && Episode->FindCarlaActor(Attached) != nullptr))
      {
        continue;  // child actors are destroyed by their component, CARLA actors by their client
      }
      bool bInteriorPart = false;
      for (const UClass* Class = Attached->GetClass(); Class != nullptr && !bInteriorPart; Class = Class->GetSuperClass())
      {
        const FString ClassName = Class->GetName();
        bInteriorPart = ClassName.Contains(TEXT("DrivingSeat")) || ClassName.Contains(TEXT("Mirrors")) ||
            ClassName.Contains(TEXT("Steering"));
      }
      if (bInteriorPart)
      {
        Attached->Destroy();
      }
    }
  }
  Super::EndPlay(EndPlayReason);
  RemoveReferenceToManager();
}

void ACarlaWheeledVehicle::OpenDoor(const EVehicleDoor DoorIdx) {
  if (int(DoorIdx) >= ConstraintsComponents.Num() && DoorIdx != EVehicleDoor::All) {
    UE_LOG(LogTemp, Warning, TEXT("This door is not configured for this car."));
    return;
  }

  if (DoorIdx == EVehicleDoor::All) {
    for (int i = 0; i < ConstraintsComponents.Num(); i++)
    {
      OpenDoorPhys(EVehicleDoor(i));
    }
    return;
  }

  OpenDoorPhys(DoorIdx);
}

void ACarlaWheeledVehicle::CloseDoor(const EVehicleDoor DoorIdx) {
  if (int(DoorIdx) >= ConstraintsComponents.Num() && DoorIdx != EVehicleDoor::All) {
    UE_LOG(LogTemp, Warning, TEXT("This door is not configured for this car."));
    return;
  }

  if (DoorIdx == EVehicleDoor::All) {
    for (int i = 0; i < ConstraintsComponents.Num(); i++)
    {
      CloseDoorPhys(EVehicleDoor(i));
    }
    return;
  }

  CloseDoorPhys(DoorIdx);
}

void ACarlaWheeledVehicle::OpenDoorPhys(const EVehicleDoor DoorIdx)
{
  UPhysicsConstraintComponent* Constraint = ConstraintsComponents[static_cast<int>(DoorIdx)];
  UPrimitiveComponent* DoorComponent = ConstraintDoor[Constraint];
  DoorComponent->DetachFromComponent(
    FDetachmentTransformRules(EDetachmentRule::KeepWorld, false));
  FTransform DoorInitialTransform =
    DoorComponentsTransform[DoorComponent] * GetActorTransform();
  DoorComponent->SetWorldTransform(DoorInitialTransform);
  DoorComponent->SetSimulatePhysics(true);
  DoorComponent->SetCollisionProfileName(TEXT("BlockAll"));
  float AngleLimit = Constraint->ConstraintInstance.GetAngularSwing1Limit();
  FRotator AngularRotationOffset = Constraint->ConstraintInstance.AngularRotationOffset;

  if (Constraint->ConstraintInstance.AngularRotationOffset.Yaw < 0.0f)
  {
    AngleLimit = -AngleLimit;
  }
  Constraint->SetAngularOrientationTarget(FRotator(0, AngleLimit, 0));
  Constraint->SetAngularDriveParams(DoorOpenStrength, 1.0, 0.0);

  Constraint->InitComponentConstraint();

  UPhysicsConstraintComponent** CollisionDisable =
    CollisionDisableConstraints.Find(DoorComponent);
  if (CollisionDisable)
  {
    (*CollisionDisable)->InitComponentConstraint();
  }

  RecordDoorChange(DoorIdx, true);
}

void ACarlaWheeledVehicle::CloseDoorPhys(const EVehicleDoor DoorIdx)
{
  UPhysicsConstraintComponent* Constraint = ConstraintsComponents[static_cast<int>(DoorIdx)];
  UPrimitiveComponent* DoorComponent = ConstraintDoor[Constraint];
  FTransform DoorInitialTransform =
    DoorComponentsTransform[DoorComponent] * GetActorTransform();
  DoorComponent->SetSimulatePhysics(false);
  DoorComponent->SetCollisionProfileName(TEXT("NoCollision"));
  DoorComponent->SetWorldTransform(DoorInitialTransform);
  DoorComponent->AttachToComponent(
    GetMesh(), FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
  RecordDoorChange(DoorIdx, false);
}

void ACarlaWheeledVehicle::RecordDoorChange(const EVehicleDoor DoorIdx, bool bIsOpen)
{
  auto* Recorder = UCarlaStatics::GetRecorder(GetWorld());
  if (Recorder && Recorder->IsEnabled())
  {
    Recorder->AddVehicleDoor(*this, DoorIdx, bIsOpen);
  }
}

void ACarlaWheeledVehicle::ApplyRolloverBehavior()
{
  auto roll = GetVehicleTransform().Rotator().Roll;

  // The angular velocity reduction is applied in 4 stages, to improve its smoothness.
  // Case 4 starts the timer to set the rollover flag, so users are notified.
  switch (RolloverBehaviorTracker) {
  case 0: CheckRollover(roll, std::make_pair(130.0, 230.0));      break;
  case 1: CheckRollover(roll, std::make_pair(140.0, 220.0));      break;
  case 2: CheckRollover(roll, std::make_pair(150.0, 210.0));      break;
  case 3: CheckRollover(roll, std::make_pair(160.0, 200.0));      break;
  case 4:
    GetWorld()->GetTimerManager().SetTimer(TimerHandler, this, &ACarlaWheeledVehicle::SetRolloverFlag, RolloverFlagTime);
    RolloverBehaviorTracker += 1;
    break;
  case 5: break;
  default:
    RolloverBehaviorTracker = 5;
  }

  // In case the vehicle recovers, reset the rollover tracker
  if (RolloverBehaviorTracker > 0 && -30 < roll && roll < 30) {
    RolloverBehaviorTracker = 0;
    FailureState = carla::rpc::VehicleFailureState::None;
  }
}

void ACarlaWheeledVehicle::CheckRollover(const float roll, const std::pair<float, float> threshold_roll) {
  if (threshold_roll.first < roll && roll < threshold_roll.second) {
    auto Root = Cast<UPrimitiveComponent>(GetRootComponent());
    auto AngularVelocity = Root->GetPhysicsAngularVelocityInDegrees();
    Root->SetPhysicsAngularVelocityInDegrees((1 - RolloverBehaviorForce) * AngularVelocity);
    RolloverBehaviorTracker += 1;
  }
}

void ACarlaWheeledVehicle::SetRolloverFlag() {
  // Make sure the vehicle hasn't recovered since the timer started
  if (RolloverBehaviorTracker >= 4) {
    FailureState = carla::rpc::VehicleFailureState::Rollover;
  }
}

carla::rpc::VehicleFailureState ACarlaWheeledVehicle::GetFailureState() const {
  return FailureState;
}

void ACarlaWheeledVehicle::AddReferenceToManager()
{
  const UObject* World = GetWorld();
  TArray<AActor*> ActorsInLevel;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), ActorsInLevel);
  for (AActor* Actor : ActorsInLevel)
  {
    AVegetationManager* Manager = Cast<AVegetationManager>(Actor);
    if (!IsValid(Manager))
      continue;
    Manager->AddVehicle(this);
    return;
  }
}

void ACarlaWheeledVehicle::RemoveReferenceToManager()
{
  const UObject* World = GetWorld();
  TArray<AActor*> ActorsInLevel;
  UGameplayStatics::GetAllActorsOfClass(World, AActor::StaticClass(), ActorsInLevel);
  for (AActor* Actor : ActorsInLevel)
  {
    AVegetationManager* Manager = Cast<AVegetationManager>(Actor);
    if (!IsValid(Manager))
      continue;
    Manager->RemoveVehicle(this);
    return;
  }
}

FRotator ACarlaWheeledVehicle::GetPhysicsConstraintAngle(
  UPhysicsConstraintComponent* Component)
{
  return Component->ConstraintInstance.AngularRotationOffset;
}

void ACarlaWheeledVehicle::SetPhysicsConstraintAngle(
  UPhysicsConstraintComponent* Component, const FRotator& NewAngle)
{
  Component->ConstraintInstance.AngularRotationOffset = NewAngle;
}

FPoseSnapshot ACarlaWheeledVehicle::GetWorldTransformedPose()
{
  if(WorldTransformedPose.bIsValid == false)
  {
    SetActorTickEnabled(true);
    GetMesh()->SnapshotPose(WorldTransformedPose);
    for(FTransform &transform : WorldTransformedPose.LocalTransforms)
    {
      transform *= GetMesh()->GetComponentTransform();
    }
  }
  return WorldTransformedPose;
}
