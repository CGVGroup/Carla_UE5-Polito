#include "ActuateMotionComponent.h"
#include "GameFramework/Actor.h"
#include "Kismet/KismetMathLibrary.h"

UActuateMotionComponent::UActuateMotionComponent()
	: enabled_(false),
	ticks_(0),
	actuateQuaternion_(),
	actuateVector_() {
	PrimaryComponentTick.bCanEverTick = true;

	// Get the actor
	actor_ = GetOwner();
	if (!actor_) {
		UE_LOG(LogTemp, Warning, TEXT("ActuateMotionComponent: Failed to obtain Actor"));
		return;
	}

	// Get a handle to Actuate shared memory client.
	hlib_ = FPlatformProcess::GetDllHandle(TEXT("libshmemclient.dll"));
	if (!hlib_) {
		UE_LOG(LogTemp, Warning, TEXT("ActuateMotionComponent: Failed to load shmemclient library"));
		return;
	}

	// Get reference to the Send function within the library
	Send_ = (SendFunc)FPlatformProcess::GetDllExport(hlib_, TEXT("Send"));
	if (!Send_) {
		UE_LOG(LogTemp, Warning, TEXT("ActuateMotionComponent: Failed to lookup Send symbol"));
		return;
	}

	// Get reference to the SendAndAck function within the library
	SendAndAck_ = (SendAndAckFunc)FPlatformProcess::GetDllExport(hlib_, TEXT("SendAndAck"));
	if (!SendAndAck_) {
		UE_LOG(LogTemp, Warning, TEXT("ActuateMotionComponent: Failed to lookup SendAndAck symbol"));
		return;
	}

	enabled_ = true;
}

UActuateMotionComponent::~UActuateMotionComponent() {
	if (enabled_) {
		SendStopMessage();
	}

	if (hlib_) {
		FPlatformProcess::FreeDllHandle(hlib_);
	}
}

void UActuateMotionComponent::BeginPlay() {
	Super::BeginPlay();

	if (!enabled_) return;

	ticks_ = 0;
	SendStopMessage();
	SendConfigureTelemTypeMessage();
	SendStartMessage();
}

void UActuateMotionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!enabled_) return;

	// Get data about the Actor.
	FQuat unrealQuaternion = actor_->GetActorRotation().Quaternion();
	FVector unrealVector = actor_->GetVelocity();

	// Calculate time, Actuate expects milliseconds.
	ticks_ += DeltaTime * 1000.0f;

	// Re-map axes for Actuate.
	actuateVector_[0] = unrealVector.Y / 100.f;
	actuateVector_[1] = unrealVector.Z / 100.f;
	actuateVector_[2] = unrealVector.X / 100.f;

	actuateQuaternion_[0] = unrealQuaternion.W;
	actuateQuaternion_[1] = unrealQuaternion.Y;
	actuateQuaternion_[2] = unrealQuaternion.Z;
	actuateQuaternion_[3] = unrealQuaternion.X;

	// Send data to Actuate.
	SendTelemMessage();

	// Update Blueprint properties
	UpdateBlueprintProperties();
}

void UActuateMotionComponent::UpdateBlueprintProperties() {
	// Update ActuateVector from the internal array
	ActuateVector = FVector(actuateVector_[0], actuateVector_[1], actuateVector_[2]);

	// Convert the quaternion to an FRotator
	FQuat Quat = FQuat(actuateQuaternion_[0], actuateQuaternion_[1], actuateQuaternion_[2], actuateQuaternion_[3]);
	ActuateRotation = Quat.Rotator();
}

void UActuateMotionComponent::SendStopMessage() {
	ActuateMsg msg = {};
	msg.msgType = TELEMMSG_MSGTYPE_STOP;

	if (!SendAndAck_(msg, ACKTIMEOUT)) {
		UE_LOG(LogTemp, Warning, TEXT("ActuateMotionComponent: Failed to send stop message, timeout?"));
	}
}

void UActuateMotionComponent::SendStartMessage() {
	ActuateMsg msg = {};
	msg.msgType = TELEMMSG_MSGTYPE_START;

	if (!SendAndAck_(msg, ACKTIMEOUT)) {
		UE_LOG(LogTemp, Warning, TEXT("ActuateMotionComponent: Failed to send start message, timeout?"));
	}
}

void UActuateMotionComponent::SendConfigureTelemTypeMessage() {
	ActuateMsg msg = {};
	msg.msgType = TELEMMSG_MSGTYPE_CONFIGURETELEMTYPE;

	ActuateMsgConfigureTelemTypePayload* payload =
		reinterpret_cast<ActuateMsgConfigureTelemTypePayload*>(msg.payload);
	payload->telemType = TELEMTYPE_VELOCITY;

	if (!SendAndAck_(msg, ACKTIMEOUT)) {
		UE_LOG(LogTemp, Warning, TEXT("ActuateMotionComponent: Failed to send configure message, timeout?"));
	}
}

void UActuateMotionComponent::SendTelemMessage() {
	ActuateMsg msg = {};
	msg.msgType = TELEMMSG_MSGTYPE_TELEMETRY;

	ActuateMsgTelemPayload* payload =
		reinterpret_cast<ActuateMsgTelemPayload*>(msg.payload);

	payload->ticks = ticks_;
	payload->vector[0] = actuateVector_[0];
	payload->vector[1] = actuateVector_[1];
	payload->vector[2] = actuateVector_[2];
	payload->rotation[0] = actuateQuaternion_[0];
	payload->rotation[1] = actuateQuaternion_[1];
	payload->rotation[2] = actuateQuaternion_[2];
	payload->rotation[3] = actuateQuaternion_[3];

	Send_(msg);
}

