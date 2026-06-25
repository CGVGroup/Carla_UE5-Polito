#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ActuateMotionComponent.generated.h"

#define TELEMMSG_MSGTYPE_START 0
#define TELEMMSG_MSGTYPE_STOP 1
#define TELEMMSG_MSGTYPE_TELEMETRY 2
#define TELEMMSG_MSGTYPE_CONFIGURETELEMTYPE 3

#define TELEMTYPE_VELOCITY 0

#define ACKTIMEOUT 250

struct ActuateMsg {
	int msgType;
	char payload[64];
};

struct ActuateMsgConfigureTelemTypePayload {
	int telemType;
};

struct ActuateMsgTelemPayload {
	unsigned long long ticks;
	float vector[3];
	float engineRpm;
	float rotation[4];
	float fuelLevel;
	float temperature;
	float tripCounter;
	float gearIndicator;
	unsigned char dashLights;
};

// send function prototypes
typedef void(*SendFunc)(ActuateMsg&);
typedef int32(*SendAndAckFunc)(ActuateMsg&, int32 timeout);

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class CARLAUNREAL_API UActuateMotionComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UActuateMotionComponent();
	~UActuateMotionComponent();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	virtual void BeginPlay() override;

private:
	AActor* actor_;
	unsigned long long ticks_;

	// Native data storage
	float actuateQuaternion_[4];
	float actuateVector_[3];

	// Exposed properties to Blueprint
	UPROPERTY(BlueprintReadOnly, Category = "Actuate Motion", meta = (AllowPrivateAccess = "true"))
	FVector ActuateVector;

	UPROPERTY(BlueprintReadOnly, Category = "Actuate Motion", meta = (AllowPrivateAccess = "true"))
	FRotator ActuateRotation;

	void SendStopMessage();
	void SendStartMessage();
	void SendConfigureTelemTypeMessage();
	void SendTelemMessage();

	bool enabled_;
	SendFunc Send_;
	SendAndAckFunc SendAndAck_;
	void* hlib_;

	// Helper function to update Blueprint properties
	void UpdateBlueprintProperties();
};
