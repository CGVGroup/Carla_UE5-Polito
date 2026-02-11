// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once



#include "Dashboard.generated.h"


USTRUCT(BlueprintType)
struct FObstacle
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
    FString Type = "";

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
    bool isDangerous = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
    FVector Position = FVector(0.0f, 0.0f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
    FVector Rotation = FVector(0.0f, 0.0f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
    float Speed = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
    float Steer = 0.0f;
};

USTRUCT(BlueprintType)
struct FPathWaypoint
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
    FVector Position = FVector(0.0f, 0.0f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
    FVector Rotation = FVector(0.0f, 0.0f, 0.0f);
};

UCLASS()
class CARLA_API ADashboard : public AActor
{

  GENERATED_BODY()

public:

  UPROPERTY(Category="Dashboard", BlueprintReadWrite, EditAnywhere)
  float speed = 0;

  UPROPERTY(Category = "Dashboard", BlueprintReadWrite, EditAnywhere)
  float steer = 0;

  UPROPERTY(Category = "Dashboard", BlueprintReadWrite, EditAnywhere)
  FString text = "";

  // Map Obstacles
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
  TMap<FString, FObstacle> ObstacleList;

  // List of Path Waypoint
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dashboard")
  TArray<FPathWaypoint> VectorPairList;


  UFUNCTION(BlueprintImplementableEvent, Category = "Events")
  void OnPathSet();


  UFUNCTION(BlueprintImplementableEvent, Category = "Events")
  void OnUpdate();


  //-----------------------------------------------------------------------------------------EVENTS
  void TriggerPathSetEvent() {
      OnPathSet();
  }

  void UpdateDashboard() {
      OnUpdate();
  }

  //----------------------------------------------------------------------------------------OBSTACLES
  // Add or update Obstacles
  UFUNCTION(BlueprintCallable)
  void UpdateObstacle(int ID, const FString& Type, bool isDanger, FVector Vector1, FVector Vector2, float obstacleSpeed, float obstacleSteer)
  {
      FObstacle Obstacle;
      Obstacle.Type = Type;
      Obstacle.isDangerous = isDanger;
      Obstacle.Position = Vector1;
      Obstacle.Rotation = Vector2;
      Obstacle.Speed = obstacleSpeed;
      Obstacle.Steer = obstacleSteer;

      ObstacleList.Add(FString::FromInt(ID), Obstacle);
  }

  // Remove an Obstacles
  UFUNCTION(BlueprintCallable)
  void RemoveIDObstacle(int ID)
  {
      ObstacleList.Remove(FString::FromInt(ID));
  }


  //----------------------------------------------------------------------------------------WAYPOINT
  // Add a waypoint to the list
  UFUNCTION(BlueprintCallable)
  void AddVectorPair(FVector P, FVector R)
  {
      FPathWaypoint Pair;
      Pair.Position = P;
      Pair.Rotation = R;
      VectorPairList.Add(Pair);
  }


  //----------------------------------------------------------------------------------------CONTROLS

  UFUNCTION(BlueprintCallable)
  void SetControlValues(float speed_value, float steering_value)
  {
      speed = speed_value;
      steer = steering_value;
      return;
  }

  //----------------------------------------------------------------------------------------TEXT
  UFUNCTION(BlueprintCallable)
  void SetTypeString(FString value)
  {
      text = value;
      return;
  }


  // Clear all data
  UFUNCTION(BlueprintCallable)
  void ClearAll()
  {
      ObstacleList.Empty();
      VectorPairList.Empty();
  }

};
