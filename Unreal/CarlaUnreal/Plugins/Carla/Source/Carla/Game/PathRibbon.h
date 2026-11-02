// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"

class AActor;
class UWorld;
class UMaterialInterface;
class UProceduralMeshComponent;

/// The global path of the route shown on the road ahead of the ego vehicle: a flat ribbon mesh laid on
/// the road surface (each vertex traced down onto the static geometry, so that bumps and the road's crown
/// never hide it), with a translucent, lit AR-HUD projection material (soft, not glowing). The client
/// sends its center line: "ribbon:x,y,z,width;x,y,z,width;..." (CARLA coordinates and widths in meters),
/// "ribbon:" (nothing) removes it, "ribbon_material:<material path>" changes the material
class FPathRibbon
{
public:

  /// Parses a "ribbon:" command (without the prefix) and rebuilds the ribbon
  void SetFromCommand(UWorld* World, const FString& Points);

  /// Rebuilds the ribbon along these center points (Unreal coordinates [cm], W: width [cm])
  void SetPoints(UWorld* World, const TArray<FVector4>& Points);

  void SetMaterial(const FString& NewMaterialPath);

  void Clear();

  /// Removes the ribbon's actor (Clear only empties its mesh)
  void Destroy();

private:

  void Build(UWorld* World, const TArray<FVector4>& Points);

  UProceduralMeshComponent* GetMesh(UWorld* World);

  TWeakObjectPtr<AActor> Actor;
  TWeakObjectPtr<UProceduralMeshComponent> Mesh;
  FString MaterialPath = TEXT("/CarlaVR_Vehicles/ARHUD_TestCase/ARHUD_Designs/M_Projection_LightBlue.M_Projection_LightBlue");
  TWeakObjectPtr<UMaterialInterface> Material;

  /// Vertices every that many cm along the ribbon
  float Step = 100.0f;
  /// Height of the ribbon above the road surface [cm]
  float Lift = 3.0f;
  /// Max height difference [cm] between the road surface found under a vertex and the route's own
  /// height there (more: not the road, e.g. a curb, a bollard, a bridge)
  float MaxRoadDeviation = 60.0f;
};
