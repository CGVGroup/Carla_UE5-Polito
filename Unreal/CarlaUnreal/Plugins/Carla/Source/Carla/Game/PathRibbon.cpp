// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/PathRibbon.h"
#include "Carla.h"

#include <util/ue-header-guard-begin.h>
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include <util/ue-header-guard-end.h>

void FPathRibbon::SetFromCommand(UWorld* World, const FString& Points)
{
  TArray<FString> Items;
  Points.ParseIntoArray(Items, TEXT(";"), true);
  TArray<FVector4> Parsed;
  for (const FString& Item : Items)
  {
    TArray<FString> Values;
    Item.ParseIntoArray(Values, TEXT(","), true);
    if (Values.Num() < 4)
    {
      continue;
    }
    // CARLA coordinates [m] -> Unreal [cm]
    Parsed.Emplace(FCString::Atod(*Values[0]) * 100.0, FCString::Atod(*Values[1]) * 100.0,
        FCString::Atod(*Values[2]) * 100.0, FCString::Atod(*Values[3]) * 100.0);
  }
  SetPoints(World, Parsed);
}

void FPathRibbon::SetPoints(UWorld* World, const TArray<FVector4>& Points)
{
  if (Points.Num() < 2)
  {
    Clear();
    return;
  }
  Build(World, Points);
}

void FPathRibbon::SetMaterial(const FString& NewMaterialPath)
{
  MaterialPath = NewMaterialPath.TrimStartAndEnd();
  Material = nullptr;
  if (Mesh.IsValid())
  {
    UMaterialInterface* NewMaterial = LoadObject<UMaterialInterface>(nullptr, *MaterialPath);
    Material = NewMaterial;
    Mesh->SetMaterial(0, NewMaterial);
  }
}

void FPathRibbon::Clear()
{
  if (Mesh.IsValid())
  {
    Mesh->ClearAllMeshSections();
  }
}

void FPathRibbon::Destroy()
{
  if (Actor.IsValid())
  {
    Actor->Destroy();
  }
  Actor = nullptr;
  Mesh = nullptr;
}

UProceduralMeshComponent* FPathRibbon::GetMesh(UWorld* World)
{
  if (Mesh.IsValid())
  {
    return Mesh.Get();
  }
  if (World == nullptr)
  {
    return nullptr;
  }
  FActorSpawnParameters SpawnParams;
  SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  AActor* NewActor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, SpawnParams);
  if (NewActor == nullptr)
  {
    return nullptr;
  }
  UProceduralMeshComponent* NewMesh = NewObject<UProceduralMeshComponent>(NewActor, TEXT("PathRibbonMesh"));
  NewMesh->SetMobility(EComponentMobility::Movable);
  NewMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  NewMesh->SetCastShadow(false);
  NewMesh->bUseAsyncCooking = true;
  NewActor->SetRootComponent(NewMesh);
  NewMesh->RegisterComponent();
  NewActor->SetActorEnableCollision(false);

  UMaterialInterface* NewMaterial = LoadObject<UMaterialInterface>(nullptr, *MaterialPath);
  if (NewMaterial == nullptr)
  {
    UE_LOG(LogCarla, Warning, TEXT("Path ribbon: material %s not found"), *MaterialPath);
  }
  Material = NewMaterial;
  NewMesh->SetMaterial(0, NewMaterial);

  Actor = NewActor;
  Mesh = NewMesh;
  return NewMesh;
}

void FPathRibbon::Build(UWorld* World, const TArray<FVector4>& Points)
{
  UProceduralMeshComponent* RibbonMesh = GetMesh(World);
  if (RibbonMesh == nullptr)
  {
    return;
  }

  // Road surface under a point: traced down onto the static geometry only (not onto the vehicles)
  FCollisionObjectQueryParams StaticObjects(ECC_WorldStatic);
  FCollisionQueryParams Params(SCENE_QUERY_STAT(PathRibbon), false);
  if (Actor.IsValid())
  {
    Params.AddIgnoredActor(Actor.Get());
  }
  // The road surface is the static surface closest to the route's own height there (the point's z, from
  // the route's waypoints), within MaxRoadDeviation of it: curbs, bollards, barriers, signs, trees or
  // bridges above / below aren't taken for the road (the ribbon would go up towards them). None found:
  // the route's height
  auto OnRoad = [&](const FVector& Point) -> FVector
  {
    TArray<FHitResult> Hits;
    const FVector Start = Point + FVector(0.0f, 0.0f, MaxRoadDeviation);
    const FVector End = Point - FVector(0.0f, 0.0f, MaxRoadDeviation);
    World->LineTraceMultiByObjectType(Hits, Start, End, StaticObjects, Params);
    const FHitResult* Best = nullptr;
    for (const FHitResult& Hit : Hits)
    {
      // Not the road: trigger volumes (e.g. the box of a stop sign, over the lane before its stop line),
      // which only report overlaps and don't block traces, and traces starting inside something
      const UPrimitiveComponent* Component = Hit.GetComponent();
      if (Hit.bStartPenetrating || Component == nullptr
          || Component->GetCollisionResponseToChannel(ECC_Visibility) != ECR_Block)
      {
        continue;
      }
      const double Deviation = FMath::Abs(Hit.ImpactPoint.Z - Point.Z);
      if (Deviation <= MaxRoadDeviation && Hit.ImpactNormal.Z > 0.7  // a surface facing up, not a side
          && (Best == nullptr || Deviation < FMath::Abs(Best->ImpactPoint.Z - Point.Z)))
      {
        Best = &Hit;
      }
    }
    return (Best != nullptr ? FVector(Point.X, Point.Y, Best->ImpactPoint.Z) : Point) + FVector(0.0f, 0.0f, Lift);
  };

  // Route points bunched up or going back (e.g. where the route stops at a stop line): dropped, the band
  // would fold over itself there
  TArray<FVector4> Kept;
  for (const FVector4& Point : Points)
  {
    if (Kept.Num() > 0)
    {
      const FVector Last(Kept.Last().X, Kept.Last().Y, 0.0);
      const FVector Delta = FVector(Point.X, Point.Y, 0.0) - Last;
      if (Delta.Size() < 50.0)
      {
        continue;
      }
      if (Kept.Num() > 1)
      {
        const FVector Before = Last - FVector(Kept[Kept.Num() - 2].X, Kept[Kept.Num() - 2].Y, 0.0);
        if (FVector::DotProduct(Before.GetSafeNormal(), Delta.GetSafeNormal()) < 0.0)
        {
          continue;
        }
      }
    }
    Kept.Add(Point);
  }
  if (Kept.Num() < 2)
  {
    RibbonMesh->ClearAllMeshSections();
    return;
  }

  // Center line resampled every Step cm
  TArray<FVector> Centers;
  TArray<float> Widths;
  for (int32 i = 0; i + 1 < Kept.Num(); ++i)
  {
    const FVector A(Kept[i].X, Kept[i].Y, Kept[i].Z);
    const FVector B(Kept[i + 1].X, Kept[i + 1].Y, Kept[i + 1].Z);
    const int32 Subdivisions = FMath::Max(1, FMath::CeilToInt(FVector::Dist(A, B) / Step));
    for (int32 s = 0; s < Subdivisions; ++s)
    {
      const float T = static_cast<float>(s) / Subdivisions;
      Centers.Add(FMath::Lerp(A, B, T));
      Widths.Add(static_cast<float>(FMath::Lerp(Kept[i].W, Kept[i + 1].W, static_cast<double>(T))));
    }
  }
  Centers.Add(FVector(Kept.Last().X, Kept.Last().Y, Kept.Last().Z));
  Widths.Add(static_cast<float>(Kept.Last().W));

  TArray<FVector> Vertices;
  TArray<int32> Triangles;
  TArray<FVector> Normals;
  TArray<FVector2D> UVs;
  TArray<FLinearColor> Colors;
  TArray<FProcMeshTangent> Tangents;
  float Along = 0.0f;
  for (int32 i = 0; i < Centers.Num(); ++i)
  {
    const FVector Previous = Centers[FMath::Max(0, i - 1)];
    const FVector Next = Centers[FMath::Min(Centers.Num() - 1, i + 1)];
    FVector Forward = Next - Previous;
    Forward.Z = 0.0f;
    Forward = Forward.GetSafeNormal();
    const FVector Right(-Forward.Y, Forward.X, 0.0f);
    const double Half = Widths[i] * 0.5;
    if (i > 0)
    {
      Along += FVector::Dist(Centers[i - 1], Centers[i]);
    }
    Vertices.Add(OnRoad(Centers[i] - Right * Half));
    Vertices.Add(OnRoad(Centers[i] + Right * Half));
    Normals.Add(FVector::UpVector);
    Normals.Add(FVector::UpVector);
    UVs.Add(FVector2D(0.0f, Along / 100.0f));
    UVs.Add(FVector2D(1.0f, Along / 100.0f));
    Colors.Add(FLinearColor::White);
    Colors.Add(FLinearColor::White);
    Tangents.Add(FProcMeshTangent(Forward, false));
    Tangents.Add(FProcMeshTangent(Forward, false));
    if (i > 0)
    {
      const int32 L0 = 2 * (i - 1), R0 = L0 + 1, L1 = 2 * i, R1 = L1 + 1;
      // Both windings: whichever faces up is the one drawn (the other is culled, the material being
      // one-sided), whatever the handedness conventions
      Triangles.Append({L0, L1, R0, R0, L1, R1});
      Triangles.Append({L0, R0, L1, R0, R1, L1});
    }
  }
  // Heights smoothed along each edge (median of 5 vertices, ~5 m): a road surface found on something
  // other than the road for a vertex or two (a mark, a manhole, a lip at the stop line) doesn't make the
  // band jump up and down there
  for (int32 Side = 0; Side < 2; ++Side)
  {
    TArray<double> Heights;
    for (int32 i = Side; i < Vertices.Num(); i += 2)
    {
      Heights.Add(Vertices[i].Z);
    }
    for (int32 k = 0; k < Heights.Num(); ++k)
    {
      TArray<double, TInlineAllocator<5>> Window;
      for (int32 j = FMath::Max(0, k - 2); j <= FMath::Min(Heights.Num() - 1, k + 2); ++j)
      {
        Window.Add(Heights[j]);
      }
      Window.Sort();
      Vertices[2 * k + Side].Z = Window[Window.Num() / 2];
    }
  }

  RibbonMesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, UVs, Colors, Tangents, false);
  if (Material.IsValid())
  {
    RibbonMesh->SetMaterial(0, Material.Get());
  }
}
