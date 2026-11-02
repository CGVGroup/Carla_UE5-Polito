// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"

class UWorld;
class UCarlaEpisode;
class IFileHandle;

/// Records the driver's gaze (VR headset eye tracking, through Unreal's Eye Tracker API, e.g. the
/// OpenXR eye gaze interaction of the VIVE Focus Vision) to a CSV file, one line per frame, with what
/// the gaze hits in the world (CARLA actor id / type, or the semantic class of the map object), the
/// head pose and the ego vehicle's state. Started and stopped by the client through the dashboard's
/// type string ("gaze:start:<name>", "gaze:stop"). Files go to <Saved>/GazeLogs/<name>_<date>.csv
/// When a recording starts, a sync cue (flagged in the 'sync' column) lets a video of the headset view (e.g.
/// OBS capturing the VR mirror window) be matched to the recording, to overlay the gaze on it afterwards,
/// and calibrates that overlay: for the first half of SyncDuration, a big magenta "SYNC" straight in front
/// of the view (where the view's forward direction is in the video), then two magenta "+" at
/// SyncMarkerAngle degrees left and right of it (the video's field of view)
class FGazeRecorder
{
public:

  ~FGazeRecorder();

  /// Starts a new recording (stopping the current one, if any). Returns the file path
  FString Start(const FString& Name);

  void Stop();

  bool IsRecording() const { return File != nullptr; }

  /// Records one sample (called every frame)
  void Tick(UWorld* World, UCarlaEpisode* Episode);

private:

  void WriteLine(const FString& Line);

  /// Shows / moves / removes the sync cue. Returns whether it is shown this frame
  bool UpdateSyncCue(UWorld* World);

  IFileHandle* File = nullptr;
  FString FilePath;
  double LastFlushTime = 0.0;
  int64 Samples = 0;
  int64 ValidSamples = 0;

  /// Max distance of the gaze ray [cm]
  float MaxGazeDistance = 30000.0f;

  /// Sync cue
  bool bSyncPending = false;
  double SyncStartTime = -1.0;
  float SyncDuration = 1.0f;  // [s]
  // Close to the eyes, in front of the dashboard and the steering wheel (further, the car's interior hides
  // it), with the same size in the view as 60 cm high at 2 m
  float SyncDistance = 30.0f;  // [cm] in front of the view
  float SyncSize = 9.0f;  // [cm] height of the text
  float SyncMarkerAngle = 20.0f;  // [deg] of the "+" markers from the view's forward direction
  float SyncMarkerSize = 6.0f;  // [cm]
  /// "SYNC", left "+", right "+"
  TArray<TWeakObjectPtr<class ATextRenderActor>> SyncActors;

  void DestroySyncActors();
};
