$env:CARLA_ROOT = "C:\Users\USER\Documents\GitHub\scenario_xai\Carla_UE5-Polito"
# The carla Python package is expected to come from `pip install` (matching the 0.10.0 server),
# not from a versioned .egg. Only PythonAPI/carla/ (for the local `agents` package) is added here.
$env:PYTHONPATH = "$env:CARLA_ROOT\PythonAPI\carla\"
$env:LEADERBOARD_ROOT = $PSScriptRoot
$env:TEAM_AGENT = "$env:LEADERBOARD_ROOT\leaderboard\autoagents\npc_agent.py"
# Use this project's own scenario_runner copy, not an external clone, so the
# project is self-contained and its scenario fixes actually take effect.
$env:SCENARIO_RUNNER_ROOT = "$env:CARLA_ROOT\scenario_runner"
$env:PYTHONPATH = "$env:PYTHONPATH;$env:SCENARIO_RUNNER_ROOT;$env:LEADERBOARD_ROOT"
$env:ROUTES = "$env:LEADERBOARD_ROOT\data\routes_Town15_long3_text.xml"
$env:ROUTES_SUBSET = "0"
$env:REPETITIONS = "1"
Write-Host $env:PYTHONPATH
python -c "import carla; print('Using carla package:', carla.__file__)"

$env:DEBUG_CHALLENGE = "0"
$env:CHALLENGE_TRACK_CODENAME = "SENSORS"
$env:CHECKPOINT_ENDPOINT = "$env:LEADERBOARD_ROOT\results.json"
# Leaderboard's CARLA recording: one file per route, <RECORD_PATH>\<route name>_rep<n>.log. The folder must
# exist and be an absolute path (the CARLA server writes the file). When set, the session logger doesn't
# record (only one recording at a time): set it back to "" to get the session logger's fuller recording
# (with the additional data and the analyses) in logs\sessions\<session>\recording.log
$env:RECORD_PATH = "$env:LEADERBOARD_ROOT\logs\recordings"
New-Item -ItemType Directory -Force $env:RECORD_PATH | Out-Null
$env:RESUME = ""

$env:RUN_LOG = "$env:LEADERBOARD_ROOT\run_log.txt"
# Simulation steps per second (synchronous mode), also the session log rate (frames.csv). Only useful if
# the server and the agent keep up: check realtime_factor in the session's summary.json (1 = real time,
# below = slow motion for the participant)
$env:FRAME_RATE = "20"
$env:PYTHONIOENCODING = "utf-8"

# Gaze video at the end of the route (tools\make_session_gaze_video.py): the OBS recording of the headset
# view with the gaze on top, in the session folder. $MakeGazeVideo = $false to skip it.
# The gaze position on the video is calibrated from the SYNC cue of each recording. These values are only
# used when it can't be measured (GAZE_HFOV also for recordings without the cue's "+" markers), or always
# with $env:GAZE_CALIBRATION = "manual"
# $EyeTracking = $false: quick VR tests without anything gaze related (no gaze recording in CARLA, no
# eye data check, no OBS recording, no gaze video)
$EyeTracking = $true
$env:GAZE_RECORD = if ($EyeTracking) { "1" } else { "0" }
$MakeGazeVideo = $EyeTracking
$env:GAZE_HFOV = "100"
$env:GAZE_CX = "0.72"
$env:GAZE_CY = "0.5"
$env:GAZE_CALIBRATION = "auto"
# OBS recording started / stopped by this script when OBS's WebSocket server is enabled (Tools > WebSocket
# Server Settings, port 4455; password, if any, in $env:OBS_PASSWORD or in obs_password.txt next to this
# script, not committed; pip install obs-websocket-py). Otherwise start / stop the OBS recording by hand.
$ObsPasswordFile = Join-Path $PSScriptRoot "obs_password.txt"
if (-not $env:OBS_PASSWORD -and (Test-Path $ObsPasswordFile)) {
    $env:OBS_PASSWORD = (Get-Content $ObsPasswordFile -Raw).Trim()
}
$RouteStart = [DateTimeOffset]::Now.ToUnixTimeSeconds()
if ($MakeGazeVideo) { python "$env:LEADERBOARD_ROOT\tools\obs_control.py" start }

# To stop the run early (e.g. the participant feels unwell): Ctrl+Shift+Q, whichever window has the focus.
# The route ends cleanly (session log, CARLA recording kept), then the steps below run as usual. Not Ctrl+C:
# it also stops this script before them
python "$env:LEADERBOARD_ROOT\leaderboard\leaderboard_evaluator.py" `
    --routes="$env:ROUTES" `
    --routes-subset="$env:ROUTES_SUBSET" `
    --repetitions="$env:REPETITIONS" `
    --track="$env:CHALLENGE_TRACK_CODENAME" `
    --checkpoint="$env:CHECKPOINT_ENDPOINT" `
    --debug-checkpoint="$env:DEBUG_CHECKPOINT_ENDPOINT" `
    --agent="$env:TEAM_AGENT" `
    --agent-config="$env:TEAM_CONFIG" `
    --debug="$env:DEBUG_CHALLENGE" `
    --frame-rate="$env:FRAME_RATE" `
    --record="$env:RECORD_PATH" `
    2>&1 | Tee-Object -FilePath $env:RUN_LOG
    # --host=158.109.9.49
    # --resume=$env:RESUME

if ($MakeGazeVideo) {
    python "$env:LEADERBOARD_ROOT\tools\obs_control.py" stop
}

# The analysis for the paper (gaze video included) is made afterwards, not here (it takes long): the exact
# command for each session of this run. Participant code: $env:PARTICIPANT (e.g. $env:PARTICIPANT = "P06")
$SessionsDir = if ($env:SESSION_LOG_DIR) { $env:SESSION_LOG_DIR } else { "$env:LEADERBOARD_ROOT\logs\sessions" }
$Sessions = Get-ChildItem -Path $SessionsDir -Directory -ErrorAction SilentlyContinue |
    Where-Object { $_.CreationTime -ge [DateTimeOffset]::FromUnixTimeSeconds($RouteStart).LocalDateTime } |
    Sort-Object CreationTime
$Participant = if ($env:PARTICIPANT) { " --participant $env:PARTICIPANT" } else { " --participant <P1..P6>" }
$Participant += if ($env:DRIVE) { " --drive $env:DRIVE" } else { " --drive <1..3>" }

# CARLA's recording of each route (replayable .log): moved into its session folder as recording.log (the
# leaderboard writes <RECORD_PATH>\<route name>_rep<n>.log, the same name at every run)
$Recordings = @()
if ($env:RECORD_PATH) {
    $Recordings = @(Get-ChildItem -Path $env:RECORD_PATH -Filter *.log -ErrorAction SilentlyContinue |
        Where-Object { $_.LastWriteTime -ge [DateTimeOffset]::FromUnixTimeSeconds($RouteStart).LocalDateTime } |
        Sort-Object CreationTime)
}
$SessionList = @($Sessions)
for ($i = 0; $i -lt [Math]::Min($Recordings.Count, $SessionList.Count); $i++) {
    $Recording = $Recordings[$i]
    $Target = Join-Path $SessionList[$i].FullName "recording.log"
    Move-Item -Force $Recording.FullName $Target
}

Write-Host ""
if ($Sessions) {
    # The same commands, also written to commands.txt in the session folder
    $Commands = @("# Run on $(Get-Date -Format 'yyyy-MM-dd HH:mm'), routes $env:ROUTES", "")
    foreach ($Session in $Sessions) {
        $Log = Join-Path $Session.FullName "recording.log"
        if (Test-Path $Log) {
            $Replay = "python `"$env:LEADERBOARD_ROOT\tools\replay_session.py`" `"$($Session.FullName)`" [--event <n>]"
            Write-Host "CARLA recording (replayable): $Log" -ForegroundColor Cyan
            Write-Host "  replay (CARLA running): $Replay"
            $Commands += "# CARLA recording (replayable): $Log", "# replay (CARLA running):", $Replay, ""
        } else {
            Write-Host "No CARLA recording for $($Session.Name) (RECORD_PATH empty?)" -ForegroundColor Yellow
            $Commands += "# No CARLA recording for $($Session.Name) (RECORD_PATH empty?)", ""
        }
    }
    Write-Host ""
    Write-Host "Analysis for the paper (protocol tables and figures, report, gaze plots and video), to run later:" -ForegroundColor Cyan
    $Commands += "# Analysis for the paper (protocol tables and figures, report, gaze plots and video)",
                 "# (add --no-video to skip the gaze video; outputs in <session folder>\paper)"
    foreach ($Session in $Sessions) {
        $Analyze = "python `"$env:LEADERBOARD_ROOT\tools\analyze_session.py`" `"$($Session.FullName)`"$Participant"
        Write-Host "  $Analyze"
        $Commands += $Analyze
    }
    Write-Host "  (add --no-video to skip the gaze video; outputs in <session folder>\paper)"
    $Aggregate = "python `"$env:LEADERBOARD_ROOT\tools\aggregate_study.py`""
    Write-Host "Once all the drives are analysed (and study\questionnaires.csv filled in): " -NoNewline -ForegroundColor Cyan
    Write-Host $Aggregate
    $Commands += "", "# Once all the drives are analysed (and study\questionnaires.csv filled in):", $Aggregate

    foreach ($Session in $Sessions) {
        $CommandsFile = Join-Path $Session.FullName "commands.txt"
        $Commands | Out-File -FilePath $CommandsFile -Encoding utf8
    }
    Write-Host ""
    Write-Host "Commands written to $CommandsFile" -ForegroundColor Cyan
} else {
    Write-Host "No session folder written by this run (in $SessionsDir)" -ForegroundColor Yellow
}
