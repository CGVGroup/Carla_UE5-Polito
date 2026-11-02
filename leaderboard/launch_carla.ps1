# Starts the CARLA server in VR when a headset is connected, and as a normal desktop window otherwise
# (same driving view, from the cockpit of the VR ego vehicle). Then run .\run_leaderboard_xai.ps1 as usual.
#
#   .\launch_carla.ps1          : checks the headset (tools\headset_connected.py) and starts CARLA accordingly
#   .\launch_carla.ps1 -VR      : VR
#   .\launch_carla.ps1 -Desktop : desktop window, without checking
#
# Before starting in VR, the VR connection is checked (asking what to do when something is wrong):
#   CARLA not already running (it keeps the VR connection it had when it started), SteamVR running, VIVE
#   Business Streaming running (eye tracking), headset available, eye tracking delivering gaze (headset on:
#   tools\check_eye_tracking.py, ~10 s). -NoEyeCheck skips the eye tracking test (quick tests)
#
# VR rendering load. In VR, CARLA starts with a VR performance profile (smooth headset view, a bit plainer
# look): no Lumen (global illumination / reflections), classic shadow maps instead of virtual ones, TAA
# instead of TSR, no motion blur / lens effects, lighter post-processing, headset view at 80% of its
# resolution (vr.PixelDensity 0.8), small mirror window on the desktop, CARLA at high CPU priority.
#   -FullQuality      : CARLA's full look (Lumen, virtual shadow maps, TSR), tuned for VR: Lumen / shadows /
#                       post-processing at their 'High' scalability instead of 'Epic', no motion blur / depth of
#                       field / lens flares / chromatic aberration / film grain, coarser volumetric fog, larger
#                       texture pool, headset view at 90% of its resolution, small mirror window
#   -FullQuality -Untouched : CARLA's Epic look as is (the headset view can judder)
#   -PixelDensity 0.6 : headset view at 60% of its resolution (1 = full) when it still judders
#   -Quality Medium   : CARLA's rendering quality (Epic = default, Low = much simpler look)
#   -Fast             : the profile on the desktop too (in VR it is already on)
#   e.g. .\launch_carla.ps1 -VR              or  .\launch_carla.ps1 -VR -PixelDensity 0.6
param(
    [switch]$VR,
    [switch]$Desktop,
    [switch]$Fast,
    [switch]$FullQuality,
    [switch]$Untouched,
    [switch]$NoEyeCheck,
    [double]$PixelDensity = 0,
    [ValidateSet("", "Low", "Medium", "High", "Epic")]
    [string]$Quality = ""
)

$CarlaExe = "C:\Users\USER\Documents\GitHub\Carla_UE5-Polito\Build\Package\Carla-0.10.0-Win64-Shipping\Windows\CarlaUnreal.exe"
# Without VR there is no VR main menu to choose the map: Town15 directly, with the VR game mode (the VR
# vehicles take the player on board only with it)
$DesktopMap = "/Game/Carla/Maps/Town15/Town15?game=/CarlaVR_Vehicles/Core/BP_CGMB_CarlaVR.BP_CGMB_CarlaVR_C"

function Confirm-Choice($Question) {
    $Answer = Read-Host "$Question [y/n]"
    return $Answer -match '^(y|yes|o|oui|s|si)$'
}

function Write-Check($Ok, $Text) {
    if ($Ok) { Write-Host "  [OK]   $Text" -ForegroundColor Green } else { Write-Host "  [!!]   $Text" -ForegroundColor Yellow }
}

$AutoMode = -not $VR -and -not $Desktop
if ($AutoMode) {
    Write-Host "Checking the VR headset..."
    python "$PSScriptRoot\tools\headset_connected.py"
    $VR = ($LASTEXITCODE -eq 0)
}

if ($VR) {
    Write-Host "`nChecking the VR connection..."

    # 1. CARLA already running: it keeps the VR connection (and eye tracking or not) it started with
    $Running = Get-Process -Name "CarlaUnreal*" -ErrorAction SilentlyContinue
    Write-Check (-not $Running) "CARLA not already running"
    if ($Running) {
        if (Confirm-Choice "  CARLA is running: close it, so that it starts again with the current VR connection?") {
            $Running | Stop-Process -Force
            Start-Sleep -Seconds 3
        } else {
            Write-Host "  CARLA left running: nothing started"
            exit 1
        }
    }

    # 2. SteamVR (the OpenXR runtime VIVE Business Streaming and Steam Link go through)
    $SteamVR = Get-Process -Name "vrserver", "vrmonitor" -ErrorAction SilentlyContinue
    Write-Check $SteamVR "SteamVR running"
    if (-not $SteamVR) {
        Write-Host "         Start SteamVR (and connect the headset) first"
    }

    # 3. VIVE Business Streaming (needed for the eye tracking; Steam Link has none)
    $Vive = Get-Process -Name "RRServer", "RRConsole" -ErrorAction SilentlyContinue
    Write-Check $Vive "VIVE Business Streaming running"
    if (-not $Vive) {
        Write-Host "         Without it there is no eye tracking (connect the headset through VIVE Business Streaming)"
    }

    # VIVE Console's SteamVR driver (VIVEDriver, for VIVE Pro Eye headsets) conflicts with VIVE Business Streaming
    $ViveConsole = Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.Path -match 'VIVEDriver' }
    Write-Check (-not $ViveConsole) "VIVE Console (VIVEDriver) not running"
    if ($ViveConsole) {
        Write-Host "         It can take over the headset from VIVE Business Streaming: close it, and keep its SteamVR"
        Write-Host "         driver disabled (SteamVR > Settings > Startup / Shutdown > Manage Add-ons > viveVR: off)"
    }

    # 4. Headset available through OpenXR (already checked in automatic mode)
    if (-not $AutoMode) {
        python "$PSScriptRoot\tools\headset_connected.py" | Out-Null
        $Headset = ($LASTEXITCODE -eq 0)
        Write-Check $Headset "Headset available"
        if (-not $Headset) {
            if (Confirm-Choice "  No headset available: start CARLA without VR (desktop window) instead?") {
                $VR = $false
            } elseif (-not (Confirm-Choice "  Start in VR anyway?")) {
                exit 1
            }
        }
    } else {
        Write-Check $true "Headset available"
    }

    # 5. Eye tracking delivering gaze (the headset has to be worn)
    if ($VR -and -not $NoEyeCheck) {
        Write-Host "`n  Eye tracking test: PUT THE HEADSET ON and look around (10 s)..."
        python "$PSScriptRoot\tools\check_eye_tracking.py" --seconds 10 | Select-String -Pattern "Result:|=>|runtime:|layer" | ForEach-Object { Write-Host "         $($_.Line.Trim())" }
        $EyeOk = ($LASTEXITCODE -eq 0)
        Write-Check $EyeOk "Eye tracking delivering gaze"
        if (-not $EyeOk -and -not (Confirm-Choice "  No eye data: start CARLA anyway (no gaze recorded)?")) {
            Write-Host "  Fix the eye tracking (VIVE Business Streaming: eye tracking on, calibration done), then run this again"
            exit 1
        }
    }
    Write-Host ""
}

$RenderArgs = @()
if ($Quality) { $RenderArgs += "-quality-level=$Quality" }
$ExecCmds = @()
if ($FullQuality -and -not $Untouched) {
    # Full look (Lumen global illumination and reflections, virtual shadow maps, TSR, Nanite), its cost cut
    # where it doesn't show in a headset:
    #  - Lumen, shadows, post-processing, effects at their 'High' scalability instead of 'Epic' (UE5's 60 fps
    #    target instead of 30 fps: same features, coarser sampling)
    #  - off: motion blur, depth of field, lens flares, chromatic aberration, film grain (blurry or sickening
    #    in VR anyway)
    #  - volumetric fog on a coarser grid, larger texture pool (no texture streaming hitches)
    $ExecCmds += "sg.GlobalIlluminationQuality 2", "sg.ReflectionQuality 2", "sg.ShadowQuality 2",
                 "sg.PostProcessQuality 2", "sg.EffectsQuality 2",
                 "r.MotionBlurQuality 0", "r.DepthOfFieldQuality 0", "r.LensFlareQuality 0",
                 "r.SceneColorFringeQuality 0", "r.FilmGrain 0",
                 "r.VolumetricFog.GridPixelSize 16", "r.Streaming.PoolSize 3000"
    if ($VR -and $PixelDensity -le 0) { $PixelDensity = 0.9 }
    if ($VR) {
        # The desktop mirror of the headset view (also what OBS records): a small window costs less
        $RenderArgs += "-windowed", "-ResX=1280", "-ResY=720"
    }
}
elseif ($Fast -or ($VR -and -not $FullQuality)) {
    # Scalability groups (1 = medium: Lumen off), then the costliest UE5 features one by one, in case the
    # groups leave them on: Lumen GI / reflections, virtual shadow maps, TSR, motion blur, lens effects
    $ExecCmds += "sg.ShadowQuality 1", "sg.GlobalIlluminationQuality 1", "sg.ReflectionQuality 1",
                 "sg.PostProcessQuality 1", "sg.EffectsQuality 1",
                 "r.DynamicGlobalIlluminationMethod 0", "r.ReflectionMethod 0", "r.Shadow.Virtual.Enable 0",
                 "r.AntiAliasingMethod 2", "r.MotionBlurQuality 0", "r.LensFlareQuality 0",
                 "r.DepthOfFieldQuality 0", "r.BloomQuality 1", "r.SSR.Quality 0"
    if ($PixelDensity -le 0) { $PixelDensity = 0.8 }
    if ($VR) {
        # The desktop mirror of the headset view (also what OBS records): a small window costs less
        $RenderArgs += "-windowed", "-ResX=1280", "-ResY=720"
    }
}
if ($VR -and $PixelDensity -gt 0) {
    $ExecCmds += "vr.PixelDensity " + $PixelDensity.ToString([System.Globalization.CultureInfo]::InvariantCulture)
}
if ($ExecCmds.Count -gt 0) { $RenderArgs += "-ExecCmds=`"$($ExecCmds -join ',')`"" }

if ($VR) {
    Write-Host "Starting CARLA in VR (choose Town15 in the VR menu) $RenderArgs"
    if ($RenderArgs.Count -gt 0) {
        $Process = Start-Process -FilePath $CarlaExe -ArgumentList $RenderArgs -PassThru
    } else {
        $Process = Start-Process -FilePath $CarlaExe -PassThru
    }
} else {
    Write-Host "Starting CARLA without VR (desktop window, Town15) $RenderArgs"
    $Process = Start-Process -FilePath $CarlaExe -ArgumentList (@("`"$DesktopMap`"", "-nohmd") + $RenderArgs) -PassThru
}

# High CPU priority: CARLA (and the frames it hands to the headset) before background programs. The
# packaged exe starts a child process that does the actual work: both
if ($VR -and $Process) {
    Start-Sleep -Seconds 5
    $Raised = 0
    Get-Process -Name "CarlaUnreal*" -ErrorAction SilentlyContinue | ForEach-Object {
        try { $_.PriorityClass = [System.Diagnostics.ProcessPriorityClass]::High; $Raised++ } catch {}
    }
    Write-Check ($Raised -gt 0) "CARLA at high CPU priority"
}

if ($VR) {
    Write-Host ""
    Write-Host "Smooth VR also depends on the stream (VIVE Business Streaming):" -ForegroundColor Cyan
    Write-Host "  - USB 3 cable (icon '3.0'), or Wi-Fi 6/6E on a router of its own, 5/6 GHz, PC on Ethernet"
    Write-Host "  - its settings: dynamic bitrate on, TCP off, 72 Hz, resolution below 2048 per eye"
    Write-Host "  - quality indicator green with CARLA running; nothing else heavy running (browser video, OBS at high quality)"
}
