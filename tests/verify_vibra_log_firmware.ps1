# @file verify_vibra_log_firmware.ps1
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-06
#
# @copyright Copyright (c) 2026
#
$ErrorActionPreference = 'Stop'

$vibraRoot = Split-Path -Parent $PSScriptRoot
$projectDir = Join-Path $vibraRoot 'project\MDK_V5'
$projectPath = Join-Path $projectDir 'VibraNode.uvprojx'
$buildLogPath = Join-Path $projectDir 'objects\VibraNode.feature_build_output.txt'
$mapPath = Join-Path $projectDir 'listings\VibraNode.map'
$axfPath = Join-Path $projectDir 'objects\VibraNode.axf'
$uv4Path = 'C:\Keil_v5\UV4\UV4.exe'

Remove-Item -LiteralPath $buildLogPath -Force -ErrorAction SilentlyContinue
& $uv4Path -j0 -r $projectPath -t 'VibraNode' -o $buildLogPath
if ($LASTEXITCODE -ne 0) {
  throw "Keil rebuild process failed with exit code $LASTEXITCODE"
}

$deadline = [DateTime]::UtcNow.AddSeconds(30)
$buildLog = ''
do {
  if (Test-Path -LiteralPath $buildLogPath) {
    $rawBuildLog = Get-Content -LiteralPath $buildLogPath -Raw
    if ($null -eq $rawBuildLog) {
      $buildLog = ''
    }
    else {
      $buildLog = [string]$rawBuildLog
    }
    if ($buildLog.Contains('Build Time Elapsed:')) {
      break
    }
  }
  Start-Sleep -Milliseconds 100
} while ([DateTime]::UtcNow -lt $deadline)

if (-not $buildLog.Contains('Build Time Elapsed:')) {
  throw "Timed out waiting for Keil build log: $buildLogPath"
}

if ($buildLog -notmatch '0 Error\(s\), 0 Warning\(s\)') {
  throw "Keil rebuild was not clean:`n$buildLog"
}

$map = Get-Content -LiteralPath $mapPath -Raw
foreach ($requiredSymbol in @('SEGGER_RTT_printf', 'SEGGER_RTT_Init', 'wk_timebase_get')) {
  if (-not $map.Contains($requiredSymbol)) {
    throw "Final image is missing symbol: $requiredSymbol"
  }
}

$imageText = [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($axfPath))
if (-not $imageText.Contains('VibraNode started')) {
  throw 'Final image is missing the VibraNode startup log text'
}

Write-Output 'Vibra log firmware image: PASS'
