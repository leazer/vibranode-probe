# @file verify_rtt_project_integration.ps1
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-06
#
# @copyright Copyright (c) 2026
#
$ErrorActionPreference = 'Stop'

$vibraRoot = Split-Path -Parent $PSScriptRoot
$projectPath = Join-Path $vibraRoot 'project\MDK_V5\VibraNode.uvprojx'
$configPath = Join-Path $vibraRoot 'middlewares\SEGGER_RTT_V798g\Config\SEGGER_RTT_Conf.h'

[xml]$project = Get-Content -LiteralPath $projectPath -Raw
$target = $project.Project.Targets.Target
$cSettings = $target.TargetOption.TargetArmAds.Cads
if ([string]$cSettings.uC99 -ne '1') {
  throw 'ArmCC C99 mode is required by SEGGER_RTT_printf.c'
}

$rttGroup = @($target.Groups.Group) | Where-Object { $_.GroupName -eq 'SEGGER RTT' }
if ($null -eq $rttGroup) {
  throw 'Missing SEGGER RTT project group'
}

$rttPaths = @($rttGroup.Files.File | ForEach-Object { [string]$_.FilePath })
$expectedPaths = @(
  '..\..\middlewares\SEGGER_RTT_V798g\RTT\SEGGER_RTT.c',
  '..\..\middlewares\SEGGER_RTT_V798g\RTT\SEGGER_RTT_printf.c'
)
foreach ($expectedPath in $expectedPaths) {
  if ($rttPaths -notcontains $expectedPath) {
    throw "Missing RTT source: $expectedPath"
  }
}
if ($rttPaths.Count -ne 2) {
  throw "SEGGER RTT group must contain exactly 2 files, found $($rttPaths.Count)"
}

$allSourcePaths = @($target.Groups.Group | ForEach-Object { $_.Files.File } |
  ForEach-Object { [string]$_.FilePath })
if ($allSourcePaths -match '\\Syscalls\\|\\Examples\\') {
  throw 'Syscalls or Examples were added to the production project'
}

$includePaths = ([string]$target.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath).Split(';')
foreach ($requiredInclude in @(
  '..\..\middlewares\SEGGER_RTT_V798g\RTT',
  '..\..\middlewares\SEGGER_RTT_V798g\Config'
)) {
  if ($includePaths -notcontains $requiredInclude) {
    throw "Missing include path: $requiredInclude"
  }
}

$config = Get-Content -LiteralPath $configPath -Raw
if ($config -notmatch 'SEGGER_RTT_PRINTF_BUFFER_SIZE\s+\(128u\)') {
  throw 'RTT printf buffer is not 128 bytes'
}
if ($config -match 'register unsigned char BASEPRI') {
  throw 'ArmCC RTT lock still uses BASEPRI'
}
$primaskCount = [regex]::Matches($config, '_SEGGER_RTT__PRIMASK __asm').Count
if ($primaskCount -lt 2) {
  throw 'ArmCC ARMv7-M lock does not use PRIMASK'
}
if ($config -notmatch 'SEGGER_RTT_MODE_DEFAULT\s+SEGGER_RTT_MODE_NO_BLOCK_SKIP') {
  throw 'RTT channel 0 is not configured as non-blocking skip'
}

Write-Output 'SEGGER RTT project integration: PASS'
