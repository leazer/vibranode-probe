# @file verify_vibra_log_macros.ps1
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-06
#
# @copyright Copyright (c) 2026
#
$ErrorActionPreference = 'Stop'

$vibraRoot = Split-Path -Parent $PSScriptRoot
$fixturePath = Join-Path $PSScriptRoot 'vibra_log_macro_contract.c'
$includePath = Join-Path $vibraRoot 'project\inc'
$rttPath = Join-Path $vibraRoot 'middlewares\SEGGER_RTT_V798g\RTT'
$rttConfigPath = Join-Path $vibraRoot 'middlewares\SEGGER_RTT_V798g\Config'
$driverPath = Join-Path $vibraRoot 'libraries\drivers\inc'
$cmsisCorePath = Join-Path $vibraRoot 'libraries\cmsis\cm4\core_support'
$cmsisDevicePath = Join-Path $vibraRoot 'libraries\cmsis\cm4\device_support'
$armccPath = 'C:\Keil_v5\ARM\ARMCC\bin\armcc.exe'

if (-not (Test-Path -LiteralPath $armccPath)) {
  throw "ArmCC not found: $armccPath"
}

$testCases = @(
  [PSCustomObject]@{
    Name = 'default_debug'
    Defines = @()
    Levels = @('E', 'W', 'I', 'D')
    Present = @('M_ERROR', 'M_WARN', 'M_INFO', 'M_DEBUG')
    Absent = @('M_VERBOSE')
    HasTimestamp = $true
    HasRtt = $true
  },
  [PSCustomObject]@{
    Name = 'timestamp_off'
    Defines = @('VLOG_TIMESTAMP_ENABLE=0')
    Levels = @('E', 'W', 'I', 'D')
    Present = @('M_ERROR', 'M_WARN', 'M_INFO', 'M_DEBUG')
    Absent = @('M_VERBOSE')
    HasTimestamp = $false
    HasRtt = $true
  },
  [PSCustomObject]@{
    Name = 'warn_only'
    Defines = @('VLOG_LEVEL=VLOG_LEVEL_WARN', 'TEST_WARN_ONLY=1')
    Levels = @('E', 'W')
    Present = @('M_ERROR', 'M_WARN')
    Absent = @('M_INFO', 'M_DEBUG', 'M_VERBOSE')
    HasTimestamp = $true
    HasRtt = $true
  },
  [PSCustomObject]@{
    Name = 'master_off'
    Defines = @('VLOG_ENABLE=0', 'TEST_MASTER_OFF=1')
    Levels = @()
    Present = @()
    Absent = @('M_ERROR', 'M_WARN', 'M_INFO', 'M_DEBUG', 'M_VERBOSE')
    HasTimestamp = $false
    HasRtt = $false
  }
)

foreach ($testCase in $testCases) {
  $stem = 'vibranode-vlog-' + $testCase.Name + '-' + [Guid]::NewGuid().ToString('N')
  $preprocessedPath = Join-Path ([IO.Path]::GetTempPath()) ($stem + '.i')
  $objectPath = Join-Path ([IO.Path]::GetTempPath()) ($stem + '.o')

  try {
    $commonArgs = @(
      '--cpu', 'Cortex-M4',
      '-DAT32F413CCU7',
      "-I$includePath",
      "-I$rttPath",
      "-I$rttConfigPath",
      "-I$driverPath",
      "-I$cmsisCorePath",
      "-I$cmsisDevicePath"
    )
    foreach ($define in $testCase.Defines) {
      $commonArgs += "-D$define"
    }

    & $armccPath @commonArgs -E $fixturePath -o $preprocessedPath
    if ($LASTEXITCODE -ne 0) {
      throw "Preprocess failed for $($testCase.Name)"
    }

    & $armccPath @commonArgs -c $fixturePath -o $objectPath
    if ($LASTEXITCODE -ne 0) {
      throw "Compile failed for $($testCase.Name)"
    }

    $expanded = Get-Content -LiteralPath $preprocessedPath -Raw
    foreach ($marker in $testCase.Present) {
      if (-not $expanded.Contains($marker)) {
        throw "$($testCase.Name): missing marker $marker"
      }
    }
    foreach ($marker in $testCase.Absent) {
      if ($expanded.Contains($marker)) {
        throw "$($testCase.Name): unexpected marker $marker"
      }
    }

    foreach ($level in $testCase.Levels) {
      if ($testCase.HasTimestamp) {
        $expectedPrefix = '"[%010u][" "' + $level + '" "][" "TEST" "] "'
      }
      else {
        $expectedPrefix = '"[" "' + $level + '" "][" "TEST" "] "'
      }
      if (-not $expanded.Contains($expectedPrefix)) {
        throw "$($testCase.Name): missing level/tag prefix for $level"
      }
    }
    if ($testCase.HasRtt -and (-not $expanded.Contains('"\r\n"'))) {
      throw "$($testCase.Name): missing CRLF line ending"
    }

    $hasTimestampCall = $expanded.Contains('wk_timebase_get()')
    if ($hasTimestampCall -ne $testCase.HasTimestamp) {
      throw "$($testCase.Name): timestamp expansion mismatch"
    }

    $hasRttCall = $expanded.Contains('((void)SEGGER_RTT_printf')
    if ($hasRttCall -ne $testCase.HasRtt) {
      throw "$($testCase.Name): RTT expansion mismatch"
    }

    $hasInitCall = $expanded.Contains('SEGGER_RTT_Init();')
    if ($hasInitCall -ne $testCase.HasRtt) {
      throw "$($testCase.Name): RTT init expansion mismatch"
    }
  }
  finally {
    Remove-Item -LiteralPath $preprocessedPath -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $objectPath -Force -ErrorAction SilentlyContinue
  }
}

Write-Output 'Vibra log macro contract: PASS (4 cases)'
