param(
  [double]$Duration = 120,
  [double]$Loss = 0.4,
  [int]$Devices = 6,
  [uint32]$Seed = 42,
  [double]$Baud = 9600,
  [int]$PacketMode = 0,        # 0=不打包(逐字节转发) 1=打包(攒包静默后整包突发上线)
  [double]$PacketIdleMs = 10,  # 打包静默门限(ms)
  [double]$IdleGuardMs = -1,   # 空闲保障窗口(ms), -1=自动(打包=门限)
  [switch]$Debug
)

$ErrorActionPreference = 'Stop'
$simDir = $PSScriptRoot
$gtDir = (Resolve-Path (Join-Path $simDir '..\src')).Path

$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
if(-not (Test-Path $vswhere)) { throw 'vswhere not found; need VS Build Tools (MSVC)' }
$vs = (& $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Select-Object -First 1)
if(-not $vs) { throw 'MSVC C++ toolset not found' }
$vcvars = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
if(-not (Test-Path $vcvars)) { throw "vcvars64.bat not found: $vcvars" }

# 清理旧目标
Get-ChildItem $simDir -Filter '*.obj' -ErrorAction SilentlyContinue | Remove-Item -Force

$env:SIMDIR = $simDir
$env:GTDIR  = $gtDir
$env:VCVARS = $vcvars

$extra = ''
if($Debug){ $extra = ' /DGT_DEBUG' }

$bat = "@echo off`r`n"
$bat += 'call "%VCVARS%" >nul 2>&1' + "`r`n"
$bat += 'cd /d "%SIMDIR%"' + "`r`n"
$bat += "cl /nologo /std:c11 /W3 /O2 /utf-8$extra /I`"%SIMDIR%`" /I`"%GTDIR%`" /c `"%GTDIR%\GridTalk.c`" /Fo`"gridtalk.obj`" || exit /b 1`r`n"
$bat += "cl /nologo /std:c11 /W3 /O2 /utf-8 /I`"%SIMDIR%`" /I`"%GTDIR%`" sim.c gridtalk.obj /Fe:sim.exe || exit /b 1`r`n"
$batPath = Join-Path $simDir '_build.bat'
[System.IO.File]::WriteAllText($batPath, $bat, [System.Text.Encoding]::Default)
cmd /c "`"$batPath`""
$code = $LASTEXITCODE
Remove-Item $batPath -Force -ErrorAction SilentlyContinue
if($code -ne 0){ throw "build failed (exit=$code)" }

# 运行仿真
& (Join-Path $simDir 'sim.exe') $Duration $Loss $Devices $Seed $Baud $PacketMode $PacketIdleMs $IdleGuardMs
