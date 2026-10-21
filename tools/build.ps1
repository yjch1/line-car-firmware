<#
  固件编译检查 —— 一条命令验证两条线都能编过
  ---------------------------------------------------------------------------
  用法（PowerShell 执行策略禁止直接跑 .ps1，所以用 -ExecutionPolicy Bypass 调用）：

      powershell -NoProfile -ExecutionPolicy Bypass -File tools\build.ps1
      powershell -NoProfile -ExecutionPolicy Bypass -File tools\build.ps1 ultrasonic
      powershell -NoProfile -ExecutionPolicy Bypass -File tools\build.ps1 whisker

  退出码：0 = 通过；1 = 有目标编不过。可直接用于 pre-commit 或 CI。
  ---------------------------------------------------------------------------
  为什么不用 Arduino IDE / arduino-cli：本机 avr-g++ 不在 PATH，只存在于
  %LOCALAPPDATA%\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7。
  这里显式用该路径，等价于 IDE 的编译选项（-Os / gnu++11 / atmega328p / F_CPU 16MHz）。
#>
param(
    [ValidateSet('', 'ultrasonic', 'whisker')]
    [string]$Only = ''
)

$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $PSScriptRoot
$ard  = Join-Path $env:LOCALAPPDATA 'Arduino15\packages\arduino'
$tool = Join-Path $ard 'tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin'
$gpp  = Join-Path $tool 'avr-g++.exe'
$szex = Join-Path $tool 'avr-size.exe'
$core = Join-Path $ard 'hardware\avr\1.8.8'

foreach ($p in @($gpp, $szex, $core)) {
    if (-not (Test-Path $p)) { throw "找不到工具链路径: $p" }
}

$utf8 = New-Object System.Text.UTF8Encoding($false)
$out  = Join-Path $env:TEMP ('fwbuild_' + $PID)
New-Item -ItemType Directory -Force -Path $out | Out-Null

$targets = @(
    [pscustomobject]@{ Name = 'ultrasonic'; Ino = (Join-Path $repo 'ultrasonic\ultrasonic.ino') },
    [pscustomobject]@{ Name = 'whisker';    Ino = (Join-Path $repo 'whisker\whisker.ino') }
)

$failed = 0
foreach ($t in $targets) {
    if ($Only -and $t.Name -ne $Only) { continue }
    if (-not (Test-Path $t.Ino)) {
        Write-Host ("{0,-11} 缺少文件 {1}" -f $t.Name, $t.Ino) -ForegroundColor Red
        $failed++; continue
    }

    # .ino 不是合法编译单元：Arduino IDE 会隐式加 #include <Arduino.h>
    $code = "#include <Arduino.h>`n#line 1 `"$($t.Name).ino`"`n" +
            [System.IO.File]::ReadAllText($t.Ino, $utf8)
    $cpp = Join-Path $out ($t.Name + '.cpp')
    $obj = Join-Path $out ($t.Name + '.o')
    [System.IO.File]::WriteAllText($cpp, $code, $utf8)

    $gargs = @(
        '-c', '-Os', '-std=gnu++11', '-fpermissive', '-fno-exceptions',
        '-ffunction-sections', '-fdata-sections', '-fno-threadsafe-statics',
        '-mmcu=atmega328p', '-DF_CPU=16000000L', '-DARDUINO=10808',
        '-DARDUINO_AVR_UNO', '-DARDUINO_ARCH_AVR',
        "-I$core\cores\arduino", "-I$core\variants\standard",
        '-Wall', '-Wextra', '-Wshadow',
        $cpp, '-o', $obj
    )

    $diag = & $gpp @gargs 2>&1
    $rc   = $LASTEXITCODE
    $lines = @($diag | ForEach-Object { "$_" } | Where-Object { $_.Trim() -ne '' })

    if ($rc -ne 0) {
        $failed++
        Write-Host ("{0,-11} 编译失败 exit={1}，诊断 {2} 行：" -f $t.Name, $rc, $lines.Count) -ForegroundColor Red
        $lines | Select-Object -First 25 | ForEach-Object { '  ' + $_ }
        continue
    }

    if ($lines.Count -gt 0) {
        Write-Host ("{0,-11} 通过（但有 {1} 行告警）：" -f $t.Name, $lines.Count) -ForegroundColor Yellow
        $lines | Select-Object -First 25 | ForEach-Object { '  ' + $_ }
    } else {
        Write-Host ("{0,-11} 通过   0 错误 0 告警" -f $t.Name) -ForegroundColor Green
    }

    # 汇总 flash/RAM。注意：带 -ffunction-sections 时没有裸 .text/.data/.bss 段，
    # 必须用不带 -A 的汇总输出（末行），否则全是 0。末尾的临时文件名去掉，免得干扰阅读。
    $s = & $szex $obj | Select-Object -Last 1
    if ($s) {
        $s = ($s -replace '\s+\S+\s*$', '') -replace '\s+', ' '
        '            text/data/bss:' + $s
    }
}

Remove-Item -Recurse -Force $out -ErrorAction SilentlyContinue

if ($failed) {
    Write-Host ""
    Write-Host "构建失败：$failed 个目标不通过" -ForegroundColor Red
    exit 1
}
Write-Host ""
Write-Host "全部通过" -ForegroundColor Green
exit 0
