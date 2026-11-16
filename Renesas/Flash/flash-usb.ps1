$ErrorActionPreference = 'Stop'

$sourceDir = Join-Path $PSScriptRoot '../TinySLAM/3.LSLidar'
$buildDir = Join-Path $sourceDir 'build/Debug'
$hexFile = Join-Path $buildDir 'LSLidar.hex'

$rfp = $null
if ($env:RFP_CLI_PATH) {
    if (-not (Test-Path -LiteralPath $env:RFP_CLI_PATH -PathType Leaf)) {
        throw "RFP_CLI_PATH does not point to a file: $env:RFP_CLI_PATH"
    }
    $rfp = $env:RFP_CLI_PATH
} else {
    $command = Get-Command rfp-cli.exe -ErrorAction SilentlyContinue
    if ($command) { $rfp = $command.Source }
}
if (-not $rfp) {
    throw 'Renesas Flash Programmer CLI was not found. Install RFP, then add rfp-cli.exe to PATH or set RFP_CLI_PATH to its full path.'
}

$ports = @(Get-PnpDevice -PresentOnly -Class Ports | Where-Object {
    $_.InstanceId -match '^USB\\VID_045B&PID_0261' -and $_.FriendlyName -match '\(COM\d+\)'
})
if ($ports.Count -ne 1) {
    throw "Expected one RA USB Boot device; found $($ports.Count). Connect only the intended target in USB boot mode."
}
$port = [regex]::Match($ports[0].FriendlyName, '\((COM\d+)\)').Groups[1].Value

if (-not (Test-Path -LiteralPath (Join-Path $buildDir 'CMakeCache.txt'))) {
    throw "Debug build is not configured: $buildDir. Configure the CMake Debug build in VS Code first."
}
Write-Host "Building Debug firmware..."
& cmake --build $buildDir --target LSLidar.elf
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
if (-not (Test-Path -LiteralPath $hexFile -PathType Leaf)) { throw "Firmware not found: $hexFile" }

Write-Host "Programming $hexFile through RA USB Boot on $port (erase, program, verify)..."
& $rfp -device RA -port $port -auto $hexFile
exit $LASTEXITCODE
