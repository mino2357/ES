param(
    [string]$Tag = "",
    [string]$OutputDir = "dist"
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$cargoTomlPath = Join-Path $repoRoot "Cargo.toml"
$cargoToml = Get-Content -Path $cargoTomlPath -Raw

$packageName = [regex]::Match($cargoToml, '(?m)^name\s*=\s*"([^"]+)"\s*$').Groups[1].Value
$packageVersion = [regex]::Match($cargoToml, '(?m)^version\s*=\s*"([^"]+)"\s*$').Groups[1].Value
if (-not $packageName) {
    throw "Failed to read package name from Cargo.toml"
}
if (-not $packageVersion) {
    throw "Failed to read package version from Cargo.toml"
}
if (-not $Tag) {
    $Tag = "v$packageVersion"
}

$binaryPath = Join-Path $repoRoot "target\release\$packageName.exe"
if (-not (Test-Path $binaryPath)) {
    throw "Release binary not found: $binaryPath. Run 'cargo build --release' first."
}

$outputRoot = Join-Path $repoRoot $OutputDir
$assetBaseName = "$packageName-windows-x64-$Tag"
$stageDir = Join-Path $outputRoot $assetBaseName
$stageConfigDir = Join-Path $stageDir "config"
$zipPath = Join-Path $outputRoot "$assetBaseName.zip"
$shaPath = Join-Path $outputRoot "$assetBaseName.sha256"

if (Test-Path $stageDir) {
    Remove-Item -Path $stageDir -Recurse -Force
}
if (Test-Path $zipPath) {
    Remove-Item -Path $zipPath -Force
}
if (Test-Path $shaPath) {
    Remove-Item -Path $shaPath -Force
}

New-Item -Path $outputRoot -ItemType Directory -Force | Out-Null
New-Item -Path $stageConfigDir -ItemType Directory -Force | Out-Null

Copy-Item -Path $binaryPath -Destination $stageDir
Copy-Item -Path (Join-Path $repoRoot "README.md") -Destination $stageDir
Copy-Item -Path (Join-Path $repoRoot "README.ja.md") -Destination $stageDir
Copy-Item -Path (Join-Path $repoRoot "LICENSE") -Destination $stageDir
Copy-Item -Path (Join-Path $repoRoot "config\sim.yaml") -Destination $stageConfigDir

Compress-Archive -Path $stageDir -DestinationPath $zipPath -CompressionLevel Optimal

$hash = (Get-FileHash -Path $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
"$hash  $([System.IO.Path]::GetFileName($zipPath))" | Set-Content -Path $shaPath -Encoding ascii -NoNewline

Write-Host "Created $zipPath"
Write-Host "Created $shaPath"

if ($env:GITHUB_OUTPUT) {
    "zip_path=$zipPath" >> $env:GITHUB_OUTPUT
    "sha256_path=$shaPath" >> $env:GITHUB_OUTPUT
    "asset_base_name=$assetBaseName" >> $env:GITHUB_OUTPUT
}
