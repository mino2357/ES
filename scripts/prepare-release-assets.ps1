param(
    [string]$Tag = "",
    [string]$OutputDir = "dist/release-assets"
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
$manifestPath = Join-Path $outputRoot "$packageName-windows-x64-$Tag.sha256"

if (Test-Path $outputRoot) {
    Remove-Item -Path $outputRoot -Recurse -Force
}

New-Item -Path $outputRoot -ItemType Directory -Force | Out-Null

$assets = @(
    @{
        Source = $binaryPath
        Name = "$packageName.exe"
    },
    @{
        Source = Join-Path $repoRoot "config\sim.yaml"
        Name = "sim.yaml"
    },
    @{
        Source = Join-Path $repoRoot "LICENSE"
        Name = "LICENSE"
    }
)

$manifestLines = @()
foreach ($asset in $assets) {
    $destination = Join-Path $outputRoot $asset.Name
    Copy-Item -Path $asset.Source -Destination $destination
    $hash = (Get-FileHash -Path $destination -Algorithm SHA256).Hash.ToLowerInvariant()
    $manifestLines += "$hash  $($asset.Name)"
    Write-Host "Staged $destination"
}

$manifestLines | Set-Content -Path $manifestPath -Encoding ascii
Write-Host "Created $manifestPath"

if ($env:GITHUB_OUTPUT) {
    "assets_dir=$outputRoot" >> $env:GITHUB_OUTPUT
    "sha256_path=$manifestPath" >> $env:GITHUB_OUTPUT
}
