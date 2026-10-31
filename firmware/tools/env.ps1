<##
.SYNOPSIS
    Select the repository-local Arm compiler and Visual Studio build tools.

.DESCRIPTION
    This script only discovers existing tools and updates the current
    PowerShell process when dot-sourced. It never downloads or installs files.

    Typical use from the repository root:

        . .\tools\env.ps1
        cmake --preset native
        cmake --build --preset native
        cmake --preset arm-none-eabi

    Use -PrintOnly to inspect the selected paths without changing the process.
##>
[CmdletBinding()]
param(
    [switch] $PrintOnly
)

function Set-SimpleClockEnvironment {
    [CmdletBinding()]
    param(
        [switch] $NoChange
    )

    $ErrorActionPreference = "Stop"

    $repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
    $lockPath = Join-Path $PSScriptRoot "toolchain.lock.json"
    $lock = Get-Content -LiteralPath $lockPath -Raw | ConvertFrom-Json

    # Use only the exact compiler named by the lock file. This prevents a
    # different compiler from being silently reported as the locked version.
    $expectedRoot = Join-Path $repoRoot ($lock.installRoot -replace "/", "\")
    $compilerPath = Join-Path $expectedRoot ($lock.compilerRelativePath -replace "/", "\")
    if (-not (Test-Path -LiteralPath $compilerPath)) {
        throw "Locked compiler is missing: $compilerPath"
    }
    $compilerPath = (Resolve-Path -LiteralPath $compilerPath).Path

    $versionLine = (& $compilerPath --version 2>$null | Select-Object -First 1)
    if ($versionLine -notmatch [regex]::Escape($lock.version)) {
        throw "Locked compiler version mismatch. Expected $($lock.version); got '$versionLine'."
    }

    $armBin = Split-Path -Parent $compilerPath
    $armRoot = Split-Path -Parent $armBin

    $vsWhereCandidates = @(
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"),
        (Join-Path ${env:ProgramFiles} "Microsoft Visual Studio\Installer\vswhere.exe")
    ) | Where-Object { $_ -and (Test-Path -LiteralPath $_) }

    $vsInstall = $null
    if ($vsWhereCandidates.Count -gt 0) {
        $vsWhere = $vsWhereCandidates | Select-Object -First 1
        $vsInstall = (& $vsWhere -latest -products * `
            -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
            -property installationPath 2>$null | Select-Object -First 1)
        if ($vsInstall) { $vsInstall = $vsInstall.Trim() }
    }

    $cmakePath = $null
    $ninjaPath = $null
    $vcvarsPath = $null
    if ($vsInstall) {
        $vsRoot = $vsInstall.Trim()
        $vcvarsCandidate = Join-Path $vsRoot "VC\Auxiliary\Build\vcvars64.bat"
        $cmakeCandidate = Join-Path $vsRoot `
            "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
        $ninjaCandidate = Join-Path $vsRoot `
            "Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
        if (Test-Path -LiteralPath $vcvarsCandidate) { $vcvarsPath = (Resolve-Path $vcvarsCandidate).Path }
        if (Test-Path -LiteralPath $cmakeCandidate) { $cmakePath = (Resolve-Path $cmakeCandidate).Path }
        if (Test-Path -LiteralPath $ninjaCandidate) { $ninjaPath = (Resolve-Path $ninjaCandidate).Path }
    }

    if (-not $cmakePath) {
        $cmakeCommand = Get-Command cmake.exe -ErrorAction SilentlyContinue
        if ($cmakeCommand) { $cmakePath = $cmakeCommand.Source }
    }
    if (-not $ninjaPath) {
        $ninjaCommand = Get-Command ninja.exe -ErrorAction SilentlyContinue
        if ($ninjaCommand) { $ninjaPath = $ninjaCommand.Source }
    }

    if (-not $NoChange) {
        # Import the complete MSVC developer environment when available. The
        # batch file runs in a child cmd.exe; parsing `set` carries its values
        # into this PowerShell process without modifying user/system settings.
        if ($vcvarsPath) {
            # vcvars itself calls vswhere.exe, which the installer does not put on PATH.
            $vcvarsOutput = cmd.exe /d /s /c ('set "PATH={1};%PATH%" && call "{0}" >nul && set' -f $vcvarsPath, (Split-Path -Parent $vsWhere))
            if ($LASTEXITCODE -ne 0) {
                throw "Visual Studio environment initialization failed: $vcvarsPath"
            }
            foreach ($line in $vcvarsOutput) {
                if ($line -match "^(?<name>[^=]+)=(?<value>.*)$") {
                    Set-Item -Path ("Env:{0}" -f $Matches.name) -Value $Matches.value
                }
            }
        }

        # CMake/Ninja use the Windows ANSI code page for localized /showIncludes
        # dependency prefixes. A UTF-8 console with a localized MSVC installation
        # can silently lose all header dependencies. Match the existing system
        # code page in this console; do not change Windows language/registry.
        if ($vcvarsPath) {
            $compilerCodePage = Get-ItemPropertyValue `
                'HKLM:\SYSTEM\CurrentControlSet\Control\Nls\CodePage' -Name ACP
            chcp.com $compilerCodePage > $null
            if ($LASTEXITCODE -ne 0) { throw 'Could not set compiler console code page.' }
        }
        $env:ARM_GNU_TOOLCHAIN_ROOT = $armRoot
        $env:ARM_NONE_EABI_GCC_PATH = $armBin
        $env:ARM_NONE_EABI_GCC_VERSION = $lock.version

        $pathEntries = @($armBin)
        if ($cmakePath) { $pathEntries += Split-Path -Parent $cmakePath }
        if ($ninjaPath) { $pathEntries += Split-Path -Parent $ninjaPath }
        $env:Path = (($pathEntries + ($env:Path -split ";")) |
            Where-Object { $_ -and $_.Trim() } |
            Select-Object -Unique) -join ";"
    }

    Write-Output "Repository: $repoRoot"
    Write-Output "Arm GNU Toolchain $($lock.version): $armRoot"
    Write-Output "Arm compiler: $compilerPath"
    if ($vsInstall) { Write-Output "Visual Studio: $vsInstall" }
    if ($vcvarsPath) { Write-Output "MSVC environment: $vcvarsPath" }
    if ($cmakePath) { Write-Output "CMake: $cmakePath" }
    if ($ninjaPath) { Write-Output "Ninja: $ninjaPath" }
    if ($NoChange) { Write-Output "Environment unchanged (-PrintOnly)." }
}

Set-SimpleClockEnvironment -NoChange:$PrintOnly
