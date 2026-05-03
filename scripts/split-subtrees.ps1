<#
.SYNOPSIS
  Creates local branches from subdirectory history via git subtree split (BYTE_VA, BYTE_VISION, extra_files).

.DESCRIPTION
  Run from the BYTE_ALL repo root after committing your changes.
  Each branch can be pushed to a new empty remote, e.g.:
    git remote add byte-va https://github.com/you/byte_va.git
    git push byte-va split-BYTE_VA

.PARAMETER Prefix
  Optional single prefix (BYTE_VA, BYTE_VISION, or extra_files). Omit to split all three.

.EXAMPLE
  .\scripts\split-subtrees.ps1
.EXAMPLE
  .\scripts\split-subtrees.ps1 -Prefix BYTE_VA
#>
param(
    [ValidateSet("BYTE_VA", "BYTE_VISION", "extra_files", "")]
    [string]$Prefix = ""
)

$ErrorActionPreference = "Stop"
Set-Location (Resolve-Path (Join-Path $PSScriptRoot ".."))

if (-not (Test-Path ".git")) {
    Write-Error "Run this script from a git clone of BYTE_ALL (no .git directory found)."
}

function Split-One {
    param([string]$Path, [string]$BranchName)
    $exists = git rev-parse --verify $BranchName 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Branch $BranchName already exists; delete it first or choose another name. Skipping."
        return
    }
    Write-Host "Splitting $Path -> branch $BranchName ..."
    git subtree split -P $Path -b $BranchName
    if ($LASTEXITCODE -ne 0) {
        Write-Error "git subtree split failed for $Path"
    }
    Write-Host "Done. Push example: git remote add <name> <url> ; git push <name> $BranchName"
}

$splits = @(
    @{ Path = "BYTE_VA";        Branch = "split-BYTE_VA" },
    @{ Path = "BYTE_VISION";    Branch = "split-BYTE_VISION" },
    @{ Path = "extra_files";   Branch = "split-extra_files" }
)

foreach ($s in $splits) {
    if ($Prefix -ne "" -and $s.Path -ne $Prefix) { continue }
    Split-One -Path $s.Path -BranchName $s.Branch
}

Write-Host "Finished subtree splits."
