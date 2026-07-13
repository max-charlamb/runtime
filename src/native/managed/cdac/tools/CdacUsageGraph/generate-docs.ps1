#!/usr/bin/env pwsh
# Generate the "Data descriptors used" and "Contracts used" tables in
# docs/design/datacontracts/*.md from the CdacUsageGraph tool output, merging in
# meanings from data-descriptor-meanings.json.
#
#   ./generate-docs.ps1          # rewrite marked blocks in place
#   ./generate-docs.ps1 -Check   # fail (exit 1) if any doc would change (for CI)
#
# Marked regions look like:
#   <!-- BEGIN GENERATED: data-descriptors contract=Thread version=c1 -->
#   ...table...
#   <!-- END GENERATED: data-descriptors contract=Thread version=c1 -->
#   <!-- BEGIN GENERATED: contracts-used contract=Thread version=c1 -->
#   ...table...
#   <!-- END GENERATED: contracts-used contract=Thread version=c1 -->
param(
    [switch]$Check,
    [string]$DocsDir  = (Join-Path $PSScriptRoot "..\..\..\..\..\..\docs\design\datacontracts"),
    [string]$Json     = (Join-Path $PSScriptRoot "output\contract-usage.json"),
    [string]$Meanings = (Join-Path $PSScriptRoot "..\..\..\..\..\..\docs\design\datacontracts\data-descriptor-meanings.json")
)

$usage = Get-Content $Json -Raw | ConvertFrom-Json
$means = @{}
if (Test-Path $Meanings) { $means = Get-Content $Meanings -Raw | ConvertFrom-Json -AsHashtable }

function Get-Entry($contractShort, $version) {
    return $usage | Where-Object { $_.contract -eq "I$contractShort" -and $_.version -eq $version } | Select-Object -First 1
}

function Meaning($contractShort, $key) {
    if ($means.ContainsKey($contractShort) -and $means[$contractShort].ContainsKey($key)) {
        return $means[$contractShort][$key]
    }
    return "_TODO: describe_"
}

function Build-DataDescriptors($contractShort, $version) {
    $entry = Get-Entry $contractShort $version
    $set = [System.Collections.Generic.HashSet[string]]::new()   # "Type.Field"
    if ($entry -and $entry.fieldUsage) {
        foreach ($p in $entry.fieldUsage.PSObject.Properties) {
            $type = $p.Name -replace '^Data\.',''
            foreach ($fld in $p.Value.PSObject.Properties.Name) { [void]$set.Add("$type.$fld") }
        }
    }
    # supplement / suppress
    if ($means.ContainsKey('_supplement') -and $means['_supplement'].ContainsKey($contractShort)) {
        foreach ($k in $means['_supplement'][$contractShort]) { [void]$set.Add($k) }
    }
    if ($means.ContainsKey('_suppress') -and $means['_suppress'].ContainsKey($contractShort)) {
        foreach ($k in $means['_suppress'][$contractShort]) { [void]$set.Remove($k) }
    }
    $rows = $set | ForEach-Object {
        $i = $_.IndexOf('.'); [pscustomobject]@{ Type = $_.Substring(0,$i); Field = $_.Substring($i+1); Key = $_ }
    } | Sort-Object Type, Field
    $sb = [System.Collections.Generic.List[string]]::new()
    $sb.Add('| Data Descriptor Name | Field | Meaning |')
    $sb.Add('| --- | --- | --- |')
    foreach ($r in $rows) {
        $sb.Add("| ``$($r.Type)`` | ``$($r.Field)`` | $(Meaning $contractShort $r.Key) |")
    }
    return $sb
}

function Build-ContractsUsed($contractShort, $version) {
    $entry = Get-Entry $contractShort $version
    $list = @()
    if ($entry -and $entry.contractsUsed) { $list = $entry.contractsUsed | Sort-Object }
    $sb = [System.Collections.Generic.List[string]]::new()
    $sb.Add('| Contract Name |')
    $sb.Add('| --- |')
    foreach ($x in $list) { $sb.Add("| ``$x`` |") }
    return $sb
}

$blockRe = '(?s)<!-- BEGIN GENERATED: (?<kind>[\w-]+) contract=(?<c>\w+) version=(?<v>\w+) -->.*?<!-- END GENERATED: \k<kind> contract=\k<c> version=\k<v> -->'

$drift = @()
$changed = @()
foreach ($md in Get-ChildItem $DocsDir -Filter *.md) {
    $text = Get-Content $md.FullName -Raw
    if ($text -notmatch 'BEGIN GENERATED: (data-descriptors|contracts-used)') { continue }
    $nl = if ($text -match "`r`n") { "`r`n" } else { "`n" }

    $evaluator = {
        param($m)
        $kind = $m.Groups['kind'].Value; $c = $m.Groups['c'].Value; $v = $m.Groups['v'].Value
        $begin = "<!-- BEGIN GENERATED: $kind contract=$c version=$v -->"
        $end   = "<!-- END GENERATED: $kind contract=$c version=$v -->"
        $table = switch ($kind) {
            'data-descriptors' { Build-DataDescriptors $c $v }
            'contracts-used'   { Build-ContractsUsed $c $v }
            default            { @() }
        }
        return ($begin, ($table -join $nl), $end) -join $nl
    }
    $new = [regex]::Replace($text, $blockRe, $evaluator)

    if ($new -ne $text) {
        if ($Check) { $drift += $md.Name }
        else { Set-Content -Path $md.FullName -Value $new -NoNewline; $changed += $md.Name }
    }
}

if ($Check) {
    if ($drift.Count -gt 0) {
        Write-Error ("Data-descriptor docs are stale for: {0}`nRun: generate-docs.ps1 (without -Check) to update." -f ($drift -join ', '))
        exit 1
    }
    Write-Output "Docs are up to date."
} else {
    if ($changed.Count -gt 0) { Write-Output ("Updated: {0}" -f ($changed -join ', ')) }
    else { Write-Output "No changes." }
}
