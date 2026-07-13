#!/usr/bin/env pwsh
# One-time helper: seed data-descriptor-meanings.json for a contract by porting
# the Meaning column from the existing hand-written table in the contract's .md,
# matching old (Type, Field) rows to the tool's native descriptor names by a
# normalized key. Unmatched native fields get a "_TODO: describe_" placeholder.
param(
    [Parameter(Mandatory)][string]$Contract,       # e.g. Thread
    [string]$Version = "c1",
    [string]$DocsDir = (Join-Path $PSScriptRoot "..\..\..\..\..\..\docs\design\datacontracts"),
    [string]$Json    = (Join-Path $PSScriptRoot "output\contract-usage.json"),
    [string]$Out     = (Join-Path $PSScriptRoot "..\..\..\..\..\..\docs\design\datacontracts\data-descriptor-meanings.json")
)

function NormKey($type, $field) {
    $t = $type
    if ($t -eq 'GCHeapSVR') { $t = 'GCHeap' }
    $t = [regex]::Replace($t, '_\d+$', '').ToLowerInvariant()
    $f = ($field -replace '^m_','' -replace '^_','' -replace '_','').ToLowerInvariant()
    return "$t.$f"
}

# 1. Parse existing doc table -> normalized (type.field) -> meaning
$md = Join-Path $DocsDir "$Contract.md"
$oldMeanings = @{}
$rowRe = '^\s*\|\s*`?(?<t>[A-Za-z_][A-Za-z0-9_]*)`?\s*\|\s*`?(?<f>[A-Za-z_][A-Za-z0-9_]*)`?\s*\|\s*(?<m>.*?)\s*\|\s*$'
foreach ($line in Get-Content $md) {
    $m = [regex]::Match($line, $rowRe)
    if ($m.Success -and $m.Groups['t'].Value -cmatch '^[A-Z]' -and $m.Groups['f'].Value -ne '---') {
        $k = NormKey $m.Groups['t'].Value $m.Groups['f'].Value
        if (-not $oldMeanings.ContainsKey($k)) { $oldMeanings[$k] = $m.Groups['m'].Value }
    }
}

# 2. Tool's native descriptor names for this contract/version
$data = Get-Content $Json -Raw | ConvertFrom-Json
$entry = $data | Where-Object { $_.contract -eq "I$Contract" -and $_.version -eq $Version }
if (-not $entry) { throw "No tool data for I$Contract $Version" }

$result = [ordered]@{}
$rows = @()
foreach ($p in $entry.fieldUsage.PSObject.Properties) {
    $type = $p.Name -replace '^Data\.',''
    foreach ($fld in $p.Value.PSObject.Properties.Name) { $rows += [pscustomobject]@{ Type = $type; Field = $fld } }
}
foreach ($r in ($rows | Sort-Object Type, Field)) {
    $key = "$($r.Type).$($r.Field)"
    $meaning = $oldMeanings[(NormKey $r.Type $r.Field)]
    if (-not $meaning) { $meaning = "_TODO: describe_" }
    $result[$key] = $meaning
}

# 3. Merge into existing sidecar (preserve other contracts)
$root = @{}
if (Test-Path $Out) { $root = Get-Content $Out -Raw | ConvertFrom-Json -AsHashtable }
$root[$Contract] = $result
$root | ConvertTo-Json -Depth 6 | Set-Content $Out
$todo = ($result.Values | Where-Object { $_ -eq '_TODO: describe_' }).Count
Write-Output "Seeded $Contract into $Out ($($result.Count) fields, $todo TODO)."
