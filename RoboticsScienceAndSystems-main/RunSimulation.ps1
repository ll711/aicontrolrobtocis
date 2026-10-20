param(
    [string]$ProcessingRoot = (Join-Path $PSScriptRoot '.tools/processing4/Processing'),
    [switch]$BuildOnly
)
$ErrorActionPreference = 'Stop'
$processingLocation = (Resolve-Path -LiteralPath $ProcessingRoot).Path
$processingResources = Join-Path $processingLocation 'app/resources'
$processingJava = Join-Path $processingResources 'jdk/bin/java.exe'
if (-not (Test-Path -LiteralPath $processingJava)) {
    throw 'Expected Processing 4 distribution with app/resources/jdk. Pass -ProcessingRoot PATH.'
}
$processingBuild = Join-Path $PSScriptRoot ('build-host/processing-' + [Guid]::NewGuid().ToString('N'))
$processingAction = if ($BuildOnly) { '--build' } else { '--run' }
& $processingJava "-Dcompose.application.resources.dir=$processingResources" '-Dprocessing.version=4.5.7' `
    -cp (Join-Path $processingLocation 'app/*') processing.mode.java.Commander `
    "--sketch=$(Join-Path $PSScriptRoot 'DigitalTwin')" "--output=$processingBuild" $processingAction
if ($LASTEXITCODE -ne 0) { throw "Processing failed with exit code $LASTEXITCODE" }
Write-Output "Processing build: $processingBuild"
