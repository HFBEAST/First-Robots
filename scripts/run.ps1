$projectRoot = Split-Path -Parent $PSScriptRoot
$sourceRoot = Join-Path $projectRoot 'src'
$previousPythonPath = $env:PYTHONPATH

try {
    $env:PYTHONPATH = if ($previousPythonPath) {
        "$sourceRoot;$previousPythonPath"
    } else {
        $sourceRoot
    }
    & py -3 -m first_robots.cli @args
    exit $LASTEXITCODE
}
finally {
    $env:PYTHONPATH = $previousPythonPath
}
