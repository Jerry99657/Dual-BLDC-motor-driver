param([switch]$Demo)
$ErrorActionPreference = 'Stop'
$consoleRoot = $PSScriptRoot
Push-Location $consoleRoot
try {
    python -c "import importlib.util,sys; sys.exit(0 if importlib.util.find_spec('serial') else 1)"
    if ($LASTEXITCODE -ne 0) {
        Write-Host 'Install dependencies: python -m pip install -r pc_tool/requirements.txt'
        exit 1
    }
    if ($Demo) { python server.py --demo } else { python server.py }
} finally { Pop-Location }
