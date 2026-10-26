param([switch]$Demo)
$ErrorActionPreference = 'Stop'
# Optional convenience launcher. The HTML itself is the complete application.
$consolePage = Join-Path $PSScriptRoot 'index.html'
if ($Demo) { $consolePage = ([Uri]$consolePage).AbsoluteUri + '#demo' }
Start-Process -FilePath $consolePage
