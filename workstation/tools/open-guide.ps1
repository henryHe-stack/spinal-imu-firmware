$root=Split-Path -Parent $PSScriptRoot
Start-Process -FilePath (Join-Path $root 'README.html')
