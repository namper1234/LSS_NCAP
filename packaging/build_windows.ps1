param([string]$Python = "python")
$ErrorActionPreference = "Stop"
Set-Location (Resolve-Path (Join-Path $PSScriptRoot ".."))

& $Python -m PyInstaller --noconfirm --clean --distpath dist --workpath build/pyinstaller LSS_NCAP_Tool.spec
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed" }
& $Python packaging/zip_release.py --platform windows
if ($LASTEXITCODE -ne 0) { throw "ZIP packaging failed" }
& $Python packaging/smoke_release.py dist/LSS_NCAP_Tool-windows-x86_64.zip --gui
if ($LASTEXITCODE -ne 0) { throw "Release smoke checks failed" }
