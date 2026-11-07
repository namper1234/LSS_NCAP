param([string]$Python = "python")
$ErrorActionPreference = "Stop"
Set-Location (Resolve-Path (Join-Path $PSScriptRoot ".."))

& $Python -c 'import sys; assert sys.version_info >= (3, 11) and sys.maxsize > 2**32, "64-bit Python 3.11+ is required"'
if ($LASTEXITCODE -ne 0) { throw "A 64-bit Python 3.11+ interpreter is required" }

& $Python -m venv .venv
if ($LASTEXITCODE -ne 0) { throw "Virtual environment creation failed" }
$BuildPython = Join-Path (Get-Location) ".venv\Scripts\python.exe"

& $BuildPython -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { throw "pip upgrade failed" }
& $BuildPython -m pip install -e . -r packaging/build-requirements.txt
if ($LASTEXITCODE -ne 0) { throw "Build dependency installation failed" }

& $BuildPython -m unittest discover -s tests -v
if ($LASTEXITCODE -ne 0) { throw "Unit tests failed" }

& $BuildPython -m PyInstaller --noconfirm --clean --distpath dist --workpath build/pyinstaller LSS_NCAP_Tool.spec
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed" }
$ArchiveOutput = @(& $BuildPython packaging/zip_release.py --platform windows)
if ($LASTEXITCODE -ne 0) { throw "ZIP packaging failed" }
$Archive = $ArchiveOutput[-1]
& $BuildPython packaging/smoke_release.py $Archive --gui
if ($LASTEXITCODE -ne 0) { throw "Release smoke checks failed" }

Write-Host "Windows executable: $(Join-Path (Get-Location) 'dist\LSS_NCAP_Tool\LSS_NCAP_Tool.exe')"
Write-Host "Portable ZIP: $Archive"
