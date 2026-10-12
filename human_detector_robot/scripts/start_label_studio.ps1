# Label Studio start karo (Windows — PATH fix ke saath)
# Usage: powershell -ExecutionPolicy Bypass -File scripts/start_label_studio.ps1

$scriptsPath = "$env:APPDATA\Python\Python313\Scripts"

if (-not (Test-Path $scriptsPath)) {
    # Python version alag ho to Scripts folder dhoondo
    $fallback = Get-ChildItem "$env:APPDATA\Python\Python*\Scripts\label-studio.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($fallback) {
        $scriptsPath = $fallback.DirectoryName
    } else {
        Write-Host "[ERROR] label-studio.exe nahi mila."
        Write-Host "Pehle install karo: python -m pip install label-studio"
        exit 1
    }
}

if ($env:Path -notlike "*$scriptsPath*") {
    $env:Path += ";$scriptsPath"
    Write-Host "[OK] PATH mein add kiya: $scriptsPath"
}

Write-Host "[START] Label Studio khul raha hai..."
Write-Host "Browser: http://localhost:8080"
Write-Host "Band karne ke liye: Ctrl+C"
Write-Host ""

& "$scriptsPath\label-studio.exe" start
