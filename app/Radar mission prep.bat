@echo off
rem Radar mission preparation -- double-click this file to start the tool.
rem Drop a mission file (.json or .csv) on it to open that mission directly.
rem Missing Python packages: the user is told and asked; only on yes, the installation by pip.
setlocal
cd /d "%~dp0"
set "TITLE=Radar mission preparation"

rem 1. A working Python 3 (python on PATH or Microsoft Store first, else the py launcher of python.org installs).
set "PY="
set "PYW="
python -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>&1 && (set "PY=python" & set "PYW=pythonw")
if not defined PY py -3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>&1 && (set "PY=py -3" & set "PYW=pyw -3")
if not defined PY (
    powershell -NoProfile -Command "Add-Type -AssemblyName PresentationFramework; [System.Windows.MessageBox]::Show('Python 3.11 or later is needed and was not found.' + [Environment]::NewLine + [Environment]::NewLine + 'Install it from the Microsoft Store (search Python) or from python.org (tick Add python.exe to PATH), then start again.', '%TITLE%', 'OK', 'Error')" >nul
    exit /b 1
)

rem 2. The packages of requirements.txt: nothing is installed unless the user agrees, and pip is needed.
set "MISSING="
for /f "delims=" %%m in ('%PY% -c "import importlib.util as u; print(' '.join(m for m in ('numpy', 'scipy', 'PyQt5', 'pyvista', 'pyvistaqt') if u.find_spec(m) is None))"') do set "MISSING=%%m"
if defined MISSING (
    powershell -NoProfile -Command "Add-Type -AssemblyName PresentationFramework; $r = [System.Windows.MessageBox]::Show('These Python packages are missing: %MISSING%.' + [Environment]::NewLine + [Environment]::NewLine + 'The tool cannot start without them. Install them now? (needs internet)', '%TITLE%', 'YesNo', 'Question'); if ($r -eq 'Yes') { exit 0 } else { exit 1 }" >nul
    if errorlevel 1 exit /b 1
    %PY% -m pip --version >nul 2>&1
    if errorlevel 1 (
        powershell -NoProfile -Command "Add-Type -AssemblyName PresentationFramework; [System.Windows.MessageBox]::Show('pip is needed and was not found.' + [Environment]::NewLine + [Environment]::NewLine + 'Install it with the command: python -m ensurepip --upgrade' + [Environment]::NewLine + 'then start again.', '%TITLE%', 'OK', 'Error')" >nul
        exit /b 1
    )
    echo %TITLE%: installing the required Python packages...
    echo.
    %PY% -m pip install --user -r requirements.txt
    if errorlevel 1 (
        echo.
        echo The installation failed: see the messages above.
        pause
        exit /b 1
    )
)

rem 3. Start without a console window (any mission file dropped on this launcher is passed on).
start "" %PYW% "%~dp0main.py" %*
exit /b 0
