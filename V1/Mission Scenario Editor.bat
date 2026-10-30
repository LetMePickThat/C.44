@echo off
rem Mission Scenario Editor -- double-click this file to start the scenario tool.
rem Drop a scenario file (.json) on it to open that scenario directly.
rem First start: missing Python packages are installed automatically (needs internet, one-off).
setlocal
cd /d "%~dp0"
set "TITLE=Mission Scenario Editor"

rem 1. A working Python 3 (python on PATH or Microsoft Store first, else the py launcher of python.org installs).
set "PY="
set "PYW="
python -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>&1 && (set "PY=python" & set "PYW=pythonw")
if not defined PY py -3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>&1 && (set "PY=py -3" & set "PYW=pyw -3")
if not defined PY (
    powershell -NoProfile -Command "Add-Type -AssemblyName PresentationFramework; [System.Windows.MessageBox]::Show('Python 3.11 or later is needed and was not found.' + [Environment]::NewLine + [Environment]::NewLine + 'Install it from the Microsoft Store (search Python) or from python.org (tick Add python.exe to PATH), then start again.', '%TITLE%', 'OK', 'Error')" >nul
    exit /b 1
)

rem 2. The packages of requirements.txt (installed for the current user on first start).
%PY% -c "import PyQt5, pyvista, pyvistaqt, numpy" >nul 2>&1
if errorlevel 1 (
    echo %TITLE%: first start, installing the required Python packages...
    echo.
    %PY% -m pip install --user -r requirements.txt
    if errorlevel 1 (
        echo.
        echo The installation failed: see the messages above.
        pause
        exit /b 1
    )
)

rem 3. A shortcut beside this file, with the tool's icon (a .bat cannot carry one); rewritten here, it follows the folder.
powershell -NoProfile -Command "$s = (New-Object -ComObject WScript.Shell).CreateShortcut('%~dp0Mission Scenario Editor.lnk'); $s.TargetPath = '%~f0'; $s.WorkingDirectory = '%~dp0'; $s.IconLocation = '%~dp0assets\icons\mission_scenario_editor.ico'; $s.Save()" >nul 2>&1

rem 4. Start without a console window (any scenario file dropped on this launcher is passed on).
start "" %PYW% "%~dp0scenario_tool.py" %*
exit /b 0
