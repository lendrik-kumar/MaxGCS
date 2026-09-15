@echo off
REM ==================================================================================
REM  Kite Ground Control (MaxGCS) - Windows Zero-Touch Deploy
REM ==================================================================================
REM  Purpose
REM    Turn a brand-new Windows laptop (nothing installed - no Node, no Rust, no
REM    Visual Studio, nothing) into one that (a) builds this app from source and
REM    (b) opens Kite Ground Control automatically every time someone logs in.
REM
REM  How to use
REM    1. Copy the WHOLE project folder onto the laptop (USB drive, network share,
REM       `git clone`, whatever - this script builds from source, it does not
REM       download the project itself).
REM    2. Right-click this file -> "Run as administrator" (or just double-click it;
REM       it will re-launch itself elevated and ask for a UAC prompt if needed).
REM    3. Wait. On a completely fresh machine the Visual Studio Build Tools step
REM       alone can take 10-20+ minutes (multi-GB download) - this is normal.
REM    4. When it finishes, the app launches once immediately so you can confirm
REM       it deployed correctly, and it will now auto-launch at every future logon.
REM
REM  What this script installs/changes on the machine (all via winget, all silent)
REM    - Node.js LTS                          (OpenJS.NodeJS.LTS)
REM    - Rust via rustup                      (Rustlang.Rustup)
REM    - Visual Studio 2022 Build Tools + the C++ (VCTools) workload
REM                                            (Microsoft.VisualStudio.2022.BuildTools)
REM    - Microsoft Edge WebView2 Runtime       (Microsoft.EdgeWebView2Runtime)
REM    - Copies the built kite-gc.exe (+ a `.portable` marker so its data folder
REM      stays right beside it) into: %LOCALAPPDATA%\Programs\KiteGC\
REM      (deliberately a per-user, user-writable folder - NOT Program Files, which
REM      a standard user cannot write into, and this app writes its own data next
REM      to the exe in portable mode)
REM    - A Scheduled Task named "KiteGC AutoStart" (Trigger: at log on, for the
REM      user who ran this script; no elevation required to run the app itself)
REM    - A matching shortcut in that user's Startup folder, as a second, independent
REM      auto-start mechanism (belt-and-braces: still works even if the Scheduled
REM      Task were ever deleted or disabled by hand)
REM
REM  Not done automatically (on purpose - this touches Windows sign-in security)
REM    "Auto-start at logon" still needs someone to reach the desktop first. If you
REM    want the app visible the instant the laptop is powered on, with nobody
REM    typing a password, you ALSO need Windows auto-logon enabled for this
REM    account. That is a deliberate, separate, security-relevant step this script
REM    will not take for you silently. To do it yourself:
REM        netplwiz.exe -> uncheck "Users must enter a user name and password..."
REM    or, for a locked-down kiosk machine, the "Assigned Access" / kiosk mode
REM    feature under Settings > Accounts > Other users is the supported way to do
REM    this properly instead of storing a plaintext password in the registry.
REM
REM  To undo everything this script did
REM    schtasks /Delete /TN "KiteGC AutoStart" /F
REM    del "%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\KiteGC.lnk"
REM    rmdir /s /q "%LOCALAPPDATA%\Programs\KiteGC"
REM  (Node/Rust/VS Build Tools/WebView2 are left in place - they're normal
REM  development tools, not deployment artifacts, and other things may use them.)
REM
REM  Safe to re-run: every step below is idempotent (already-installed prerequisites
REM  are detected and skipped; the scheduled task and shortcut are recreated fresh).
REM ==================================================================================

setlocal EnableExtensions EnableDelayedExpansion
title Kite Ground Control - Windows Deploy

set "APP_TITLE=Kite Ground Control"
set "EXE_NAME=kite-gc.exe"
set "TASK_NAME=KiteGC AutoStart"
set "INSTALL_DIR=%LOCALAPPDATA%\Programs\KiteGC"
set "REPO_ROOT=%~dp0.."
pushd "%REPO_ROOT%" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Could not resolve the project root from "%~dp0..".
    pause
    exit /b 1
)
set "REPO_ROOT=%CD%"
popd >nul 2>&1
set "LOGFILE=%~dp0deploy-windows.log"

call :log "==================================================================="
call :log " %APP_TITLE% - Windows Zero-Touch Deploy   (%DATE% %TIME%)"
call :log "==================================================================="

REM --- Sanity check: are we actually sitting inside the project? -------------------
if not exist "%REPO_ROOT%\package.json" (
    call :log "[ERROR] No package.json found at %REPO_ROOT%."
    call :log "        Copy the WHOLE project folder onto this machine first, then run"
    call :log "        this script from inside its scripts\ folder."
    pause
    exit /b 1
)
if not exist "%REPO_ROOT%\src-tauri" (
    call :log "[ERROR] No src-tauri\ folder found at %REPO_ROOT% - this doesn't look like a complete copy of the project."
    pause
    exit /b 1
)

REM --- Elevate if needed -------------------------------------------------------------
net session >nul 2>&1
if not "%errorlevel%"=="0" (
    call :log "[INFO] Administrator rights are required - requesting elevation..."
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs" >nul 2>&1
    if errorlevel 1 (
        echo.
        echo This script must run elevated. Right-click deploy-windows.bat and choose
        echo "Run as administrator", or accept the UAC prompt when it appears.
        pause
    )
    exit /b
)

REM --- winget must exist --------------------------------------------------------------
where winget >nul 2>&1
if errorlevel 1 (
    call :log "[ERROR] winget (Windows Package Manager) was not found on this machine."
    call :log "        Install 'App Installer' from the Microsoft Store, then re-run this script:"
    call :log "        https://apps.microsoft.com/detail/9nblggh4nns1"
    pause
    exit /b 1
)

echo.
call :log "This will install Node.js, Rust, Visual Studio Build Tools and WebView2 if"
call :log "they are missing, then build and deploy %APP_TITLE%. On a fresh machine this"
call :log "can take 15-30+ minutes (Build Tools alone is a multi-GB download) - please"
call :log "leave this window open."
echo.

REM ==================================================================================
REM  [1/6] Node.js LTS
REM ==================================================================================
call :log "[1/6] Node.js (LTS) ..."
where node >nul 2>&1
if errorlevel 1 (
    call :log "      not found - installing via winget..."
    winget install --id OpenJS.NodeJS.LTS -e --silent --accept-package-agreements --accept-source-agreements
) else (
    call :log "      already installed."
)
call :refreshpath
where node >nul 2>&1
if errorlevel 1 (
    call :log "[ERROR] node.js is still not on PATH after installing it. Aborting."
    call :log "        Try closing this window and re-running the script once more"
    call :log "        (a fresh PATH is sometimes only picked up on the next launch)."
    pause
    exit /b 1
)
for /f "delims=" %%V in ('node -v') do call :log "      node %%V"

REM ==================================================================================
REM  [2/6] Rust (rustup + the MSVC stable toolchain)
REM ==================================================================================
call :log "[2/6] Rust (rustup) ..."
where cargo >nul 2>&1
if errorlevel 1 (
    call :log "      not found - installing via winget..."
    winget install --id Rustlang.Rustup -e --silent --accept-package-agreements --accept-source-agreements
    call :refreshpath
) else (
    call :log "      already installed."
)
where rustup >nul 2>&1
if not errorlevel 1 (
    call :log "      ensuring the stable-x86_64-pc-windows-msvc toolchain is installed and default..."
    rustup toolchain install stable-x86_64-pc-windows-msvc -q
    rustup default stable-x86_64-pc-windows-msvc >nul 2>&1
)
call :refreshpath
where cargo >nul 2>&1
if errorlevel 1 (
    call :log "[ERROR] cargo is still not on PATH after installing Rust. Aborting."
    pause
    exit /b 1
)
for /f "delims=" %%V in ('cargo --version') do call :log "      %%V"

REM ==================================================================================
REM  [3/6] Visual Studio Build Tools + the C++ (VCTools) workload - this is the slow one
REM ==================================================================================
call :log "[3/6] Visual Studio Build Tools (MSVC linker + Windows SDK) ..."
call :log "      This step alone can take 10-20+ minutes on a fresh machine. Do not close this window."
winget install --id Microsoft.VisualStudio.2022.BuildTools -e --silent --accept-package-agreements --accept-source-agreements --override "--wait --quiet --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
call :log "      Build Tools step finished (a non-zero result here usually just means it was already installed)."

REM ==================================================================================
REM  [4/6] WebView2 Runtime (usually already present on Win10 21H2+/Win11, but make sure)
REM ==================================================================================
call :log "[4/6] Microsoft Edge WebView2 Runtime ..."
winget install --id Microsoft.EdgeWebView2Runtime -e --silent --accept-package-agreements --accept-source-agreements
call :log "      WebView2 step finished."

REM ==================================================================================
REM  [5/6] Build the app (delegates to the project's own build-windows.ps1, which
REM        also handles the OneDrive CARGO_TARGET_DIR workaround and collect-release)
REM ==================================================================================
call :log "[5/6] Building %APP_TITLE% (npm install + tauri build - several minutes on first run)..."
pushd "%REPO_ROOT%" >nul 2>&1
call powershell -NoProfile -ExecutionPolicy Bypass -File "scripts\build-windows.ps1"
set "BUILD_RC=%errorlevel%"
popd >nul 2>&1
if not "%BUILD_RC%"=="0" (
    call :log "[ERROR] The build failed (exit code %BUILD_RC%). See the output above for details."
    pause
    exit /b 1
)

REM Locate the built exe. build-windows.ps1 redirects to D:\cargo-target\kite-gc when the
REM project lives under a OneDrive-synced path (cargo dislikes that) - check both spots.
set "BUILT_EXE=%REPO_ROOT%\src-tauri\target\release\%EXE_NAME%"
if not exist "%BUILT_EXE%" set "BUILT_EXE=D:\cargo-target\kite-gc\release\%EXE_NAME%"
if not exist "%BUILT_EXE%" (
    call :log "[ERROR] Could not find %EXE_NAME% after a successful-looking build. Looked in:"
    call :log "          %REPO_ROOT%\src-tauri\target\release\"
    call :log "          D:\cargo-target\kite-gc\release\   (OneDrive workaround path)"
    pause
    exit /b 1
)
call :log "      Built: %BUILT_EXE%"

REM ==================================================================================
REM  [6/6] Install to a user-writable folder + register auto-start (Scheduled Task
REM        AND a Startup-folder shortcut, for redundancy)
REM ==================================================================================
call :log "[6/6] Installing to %INSTALL_DIR% and registering auto-start..."
if not exist "%INSTALL_DIR%" mkdir "%INSTALL_DIR%"
copy /Y "%BUILT_EXE%" "%INSTALL_DIR%\%EXE_NAME%" >nul
if errorlevel 1 (
    call :log "[ERROR] Failed to copy %EXE_NAME% into %INSTALL_DIR%."
    pause
    exit /b 1
)
REM The .portable marker tells the app to keep its own data/settings in a data\ folder
REM right beside the exe, instead of %APPDATA% - see scripts\collect-release.ps1.
type nul > "%INSTALL_DIR%\.portable"

schtasks /Query /TN "%TASK_NAME%" >nul 2>&1
if not errorlevel 1 schtasks /Delete /TN "%TASK_NAME%" /F >nul 2>&1
schtasks /Create /TN "%TASK_NAME%" /SC ONLOGON /RU "%USERNAME%" /RL LIMITED /TR "\"%INSTALL_DIR%\%EXE_NAME%\"" /F >nul
if errorlevel 1 (
    call :log "[ERROR] Failed to register the '%TASK_NAME%' scheduled task."
    pause
    exit /b 1
)
call :log "      Scheduled task '%TASK_NAME%' created - runs %EXE_NAME% at every logon for %USERNAME%."

set "STARTUP_LNK=%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\KiteGC.lnk"
powershell -NoProfile -Command "try { $s = (New-Object -ComObject WScript.Shell).CreateShortcut('%STARTUP_LNK%'); $s.TargetPath = '%INSTALL_DIR%\%EXE_NAME%'; $s.WorkingDirectory = '%INSTALL_DIR%'; $s.Save() } catch { exit 1 }" >nul 2>&1
if errorlevel 1 (
    call :log "[WARN] Could not create the Startup-folder shortcut (non-fatal - the scheduled task still covers auto-start)."
) else (
    call :log "      Startup shortcut created at: %STARTUP_LNK%"
)

call :log ""
call :log "==================================================================="
call :log " Deploy complete."
call :log "==================================================================="
call :log " Installed to : %INSTALL_DIR%\%EXE_NAME%"
call :log " Auto-starts  : at every logon for '%USERNAME%' (Scheduled Task + Startup shortcut)"
call :log ""
call :log " NOTE: this makes the app open once someone reaches the desktop. If you"
call :log " want it visible the instant the laptop powers on with no one typing a"
call :log " password, you still need to enable Windows auto-logon yourself (see the"
call :log " comment block at the top of this script for how, and why that's a"
call :log " separate manual step)."
call :log "==================================================================="
echo.
call :log "Launching %APP_TITLE% now to confirm the deployment..."
start "" "%INSTALL_DIR%\%EXE_NAME%"

echo.
pause
exit /b 0

REM ==================================================================================
REM  Subroutines
REM ==================================================================================

:log
echo(%~1
>> "%LOGFILE%" echo(%~1
exit /b 0

:refreshpath
REM Pulls PATH from HKLM (system) + HKCU (user) fresh out of the registry so this
REM already-running cmd session sees tools an installer just added, without needing
REM to close and reopen the window. Also appends the well-known default install
REM locations for cargo/node as a second line of defence in case a registry read
REM above raced the installer's own write.
set "SYS_PATH="
set "USR_PATH="
for /f "tokens=1,2,*" %%A in ('reg query "HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment" /v Path 2^>nul') do if /i "%%A"=="Path" set "SYS_PATH=%%C"
for /f "tokens=1,2,*" %%A in ('reg query "HKCU\Environment" /v Path 2^>nul') do if /i "%%A"=="Path" set "USR_PATH=%%C"
REM The registry's raw Path value is REG_EXPAND_SZ and can contain literal, unexpanded tokens
REM like "%SystemRoot%" - a plain "set PATH=%SYS_PATH%..." would bake that literal text into
REM PATH instead of resolving it. `call set` forces a second parsing pass, which expands them.
call set "SYS_PATH=%SYS_PATH%"
call set "USR_PATH=%USR_PATH%"
set "PATH=%SYS_PATH%;%USR_PATH%;%USERPROFILE%\.cargo\bin;%ProgramFiles%\nodejs;%LOCALAPPDATA%\Programs\nodejs"
exit /b 0
