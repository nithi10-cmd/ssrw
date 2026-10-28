@echo off
rem ตรวจเครื่องอ่านบัตรประชาชน 30 วินาที (ปิดหน้าต่างแอป RehabGlove ก่อน)
cd /d "%~dp0"
chcp 65001 >nul
set PY=python
where python >nul 2>nul || set PY=py
%PY% thai_id_bridge.py --diag
pause
