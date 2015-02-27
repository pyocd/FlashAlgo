@echo off
:: Script to setup local envirnoment.
set CUR_PATH=%~dp0
set PYTHONPATH=%CUR_PATH%;%CUR_PATH%tools;%PYTHONPATH%
set PATH=%CUR_PATH%;%CUR_PATH%tools\;%PATH%
cmd /K cd %CD%