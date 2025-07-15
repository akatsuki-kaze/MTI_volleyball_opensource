@echo off
chcp 65001 > nul
cd /d "%~dp0"
echo 当前目录:%cd%
echo 正在运行 d415.py...
python d415.py
if %errorlevel% neq 0 (
    echo 执行过程中出现错误,错误码:%errorlevel%
) else (
    echo d415.py 已成功运行完毕。
)