@echo off
setlocal enabledelayedexpansion

REM 设置默认分支为 master
set target_branch=master

REM 检查是否传入分支参数
if "%~1" neq "" (
    set target_branch=%1
)

REM 推送到 origin 远程仓库
echo 正在推送到 origin 的 %target_branch% 分支...
git push -u origin %target_branch%

REM 推送到 origithub 远程仓库
echo 正在推送到 origithub 的 %target_branch% 分支...
git push -u origithub %target_branch%

:: pause