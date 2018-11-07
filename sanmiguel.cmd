@echo off
set LOG=logsanmiguel.log

rem Find executable.

set EXE=rt_x64_Release.exe
if not exist %EXE% set EXE=rt.exe

%EXE% benchmark --log=%LOG% --mesh=scenes/rt/sanmiguel/sanmiguel.obj --sbvh-alpha=1.0e-6 --ao-radius=1.5 --kernel=kepler_dynamic_fetch --camera="Yciwz1oRQmz/Xvsm005CwjHx/b70nx18tVI7005frY108Y/:x/v3/z100" --camera="NhL2/2tO1w/0OIZh005DPZMz/xC9Cz18tVI7005frY108Y/:x/v3/z100" --camera="AbE3/0LWiZz/4Ccj005X5X1z1qJ13x/8BfRky/5frY108Y/:x/v3/z100"

:done
echo Done.
