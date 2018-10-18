@echo off
set LOG=foo_benchmark.log

rem Find executable.

set EXE=rt_x64_Release_foo.exe
if not exist %EXE% set EXE=rt_x64_Release.exe
if not exist %EXE% set EXE=rt.exe

rem Benchmark sibenik, fairyforest, conference, and san miguel.

%EXE% benchmark --log=%LOG% --mesh=scenes/rt/sibenik/sibenik.obj --sbvh-alpha=1.0e-5 --ao-radius=5 --kernel=kepler_dynamic_fetch --camera="ytIa02G35kz1i:ZZ/0//iSay/5W6Ex19///c/05frY109Qx7w////m100" --camera=":Wp802ACAD/2x9OQ/0/waE8z/IOKbx/9///c/05frY109Qx7w////m100" --camera="CFtpy/s6ea/28btX/0172CFy/K5g1z/9///c/05frY109Qx7w////m100" --camera="steO/0TlN1z1tsDg/03InaMz/bqZxx/9///c/05frY109Qx7w////m100" --camera="HJv//034:Rx1S4Xh/03dpXux1BVmGw/9///c/05frY109Qx7w////m100"
%EXE% benchmark --log=%LOG% --mesh=scenes/rt/fairyforest/fairyforest.obj --sbvh-alpha=1.0e-5 --ao-radius=0.3 --kernel=kepler_dynamic_fetch --camera="cIxMx/sK/Ty/EFu3z/5m9mWx/YPA5z/8///m007toC10AnAHx///Uy200" --camera="KI/Qz/zlsUy/TTy6z13BdCZy/LRxzy/8///m007toC10AnAHx///Uy200" --camera="mF5Gz1SuO1z/ZMooz11Q0bGz/CCNxx18///m007toC10AnAHx///Uy200" --camera="vH7Jy19GSHx/YN45x//P2Wpx1MkhWy18///m007toC10AnAHx///Uy200" --camera="ViGsx/KxTFz/Ypn8/05TJTmx1ljevx18///m007toC10AnAHx///Uy200"
%EXE% benchmark --log=%LOG% --mesh=scenes/rt/conference/conference.obj --sbvh-alpha=1.0e-5 --ao-radius=5 --kernel=kepler_dynamic_fetch --camera="6omr/04j3200bR6Z/0/3ZEAz/x4smy19///c/05frY109Qx7w////m100" --camera="Lpmr/07k3200CS6Z/0/QqOIz1qfnsx19///c/05frY109Qx7w////m100" --camera="Y1BR00IkZd/0aA9X/0/Gy8Px1ca7Tw19///c/05frY109Qx7w////m100" --camera="XYDl00Gqv600byxY/00IQE4x/jN1jx/9///c/05frY109Qx7w////m100" --camera="w:ie00yxXX00ND1b/03TZ6qy1egt3x/9///c/05frY109Qx7w////m100"
%EXE% benchmark --log=%LOG% --mesh=scenes/rt/sanmiguel/sanmiguel.obj --sbvh-alpha=1.0e-6 --ao-radius=1.5 --kernel=kepler_dynamic_fetch --camera="Yciwz1oRQmz/Xvsm005CwjHx/b70nx18tVI7005frY108Y/:x/v3/z100" --camera="NhL2/2tO1w/0OIZh005DPZMz/xC9Cz18tVI7005frY108Y/:x/v3/z100" --camera="AbE3/0LWiZz/4Ccj005X5X1z1qJ13x/8BfRky/5frY108Y/:x/v3/z100"

:done
echo Done.
