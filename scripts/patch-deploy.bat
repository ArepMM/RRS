rem Копируем бинарные файлы

xcopy /Y  ..\..\bin\*.exe %RRS_DEV_ROOT%\bin\
xcopy /Y ..\..\lib\*.dll %RRS_DEV_ROOT%\bin\
move %RRS_DEV_ROOT%\bin\rkf5.dll %RRS_DEV_ROOT%\lib\rkf5.dll
move %RRS_DEV_ROOT%\bin\rk4.dll %RRS_DEV_ROOT%\lib\rk4.dll
move %RRS_DEV_ROOT%\bin\euler2.dll %RRS_DEV_ROOT%\lib\euler2.dll
move %RRS_DEV_ROOT%\bin\euler.dll %RRS_DEV_ROOT%\lib\euler.dll

xcopy /Y  ..\..\modules\*.dll %RRS_DEV_ROOT%\modules
xcopy /Y  ..\..\modules\freightcar\*.dll %RRS_DEV_ROOT%\modules\freightcar\
xcopy /Y  ..\..\modules\passcar\*.dll %RRS_DEV_ROOT%\modules\passcar\
xcopy /Y  ..\..\modules\vl60k\*.dll %RRS_DEV_ROOT%\modules\vl60k\
xcopy /Y  ..\..\modules\vl60pk\*.dll %RRS_DEV_ROOT%\modules\vl60pk\

xcopy /Y  ..\..\plugins\*.dll %RRS_DEV_ROOT%\plugins

rem Копируем конфиги

xcopy /Y ..\cfg\*.xml %RRS_DEV_ROOT%\cfg\
del %RRS_DEV_ROOT%\cfg\control-panel.xml
xcopy /Y ..\cfg\devices\*.xml %RRS_DEV_ROOT%\cfg\devices\
xcopy /Y ..\cfg\main-resist\*.xml %RRS_DEV_ROOT%\cfg\main-resist\
xcopy /Y ..\cfg\wheel-rail-friction\*.xml %RRS_DEV_ROOT%\cfg\wheel-rail-friction\

xcopy /Y ..\cfg\vehicles\vl60pk-1543\*.* %RRS_DEV_ROOT%\cfg\vehicles\vl60pk-1543\
xcopy /Y ..\cfg\vehicles\VL60k-1737\*.* %RRS_DEV_ROOT%\cfg\vehicles\VL60k-1737\
xcopy /Y ..\cfg\vehicles\IMR_pass_rzd-11100\*.* %RRS_DEV_ROOT%\cfg\vehicles\IMR_pass_rzd-11100\
xcopy /Y ..\cfg\vehicles\IMR_pass_rzd-13819\*.* %RRS_DEV_ROOT%\cfg\vehicles\IMR_pass_rzd-13819\
xcopy /Y ..\cfg\vehicles\IMR_pass_rzd-16733\*.* %RRS_DEV_ROOT%\cfg\vehicles\IMR_pass_rzd-16733\
xcopy /Y ..\cfg\vehicles\IMR_pass_rzd-17669\*.* %RRS_DEV_ROOT%\cfg\vehicles\IMR_pass_rzd-17669\
xcopy /Y ..\cfg\vehicles\IMR_pass_rzd-21071\*.* %RRS_DEV_ROOT%\cfg\vehicles\IMR_pass_rzd-21071\
xcopy /Y ..\cfg\vehicles\IMR_pass_rzd-25924\*.* %RRS_DEV_ROOT%\cfg\vehicles\IMR_pass_rzd-25924\
xcopy /Y ..\cfg\vehicles\IMR_pass_rzd-65361\*.* %RRS_DEV_ROOT%\cfg\vehicles\IMR_pass_rzd-65361\
xcopy /Y ..\cfg\vehicles\Fr_hopper_RZD-2851\*.* %RRS_DEV_ROOT%\cfg\vehicles\Fr_hopper_RZD-2851\

xcopy /Y ..\cfg\trains\vl60pk-1543.xml %RRS_DEV_ROOT%\cfg\trains\
xcopy /Y ..\cfg\trains\vl60pk-1543-T65_17.xml %RRS_DEV_ROOT%\cfg\trains\
xcopy /Y ..\cfg\trains\VL60k-1737.xml %RRS_DEV_ROOT%\cfg\trains\
xcopy /Y ..\cfg\trains\vl60k-1737-frEmpties.xml %RRS_DEV_ROOT%\cfg\trains\
xcopy /Y ..\cfg\trains\vl60k-1737-frLoads.xml %RRS_DEV_ROOT%\cfg\trains\

