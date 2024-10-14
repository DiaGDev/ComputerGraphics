@echo raaCF %1 %2 %3
@echo Copy Headers to %3\include\%2
@mkdir %3\include\%2
@copy *.h %3\include\%2\.

@ goto %1

:debugdll
@copy ..\x64\debug\%2.lib %3\lib\.
@copy ..\x64\debug\%2.dll %3\bin\.
@goto end

:releasedll
@copy ..\x64\release\%2.lib %3\lib\.
@copy ..\x64\release\%2.dll %3\bin\.
@goto end

:debugexe
@copy ..\x64\debug\%2.exe %3\bin\.
@goto end

:releaseexe
@copy ..\x64\release\%2.exe %3\bin\.
@goto end

:debuglib
@copy ..\x64\debug\%2.lib %3\lib\.
@goto end

:releaselib
@copy ..\x64\release\%2.lib %3\lib\.

:end
@echo raaCF %1 Complete