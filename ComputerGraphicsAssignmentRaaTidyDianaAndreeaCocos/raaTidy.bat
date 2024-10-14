rmdir /S /Q .\.vs
rmdir /S /Q .\x64
rmdir /S /Q .\OpenSceneGraph-3.6.5-VC2022-64-Debug
rmdir /S /Q .\OpenSceneGraph-Data

call :treeProc
goto :eof

:treeProc

for /D %%d in (*) do (

    rmdir /S /Q .vs .\x64

    cd %%d
    call :treeProc

)


exit /b