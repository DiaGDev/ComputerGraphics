SETLOCAL
cd ..
powershell -command "if(Test-Path OpenSceneGraph-3.6.5-VC2022-64-Debug\include){Write-Host 'osg-exists'}else{Write-Host 'installing OSG - will take some time';Start-BitsTransfer -Source https://objexx.com/OpenSceneGraph/OpenSceneGraph-3.6.5-VC2022-64-Debug.7z -Destination folder.7z;set-alias sz $env:ProgramFiles\7-Zip\7z.exe; sz x folder.7z;Remove-Item folder.7z}"
powershell -command "if(Test-Path OpenSceneGraph-Data){Write-Host 'osg-data-exists'}else{Write-Host 'installing OSG Data - will take some time';Start-BitsTransfer -Source https://www.openscenegraph.com/downloads/stable_releases/OpenSceneGraph-3.4.0/data/OpenSceneGraph-Data-3.4.0.zip -Destination data.7z;set-alias sz $env:ProgramFiles\7-Zip\7z.exe; sz x data.7z;Remove-Item data.7z}"
ENDLOCAL