powershell -command "if(Test-Path OpenSceneGraph-3.6.5-VC2022-64-Debug) {Write-Host 'osg-exists'}else {Start-BitsTransfer -Source https://objexx.com/OpenSceneGraph/OpenSceneGraph-3.6.5-VC2022-64-Debug.7z -Destination folder.7z; set-alias sz $env:ProgramFiles\7-Zip\7z.exe; sz x folder.7z}"

