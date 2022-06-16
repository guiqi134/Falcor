rem If running this gives "can't find model file XXXXXX", make sure to
rem (a) grab the appropriate model from p4research:2017 in //research/graphics/media/Main/
rem (b) tell Falcor where to find the model.  You can do this by setting the FALCOR_MEDIA_FOLDERS
rem     environment variable (e.g., to "F:\p4\workspace\graphics\media\Main" or similar) or by
rem     editing the Python script on the following line so the model file it loads is specified
rem     using absolute (rather than relative) paths
..\..\..\..\Bin\x64\Release\Mogwai.exe --script __arcade.py
