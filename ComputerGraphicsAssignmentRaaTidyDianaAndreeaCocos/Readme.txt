This is the template project for work in lab sessions and the assignment. It comtaines the following projects

Assignment(StaticLib)               - this is where all you classes should be implmented. If you wish to modify/extend 
                                    classes from other libraries these should be implmented here
Common(StaticLib)                   - Common tools that will be useful for the assignment. These will be explored in the 
                                    lab sessions (do NOT change or add classes to this)
Shader(StaticLib)                   - Useful tools for implmenting and managing shaders. (do NOT change or add classes to this)
ShaderViewer(Application)           - Contains the main function for the shader viewer. Set this as the startup project for the 
                                    shader viewer. You can add or modify this
TrafficSystem(StaticLib)            - Useful tools for the assignment traffic system. (do not change of add classes to this)
TrafficSystemViewer(Application)    - Contains the main function for the assignment template. You can modify or change this. 
                                    There are two versions of the main function: a simple viewer and one that uses the 
                                    additional tools I have provided. To switch between them comment/uncomment the '#simple'
                                    define on line 35.

You should select either startup project and build this. The libraries are linked dependencies and will rebuild as needed. 

The OSG library and data directory are downloaded and installed when the Common lib is built, if they do not already exist on the 
system. On first build, and any that need to re-download OSG this may mean the first compile will take some time. This requires 
7zip to be installed on the machine you are using. Please be patient.

In the solution dir you will find two additional directories
Data - this is not any 3D content files you wish to add to the project
shaders - this contains the source code for the rendering shaders. If you are implmenting your won add them here

While the source code and project files are quite small, and will compress into a zip file that can be submitted to moodle. The 
compilation files and OSG directory will make this repository very large. Before you zip up the project you should clean these. 
Using the clean build in visual studio will only remove the temporary project files. 

To fully reduce the size of the project directory for zipping up to save AND for assignment submission follow these steps:

1. Make sure all the source files are saved
2. For assignment submission, make sure the solution compiles and runs from within visual studio
3. Close Visual Studio
4. Open the solution directory in windows explorer (file viewer)
5. Run the 'raaTidy.bat' file - you can do this by double clicking on it or from the commandline/console
6. move up one diectory and zip the project solution dir

For the assignment submission check the zip file is less than 100Mb. If not you can remove the 'Shader' and ShaderViewer' 
projects from the solution (Re-open the proj in visual studio, remove the projects from the solution, close visual studio, 
manually delete these projects from the solution dir). Follow steps 2-6 to rezip

Please note: I cannot mark any projects that will not unzip, open in visual studio and compile/run from within visual studio 
(ie press the big green arrow). Check before you submit.

If you have incomplete code, or code that does not compile or run, but want this to be considered in the assignment marking:
1. comment it out - I am quite happy to read through it. In some cases I may fix it before marking
2. Add some comments to explin what you were trying to do and what you think the problem is causing this to not compile or run 