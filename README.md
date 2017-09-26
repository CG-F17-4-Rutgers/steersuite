# Group 4 Steersuite Repository

## Members
Taichi Aritomo (ta340), Justin Wu (jw879), Michael Parrilla (map565), Kian Jackson (kmj129)


## Setup
1. **Clone** this repository
2. **Download premake.exe** from https://sourceforge.net/projects/premake/files/Premake/4.4/premake-4.4-beta5-windows.zip/download
3. Place premake.exe in build/
4. In Command Prompt, navigate to build/ and **run premake**:
   - cd *path_to_steersuite*/build
   - premake4 --platform=x64 vs2012
5. You should now have a folder in build/ named "vs2012". This folder is excluded from push/pull (see [.gitignore](.gitignore))
6. **Open steersuite.sln** from the vs2012 folder, in Visual Studio. Upgrade if prompted.
7. In the menu, go to Build > Configuration Manager. Set the *Active solution platform* to x64.
8. On the right side, you'll see the Solution Explorer. Right-click *Solution steersuite*, and select *Properties*. On the left-hand side menu, select *Configuration Properties*. Set *Configuration* to Active(Release) and *Platform* to x64.
9. For the projects *steerlib*, *util*, *steersim*, and *steersimlib*, right-click on each project and go to Properties > C/C++ > All Options and set *Smaller Type Check* to NO.
10. **Build the solution** by right-clicking *Solution steersuite* > Build

You should now be able to run steersuite from build/bin, which is also excluded from push/pull.
You can try it by navigating to build/bin and running: steersim -testcase ../../testcases/3-squeeze.xml -ai pprAI

**Note about changing code**
When you make changes to the C++ code in, say, [steerlib/src/Curve.cpp](steerlib/src/Curve.cpp), those changes won't take effect until you Build the steersuite solution again in Visual Studio.


## Notes
Notes and code snippets: [Dropbox Paper Doc](https://paper.dropbox.com/doc/Steersuite-Assignments-4pWAWAmWmpfuaMozsfF6y?_tk=share_copylink)
