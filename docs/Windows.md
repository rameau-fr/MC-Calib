# MC-Calib Installation for Windows Users
**By @Harvendois**  

This documentation is meant to guide the installation of the MC-Calib toolbox, specifically for Windows OS users. Therefore, it is recommended to the readers to first read through the original MC-Calib toolbox github README before going through any of the steps recorded in this documentation. Furthermore, this documentation is meant to guide users who are not familiar to C++ application installation through a step-by-step guideline.

## Setting up development environment in Visual Studio Code:
- Install Docker extension 
- Install Powershell extension 

*Note: user may also use powershell/CMD app directly*

## Installation Steps:

1. **Install Docker Desktop for Windows**  
   Windows 10 or above will be supported by WSL2 backend while any Windows below 10 should be using Hyper-V backend. If you are using Windows 8 or below, you should additionally turn on the Hyper-V and Containers Windows features. You can follow [Docker instructions](https://docs.docker.com/desktop/install/windows-install/) for this step.


2. **Download the MC-Calib repository from GitHub**  
   The repository can then be placed in a separate folder/directory that the user will later mount in docker to set as '/home' for the docker container run. Copy the absolute address of this Windows directory where the repository is located because it will be our `$(PWD)`.

3. **Pulling Docker Image**  
   Using Windows Powershell or CMD, we pull the docker images using the commands given in the README.  
   
   First, we move to the directory where our downloaded repository is located.  
   The command for this is:  
   `cd (copied absolute path)`  

   Then pull the docker image using either one of the commands given below.

```bash
docker pull bailool/mc-calib-prod:opencv410 # production environment
docker pull bailool/mc-calib-dev:opencv410 # development environment
```

4. **Running Pulled Image using Docker**  
In order to avoid running the image manually every time, we can create a `*.ps1` file containing the necessary docker run commands (or enter the commands manually in Windows Powershell or CMD). Below are the commands necessary. Set `PATH_TO_REPO_ROOT` and `PATH_TO_DATA` appropriately.


```bash
Docker run `
    -ti --rm `
    --volume=”$PATH_TO_REPO_ROOT:/home/MC-Calib” `
    --volume=”$PATH_TO_DATA:/home/MC-Calib/data” `
    bailool/mc-calib-prod:opencv410
```

### User Personalization

- `--volume=”$(PWD):/home/MC-Calib”` : 
Mounts a volume from the host machine to the docker container. `$(PWD)` refers to the current directory on the host machine (that the user is located in his/her powershell/cmd). Any data or files within that directory on the host machine will then be able to be accessed/mapped to `/home/MC-Calib` inside the Docker container. It is hence recommended that `*.ps1` file containing the command lines above is located in the exact Windows directory that the user intends to make as /home to docker container.

- `--volume=”PATH_TO_DATA:/home/MC-Calib/data”` : 
Another volume mapping. This line maps the necessary data in our Windows directory to the Docker directory specified above. It is recommended that the docker directory address is not changed.

However, it is important to set the appropriate `PATH_TO_DATA` to a correct directory that actually contains the images data that the user intends to calibrate with. While the location of the images are completely given as user’s freedom, the images are required to be contained in a certain prefixed directory within the chosen location. Depending on how many cameras we have, we separate each camera’s images into different subdirectories named `Cam_001`, `Cam_002`, and so forth. The prefix (`Cam_`) is essential.

For example, if we choose to save our images in `D:\project\calibration\test\images` for 2 cameras, we create two subdirectories as follows:

```bash
D:\project\calibration\test\images\Cam_001
D:\project\calibration\test\images\Cam_002
```
After personalization, user can run the '*.ps1' file in the Powershell/CMD/etc.

```bash
.\calib.ps1
```

## Compiling the MC-Calib repository

Once we are in the docker container, we are ready to compile the MC-Calib repository. We will utilize CMake to link our files together and make a compile-ready object file for us.

First, head to `/home/MC-Calib` directory, where the user should already have placed the downloaded github repository of our toolbox.

As it is a convention in compiling and building applications using CMake, we create the build directory and start compiling in it.

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j10
```

If we need to recompile after some revisions, we can repeat `make -j10` in the build directory again.

## Testing charuco board generation

In `/home/MC-Calib/build` directory, using the command below should generate and save 7 charuco boards.

```bash
./apps/create_charuco_boards/generate_charuco ../configs/Real_images/calib_param_Seq01_Non-overlapping.yml
```

The generated charuco boards can be found in the Windows directory `\build\apps\create_charuco`.

Once you can confirm that charuco boards are generated and saved successfully, installation is finished.
