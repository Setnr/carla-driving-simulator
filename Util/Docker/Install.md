# General

This document provides a short summary of the Carla tutorial for installing the regular Carla Docker and building this simulator using the Docker engine. After setting up both Docker environments, you will learn how to extract the required files from the simulator’s Docker container and integrate them into the regular Carla Docker image to run the complete system.

---

# 1. Install the Carla Docker

## Requirements

- **Docker** must be installed.
- **NVIDIA Container Kit** must be installed.

## Installation Steps

1. **Pull the docker image**

   Pull the 0.9.15 image from Docker Hub:
```
docker pull carlasim/carla:0.9.15
```

2. **Start the docker container**

Run the following command to start the container:
```
sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.15 /bin/bash ./CarlaUE4.sh
```

For further information, see the [build_docker.md](https://github.com/carla-simulator/carla/blob/0.9.15/Docs/build_docker.md).

---

# 2. Build this Simulator

## Requirements

- **Operating System:** Ubuntu 16.04+ (64 bit)
- **Memory:** At least 8 GB of RAM
- **Disk Space:** Approximately 600 GB
- **Docker:** Installed (from previous steps)
- **Python:** Version 3.6+
- **GitHub Account Setup:** You must set up your GitHub account for Unreal Engine access by following this [guide](https://www.unrealengine.com/en-US/ue4-on-github).

## Installation Steps

1. **Clone the repository**

Clone the repository from the 9.15-Update branch:
```
Git clone -b 9.15-Update https://github.com/Setnr/carla-driving-simulator.git
```

2. **Build the prerequisites docker image**

Switch to the Docker folder:
```
cd Util/Docker
```
Build the `carla-prerequisites` image (this step may take a while):
```
docker build --build-arg EPIC_USER=<GitHubUserName> --build-arg EPIC_PASS=<GitHubPassword> -t carla-prerequisites -f Prerequisites.Dockerfile .
```

For further information, see the [build_docker_unreal.md](https://github.com/carla-simulator/carla/blob/0.9.15/Docs/build_docker_unreal.md).

3. **Build the main docker image**

Now build the main `carla` image with:
```
docker build -t carla -f Carla.Dockerfile . --build-arg GIT_BRANCH=9.15-Update
```

---

# 3. Setup the original Simulator to run the modified Simulator

To run the failure modes of the updated simulator on the original simulator, follow these steps:

1. **Extract files from the modified simulator**

Run the modified simulator’s docker container with a mounted volume:
```
docker run -v <path/to/a/place/you/want/to/Save/the/Files>:/home/carla/tmp --rm -it carla-error:latest /bin/bash
```

Once inside the container, execute the following commands:
```
cd /home/carla/carla cp -R ./Unreal/CarlaUE4/Binaries /home/carla/tmp/Binaries cp -R ./PythonAPI/ /home/carla/tmp/PythonAPI cp -R ./CarlaUE4/Plugins/carla/Content/PostProcessingMaterials /home/carla/tmp/PostProcessingMaterials
```

2. **Run the modified simulator**

Change to the directory where the files have been saved:
```
cd <path/to/a/place/you/want/to/save/the/files>
```

Then run the following command:
```
docker run --privileged --rm --gpus all --net=host -e DISPLAY=$DISPLAY -e SDL_AUDIODRIVER=dummy \ -u $(id -u):$(id -g)
-v ./Binaries:/home/carla/CarlaUE4/Binaries
-v ./PythonAPI:/home/carla/PythonAPI
-v ./PostProcessingMaterials:/home/carla/CarlaUE4/Plugins/Carla/Content/PostProcessingMaterials
carlasim/carla:0.9.15 /bin/bash ./CarlaUE4.sh
```

---

# 4. FAQ

**Q:** *What if I have trouble extracting the working files from the modified simulator?*

**A:** You can try rebuilding Carla within the docker container itself:

1. **Run the container with a mounted volume**
```
docker run -v <path/to/a/place/you/want/to/Save/the/Files>:/home/carla/tmp --rm -it carla-error:latest /bin/bash
```

2. **Build the package**

Inside the container, run:
```
cd /home/carla/carla 
make package
```

This will create a folder under:
```
/home/carla/carla/Dist/CARLA_SHIPPING_XYZ/LinuxNoEditor
```

(where `XYZ` corresponds to the version of the simulator).

In the `LinuxNoEditor` folder, you will find the Binaries, PythonAPI files, and PostProcessingMaterials. Copy these files to your local disk and split them into the folders needed to execute the edited simulator.
