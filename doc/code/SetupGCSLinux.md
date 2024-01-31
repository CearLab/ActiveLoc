# SeupGCSLinux.md

# This file goes through the main steps for the setup of the GCS on a linux PC. 
# Ubuntu 20.04 focal is used as reference distribution. However, we will develop 
# entirely on docker, so a later/previous version shouldn't be an issue, as long 
# as docker and docker-compose can be installed

# GEDIT
# Silly thing, I like gedit
    
    $ sudo apt install gedit

# SSH SERVER
# The GCS will be the server for the rosmaster.

    $ sudo apt install openssh-server

# Edit /etc/ssh/sshd_config with your preferred settings and then restart the
# service.

    $ sudo systemctl restart sshd 

# Now you can connect from ssh to the GCS, known the IP. It is suggested to set 
# a static IP. To do so, get in the router settings.

# ADD SSH KEY
# Here we generate and add a ssh key to push to the git repo. If you already
# have setup one you can skip this step

# Key type ed25519 works fine
    
    $ ssh-keygen -t ed25519 
    
# (follow the prompt instructions. Store the key in /home/$USER/.ssh). Now we 
# add the key to the current terminal 

    $ eval $(ssh-agent -s)
    $ ssh-add ~/.ssh/keyname

# Now we add the key to the github account

    $ cat ~/.ssh/keyname.pub 
    
# Copy the output and add the key on your github.com account. Now we can add the
# key to the ssh config file (bu doing this you won't have to add it every time
# you open a new terminal)

# First, check if ~/.ssh/config exists. If yes, open it, otherwise create it

    $ touch ~/.ssh/config

# Now add the following entries in the file

    Host github.com
        HostName github.com
        User git
        IdentityFile ~/.ssh/keyname

# Now the key is loaded everytime you open a new terminal

# Before proceeding, we create a group which will be used to 
# grant permisisons to the files in the docker container. 
# Thus, we proceed like this:

    $ sudo groupadd -g 1042 developers

# Now add your user to the group

    $ sudo usermod -aG developers $USER

# Now reboot anc check if $USER is in developers. If so, we run the following 
# (with sudo if necessary):

    $ chown -R $USER:developers ~/.ssh
    $ chmod -R 700 ~/.ssh

# Remark: if you're not $USER but you're member of "developers" you still can't
# push/pull git repositories. For this reason, all the times that we need to 
# push/pull, we will do it from local. 

# INSTALL DOCKER
# We now proceed installing docker and docker-compose, which we will use for the
# development environment. See https://docs.docker.com/engine/install/ubuntu/

# First start by removing all previous docker installations

    $ for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
    $ sudo rm -rf /var/lib/docker
    $ sudo rm -rf /var/lib/containerd

# Proceed with a fresh install. The next command will add the repository to the 
# sources.list directory

    $ sudo apt update
    $ sudo apt install apt-transport-https ca-certificates curl software-properties-common
    $ sudo install -m 0755 -d /etc/apt/keyrings
    $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    $ sudo chmod a+r /etc/apt/keyrings/docker.gpg
    $ echo \
        "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
        "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Now update the mirrors and install the packages

    $ sudo apt update
    $ sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Now that everything has been installed we run a sanity check

    $ sudo docker run hello-world

# Now we add the user to the docker group. Run "$ groups" and read the groups 
# your user has access to. Now add the docker group

    $ sudo groupadd docker 
    $ sudo usermod -aG docker $USER 

# Now reboot and run "$ groups" and check again. You should see docker listed 
# Now check that you can call docker without sudo

    $ docker run hello-world

# Now enable docker at startup

    $ sudo systemctl enable docker.service
    $ sudo systemctl enable containerd.service

# CLONE REPOSITORY
# Check to have git installed and a proper ssh-key working. Place yourself where 
# you want to clone the repository

    $ git clone git@github.com:CearLab/ActiveLoc.git

# In the repository, place yourself where you can see the "src" folder, and
# run the following

    $ chown -R $USER:developers src
    $ chmod -R 774 src

# If needed, use sudo. By doing so, also the members of "developers" can wrx the
# files on the repository.
# The same thing should be done for .Xauthority to be able to run graphic stuff

    $ chown $USER:developers ~/.Xauthority
    $ chmod 764 ~/.Xauthority

# If needed, use sudo.

# BUILD DOCKER IMAGE FOR FOCAL (20.04) WITH ROS-NOETIC
# Before starting, it might be usefu to get acquainted with docker and its main 
# concepts. Guides can be found here: https://docker-curriculum.com/

# We use docker-compose to manage images and containers. In the src/docker folder 
# there is a docker-compose.yml file. Each service in the .yml is linked to a 
# Dockerfile where the image is built. The structure is the following: 

#   base environment:   clean installation with only Ubuntu and ROS.
#   dev environment:    built on top of base. It is meant for testing

# When images are built, you can also push them in your docker hub. 
# See https://hub.docker.com/

# Starting from the base environment, you need to run a detached container, 
# the image will build and tag autonomously. 

    $ docker compose --file src/docker/docker-compose.yml up -d ros-noetic-base --remove-orphans
    $ docker compose --file src/docker/docker-compose.yml up -d ros-noetic-dev --remove-orphans

# Now you should have 2 images and 2 containers. Check with

    $ docker images
    $ docker ps -a

# From now on we will use the dev container.

# Remark: if you want to instantiate a new container, just run docker compose 
# for jackal-noetic-dev again. 

# Remark: for nvidia jackal, check the jackal-noetic-nvidia service in 
# the docker-compose.yml

# Remark: you can log in the container from vscode:
#   1) check that the container is running. if not start it (docker start ID)
#   2) In Vs Code bottom left corner, click on the green rectancle
#   3) attach to running container --> select the container

# SETUP ROS CLEARPATH

# Now that we are logged in our ros container we start installing the main ros 
# packages that we will use for the development. We install the Clearpath 
# packages as we will use Jackals and Huskies for the swarm

# We start by updating the system and adding the clearpath repositories
# Remark: all this shall be done IN THE DOCKER CONTAINER, not locally

    $ sudo apt update
    $ sudo apt upgrade
    $ wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
    $ sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
    $ sudo apt update

# We start with the Jackals

    $ sudo apt install ros-noetic-jackal-robot --fix-missing
    $ sudo apt-get install ros-noetic-jackal-simulator --fix-missing
    $ sudo apt-get install ros-noetic-jackal-desktop --fix-missing
    $ sudo apt-get install ros-noetic-jackal-navigation --fix-missing

# Then we also install the Husky

    $ sudo apt install ros-noetic-husky-robot --fix-missing
    $ sudo apt-get install ros-noetic-husky-simulator --fix-missing
    $ sudo apt-get install ros-noetic-husky-desktop --fix-missing
    $ sudo apt-get install ros-noetic-husky-navigation --fix-missing

# We now proceed installing general python stuff, after an update (safety)

    $ sudo apt update && sudo apt upgrade
    $ sudo apt install python-pygments
    $ sudo apt install python3-rosinstall-generator 
    $ sudo apt install ros-noetic-rqt ros-noetic-rqt-common-plugins

# Now, we have al the utils installed. Check the path

    $ cd ~/workspace
    $ echo $ROS_PACKAGE_PATH

# We set nano as the default ROS editor 
    
    $ export EDITOR='nano -w'

# Now we start with the project build. First, we install some general 
# optimization general

    $ sudo apt install libopenblas-dev liblapack-dev

# Now let's make and go in the setup/lib folder and start building stuff

    $ mkdir setup/lib
    $ cd setup/lib

# We now install ALGLIB: general optimization (check link: 18/01/2024)
    
    $ wget https://www.alglib.net/translator/re/alglib-4.01.0.cpp.gpl.tgz
    $ tar -xf alglib-4.01.0.cpp.gpl.tgz
    $ rm alglib-4.01.0.cpp.gpl.tgz

# We now install ARMADILLO: general matrix computations and optimization 
# (check link: 30/05/2023)

    $ wget https://sourceforge.net/projects/arma/files/armadillo-12.6.7.tar.xz
    $ tar -xf armadillo-12.6.7.tar.xz
    $ rm armadillo-12.6.7.tar.xz
    $ cd armadillo-12.6.7

# The standard installation will be placed in /usr/lib

    $ cmake .
    $ sudo make install

# Check if the installation finished correctly (you should see armadillo files)

    $ ls /usr/lib | grep arma 

# If so, we can export the path

    $ export ARMA_INCLUDE_PATH=/usr/lib && cd ..

# We now install OPTIMLIB, a general matrix computations and optimization librry
# (check link: 21/01/2024 - following README). We first need to pull a repo. As
# previously explained, we can push/pull from local. So, first we chown lib to 
# $USER, then we pull the repo. To do so, go in the local prompt (any terminal)
# from your local PC. Navigate to "ActiveLoc/src/setup" and run the folowing:

    $ chown -R $USER:developers ./lib
    $ git clone --recurse-submodules git@github.com:kthohr/optim.git
    $ sudo chown -R $USER:developers optim/

# Now you can go back to your docker container and proceed with the installation

    $ cd optim
    $ ./configure -i "/usr/local" -l arma -p
    $ sudo make
    $ sudo make install

# Check installation

    $ ls /usr/local/lib | grep optim
    $ cd ..

# Now we install CERES, a nonlinear optimization toolbox 
# (check link: 30/05/2023 - following install instructions)
# We first start by installing some dependencies

    $ sudo apt-get install libgoogle-glog-dev libgflags-dev
    $ sudo apt-get install libatlas-base-dev
    $ sudo apt-get install libeigen3-dev
    $ sudo apt-get install libsuitesparse-dev

# Now we need to download and install stuff, please go back to your local prompt

    $ wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz
    $ tar -xf ceres-solver-2.2.0.tar.gz
    $ rm ceres-solver-2.2.0.tar.gz
    $ mkdir CERES
    $ mkdir CERES/ceres-bin
    $ mv ceres-solver-2.2.0 CERES/
    $ sudo chown -R $USER:developers CERES/
    $ cd CERES/ceres-bin

# Now we build the library, so we need to be back in the docker container. 

    $ sudo cmake -DBUILD_SHARED_LIBS='ON' ../ceres-solver-2.2.0
    $ sudo make
    $ sudo make install 
    
# These last commands might be very slow, check the RAM consumption because it 
# could be an issue. 
# If you can't compile, a firts option is to extend the swap partition using a 
# swap file. By doing so, you should be able to get to the end of the build.
# Swap memory is used to store infrequently used data 
# currently in the RAM, to decrease the burden on it. All the following should 
# be run in local, as the container shares the host kernel and swap 
# To see how much swap you have run 

    $ free -h

# To see the currently active files run 

    $ cat /proc/swaps

# Now we create our own swap file. Here the size is the block dimension (bs) 
# multiplied with the number of blocks (count). In this case we allocate 5GB

    $ sudo dd if=/dev/zero of=/swapros bs=1MB count=5120

# Now set the permissions and make the file swap. Then enable and check the swap

    $ sudo chmod 600 /swapros
    $ sudo mkswap /swapros
    $ sudo swapon /swapros

# Now if you run again "free -h" and "cat /proc/swaps" you should see the new
# swap file. Now try again with the build. If it succeeds you can unmount the 
# swap file

    $ sudo swapoff /swapros

# Whenever you need some swap more just swapon this file

# If you don't have space for a swap f file, build on a generic powerful pc 
# and copy in the docker container:

# the .so files in /usr/local/lib
# the .h files in /usr/local/include/ceres
# the .cmake files in /usr/local/lib/cmake/Ceres

# Chekc the CERES installation

    $ ls /usr/local/lib | grep ceres

# If everything is fine we can run an easy upgrade and update

    $ cd ~/workspace
    $ sudo apt update
    $ sudo apt upgrade

# We add the last ros packages that we need

    $ sudo apt install ros-noetic-roslint
    $ sudo apt install ros-noetic-rqt-tf-tree
    $ sudo apt install ros-noetic-roscpp-tutorials
    $ sudo apt install ros-noetic-tf2-web-republisher
    $ sudo apt install ros-noetic-rosbridge-suite

# We now proceed with the installation of the Vicon SDK for C++
# First go tohttps://www.vicon.com/software/datastream-sdk/?section=downloads 
# and download the .zip (link checked on 24/01/2024). Then move the .zip in the 
# docker container, specifically in "workspace/setup/lib". 

# Now, go on a local PC terminal and do the following (FILENAME is the name
# of what you downloaded)

    $ unzip FILENAME.zip
    $ mv FILENAME.zip ~/Downloads/
    $ mkdir ViconAPI
    $ mv EXTRAXTDIRNAME ViconAPI/F1
    $ cd ViconAPI/F1/Release/Linux64

# Install p7zip for .7z files

    $ sudo apt install p7zip-full

# Extract source.7z and thirdparty.7z (check the correct name)

    $ mkdir Source
    $ mv FILENAME-source.7z Source/
    $ cd Source
    $ 7z x FILENAME-source.7z 
    $ cd ..
    
    $ mkdir Thirdparty
    $ mv FILENAME-thirdparty.7z Thirdparty/
    $ cd Thirdparty
    $ 7z x FILENAME-thirdparty.7z
    $ cd ..

    $ cd ../../../../
    $ sudo chown -R $USER:developers ViconAPI

# Now, all the APIs are already present in these directories. However, they 
# could have been compiled on some linux distro with different glibc version
# that the one you're running. This might be the case if you're using an old 
# linux distro on the jackal. So, we recompile the whole thing (yay). 

# We compile in the docker container, because we want to work from there in the
# end. So, place yourself in a docker container prompt. 

    $ cd ~/workspace/setup/lib/ViconAPI/F1/Release/Linux64/Source

# Open the makefile and add the following lines:

    INCLUDES = -I/home/ros/workspace/setup/lib/boost_1_82_0/boost_install/include \
     		   -I/home/ros/workspace/setup/lib/ViconAPI/F1/Release/Linux64/Source/Vicon/CrossMarket/DataStream \
     		   -I/home/ros/workspace/setup/lib/ViconAPI/F1/Release/Linux64/Source/Vicon/CrossMarket

    LDFLAGS = -L/home/ros/workspace/setup/lib/boost_1_82_0/boost_install/lib \
     		  -L/home/ros/workspace/setup/lib/ViconAPI/F1/Release/Linux64/ThirdParty/thirdparty/Boost/boost-1.75.0-linux-x64/lib \
    		  -L/home/ros/workspace/setup/lib/ViconAPI/F1/Release/Linux64/Source/lib/Debug \
    		  -L/home/ros/workspace/setup/lib/ViconAPI/F1/Release/Linux64/Source/bin/Debug

# Now add at the end of every make target 
# (all the @$(MAKE) statements) the following: 

    CXXFLAGS="$(INCLUDES)" CFLAGS="$(INCLUDES)" LDFLAGS="$(LDFLAGS)"

# As you can see in the INCLUDES, we are using libboost1.82 as glibc version. 

# First we download it. Go to your local prompt in setup/lib and run

    $ wget https://boostorg.jfrog.io/artifactory/main/release/1.82.0/source/boost_1_82_0.tar.gz
    $ tar -xf boost_1_82_0.tar.gz
    $ mv boost_1_82_0.tar.gz ~/Downloads
    $ sudo chown -R $USER:developers boost_1_82_0

# so we first install it (in the docker, this is important. if you change 
# glibc version on your local you could regret it...)

    $ cd ~/workspace/setup/lib/boost_1_82_0
    $ sudo mkdir boost_install
    $ sudo ./bootstrap.sh --prefix=/home/ros/workspace/setup/lib/boost_1_82_0/boost_install
    $ sudo ./b2
    $ sudo ./b2 install
    $ sudo chown -R ros:developers ./boost_install
    $ sudo chown -R ros:developers b2
    $ sudo chown -R ros:developers bin.v2
    $ sudo chown -R ros:developers project-config.jam
    $ sudo chown -R ros:developers stage

# Now we can build ViconAPI. Still in the docker container

    $ cd /home/ros/workspace/setup/lib/
    $ sudo chown -R ros:developers ViconAPI
    $ cd /home/ros/workspace/setup/lib/ViconAPI/F1/Release/Linux64/Source
    $ make clean
    $ make ViconDataStreamSDK_CPP

# If you get errors like: 
#    (e.g.) /usr/bin/ld: cannot find -lboost_test_exec_monitor-mt-d-x64
# go to "/home/ros/workspace/setup/lib/boost_1_82_0/boost_install" and check if 
# there is a similar lib installed

    $ ll | grep test_exec 

# if you find that there is a similar lib, just create a symlink with the
# expected name. For example, if you find that there is a "libboost_test_exec_monitor.a"
# do the following: 

    $ ln -s libboost_test_exec_monitor.a libboost_test_exec_monitor-mt-d-x64.a

# If you run again line 467 you should see the error disappeared. Proceed in the 
# same way for similar errors. Other errors? Well I have no clue eheh. 

# If you managed to compile ViconAPI well kudos, you should now be ready to work
# on the project. 

# Let's get to work.

    $ cd ~/workspace


