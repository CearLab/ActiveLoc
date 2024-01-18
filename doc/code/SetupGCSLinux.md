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

# Before building the images, we need to create a group which will be used to 
# grant permisisons to the files in the docker container. 

    $ sudo groupadd -g 1042 developers

# Now add your user to the group

    $ sudo usermod -aG $USER developers

# Now reboot anc check if $USER is in developers
# In the repository, place yourself where you can see the "src" folder, and
# run the following

    $ chown -R $USER:developers src
    $ chmod -R g+rwx src

# If needed, use sudo.
# The same thing should be done for .Xauthority to be able to run graphic stuff

    $ chown $USER:developers ~/.Xauthority
    $ chmod g+rw ~/.Xauthority

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