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

# CLONE REPOSITORY