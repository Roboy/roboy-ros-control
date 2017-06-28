### jupyter and pydy via anaconda 
Anaconda packs a lot of packages already and this is the easiest way to get started
follow instructions here: http://jupyter.readthedocs.io/en/latest/install.html#id3
this is the short version:
* Download anaconda https://www.continuum.io/downloads#linux for python2.7
* bash Anaconda2-4.4.0-Linux-x86_64.sh 
* pip install pydy

### pydy
if you havent installed via anaconda:
* download archive from https://github.com/pydy/pydy/releases/tag/v0.4.0
* extract 
* sudo python setup.py install
* sudo pip install mpmath

### Run it
#### jupyter
* source anacond (was offered to be put into .bashrc during installation of anaconda)
* verify its anaconda python you are using: 
* which python  
* should give you something like this: /home/username/anaconda2/bin/python
* jupyter notebook
* choose PaBiRoboy_dynamics.ipynb
#### spyder is a pretty good python IDE:
* sudo pip install --upgrade html5lib==1.0b8
* sudo apt install spyder
#### direct
python PaBiRoboy_dynamics_simulation.py


