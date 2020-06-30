# Trajectory: Controller   performance   analysis

This repository brings the analysis and results of trajectory tracking for the linear controllers PD and GPC, as well as the addition of the non-linear feedforward portion based on a fuzzy controller.
For this, the integration of two free software was used: GNU Octave and CoppeliaSim from Coppelia Robotics.


* [GNU Octave homepage](https://www.gnu.org/software/octave/)
* [Coppelia Robotics homepage](https://www.coppeliarobotics.com/)

### Citation
```
@Misc{trajectory2020,
author = {Felipe Vascolncelos and Iury Filgueiras and Davi Alexandre},
title = {{Trajectory}: 
Controller performance analysis},
howpublished = {\url{https://github.com/Iury-Amorim/Trajectory}},
year = {2020}
}

```


## Getting started 

### GNU Octave

The simple way to install the software is using the following commands for ubuntu users:

```bash
sudo apt install octave
```

Para a execução dos programas é necessário a utilizaço de alguns pacotes, desta forma deve-se fazer:
```bash
sudo apt install octave-symbolic
sudo apt install octave-control
sudo apt install octave-geometry
sudo apt install octave-signal
sudo apt install octave-io
```
## Dependencies:

  - Python
  - liboctave-dev
  
  
## Acknowledgements
* The authors thank DEE-UFC and CAPES for fi-nancial support to the development of this project.


