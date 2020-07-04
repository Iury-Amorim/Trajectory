# Trajectory: Controller   performance   analysis

This repository brings the analysis and results of trajectory tracking for the linear controllers PD and GPC, as well as the addition of the non-linear feedback portion based on a fuzzy compesator.
For this, the integration of two free software was used: GNU Octave and CoppeliaSim from Coppelia Robotics.


* [GNU Octave homepage](https://www.gnu.org/software/octave/)
* [Coppelia Robotics homepage](https://www.coppeliarobotics.com/)

### Citation
```
@Misc{trajectory2020,
author = {Felipe José de Sousa Vasconcelos and Iury de Amorim Gaspar Filgueiras and Davi Alexandre Paiva},
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


For the execution of the programs it is necessary to use some packages, this way you must do:
```bash
sudo apt install octave-symbolic
sudo apt install octave-control
sudo apt install octave-geometry
sudo apt install octave-signal
sudo apt install octave-io
```

In Octave's Command Window, load packages using the command:

```bash
pkg load (namepackage)
```
### CoppeliaSim
On the [Coppelia Robotics](https://www.coppeliarobotics.com/) website, download CoppeliaSim EDU version. 
The simulator will come as an executable file.

### API-integration
For the integration between the GNU Octave and CoppeliaSim read the file "API_instructions".

## Dependencies:

  - Python
  - liboctave-dev
  
  
## Acknowledgements
* The authors thank DEE-UFC and CAPES for fi-nancial support to the development of this project.


