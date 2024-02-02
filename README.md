# Solar Sail Navigation

## SetRRT Solution Visualization
The spacecraft is initially at a periodic orbit (black dash). SetRRT finds the state and control trajectory from the initial orbit to an asteroid (green ball). During planning, it does not consider any uncertainty. 

Four different initial orbits and environments are included. The blue line is the nominal controlled trajectory. The red dash line is the controlled trajectory with process noise. The first row has no unsafe space. The second row has multiple unsafe space.

1. unconstrained state space
<table>
  <tr>
    <td><img src="images/SetRRT_Solarsail_env1.png" alt="Image 1" style="width: 200px; height: 200px;"></td>
    <td><img src="images/SetRRT_Solarsail_env2.png" alt="Image 2" style="width: 200px; height: 200px;"></td>
    <td><img src="images/SetRRT_Solarsail_env3.png" alt="Image 1" style="width: 200px; height: 200px;"></td>
    <td><img src="images/SetRRT_Solarsail_env4.png" alt="Image 2" style="width: 200px; height: 200px;"></td>
  </tr>
</table>

2. state space with circular unsafe regions
<table>
  <tr>
   <td><img src="images/SetRRT_Solarsail_env1_unsafe3.png" alt="Image 1" style="width: 200px; height: 200px;"></td>
    <td><img src="images/SetRRT_Solarsail_env2_unsafe3.png" alt="Image 2" style="width: 200px; height: 200px;"></td>
    <td><img src="images/SetRRT_Solarsail_env3_unsafe3.png" alt="Image 1" style="width: 200px; height: 200px;"></td>
    <td><img src="images/SetRRT_Solarsail_env4_unsafe3.png" alt="Image 2" style="width: 200px; height: 200px;"></td>
  </tr>
</table>

# C++

## build the project
```
cd root folder
cmake -B build .
cmake --build build
./build/bin/test_main
```

# Docker

## build dockerfile
```console
docker build -t jordan787878/solarsail-landing:tag .
```

## run dockerfile
``` 
docker run -it jordan787878/solarsail-landing bash
./build/bin/test_main
```
or run it on the docker volume: solarsail-volume
```
docker run -v solarsail-volume:/app/outputs -it jordan787878/solarsail-landing bash
```

## push dockerfile
```
docker push jordan787878/solarsail-landing
```

# Developingv
## output filesystem for python visualization
Create a docker volume: solarsail-volume to store output files
Go to Docker Desktop/Volumes to access the output files

## Put source code into Github, and follow the Gitflow 
Need to think about the output file and python visual scripts
