# Solar Sail Navigation

## SetRRT Solution Visualization
The spacecraft is initially at a periodic orbit (black dash). SetRRT finds the state and control trajectory from the initial orbit to an asteroid (green ball). During planning, it does not consider any uncertainty. 

Four different initial orbits and environments are included. The blue line is the nominal controlled trajectory. The red dash line is the controlled trajectory with process noise. The first row has no unsafe space. The second row has multiple unsafe space.

1. unconstrained state space
<table>
  <tr>
    <td><img src="images/SetRRT_Solarsail_env1.png" alt="Image 1" style="width: 200px; height: 150px;"></td>
    <td><img src="images/SetRRT_Solarsail_env2.png" alt="Image 2" style="width: 200px; height: 150px;"></td>
    <td><img src="images/SetRRT_Solarsail_env3.png" alt="Image 1" style="width: 200px; height: 150px;"></td>
    <td><img src="images/SetRRT_Solarsail_env4.png" alt="Image 2" style="width: 200px; height: 150px;"></td>
  </tr>
</table>

2. state space with circular unsafe regions
<table>
  <tr>
   <td><img src="images/SetRRT_Solarsail_env1_unsafe3.png" alt="Image 1" style="width: 200px; height: 150px;"></td>
    <td><img src="images/SetRRT_Solarsail_env2_unsafe3.png" alt="Image 2" style="width: 200px; height: 150px;"></td>
    <td><img src="images/SetRRT_Solarsail_env3_unsafe3.png" alt="Image 1" style="width: 200px; height: 150px;"></td>
    <td><img src="images/SetRRT_Solarsail_env4_unsafe3.png" alt="Image 2" style="width: 200px; height: 150px;"></td>
  </tr>
</table>

3. 10 Unsafe Regions: State and Control Trajectory
<table>
  <tr>
   <td><img src="images/solarsail_trajs.png" alt="Image 1" style="width: 400px; height: 300px;"></td>
    <td><img src="images/solarsail_controls.png" alt="Image 2" style="width: 400px; height: 300px;"></td>
  </tr>
</table>

4. Example Animation 
[Click to Play Video](https://jordan787878.github.io/solar-sail/videos.html)

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
docker build -t jordan787878/solarsail:tag .
```

## run dockerfile to init develop environment
```console
docker run -it --name solarsail-dev --rm -v $(pwd):/develop jordan787878/solarsail:tag
```

## build after coding
```console
cd build
cmake ..
make
```

## push dockerfile
```
docker push jordan787878/solarsail:tag
```