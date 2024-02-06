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

4. Animation

https://github.com/jordan787878/solar-sail/assets/17584164/d9b81a1a-f746-4272-95a6-6b58f85204bf


# Generalize to other Problem
The framework can be applied to other problems, provided their ODEs.
## Marine Vessel Navigation
A marine vessel controlled by forward velocity (u1) and yaw rate (u3). The mission is to reach the green goal and avoid the red unsafe regions.
<table>
  <tr>
   <td><img src="images/marine_vessel_2_traj.png" alt="Image 1" style="width: 400px; height: 300px;"></td>
    <td><img src="images/marine_vessel_2_control.png" alt="Image 2" style="width: 400px; height: 300px;"></td>
  </tr>
</table>

Animation

https://github.com/jordan787878/solar-sail/assets/17584164/b0716dbc-bea3-4bb8-bcaa-bb576a97cb03

## Cart and Inverted Pendulum
An inverted pendulum is attached to a cart. The cart is controlled by a horizontal force u. The system has four states: angle and angular rate of the inverted pendulum; position and velocity of the cart. The goal is to: 
1. swing up the inverted pendulum such that it is at the upright position. 
2. the position and the velocity of the cart should also reach a certain region.
<table>
  <tr>
   <td><img src="images/invertpendulum_simple_1_state.png" alt="Image 1" style="width: 400px; height: 300px;"></td>
    <td><img src="images/invertpendulum_simple_1_control.png" alt="Image 2" style="width: 400px; height: 300px;"></td>
  </tr>
</table>

Animation

https://github.com/jordan787878/solar-sail/assets/17584164/792cc468-f7bf-4b48-9427-c15772dc625e

https://github.com/jordan787878/solar-sail/assets/17584164/afa2f726-c145-4ed2-8616-7e9e73bec88f

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
