# PETcat - Obstacle Avoidance

## Path Planning with Gaussian Potential Maps
The code can be found <a href = "/obs-avd">here</a>.</br>
To determine the optimum value for gamma, we assigned rewards for a path; depending on the number of collisions with obstacles, the number of steps required to reach the goal and whether the agent was able to reach the goal under a threshold number of steps.</br>
Preliminary tests showed that the value for gamma should lie between 0 and 1.5.</br>
### Results :
##### Test 1 
[Moving from (0,0) to (4,-2)]</br>
![Rewards VS Gamma](obs-avd/samples/RewardsVsGamma1.png)</br>
  Best Path (Gamma = 1.47) : </br>
![Best Path](obs-avd/samples/BestPath1.png)</br>

##### Test 2
[Moving from (0, 0) to (5, 5)] </br>
![Reward Vs Gamma](obs-avd/samples/RvsG2.png)</br>
  Best Path (Gamma = 0.59) : </br>
![Best Path](obs-avd/samples/BestPath2.png)</br>

Both environments work well with Gamma = 1 </br>
![Test 1 with Gamma = 1](obs-avd/samples/T1_Gas1.png)</br>
![Test 2 with Gamma = 1](obs-avd/samples/T2_Gas1.png)</br>

### With Mobile Obstacles
We realised the need to use lower values for Gamma.</br>
Best Path : Gamma = 0.59</br>
![Results with Gamma = 0.59](obs-avd/samples/Mobile.gif)</br>

#### Velocity Control

![Velocity control additions](obs-avd/samples/vel_c.gif)