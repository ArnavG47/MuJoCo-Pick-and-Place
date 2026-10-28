### Lesson 5
- the limit of left_knuckle_joint is from 0 to 0.8
- the right_knuckle_joint mimics the left but mirrored
- right finger tip joints also mimic this left_knuckle_joint
- .dae meshes are just the ones used for visual whereas .stl are meshes used in collision physics

- fusestatic is a complier setting which merges linkages that dont have joint between them to save computing power.
- with fusestatic ON, the base link got merged with <worldbody> which has no mass. This removed the base links mass from total mass.
- when attaching the gripper to the mocap body directly it caused the gripper to have no calculated velocity in the simulation. This meant there was no friction to pick up the block. 
- the solution was to create a real dynamic body to "weld" to the mocap and it is referred to as 'tool'. This tool is then attached to the EOAT and easily swapped. 

- in trial runner file, a dictionary was created to handle the variations in values for grasp offset, actual grasp location and actuator names. The dictionary is then used throughout the file and in the terminal to choose which simulation to run