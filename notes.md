### Lesson 5
- the limit of left_knuckle_joint is from 0 to 0.8
- the right_knuckle_joint mimics the left but mirrored
- right finger tip joints also mimic this left_knuckle_joint
- .dae meshes are just the ones used for visual whereas .stl are meshes used in collision physics

- fusestatic is a complier setting which merges linkages that dont have joint between them to save computing power.
- with fusestatic ON, the base link got merged with <worldbody> which has no mass. This removed the base links mass from total mass.
