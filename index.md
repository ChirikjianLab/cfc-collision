[Sipu Ruan](https://ruansp.github.io/){:target="_blank"}, [Xiaoli Wang](https://github.com/lily983){:target="_blank"}, [Gregory Chirikjian](https://www.eng.nus.edu.sg/me/staff/chirikjian-gregory-s/){:target="_blank"}

Department of Mechanical Engineering, National University of Singapore

Published in __IEEE Robotics and Automation Letters (RA-L)__

## Abstract
This paper studies the narrow phase collision detection problem for two general unions of convex bodies encapsulated by smooth surfaces. The approach, namely CFC (Closed-Form Contact space), is based on parameterizing their contact space in closed-form. The first body is dilated to form the contact space while the second is shrunk to a point. Then, the collision detection is formulated as finding the closest point on the parametric contact space with the center of the second body. Numerical solutions are proposed based on the point-to-surface distance as well as the common-normal concept. Furthermore, when the two bodies are moving or under linear deformations, their first time of contact is solved continuously along the time-parameterized trajectories. Benchmark studies are conducted for the proposed algorithms in terms of solution stability and computational cost. Applications of the sampling-based motion planning for robot manipulators are demonstrated.

## Links
- [Paper](https://ieeexplore.ieee.org/document/9829274){:target="_blank"}
- Code: 
  - [Core C++ library](https://github.com/ChirikjianLab/cfc-collision.git){:target="_blank"}, [API documentation](/resources/doc/v0.1.0){:target="_blank"}: templated header-only library for collision detection algorithms
  - [App](https://github.com/ChirikjianLab/cfc-collision-app.git){:target="_blank"}: application in motion planning, including visualization
  - [MATLAB implementation](https://github.com/ChirikjianLab/cfc-collision-matlab.git){:target="_blank"}: MATLAB version for algorithms. It also includes visualizaions for figures in the paper and benchmark results from C++ implementations.

## Supplementary Video
<iframe width="640" height="340" src="https://www.youtube.com/embed/qcjZRinQ66k" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
