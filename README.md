# CFC: Collision detection based on closed-form contact space parameterization
Collision detection, distance queries (penetration depth), closes points computations via closed-form contact space (CFC) for unions of convex bodies with smooth boundaries.

(Source code coming soon!)

## Introduction
This is the C++ implementation for the narrow phase collision detection problem between two general unions of convex bodies encapsulated by smooth surfaces. The approach, namely CFC (Closed-Form Contact space), is based on parameterizing their contact space in closed-form. The first body is dilated to form the contact space while the second is shrunk to a point. Then, the collision detection is formulated as finding the closest point on the parametric contact space with the center of the second body. Numerical solutions are proposed based on the point-to-surface distance as well as the common-normal concept. Furthermore, when the two bodies are moving or under linear deformations, their first time of contact is solved continuously along the time-parameterized trajectories. Benchmark studies are conducted for the proposed algorithms in terms of solution stability and computational cost.

- Paper: [IEEE Robotics and Automation Letters (RA-L)](https://ieeexplore.ieee.org/document/9829274)
- Project page: [https://chirikjianlab.github.io/cfc-collision/](https://chirikjianlab.github.io/cfc-collision/)
- MATLAB implementation: [https://github.com/ChirikjianLab/cfc-collision-matlab](https://github.com/ChirikjianLab/cfc-collision-matlab)
- Data: [Benchmark data in the paper](https://drive.google.com/drive/folders/17jSSC-EIhiSTqXSgfoEOs4R7mzKy1d1i?usp=sharing)

## Authors
[Sipu Ruan](https://ruansp.github.io), Xiaoli Wang and [Gregory S. Chirikjian](https://scholar.google.com/citations?user=qoIuyMoAAAAJ&hl=en)

## Reference
If you find our work useful in your research, please consider citing:

- S. Ruan, X. Wang and G. S. Chirikjian, "Collision Detection for Unions of Convex Bodies With Smooth Boundaries Using Closed-Form Contact Space Parameterization," in IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 9485-9492, Oct. 2022, doi: 10.1109/LRA.2022.3190629.

- BibTeX
```
@ARTICLE{ruan2022collision,
  author={Ruan, Sipu and Wang, Xiaoli and Chirikjian, Gregory S.},
  journal={IEEE Robotics and Automation Letters}, 
  title={Collision Detection for Unions of Convex Bodies With Smooth Boundaries Using Closed-Form Contact Space Parameterization}, 
  year={2022},
  volume={7},
  number={4},
  pages={9485-9492},
  doi={10.1109/LRA.2022.3190629}}
```

