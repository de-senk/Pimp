# Pimp
Positional Immediate mode physics

### Current Situation

- [x] Collsion Constraints
  - [ ] figure out a good collision constraint and contact caching.
- [x] Increase code quality
  - [ ] broad and narrow phase
- [x] Separating Axis Tests and Contact point determination
  - [✓] sphere vs sphere, sphere vs capsule,
  - [x] sphere vs hull (to improve.)
  - [x] capsule vs capsule, capsule vs hull (to improve)
  - [✓] hull vs hull with gaussmap (done)
  - [ ] any vs triangle_mesh
- [x] QuickHull
  - [✓] Initial tetrahedron
  - [ ] in the half edge linked list, dont link twins, so we save for loops iterations
  - [ ] Build horizon and other parts of quickhull algorithm (a lot of bookeeping)
  - [ ] figure out what to do if user sends points such that the centroid is not in the origin





# Initial tetrahedron, reference
<img height="256"   alt="Image" src="https://github.com/user-attachments/assets/f90a071b-6202-4308-bdff-128d08550402" />

<img height="256"  alt="Image" src="https://github.com/user-attachments/assets/f5f115e9-4564-467b-955f-51ddaf6ec6f5" />

