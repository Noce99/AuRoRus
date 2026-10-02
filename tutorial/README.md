# Tutorials

Step-by-step tutorials that each write one autonomous driving algorithm from
an empty file. Both explain the theory, the code, and how the algorithm talks
to the rest of the project. No Rust experience is assumed, and the code is
written to be easy to read, not to be fast.

| Tutorial | What it builds | What it needs to drive |
|---|---|---|
| [1 - Gap follower](01_gap_follower.md) | Steers toward the middle of the widest opening the LIDAR sees | Only the LIDAR |
| [2 - Pure pursuit](02_pure_pursuit.md) | Follows the race line toward a point a fixed distance ahead | A race line and the car's pose on the map |

Start with tutorial 1: it explains the Rust syntax and the executor
framework in more detail.

The project already contains `gap_follower.rs` and `pure_pursuit.rs`. The
tutorials write `my_gap_follower.rs` and `my_pure_pursuit.rs` instead, so the
results can be compared with the project's own versions.

Reference documentation is in [documentation/](../documentation/).
