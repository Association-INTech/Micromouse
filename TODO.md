# TODO

## High Level

- [x] basic Maze class with add_wall and get_wall functions
- [x] very basic Coord\<Rank\>, Node\<Rank\> and Edge\<Rank\>
- [x] an abstract Graph\<Rank\> with neighbors and shortest_path functions
- [x] Curve<Rank> class that abstracts away a variant of some possible types of curves
- [x] CurveStraight for Rank=1
- [x] Trajectory<Rank> class that merges multiple curves after another
- [ ] add shortcuts for Rank=1
- [ ] CurveCircle and CurveStraight with no stop for Rank=2
- [ ] add shortcuts for Rank=2 straight lines
- [ ] Curve<3>
- [ ] add shortcuts for Rank=3 straight lines
- [ ] add shortcuts for Rank=3 diagonals

## Low Level

- [ ] MCU peripherals configuration for motors, encoders, LEDs etc
- [ ] command line interface to interact with the MCU via USB CDC
- [ ] robot kinematics
- [ ] motor PID controler
- [ ] RAMSETE controler
- [ ] IR sensors full hardware implementation
- [ ] wall detection and maze building for the first run
- [ ] use IR sensors to estimate state error
- [ ] EKF to merge IR sensors, encoders and IMU
