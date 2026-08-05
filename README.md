

The project presents the design, mathematical modeling, and embedded implementation of a miniature, indoor VTOL (Vertical Takeoff and Landing) robotic blimp. The vehicle operates under quasi-neutral buoyancy using a 50-inch helium-filled Mylar balloon, which provides the baseline lift.

To minimize payload and complexity, the blimp uses a minimal-sensing architecture managed by an ESP32-C6 microcontroller, relying on a VL53L1X Time-of-Flight sensor for altitude and an MPU6050 IMU for heading estimation. Its propulsion system consists of two counter-rotating coreless DC motors mounted on a 180-degree servo-actuated vectored-thrust mechanism. This design allows the same motors to control both vertical lift and horizontal maneuvering. Ultimately, the paper provides a complete, reproducible engineering design and experimentally identifies the dynamic models for the vehicle's vertical and yaw motions to aid future research in aerial robotics.
