"""Webots RosBot device setup.

Instantiates the Webots Robot and enables motors/sensors. The mission logic
imports these device handles. No Supervisor is used.
"""

from controller import Camera, Robot

# ============================================================
# Webots devices
# ============================================================

robot = Robot()
time_step = int(robot.getBasicTimeStep())


def required_device(name):
    device = robot.getDevice(name)
    if device is None:
        raise RuntimeError(f'Device "{name}" was not found.')
    return device


fl_motor = required_device("fl_wheel_joint")
fr_motor = required_device("fr_wheel_joint")
rl_motor = required_device("rl_wheel_joint")
rr_motor = required_device("rr_wheel_joint")

left_motors = [fl_motor, rl_motor]
right_motors = [fr_motor, rr_motor]
all_motors = left_motors + right_motors

for motor in all_motors:
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

MAX_MOTOR_VELOCITY = min(
    motor.getMaxVelocity()
    for motor in all_motors
)

fl_encoder = required_device("front left wheel motor sensor")
fr_encoder = required_device("front right wheel motor sensor")
rl_encoder = required_device("rear left wheel motor sensor")
rr_encoder = required_device("rear right wheel motor sensor")
encoders = [fl_encoder, fr_encoder, rl_encoder, rr_encoder]

for encoder in encoders:
    encoder.enable(time_step)

compass = required_device("imu compass")
compass.enable(time_step)

fl_range = required_device("fl_range")
fr_range = required_device("fr_range")
rl_range = required_device("rl_range")
rr_range = required_device("rr_range")
range_sensors = [fl_range, fr_range, rl_range, rr_range]

for sensor in range_sensors:
    sensor.enable(time_step)

lidar = required_device("laser")
lidar.enable(time_step)
lidar.enablePointCloud()

rgb_camera = required_device("camera rgb")
rgb_camera.enable(time_step)

depth_camera = required_device("camera depth")
depth_camera.enable(time_step)


