from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

name = robot.getName()

# --- Get the four wheel motors (names from your console) ---
fr = robot.getDevice('front_right_wheel_joint')
fl = robot.getDevice('front_left_wheel_joint')
br = robot.getDevice('back_right_wheel_joint')
bl = robot.getDevice('back_left_wheel_joint')

motors = [fr, fl, br, bl]

for m in motors:
    m.setPosition(float('inf'))          # velocity control mode

# Try a safe speed first (you can tune this)
BASE_SPEED = 15.0   # rad/s, usually <= max vel (~10-ish)

while robot.step(timestep) != -1:
    # Simple forward motion: same velocity on all wheels
    for m in motors:
        m.setVelocity(BASE_SPEED)
