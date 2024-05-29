from fanucpy import Robot

# connecting to a physical robot
robot = Robot(
    robot_model="Fanuc",
    host="192.168.0.109",
    port=18735,
    ee_DO_type="RDO",
    ee_DO_num=7,
)
robot.connect()

# move in joint space
robot.move(
    "joint",
    vals=[19.0, 66.0, -33.0, 18.0, -30.0, -33.0],
    velocity=100,
    acceleration=100,
    cnt_val=0,
    linear=False,
)

# move in cartesian space
robot.move(
    "pose",
    vals=[0.0, -28.0, -35.0, 0.0, -55.0, 0.0],
    velocity=50,
    acceleration=50,
    cnt_val=0,
    linear=False,
)

# open gripper
robot.gripper(True)

# close gripper
robot.gripper(False)

# get robot state
print(f"Current pose: {robot.get_curpos()}")
print(f"Current joints: {robot.get_curjpos()}")
print(f"Instantaneous power: {robot.get_ins_power()}")
print(f"Get gripper state: {robot.get_rdo(7)}")
