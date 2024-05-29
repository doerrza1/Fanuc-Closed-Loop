from fanucpy import Robot # type: ignore

def coordinate(x, y, z, w, p, r):

    coords = [x, y, z, w, p, r]
    
    print(coords)

    return coords

robot = Robot(
    robot_model = "Fanuc",
    host="192.168.0.3",
    port=18735,
    ee_DO_type="RDO",
    ee_DO_num=7,
)

robot.connect()

#loop to continuosly enter points for robot to move, enter y to continue

while(input("Would you like to move the robot?") == "y"): 
    
    # Print out current position at the beginning of each loop
    print("Current poses: ")
    cur_pos = robot.get_curpos()
    cur_jpos = robot.get_curjpos()
    print(f"Current pose: {cur_pos}")
    print(f"Current joints: {cur_pos}")

    
    # Inputs for each postion/angle
    x = int(input("Enter x position: "))
    y = int(input("Enter y postion: "))
    z = int(input("Enter z position: "))
    yaw = int(input("Enter yaw angle: "))
    pitch = int(input("Enter pitch angle: "))
    roll = int(input("Enter roll angle: "))

    # Calls function to create coordinate point
    point = coordinate(x, y, z, yaw, pitch, roll)

    # Moves the robot to the given point
    robot.move(
        "pose",
        vals = point,
        velocity = 20,
        acceleration = 20,
        cnt_val = 0,
        linear=False,
    )