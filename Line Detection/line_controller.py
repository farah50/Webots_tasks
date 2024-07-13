from controller import Robot, DistanceSensor, Motor

def run_robot(robot):
    time_step = 32
    max_speed = 6.28
    # Motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Enable ir sensors
    ir_sensors = {}
   
    ir_sensors['irL'] = robot.getDevice('irL')
    ir_sensors['irL'].enable(time_step)

    ir_sensors['irR'] = robot.getDevice('irR')
    ir_sensors['irR'].enable(time_step)

    # Step simulation
    while robot.step(time_step) != -1:
        # Read ir sensor
        sensor_values = {name: sensor.getValue() for name, sensor in ir_sensors.items()}

        print(" irL: {} irR: {}".format(sensor_values['irL'], sensor_values['irR']))
        left_speed = max_speed * 0.25
        right_speed = max_speed * 0.25

        if (sensor_values['irR'] > sensor_values['irL']) and (sensor_values['irR'] > 900):
            print("Go left")
            left_speed = -max_speed * 0.25
            right_speed = max_speed * 0.25
        elif (sensor_values['irL'] > sensor_values['irR']) and (sensor_values['irL'] > 900):
            print("Go right")
            left_speed = max_speed * 0.25
            right_speed = -max_speed * 0.25
       

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot) 
