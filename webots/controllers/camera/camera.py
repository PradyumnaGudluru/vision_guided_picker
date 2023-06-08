from controller import Robot

time_step = 32

robot = Robot()

camera = robot.getDevice("camera")

if camera:
    camera.enable(time_step)

range_finder = robot.getDevice("range-finder")

if range_finder:
    range_finder.enable(time_step)


while robot.step(32) != -1:
    pass
