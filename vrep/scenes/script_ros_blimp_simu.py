#python

publisher_position_camera = None
publisher_position_head = None
publisher_image = None

def sysCall_init():
    global publisher_position_camera, publisher_position_head, publisher_image
    # Prepare the publishers and subscribers 
    if simROS:
        publisher_position_camera = simROS.advertise('/blimp','geometry_msgs/Vector3')
        publisher_position_head = simROS.advertise('/head','geometry_msgs/Vector3')
        publisher_image = simROS.advertise('/camera','sensor_msgs/Image')

def sysCall_actuation():
    global publisher_position_camera, publisher_position_head, publisher_image
    # Send updated data
    camera = sim.getObject("/View_sensor")
    blimp = sim.getObject("/Blimp")
    head = sim.getObject("/Sphere")
    pos_cam = sim.getObjectPosition(camera, -1)
    pos_head = sim.getObjectPosition(head, -1)
    img = sim.getVisionSensorImg(camera)
    if simROS:
       simROS.publish(publisher_position_camera,{'data':pos_cam})
       simROS.publish(publisher_position_head,{'data':pos_head})
       print(img)
    pass

def sysCall_cleanup():
    global publisher_position_camera, publisher_position_head, publisher_image
    # Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if simROS:
        simROS.shutdownPublisher(publisher_position_camera)
        simROS.shutdownPublisher(publisher_position_head)
        simROS.shutdownPublisher(publisher_image)
