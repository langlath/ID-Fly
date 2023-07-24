function pos_callback(msg)
    -- blimp = sim.getObject('/Blimp')
    pos = sim.getObjectPosition(blimp, -1)
    orient = sim.getObjectOrientation(blimp, -1)
    yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(orient[1], orient[2], orient[3])
    data = msg['data']
    -- print(pos[1], data[1])
    -- print('yaw = ', yaw)
    new_pos = {0, 0, 0}
    new_pos[2] = pos[2] + dt * (data[2] * math.cos(yaw + math.pi / 2) + data[1] * math.sin(yaw + math.pi / 2))
    new_pos[1] = pos[1] + dt * (data[1] * math.cos(yaw + math.pi / 2) + data[2] * math.sin(yaw + math.pi / 2))
    new_pos[3] = pos[3] + dt * data[3]
    sim.setObjectPosition(blimp, -1, new_pos)

    yaw = yaw + dt * data[4]
    new_alpha, new_beta, new_gamma = sim.yawPitchRollToAlphaBetaGamma(yaw, pitch, roll)
    new_orient = {new_alpha, new_beta, new_gamma}
    sim.setObjectOrientation(blimp, -1, new_orient)
end


function sysCall_init()        
    -- Get some handles (as usual !):
    camera = sim.getObject('/Vision_sensor')
    head = sim.getObject('/Sphere')    
    blimp = sim.getObject('/Blimp')
    -- Enable an image publisher and subscriber:
    publisher_image = simROS.advertise('/image', 'sensor_msgs/Image')
    publisher_position_camera = simROS.advertise('/camera', 'geometry_msgs/Vector3')
    publisher_position_head = simROS.advertise('/head', 'geometry_msgs/Vector3')
    publisher_position_blimp = simROS.advertise('/blimp', 'geometry_msgs/Vector3')
    simROS.publisherTreatUInt8ArrayAsString(publisher_image) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua) 
        
    subscriber_speed_blimp = simROS.subscribe('/speed_blimp', 'std_msgs/Float64MultiArray', 'pos_callback')

    dt = 0.1
end
 
function sysCall_sensing()
    -- Publish the image of the vision sensor:
    local img_data,w,h = sim.getVisionSensorCharImage(camera)
    sim.transformImage(img_data, {w, h}, 4)
    img_msg = {}
    img_msg['header'] = {seq=0, stamp=simROS.getTime(), frame_id="a"}
    img_msg['height'] = h
    img_msg['width'] = w
    img_msg['encoding'] = 'rgb8'
    img_msg['is_bigendian'] = 1
    img_msg['step'] = w*3
    img_msg['data'] = img_data
    simROS.publish(publisher_image, img_msg)

    pos_cam = sim.getObjectPosition(camera, -1)
    cam_msg = {}
    cam_msg['x'] = pos_cam[1]
    cam_msg['y'] = pos_cam[2]
    cam_msg['z'] = pos_cam[3]
    simROS.publish(publisher_position_camera, cam_msg)

    pos_head = sim.getObjectPosition(head, -1)
    head_msg = {}
    head_msg['x'] = pos_head[1]
    head_msg['y'] = pos_head[2]
    head_msg['z'] = pos_head[3]
    simROS.publish(publisher_position_head, head_msg)

    pos_blimp = sim.getObjectPosition(blimp, -1)
    blimp_msg = {}
    blimp_msg['x'] = pos_blimp[1]
    blimp_msg['y'] = pos_blimp[2]
    blimp_msg['z'] = pos_blimp[3]
    simROS.publish(publisher_position_blimp, blimp_msg)

end
 
function sysCall_cleanup()
    -- Shut down publisher and subscriber. Not really needed from a simulation script (automatic shutdown)
    simROS.shutdownPublisher(publisher_image)
    simROS.shutdownPublisher(publisher_position_camera)
    simROS.shutdownPublisher(publisher_position_head)
    simROS.shutdownPublisher(publisher_position_blimp)

    simROS.shutdownSubscriber(subscriber_speed_blimp)
end