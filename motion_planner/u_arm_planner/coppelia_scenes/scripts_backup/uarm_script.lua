function sysCall_init()
    motorHandles={-1,-1,-1,-1}
    for i=1,4,1 do
        motorHandles[i]=sim.getObject('../motor'..i)
    end
    
    auxMotor1=sim.getObject('../auxMotor1')
    auxMotor2=sim.getObject('../auxMotor2')
    
    gripperHandle=sim.getObject('../uarmVacuumGripper')

    if simROS2 then
        sim.addLog(sim.verbosity_scriptinfos, "ROS 2 Connected!")
        
        pub = simROS2.createPublisher('/joint_states', 'sensor_msgs/msg/JointState')
        
        sub = simROS2.createSubscription('/uarm/command', 'std_msgs/msg/Float64MultiArray', 'cmd_callback')
    else
        sim.addLog(sim.verbosity_scripterrors, "ERROR: ROS2 plugin doesnt loaded!")
    end
end

function cmd_callback(msg)
    if msg.data and #msg.data >= 4 then
        for i=1,4,1 do
            sim.setJointTargetPosition(motorHandles[i], msg.data[i])
        end
    end
end

function sysCall_actuation()
    if simROS2 then
        local positions = {}
        local names = {}
        
        for i=1,4,1 do
            table.insert(positions, sim.getJointPosition(motorHandles[i]))
            table.insert(names, 'motor'..i)
        end
        
        local data={}
        data.header={stamp=simROS2.getTime(), frame_id='world'}
        data.name=names
        data.position=positions
        data.velocity={}
        data.effort={}
        
        simROS2.publish(pub, data)
    end
end


function sysCall_joint(inData)
    local errorValue = 0
    
    if inData.handle==auxMotor1 then
        local t2 = -sim.getJointPosition(motorHandles[2]) + 104*math.pi/180
        local t3 = sim.getJointPosition(motorHandles[3]) - 59.25*math.pi/180
        errorValue = t3 - t2 - inData.pos
    end
    
    if inData.handle==auxMotor2 then
        local t3 = sim.getJointPosition(motorHandles[3]) - 59.25*math.pi/180
        errorValue = -t3 - inData.pos
    end
    
    local ctrl = errorValue * 20
    local maxVelocity = ctrl
    
    if (maxVelocity > inData.maxVel) then
        maxVelocity = inData.maxVel
    end
    if (maxVelocity < -inData.maxVel) then
        maxVelocity = -inData.maxVel
    end
    
    local outData={}
    outData.vel = maxVelocity
    outData.force = inData.maxForce
    return outData
end

function sysCall_cleanup()

end