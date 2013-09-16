require "rttlib"
require "rfsm_rtt"
require "rttros"

tc=rtt.getTC()
depl=tc:getPeer("Deployer")
depl:import("communicator")

-- play around with this to try and get youbot peer without starting the service yourself
-- might be only yb=depl:getPeer("youbot")
ya_file = rttros.find_rospack("youbot_driver_rtt") .. "/lua/youbot_test.lua"
dofile(ya_file) 

depl:loadComponent("com", "communicator::Communicator")
com=depl:getPeer("com")

-- connect the youbot ports with the communicator
depl:connect("youbot.Arm1.motor_states","com.arm_motor_states",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Arm1.jointstate","com.joint_states_in",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Arm1.events","com.arm_events",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Base.odometry","com.odom_in",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Base.motor_states","com.base_motor_states",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Base.events","com.base_events",rtt.Variable("ConnPolicy"))

depl:connect("youbot.Base.cmd_twist","com.cmd_vel_out",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Base.cmd_current","com.base_current_command_oro",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Arm1.joint_velocity_command","com.arm_vel_com_oro",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Arm1.joint_position_command","com.arm_pos_com_oro",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Arm1.joint_effort_command","com.arm_tor_com_oro",rtt.Variable("ConnPolicy"))
depl:connect("youbot.Arm1.gripper_cmd","com.arm_gri_pos_oro",rtt.Variable("ConnPolicy"))


print("Configuring communicator")

if not com:configure() then
   error("failed to configure communicator component")
end

--Define ROS topics
depl:stream("com.cmd_vel", rtt.provides("ros"):topic("cmd_vel"))
depl:stream("com.base_current_command", rtt.provides("ros"):topic("base_cmd_current"))
depl:stream("com.arm_position_command", rtt.provides("ros"):topic("/arm_1/arm_controller/position_command"))
depl:stream("com.arm_velocity_command", rtt.provides("ros"):topic("/arm_1/arm_controller/velocity_command"))
depl:stream("com.arm_torques_command", rtt.provides("ros"):topic("/arm_1/arm_controller/torques_command"))
depl:stream("com.grip_position_command", rtt.provides("ros"):topic("/arm_1/gripper_controller/position_command"))

depl:stream("com.joint_states", rtt.provides("ros"):topic("joint_states"))
depl:stream("com.odom", rtt.provides("ros"):topic("odom"))
depl:stream("com.base_motor_states_out", rtt.provides("ros"):topic("base_motor_states"))
depl:stream("com.arm_motor_states_out", rtt.provides("ros"):topic("arm_motor_states"))

print("Starting Communicator")
com:start()


