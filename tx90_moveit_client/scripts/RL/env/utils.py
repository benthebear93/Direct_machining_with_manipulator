import kinpy as kp

chain = kp.build_chain_from_urdf(open("/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_moveit_client/scripts/RL/urdf/tx90.urdf").read())
print(chain)