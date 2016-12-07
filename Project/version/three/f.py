	'''Check if we are ontop of the block, otherwise move towards block'''
	# x, y, p = get_closest_block("red")
	# while abs(x-drop_x) >= 100.0 or abs(y-drop_y) >= 100.0:
	# 	print("not there yet", x, y)
	# 	if x-drop_x < -100.0:
	# 		move_y -= .02
	# 	if x-drop_x > 100.0:
	# 		move_y += .02
	# 	if y-drop_y < -100.0:
	# 		move_x += .02
	# 	if y-drop_y > 100.0:
	# 		move_x -= .02

	# 	go_to(move_x, move_y, start[2])
	# 	x, y, p = get_closest_block("red", True)

	# x, y, p = get_closest_block("red", True)
	# print("x:", x, "y:", y)

	# return

	# # Move to right on top of the block
	# x, y, p = get_closest_block("red", False)

	# move_x = start[0] + calc_x(y) + 0.013
	# move_y = calc_y(x) + .025

	# go_to(move_x, move_y, start[2])
	# x, y, p = get_closest_block("red", True)
	# print x, y, x-480, y-300
	# print("done")
	# return

	# '''Pick up block'''
	# go_to(move_x, move_y, table_h)
	# gripper.close()
	# rospy.sleep(1)
	# go_to(move_x, move_y, 0)
	# time.sleep(1)
	# go_to(move_x, move_y, table_h)
	# time.sleep(1)
	# gripper.open()
	# go_to(move_x, move_y, 0)
	# go_to(*start)
