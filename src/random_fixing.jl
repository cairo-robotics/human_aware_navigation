vl = 0.324
vv = 4.0
steer_angle = 0.06353163608639502
steer_angle = -0.0846204431870001
initial_state = [10.0,25.0,0.0]
extra_parameters = [vv, vl, steer_angle]
x,y,theta = get_intermediate_points(initial_state, 1.0, extra_parameters);
plot(x,y)

delta_theta = pi/3
sa = atan(vl*delta_theta/vv)

println("Required steering angle = ", sa, " ", sa*180/pi)


# Fix rollout policy
# Fix steering angle issue
