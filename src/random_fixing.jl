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


function find_vehicle_path(start_x,start_y,start_theta)

end

pts  = update_cart_position_pomdp_planning_2D_action_space_using_HJB(cart_state(10.0, 25.0, pi/2, 0.0, 0.324, location(100.0, 75.0)), 1.0, 100.0,100.0, U_HJB, HJB_action_list, O, HJB_env, HJB_vehicle,10)
xpts = []
ypts = []
thetapts = []
for pt in pts
    push!(xpts, pt[1])
    push!(ypts, pt[2])
end
plot(xpts,ypts)

@profview update_cart_position_pomdp_planning_2D_action_space_using_HJB(cart_state(10.0, 25.0, pi/2, 0.0, 0.324, location(100.0, 75.0)), 1.0, 100.0,100.0, U_HJB, HJB_action_list, O, HJB_env, HJB_vehicle,10)

#
# scatter!([env.cart.x], [env.cart.y], shape=:circle, color="blue", msize= 0.3*plot_size*cart_size/env.length)
# quiver!([env.cart.x],[env.cart.y],quiver=([cos(env.cart.theta)],[sin(env.cart.theta)]), color="blue")

[4.346092283518907, 2.0907808788783315, 3.718524340685583]

start_p = [10.0,25.0,0.0]
# start_p = [4.346092283518907, 2.0907808788783315, 3.718524340685583]
vel = 1.0
x_path_points = [0.0]
y_path_points = [0.0]
theta_path_points = [0.0]
goal = [100.0,75.0]
curr = deepcopy(start_p)
num_steps = 0
while(!is_within_range_check_with_points(curr[1],curr[2], goal[1], goal[2], 2.0) && num_steps<100)

    a = HJB_action(curr, U_HJB, HJB_action_list, O,HJB_env, HJB_vehicle)
    next_points = update_cart_position_pomdp_planning_2D_action_space_using_HJB(cart_state(curr[1], curr[2],curr[3], a[1], 0.324, location(100.0, 75.0)), a[1], 100.0,100.0, U_HJB, HJB_action_list, O, HJB_env, HJB_vehicle,10)
    for pt in next_points
        push!(x_path_points, pt[1])
        push!(y_path_points, pt[2])
        push!(theta_path_points, pt[3])
    end
    curr = [ next_points[end][1], next_points[end][2], next_points[end][3] ]
    num_steps +=1
end

plot(x_path_points, y_path_points)


update_cart_position_pomdp_planning_2D_action_space_using_HJB(cart_state(curr[1], curr[2],curr[3], vel, 0.324, location(100.0, 75.0)), -2, 100.0,100.0, U_HJB, HJB_action_list, O, HJB_env, HJB_vehicle,20)
HJB_action(start_p, U_HJB, HJB_action_list, O,HJB_env, HJB_vehicle)
