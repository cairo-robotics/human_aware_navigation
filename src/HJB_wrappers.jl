using BellmanPDEs
using LazySets
using GridInterpolations
using StaticArraysCore

function get_HJB_environment(vehicle_params,exp_details)

    world = exp_details.env
    workspace = VPolygon([[0.0, 0.0], [world.length, 0.0], [world.length, world.breadth], [0.0, world.breadth]])
    obstacle_list = Array{Any,1}()
    for obs in world.obstacles
        push!(obstacle_list, VPolyCircle([obs.x,obs.y], obs.r+0.1))
    end
    goal = VPolyCircle([vehicle_params.goal.x, vehicle_params.goal.y], exp_details.radius_around_vehicle_goal)
    env = define_environment(workspace, obstacle_list, goal)
    return env
end

function get_HJB_vehicle(vehicle_params)
    body_dims = [vehicle_params.length, vehicle_params.breadth]
    origin_to_cent = [vehicle_params.dist_origin_to_center, 0.0]
    veh_body = define_vehicle(vehicle_params.wheelbase, body_dims, origin_to_cent, vehicle_params.max_steering_angle, vehicle_params.max_speed)
    return veh_body
end

function get_state_grid(world, vehicle_params)
    state_space = [[0.0, world.length], [0.0, world.breadth], [-pi, pi], [0.0, vehicle_params.max_speed]]
    dx_sizes = [0.5, 0.5, deg2rad(18.0), 1/3]
    angle_wrap = [false, false, true, false]
    sg = define_state_grid(state_space, dx_sizes, angle_wrap)
    return sg
end

function find_HJB_path(vehicle, vehicle_params, rollout_guide, exp_details)

    current_state = SVector(vehicle.x,vehicle.y,wrap_between_negative_pi_to_pi(vehicle.theta),vehicle.v)
    current_vehicle = vehicle
    path_x = [current_state[1]]
    path_y = [current_state[2]]
    path_theta = [current_state[3]]
    i = 1

    while(!is_within_range(current_state[1],current_state[2], vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal) && i<100)
        vehicle_action = HJB_policy(current_state,0.0,rollout_get_actions,rollout_get_cost,exp_details.one_time_step,rollout_guide.q_value_array,
                                rollout_guide.value_array,rollout_guide.veh,rollout_guide.state_grid)

        new_vehicle_position = propogate_vehicle(current_vehicle, vehicle_params, vehicle_action[2], vehicle_action[1], rollout_guide.Dt)
        current_state = SVector(new_vehicle_position.x,new_vehicle_position.y,wrap_between_negative_pi_to_pi(new_vehicle_position.theta),new_vehicle_position.v)
        current_vehicle = new_vehicle_position
        push!(path_x, current_state[1])
        push!(path_y, current_state[2])
        push!(path_theta, current_state[3])
        i +=1
    end

    return path_x,path_y,path_theta
end

function rollout_get_actions(x, Dt, veh)
    # set change in velocity (Dv) limit
    Dv_lim = 0.5
    # set steering angle (phi) limit
    phi_max = 0.475
    Dtheta_lim = deg2rad(45)

    v = x[4]
    vp = v + Dv_lim
    vn = v - Dv_lim

    phi_lim = atan(Dtheta_lim * 1/Dt * 1/abs(v) * veh.l)
    phi_lim = clamp(phi_lim, 0.0, phi_max)

    phi_lim_p = atan(Dtheta_lim * 1/Dt * 1/abs(vp) * veh.l)
    phi_lim_p = clamp(phi_lim_p, 0.0, phi_max)

    phi_lim_n = atan(Dtheta_lim * 1/Dt * 1/abs(vn) * veh.l)
    phi_lim_n = clamp(phi_lim_n, 0.0, phi_max)

    actions = SVector{21, SVector{2, Float64}}(
        (-phi_lim_n, -Dv_lim),        # Dv = -Dv
        (-2/3*phi_lim_n, -Dv_lim),
        (-1/3*phi_lim_n, -Dv_lim),
        (0.0, -Dv_lim),
        (1/3*phi_lim_n, -Dv_lim),
        (2/3*phi_lim_n, -Dv_lim),
        (phi_lim_n, -Dv_lim),

        (-phi_lim, 0.0),       # Dv = 0.0
        (-2/3*phi_lim, 0.0),
        (-1/3*phi_lim, 0.0),
        (0.0, 0.0),
        (1/3*phi_lim, 0.0),
        (2/3*phi_lim, 0.0),
        (phi_lim, 0.0),

        (-phi_lim_p, Dv_lim),        # Dv = +Dv
        (-2/3*phi_lim_p, Dv_lim),
        (-1/3*phi_lim_p, Dv_lim),
        (0.0, Dv_lim),
        (1/3*phi_lim_p, Dv_lim),
        (2/3*phi_lim_p, Dv_lim),
        (phi_lim_p, Dv_lim)
        )

    ia_set = collect(1:length(actions))

    return actions,ia_set
end

function rollout_get_cost(x, a, Dt, veh)
    # reward_x_a = 0
    # low speed penalty
    # reward_x_a += -(veh.v_max - abs(x[4] + a[2]))/veh.v_max
    # time penalty
    # reward_x_a += -1.0
    # return reward_x_a
    return -Dt
end

function HJBPolicy(HJB_planning_details, exp_details, vehicle_params)

    HJB_env = get_HJB_environment(vehicle_params,exp_details)
    HJB_veh = get_HJB_vehicle(vehicle_params)
    HJB_sg = get_state_grid(exp_details.env, vehicle_params)

    #Solve HJB
    q_value_array,value_array = solve_HJB_PDE(rollout_get_actions, rollout_get_cost, HJB_planning_details.Dt, HJB_env, HJB_veh,
                                                HJB_sg, HJB_planning_details.Dval_tol, HJB_planning_details.max_solve_steps)

    return HJBPolicy(
        HJB_planning_details.Dt,
        value_array,
        q_value_array,
        rollout_get_actions,
        rollout_get_cost,
        HJB_env,
        HJB_veh,
        HJB_sg
        )
end

#=
START_X = SVector(3.21, 1.5, deg2rad(60),0.0)
rollout_policy(START_X, -0.5, rollout_get_actions, rollout_get_cost, 0.5, rollout_guide.value_array, rollout_guide.veh, rollout_guide.state_grid)
=#
