import BellmanPDEs as BPDE
using LazySets
using StaticArrays

struct HJBEnvironment{P,Q,R}
    workspace::P
    obstacle_list::Q
    goal::R
end

function get_HJB_environment(vehicle_params,exp_details)

    l = exp_details.env.length
    b = exp_details.env.breadth
    workspace = VPolygon([ SVector(0.0, 0.0),
                           SVector(l, 0.0),
                           SVector(l, b),
                           SVector(0.0, b)]
                            )

    workspace_obstacles = exp_details.env.obstacles
    obstacle_list = Array{VPolygon,1}()
    for obs in workspace_obstacles
        push!(obstacle_list, VPolyCircle((obs.x,obs.y),obs.r+0.1))
    end
    obstacle_list = SVector{length(workspace_obstacles),VPolygon{Float64, SVector{2, Float64}}}(obstacle_list)

    workspace_goal = (vehicle_params.goal.x, vehicle_params.goal.y)
    goal = VPolyCircle(workspace_goal, exp_details.radius_around_vehicle_goal)

    env = HJBEnvironment(workspace, obstacle_list, goal)
    return env
end

struct HJBVehicleParameters{T}
    l::Float64
    body_dims::SVector{2,Float64}
    radius_vb::Float64
    origin_to_cent::SVector{2,Float64}
    origin_body::T
    phi_max::Float64
    delta_theta_max::Float64
    v_max::Float64
    delta_speed::Float64
end


function get_HJB_vehicle(vehicle_params)
    body_dims = SVector(vehicle_params.length, vehicle_params.breadth)
    radius_around_vehicle = sqrt((0.5*body_dims[1])^2 + (0.5*body_dims[2])^2)
    # (x,y) distance of the vehicle center to the origin of the vehicle which is the center point of the rear axis
    origin_to_cent = SVector(vehicle_params.dist_origin_to_center, 0.0) 
    vehicle_body_at_origin = get_vehicle_body(body_dims, origin_to_cent)
    
    veh_body = HJBVehicleParameters(
                    vehicle_params.wheelbase,
                    body_dims,
                    radius_around_vehicle,
                    origin_to_cent,
                    vehicle_body_at_origin,
                    vehicle_params.max_steering_angle,
                    pi/4,
                    vehicle_params.max_speed,
                    0.5
                    )
    return veh_body
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

function HJB_action_function(x, Dt, veh)
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

    num_actions = 21
    actions = SVector{num_actions, SVector{2, Float64}}(
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

    return actions
end

function HJB_cost_function(x, a, Dt, veh)
    # reward_x_a = 0
    # low speed penalty
    # reward_x_a += -(veh.v_max - abs(x[4] + a[2]))/veh.v_max
    # time penalty
    # reward_x_a += -1.0
    # return reward_x_a
    return -Dt
end

function HJBPolicy(HJB_planning_details, exp_details, vehicle_params)

    Dt = HJB_planning_details.Dt
    Dval_tol = HJB_planning_details.Dval_tol
    max_solve_steps = HJB_planning_details.max_solve_steps
    world = exp_details.env

    l = world.length
    b = world.breadth
    max_speed = 2.0
    state_space = SVector{4,Tuple{Float64,Float64}}([
                    (0.0,l), #Range in x
                    (0.0,b), #Range in y
                    (-pi,pi), #Range in theta
                    (0.0,vehicle_params.max_speed) #Range in v
                    ])
    dx_sizes = SVector(0.5, 0.5, deg2rad(18.0), 0.5)
    angle_wrap = SVector(false, false, true, false)

    HJB_env = get_HJB_environment(vehicle_params,exp_details)
    HJB_veh = get_HJB_vehicle(vehicle_params)
    HJB_sg = get_state_grid(state_space,dx_sizes,angle_wrap)

    #Solve HJB
    all_states = get_all_states(HJB_sg.state_grid)
    iterators = get_iterators(state_space,dx_sizes)
    Q_values,V_values = solve_HJB_PDE(rollout_get_actions, rollout_get_cost, Dt, HJB_env, HJB_veh,
                                        HJB_sg, all_states, iterators, Dval_tol, max_solve_steps)

    return HJBPolicy(
        Dt,
        V_values,
        Q_values,
        rollout_get_actions,
        rollout_get_cost,
        HJB_env,
        HJB_veh,
        HJB_sg
        )

        # return HJBPolicy(
        # Dt,
        # V_values,
        # Q_values,
        # HJB_env,
        # HJB_veh,
        # HJB_sg
        # )
end

#=
START_X = SVector(3.21, 1.5, deg2rad(60),0.0)
rollout_policy(START_X, -0.5, rollout_get_actions, rollout_get_cost, 0.5, rollout_guide.value_array, rollout_guide.veh, rollout_guide.state_grid)
=#


function get_HJB_rollout_guide(HJB_planning_details, exp_details, vehicle_params)

    Δt = HJB_planning_details.Dt
    ϵ = HJB_planning_details.Dval_tol
    max_solve_steps = HJB_planning_details.max_solve_steps
    world = exp_details.env

    l = world.length
    b = world.breadth
    max_speed = vehicle_params.max_speed
    state_range = SVector{4,Tuple{Float64,Float64}}([
                    (0.0,l), #Range in x
                    (0.0,b), #Range in y
                    (-pi,pi), #Range in theta
                    (0.0,max_speed) #Range in v
                    ])
    dx_sizes = SVector(0.5, 0.5, deg2rad(18.0), 0.5)

    HJB_env = get_HJB_environment(vehicle_params,exp_details)
    HJB_veh = get_HJB_vehicle(vehicle_params)

    P = BPDE.Problem(state_range,dx_sizes,HJB_env,HJB_veh,HJB_action_function,HJB_cost_function)

    hjb_solver = BPDE.HJBSolver(P,
                max_steps=max_solve_steps,
                ϵ=ϵ,
                Δt=Δt
                )
    BPDE.solve(hjb_solver,P)

    safe_value_limit = 750.0
    hjb_planner = BPDE.HJBPlanner(hjb_solver,P,safe_value_limit)
    return hjb_planner
end