include("struct_definition.jl")
include("environment.jl")
include("utils.jl")
include("pomdp_planning.jl")
include("belief_tracker.jl")
include("simulator.jl")
using DataStructures
using FileIO
using JLD2
using D3Trees

#Define some global variables
GLOBAL_RADIUS_AROUND_GOAL = 1.0
GLOBAL_TIME_STEP = 0.5

function main()

    config_rng = MersenneTwister(111)
    env = generate_environment(5.518, 11.036, obstacle_location[])
    env = generate_environment(50.0, 50.0, obstacle_location[])
    list_human_goals = get_human_goals(env)
    veh = Vehicle(2.75, 0.5, 0.0, 0.0)
    veh_sensor_data = vehicle_sensor(human_state[],Int64[],belief_over_human_goals[])
    veh_goal = location(env.length/2,env.breadth)
    veh_params = es_vehicle_parameters(0.3,3.0,veh_goal)
    num_humans = 200
    env_humans, env_humans_params = generate_humans(env,veh,1.0,list_human_goals,num_humans,config_rng)
    current_time_value = 0.0
    current_vehicle_speed = 0.0
    current_vehicle_steering_angle = 0.0
    current_action = action_extended_space_POMDP_planner(0.0,0.0)

    pomdp_details = pomdp_planning_details(
        6, #=num_nearby_humans::Int64=#
        2*pi/3, #=cone_half_angle::Float64=#
        1.0, #=min_safe_distance_from_human::Float64=#
        1.0, #=min_safe_distance_from_obstacle::Float64=#
        1.0, #=radius_around_vehicle_goal::Float64=#
        -100.0, #=human_collision_penalty::Float64=#
        -100.0, #=obstacle_collision_penalty::Float64=#
        1000.0, #=goal_reached_reward::Float64=#
        3.0, #=max_vehicle_speed::Float64=#
        10, #=num_segments_in_one_time_step::Int64=#
        1.0, #=observation_discretization_length::Float64=#
        0.3, #=planning_time::Float64=#
        100, #=tree_search_max_depth::Int64=#
        100, #=num_scenarios::Int64=#
        0.97, #=discount_factor::Float64=#
        GLOBAL_TIME_STEP #=one_time_step::Float64=#
    )

    exp_details = experiment_details(
        config_rng, #=user_defined_rng::AbstractRNG=#
        0, #=number_sudden_stops::Int64=#
        0, #=number_risky_scenarios::Int64=#
        num_humans, #=num_humans_env::Int64=#
        GLOBAL_TIME_STEP, #=one_time_step::Float64=#
        0.1, #=simulator_time_step::Float64=#
        30.0, #=lidar_range::Float64=#
        200.0, #=MAX_TIME_LIMIT::Float64=#
        0.0, #=time_taken::Float64=#
        1.0, #=min_safe_distance_from_human::Float64=#
        1.0, #=radius_around_vehicle_goal::Float64=#
        0.5, #=max_risk_distance::Float64=#
        0.1, #=update_sensor_data_time_interval::Float64=#
        0.2, #=buffer_time::Float64=#
        false, #=vehicle_ran_into_boundary_wall::Bool=#
        false, #=vehicle_ran_into_obstacle::Bool=#
        false, #=vehicle_reached_goal::Bool=#
        list_human_goals, #=human_goal_locations::Array{location,1}=#
        OrderedDict(), #=pomdp_planners::OrderedDict=#
        OrderedDict(), #=nearby_humans::OrderedDict=#
        OrderedDict(), #=despot_trees::OrderedDict=#
        OrderedDict(), #=vehicle_actions::OrderedDict=#
        OrderedDict(), #=sim_objects::OrderedDict=#
        OrderedDict(), #=risky_scenarios::OrderedDict=#
        env#=env::experiment_environment=#
    )

    extended_space_pomdp = extended_space_POMDP_planner(0.97,0.5,-100.0,2.0,-100.0,GLOBAL_RADIUS_AROUND_GOAL,1000.0,veh_params.max_speed,
                                    GLOBAL_TIME_STEP,10,1.0,env)

    pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound_pomdp_planning(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                        calculate_upper_bound_pomdp_planning,check_terminal=true),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,T_max=2.0,tree_in_info=true)
    pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);

    current_sim_obj = simulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)
    exp_details.vehicle_actions[current_time_value] = current_action
    exp_details.sim_objects[current_time_value] = current_sim_obj
    exp_details.nearby_humans[current_time_value] = nearby_humans(human_state[], Int64[], belief_over_human_goals[])
    # return exp_details
    # try
    while(!stop_simulation!(current_sim_obj,current_time_value,exp_details))

        println("Current time value : ", current_time_value)
        println("Current Vehicle State: ", current_sim_obj.vehicle)
        println("Action to be executed now : ", current_action)
        #=
        Simulate for (one_time_step - pomdp_planning_time - buffer_time) seconds
        =#
        time_duration_until_planning_begins = exp_details.one_time_step - (pomdp_details.planning_time + exp_details.buffer_time)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                current_time_value, time_duration_until_planning_begins, exp_details)
        current_time_value += time_duration_until_planning_begins

        #=
        Take the current status of the environment.
        Predict the vehicle's future position using current_action after (pomdp_planning_time + buffer_time) seconds.
        Initialize scenarios with sampled human goals and predicted human positions after (pomdp_planning_time + buffer_time) seconds.
        Solve the POMDP and find the action.
        =#
        time_duration_until_pomdp_action_determined = pomdp_details.planning_time + exp_details.buffer_time
        predicted_vehicle_state = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params,
                                            current_vehicle_steering_angle, current_vehicle_speed, time_duration_until_pomdp_action_determined)
        # println(predicted_vehicle_state)
        modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params)
        nearby_humans = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,pomdp_details.min_safe_distance_from_human,
                                                pomdp_details.cone_half_angle)
        b = tree_search_scenario_parameters(predicted_vehicle_state.x,predicted_vehicle_state.y,predicted_vehicle_state.theta,predicted_vehicle_state.v,
                            modified_vehicle_params, exp_details.human_goal_locations, nearby_humans.position_data, nearby_humans.belief)
        next_action, info = action_info(pomdp_planner, b)
        exp_details.despot_trees[current_time_value] = info
        exp_details.nearby_humans[current_time_value] = nearby_humans

        #=
        Simulate for (pomdp_planning_time + buffer_time) seconds
        =#
        time_duration_until_next_action_is_applied =  pomdp_details.planning_time + exp_details.buffer_time
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                current_time_value, time_duration_until_next_action_is_applied, exp_details)

        current_vehicle_speed = clamp(current_vehicle_speed+next_action.delta_speed, 0.0, pomdp_details.max_vehicle_speed)
        current_vehicle_steering_angle = get_steering_angle(current_sim_obj.vehicle_params.L, next_action.delta_heading_angle, current_vehicle_speed, exp_details.one_time_step)
        current_action = next_action
        current_time_value += time_duration_until_next_action_is_applied
        #=
        Store relevant values in exp_details
        =#
        exp_details.vehicle_actions[current_time_value] = current_action
        if(current_action.delta_speed == -10.0)
            exp_details.number_sudden_stops += 1
        end
    end

    # catch e
    #     println("\n Things failed during the simulation. \n The error message is : \n ")
    #     println(e)
    #     return exp_details
    # end
    exp_details.time_taken = current_time_value
    println("DONE")
    return exp_details
end

a = main()
create_gif_flag = true
if(create_gif_flag)
    anim = @animate for k ∈ keys(a.sim_objects)
        display_env(a, k);
        #savefig("./plots_just_2d_action_space_pomdp_planner/plot_"*string(i)*".png")
    end
    gif(anim, "es_planner.gif", fps = 10)
end
