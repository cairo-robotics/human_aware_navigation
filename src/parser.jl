function get_details_from_input_parameters(obj)

    pomdp_details = pomdp_planning_details(
        obj.num_nearby_humans, #=num_nearby_humans::Int64=#
        obj.cone_half_angle, #=cone_half_angle::Float64=#
        obj.min_safe_distance_from_human, #=min_safe_distance_from_human::Float64=#
        obj.min_safe_distance_from_obstacle, #=min_safe_distance_from_obstacle::Float64=#
        obj.radius_around_vehicle_goal, #=radius_around_vehicle_goal::Float64=#
        obj.human_collision_penalty, #=human_collision_penalty::Float64=#
        obj.obstacle_collision_penalty, #=obstacle_collision_penalty::Float64=#
        obj.goal_reached_reward, #=goal_reached_reward::Float64=#
        obj.veh_max_speed, #=max_vehicle_speed::Float64=#
        obj.num_segments_in_one_time_step, #=num_segments_in_one_time_step::Int64=#
        obj.observation_discretization_length, #=observation_discretization_length::Float64=#
        obj.d_near, #=d_near::Float64=#
        obj.d_far, #=d_far::Float64=#
        obj.pomdp_planning_time, #=planning_time::Float64=#
        obj.tree_search_max_depth, #=tree_search_max_depth::Int64=#
        obj.num_scenarios, #=num_scenarios::Int64=#
        obj.pomdp_discount_factor, #=discount_factor::Float64=#
        obj.one_time_step #=one_time_step::Float64=#
    )

    exp_details = experiment_details(
        obj.rng, #=user_defined_rng::AbstractRNG=#
        obj.num_humans_env, #=num_humans_env::Int64=#
        obj.human_start_v, #=human_start_v::Float64=#
        obj.one_time_step, #=one_time_step::Float64=#
        obj.simulator_time_step, #=simulator_time_step::Float64=#
        obj.lidar_range, #=lidar_range::Float64=#
        obj.MAX_TIME_LIMIT, #=MAX_TIME_LIMIT::Float64=#
        obj.min_safe_distance_from_human, #=min_safe_distance_from_human::Float64=#
        obj.radius_around_vehicle_goal, #=radius_around_vehicle_goal::Float64=#
        obj.max_risk_distance, #=max_risk_distance::Float64=#
        obj.update_sensor_data_time_interval, #=update_sensor_data_time_interval::Float64=#
        obj.buffer_time, #=buffer_time::Float64=#
        location[], #=human_goal_locations::Array{location,1}=#
        experiment_environment(0.0,0.0,obstacle_location[])#=env::experiment_environment=#
    )

    output = Output(
        0, #=number_sudden_stops::Int64=#
        0, #=number_risky_scenarios::Int64=#
        0.0, #=time_taken::Float64=#
        false, #=vehicle_ran_into_boundary_wall::Bool=#
        false, #=vehicle_ran_into_obstacle::Bool=#
        false, #=vehicle_reached_goal::Bool=#
        OrderedDict(), #=pomdp_planners::OrderedDict=#
        OrderedDict(), #=nearby_humans::OrderedDict=#
        OrderedDict(), #=b_root::OrderedDict=#
        OrderedDict(), #=despot_trees::OrderedDict=#
        OrderedDict(), #=vehicle_actions::OrderedDict=#
        OrderedDict(), #=sim_objects::OrderedDict=#
        OrderedDict(), #=risky_scenarios::OrderedDict=#
    )

    return exp_details, pomdp_details, output
end
