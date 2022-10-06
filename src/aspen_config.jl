input =  input_parameters(
    MersenneTwister(111), #=rng::AbstractRNG=#
    5.518, #=env_length::Float64=#
    11.036, #=env_breadth::Float64=#
    obstacle_location[], obstacles::Array{obstacle_location,1}=#
    200, #=num_humans::Int64=#
    1.0, #=human_start_v::Float64=#
    2.75, #=veh_start_x::Float64=#
    0.5, #=veh_start_y::Float64=#
    0.0, #=veh_start_theta::Float64=#
    0.0, #=veh_start_v::Float64=#
    2.759, #=veh_goal_x::Float64=#
    11.0, #=veh_goal_y::Float64=#
    0.3, #=veh_L::Float64=#
    3.0, #=veh_max_speed::Float64=#
    30.0, #=lidar_range::Float64=#
    6, #=num_nearby_humans::Int64=#
    2*pi/3, #=cone_half_angle::Float64=#
    0.07, #=pomdp_discount_factor::Float64=#
    1.0, #=min_safe_distance_from_human::Float64=#
    1.0, #=min_safe_distance_from_obstacle::Float64=#
    1.0, #=radius_around_vehicle_goal::Float64=#
    0.5, #=max_risk_distance::Float64=#
    -100.0, #=human_collision_penalty::Float64=#
    -100.0, #=obstacle_collision_penalty::Float64=#
    1000.0, #=goal_reached_reward::Float64=#
    10, #=num_segments_in_one_time_step::Int64=#
    1.0, #=observation_discretization_length::Float64=#
    100, #=tree_search_max_depth::Int64=#
    100, #=num_scenarios::Int64=#
    0.3, #=pomdp_planning_time::Float64=#
    0.5, #=one_time_step::Float64=#
    0.1, #=simulator_time_step::Float64=#
    0.1, #=update_sensor_data_time_interval::Float64=#
    0.2, #=buffer_time::Float64=#
    200.0 #=MAX_TIME_LIMIT::Float64=#
end

function get_details_from_config(config_name)

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
