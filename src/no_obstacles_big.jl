scenario1_big =  InputParameters(
    MersenneTwister(), #=rng::AbstractRNG=#
    100.0, #=env_length::Float64=#
    100.0, #=env_breadth::Float64=#
    ObstacleLocation[], #=obstacles::Array{obstacle_location,1}=#
    400, #=num_humans::Int64=#
    1.0, #=human_start_v::Float64=#
    2.75, #=veh_start_x::Float64=#
    10, #=veh_start_y::Float64=#
    0.0, #=veh_start_theta::Float64=#
    0.0, #=veh_start_v::Float64=#
    99.0, #=veh_goal_x::Float64=#
    75.0, #=veh_goal_y::Float64=#
    0.3, #=veh_L::Float64=#
    3.0, #=veh_max_speed::Float64=#
    30.0, #=lidar_range::Float64=#
    6, #=num_nearby_humans::Int64=#
    2*pi/3, #=cone_half_angle::Float64=#
    0.97, #=pomdp_discount_factor::Float64=#
    1.0, #=min_safe_distance_from_human::Float64=#
    1.0, #=min_safe_distance_from_obstacle::Float64=#
    1.0, #=radius_around_vehicle_goal::Float64=#
    1.0, #=max_risk_distance::Float64=#
    -100.0, #=human_collision_penalty::Float64=#
    -100.0, #=obstacle_collision_penalty::Float64=#
    1000.0, #=goal_reached_reward::Float64=#
    10, #=num_segments_in_one_time_step::Int64=#
    1.0, #=observation_discretization_length::Float64=#
    100, #=tree_search_max_depth::Int64=#
    100, #=num_scenarios::Int64=#
    4.0, #=d_near::Float64 = 0.5=#
    6.0, #=d_far::Float64 = 0.5=#
    0.5, #=pomdp_planning_time::Float64=#
    0.5, #=one_time_step::Float64=#
    0.1, #=simulator_time_step::Float64=#
    0.1, #=update_sensor_data_time_interval::Float64=#
    0.0, #=buffer_time::Float64=#
    100.0 #=MAX_TIME_LIMIT::Float64=#
)
