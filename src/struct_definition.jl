using Random
using DataStructures
using Parameters

#Various different Struct definitions

struct location
    x::Float64
    y::Float64
end

struct obstacle_location
    x::Float64
    y::Float64
    r::Float64 #Radius of the obstacle which is assumed to be a circle
end

#=
Used to store belief over goals for a single human
Need this for every human
=#
struct belief_over_human_goals
    pdf::Array{Float64,1}
end

struct human_state
    x::Float64
    y::Float64
    v::Float64
    goal::location
end

struct human_parameters
    id::Int64
end

struct nearby_humans
    position_data::Array{human_state,1}
    ids::Array{Int64,1}
    belief::Array{belief_over_human_goals,1}
end

struct Vehicle
    x::Float64
    y::Float64
    theta::Float64
    v::Float64
end

struct vehicle_sensor
    lidar_data::Array{human_state,1}
    ids::Array{Int64,1}
    belief::Array{belief_over_human_goals,1}
end

struct es_vehicle_parameters
    L::Float64
    max_speed::Float64
    goal::location
end

struct ls_vehicle_parameters
    L::Float64
    max_speed::Float64
    goal::location
    hybrid_astar_path::Array{Float64,1}
end

struct experiment_environment
    length::Float64
    breadth::Float64
    obstacles::Array{obstacle_location,1}
end

mutable struct experiment_details
    user_defined_rng::AbstractRNG
    num_humans_env::Int64
    human_start_v::Float64
    one_time_step::Float64
    simulator_time_step::Float64
    lidar_range::Float64
    MAX_TIME_LIMIT::Float64
    min_safe_distance_from_human::Float64
    radius_around_vehicle_goal::Float64
    max_risk_distance::Float64
    update_sensor_data_time_interval::Float64
    buffer_time::Float64
    human_goal_locations::Array{location,1}
    env::experiment_environment
end

mutable struct pomdp_planning_details
    num_nearby_humans::Int64
    cone_half_angle::Float64
    min_safe_distance_from_human::Float64
    min_safe_distance_from_obstacle::Float64
    radius_around_vehicle_goal::Float64
    human_collision_penalty::Float64
    obstacle_collision_penalty::Float64
    goal_reached_reward::Float64
    max_vehicle_speed::Float64
    num_segments_in_one_time_step::Int64
    observation_discretization_length::Float64
    d_near::Float64
    d_far::Float64
    planning_time::Float64
    tree_search_max_depth::Int64
    num_scenarios::Int64
    discount_factor::Float64
    one_time_step::Float64
end

@with_kw mutable struct input_parameters
    rng::AbstractRNG = MersenneTwister(rand(UInt32))
    env_length::Float64 = 5.518
    env_breadth::Float64 = 11.036
    obstacles::Array{obstacle_location,1} = obstacle_location[]
    num_humans_env::Int64 = 200
    human_start_v::Float64 = 1.0
    veh_start_x::Float64 = 2.75
    veh_start_y::Float64 = 0.5
    veh_start_theta::Float64 = 0.0
    veh_start_v::Float64 = 0.0
    veh_goal_x::Float64 = 2.759
    veh_goal_y::Float64 = 11.0
    veh_L::Float64 = 0.3
    veh_max_speed::Float64 = 3.0
    lidar_range::Float64 = 30.0
    num_nearby_humans::Int64 = 6
    cone_half_angle::Float64 = 2*pi/3
    pomdp_discount_factor::Float64 = 0.97
    min_safe_distance_from_human::Float64 = 1.0
    min_safe_distance_from_obstacle::Float64 = 1.0
    radius_around_vehicle_goal::Float64 = 1.0
    max_risk_distance::Float64 = 0.5
    human_collision_penalty::Float64 = -100.0
    obstacle_collision_penalty::Float64 = -100.0
    goal_reached_reward::Float64 = 1000.0
    num_segments_in_one_time_step::Int64 = 10
    observation_discretization_length::Float64 = 1.0
    tree_search_max_depth::Int64 = 100
    num_scenarios::Int64 = 100
    d_near::Float64 = 0.5
    d_far::Float64 = 0.5
    pomdp_planning_time::Float64 = 0.3
    one_time_step::Float64 = 0.5
    simulator_time_step::Float64 = 0.1
    update_sensor_data_time_interval::Float64 = 0.1
    buffer_time::Float64 = 0.2
    MAX_TIME_LIMIT::Float64 = 200.0
end

mutable struct Output
    number_sudden_stops::Int64
    number_risky_scenarios::Int64
    time_taken::Float64
    vehicle_ran_into_boundary_wall::Bool
    vehicle_ran_into_obstacle::Bool
    vehicle_reached_goal::Bool
    pomdp_planners::OrderedDict
    nearby_humans::OrderedDict
    b_root::OrderedDict
    despot_trees::OrderedDict
    vehicle_actions::OrderedDict
    sim_objects::OrderedDict
    risky_scenarios::OrderedDict
end
