using Random
using DataStructures

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
    number_sudden_stops::Int64
    number_risky_scenarios::Int64
    num_humans_env::Int64
    one_time_step::Float64
    simulator_time_step::Float64
    lidar_range::Float64
    MAX_TIME_LIMIT::Float64
    time_taken::Float64
    min_safe_distance_from_human::Float64
    radius_around_vehicle_goal::Float64
    max_risk_distance::Float64
    update_sensor_data_time_interval::Float64
    buffer_time::Float64
    vehicle_ran_into_boundary_wall::Bool
    vehicle_ran_into_obstacle::Bool
    vehicle_reached_goal::Bool
    human_goal_locations::Array{location,1}
    pomdp_planners::OrderedDict
    nearby_humans::OrderedDict
    despot_trees::OrderedDict
    vehicle_actions::OrderedDict
    sim_objects::OrderedDict
    risky_scenarios::OrderedDict
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
    planning_time::Float64
    tree_search_max_depth::Int64
    num_scenarios::Int64
    discount_factor::Float64
    one_time_step::Float64
end
