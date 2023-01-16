using POMDPs
using Distributions
using Random
import POMDPs: initialstate_distribution, actions, gen, discount, isterminal
using POMDPModels, POMDPTools, ARDESPOT
using ParticleFilters
using BenchmarkTools
using Debugger
using LinearAlgebra
using DifferentialEquations

#=
Function to check terminal state
=#
function is_terminal_state(s,terminal_state)
    if(terminal_state.x == s.vehicle_x && terminal_state.y == s.vehicle_y)
        return true
    else
        return false
    end
end

#=
************************************************************************************************
Generate Initial POMDP state based on the scenario provided by random operator.
=#
struct TreeSearchScenarioParameters{P}
    vehicle_x::Float64
    vehicle_y::Float64
    vehicle_theta::Float64
    vehicle_v::Float64
    vehicle_params::P
    human_goals::Array{Location,1}
    num_nearby_humans::Int64
    nearby_humans::Array{HumanState,1}
    nearby_humans_belief::Array{HumanGoalsBelief,1}
    world_length::Float64
    world_breadth::Float64
    time_duration::Float64
end

#=
************************************************************************************************
Reward functions for the POMDP model
=#
# Human Collision Penalty
function human_collision_penalty(human_collision_flag::Bool, penalty::Float64)
    if(human_collision_flag)
        return penalty
    else
        return 0.0
    end
end

# Obstacle Collision Penalty
function obstacle_collision_penalty(obstacle_collision_flag::Bool, penalty::Float64)
    if(obstacle_collision_flag)
        return penalty
    else
        return 0.0
    end
end

# Goal Reward
function vehicle_goal_reached_reward(goal_reached_flag::Bool, goal_reward::Float64)
    if(goal_reached_flag)
        return goal_reward
    else
        return 0.0
    end
end

# Low Speed Penalty
function low_speed_penalty(current_vehicle_speed::Float64, max_vehicle_speed::Float64)
    return (current_vehicle_speed - max_vehicle_speed)/max_vehicle_speed
end

#Immediate Stop penalty
function immediate_stop_penalty(immediate_stop_flag::Bool, penalty::Float64)
    if(immediate_stop_flag)
        return penalty/10.0
        # return 0.0
    else
        return 0.0
    end
end

#Penalty for heading angle changes
function heading_angle_change_penalty(steering_angle::Float64)
    if(steering_angle != 0.0)
        return -10.0
    else
        return 0.0
    end
end

# Penalty for excessive acceleration and deceleration
function unsmooth_motion_penalty(a)
    if(a.delta_speed!=0.0)
        return -1.0
    else
        return 0.0
    end
end
