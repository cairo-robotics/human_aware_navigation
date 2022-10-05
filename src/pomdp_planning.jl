using ProfileView
using POMDPs
using Distributions
using Random
import POMDPs: initialstate_distribution, actions, gen, discount, isterminal
using POMDPModels, POMDPSimulators, ARDESPOT, POMDPModelTools, POMDPPolicies
using ParticleFilters
using BenchmarkTools
using Debugger
using LinearAlgebra
using DifferentialEquations

#Struct for POMDP State
struct state_extended_space_POMDP_planner
    vehicle_x::Float64
    vehicle_y::Float64
    vehicle_theta::Float64
    vehicle_v::Float64
    vehicle_L::Float64
    vehicle_goal::location
    nearby_humans::Array{human_state,1}
end

#Struct for POMDP Action
struct action_extended_space_POMDP_planner
    delta_heading_angle::Float64
    delta_speed::Float64
end

#Struct for POMDP
struct extended_space_POMDP_planner <: POMDPs.POMDP{state_extended_space_POMDP_planner,action_extended_space_POMDP_planner,Array{location,1}}
    discount_factor::Float64
    min_safe_distance_from_human::Float64
    human_collision_penalty::Float64
    min_safe_distance_from_obstacle::Float64
    obstacle_collision_penalty::Float64
    radius_around_vehicle_goal::Float64
    goal_reached_reward::Float64
    max_vehicle_speed::Float64
    one_time_step::Float64
    num_segments_in_one_time_step::Int64
    observation_discretization_length::Float64
    world::experiment_environment
end

#Function to check terminal state
function is_terminal_state(s,terminal_state)
    if(terminal_state.x == s.vehicle_x && terminal_state.y == s.vehicle_y)
        return true
    else
        return false
    end
end

#************************************************************************************************
#Generate Initial POMDP state based on the scenario provided by random operator.

struct tree_search_scenario_parameters
    vehicle_x::Float64
    vehicle_y::Float64
    vehicle_theta::Float64
    vehicle_v::Float64
    vehicle_params::Union{es_vehicle_parameters,ls_vehicle_parameters}
    human_goals::Array{location,1}
    nearby_humans::Array{human_state,1}
    nearby_humans_belief::Array{belief_over_human_goals,1}
end

function Base.rand(rng::AbstractRNG, scenario_params::tree_search_scenario_parameters)
    humans = Array{human_state,1}()
    for i in 1:length(scenario_params.nearby_humans)
        sampled_goal = Distributions.rand(rng, SparseCat(scenario_params.human_goals,scenario_params.nearby_humans_belief[i].pdf))
        new_human = human_state(scenario_params.nearby_humans[i].x,scenario_params.nearby_humans[i].y,scenario_params.nearby_humans[i].v,sampled_goal)
        new_human, dis_o = update_human_position_pomdp_planning(new_human,50.0,50.0,0.5,1.0,rng)
        push!(humans, new_human)
    end
    return state_extended_space_POMDP_planner(scenario_params.vehicle_x,scenario_params.vehicle_y,scenario_params.vehicle_theta,
                                scenario_params.vehicle_v,scenario_params.vehicle_params.L,scenario_params.vehicle_params.goal,humans)
end

#************************************************************************************************
#Simulate humans one step forward in POMDP planning

function update_human_position_pomdp_planning(human::human_state,world_length::Float64,world_breadth::Float64,time_step::Float64,discretization_step_length::Float64,rng::AbstractRNG)

    rand_num = (rand(rng) - 0.5)*0.2
    #rand_num = 0.0
    #First Quadrant
    if(human.goal.x >= human.x && human.goal.y >= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y + (human.v)*time_step + rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x + (human.v)*time_step + rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x + ((human.v)*time_step + rand_num)*cos(heading_angle)
            new_y = human.y + ((human.v)*time_step + rand_num)*sin(heading_angle)
        end
    #Second Quadrant
    elseif(human.goal.x <= human.x && human.goal.y >= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y + (human.v)*time_step + rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x - (human.v)*time_step - rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x - ((human.v)*time_step + rand_num)*cos(heading_angle)
            new_y = human.y - ((human.v)*time_step + rand_num)*sin(heading_angle)
        end
    #Third Quadrant
    elseif(human.goal.x <= human.x && human.goal.y <= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y - (human.v)*time_step - rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x - (human.v)*time_step - rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x - ((human.v)*time_step + rand_num)*cos(heading_angle)
            new_y = human.y - ((human.v)*time_step + rand_num)*sin(heading_angle)
        end
    #Fourth Quadrant
    else(human.goal.x >= human.x && human.goal.y <= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y - (human.v)*time_step - rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x + (human.v)*time_step + rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x + ((human.v)*time_step + rand_num)*cos(heading_angle)
            new_y = human.y + ((human.v)*time_step + rand_num)*sin(heading_angle)
        end
    end

    new_x = clamp(new_x,0.0,world_length)
    new_y = clamp(new_y,0.0,world_breadth)
    discrete_new_x = floor(new_x/discretization_step_length) * discretization_step_length
    discrete_new_y = floor(new_y/discretization_step_length) * discretization_step_length
    new_human_state = human_state(new_x,new_y,human.v,human.goal)
    observed_location = location(discrete_new_x, discrete_new_y)

    return new_human_state,observed_location
end
#=
unit_test_human = human_state(10.0,10.0,1.0,location(100.0,100.0),7.0)
update_human_position_pomdp_planning(unit_test_human,env.length,env.breadth,1.0,1.0,MersenneTwister(1234))
update_human_position_pomdp_planning(unit_test_human,100.0,100.0,1.0,1.0,MersenneTwister(1234))
@code_warntype update_human_position_pomdp_planning(unit_test_human,100.0,100.0,1.0,1.0,MersenneTwister(1234))
=#


#************************************************************************************************
#Simulate the vehicle one step forward in POMDP planning

function update_vehicle_position_pomdp_planning(vehicle_x::Float64, vehicle_y::Float64, vehicle_theta::Float64, vehicle_L::Float64,
                                                            delta_heading_angle::Float64, new_vehicle_speed::Float64, world_length::Float64,
                                                            world_breadth::Float64, one_time_step::Float64, num_time_segments::Int64 = 10)

    current_x, current_y, current_theta = vehicle_x, vehicle_y, vehicle_theta
    if(new_vehicle_speed == 0.0)
        vehicle_path = Tuple{Float64,Float64,Float64}[ (current_x, current_y, current_theta) ]
        vehicle_path = repeat(vehicle_path, num_time_segments+1)
    else
        vehicle_path = Tuple{Float64,Float64,Float64}[ (current_x, current_y, current_theta) ]
        # push!(vehicle_path,(Float64(current_x), Float64(current_y), Float64(current_theta)))
        arc_length = new_vehicle_speed * one_time_step
        steering_angle = atan((vehicle_L*delta_heading_angle)/arc_length)
        for i in (1:num_time_segments)
            if(steering_angle == 0.0)
                new_theta = current_theta
                new_x = current_x + new_vehicle_speed*cos(current_theta)*(one_time_step/num_time_segments)
                new_y = current_y + new_vehicle_speed*sin(current_theta)*(one_time_step/num_time_segments)
            else
                new_theta = current_theta + (new_vehicle_speed * tan(steering_angle) * (one_time_step/num_time_segments) / vehicle_L)
                new_theta = wrap_between_0_and_2Pi(new_theta)
                new_x = current_x + ((vehicle_L / tan(steering_angle)) * (sin(new_theta) - sin(current_theta)))
                new_y = current_y + ((vehicle_L / tan(steering_angle)) * (cos(current_theta) - cos(new_theta)))
            end
            push!(vehicle_path,(Float64(new_x), Float64(new_y), Float64(new_theta)))
            current_x,current_y,current_theta = new_x,new_y,new_theta
            if(current_x>world_length || current_y>world_breadth || current_x<0.0 || current_y<0.0)
                for j in i+1:num_time_segments
                    push!(vehicle_path,(current_x, current_y, current_theta))
                end
                return vehicle_path
            end
        end
    end
    # println(vehicle_x, vehicle_y, vehicle_theta, vehicle_L, delta_heading_angle, new_vehicle_speed)
    # println(vehicle_path)
    return vehicle_path
end
#=
unit_test_vehicle = Vehicle(2.0,2.0,0.0,1.0,1.0,location(99.0,74.0),human_state[],Float64[])
unit_test_delta_angle = 0.0
unit_test_new_velocity = 4.0
unit_test_world_length, unit_test_world_breadth = 100.0,100.0
update_vehicle_position_pomdp_planning(unit_test_vehicle.x,unit_test_vehicle.y,unit_test_vehicle.theta,unit_test_vehicle.L,unit_test_delta_angle,
                                                        unit_test_new_velocity,unit_test_world_length, unit_test_world_breadth,1.0)
=#


#************************************************************************************************
# Reward functions for the POMDP model
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
    else
        return 0.0
    end
end

#Penalty for heading angle changes
function heading_angle_change_penalty(delta_heading_angle::Float64)
    if(delta_heading_angle != 0.0)
        return -1.0
    else
        return 0.0
    end
end


#************************************************************************************************
#POMDP Generative Model

#parent = Dict()
function POMDPs.gen(m::extended_space_POMDP_planner, s, a, rng)

    vehicle_reached_goal_flag = false
    collision_with_human_flag = false
    collision_with_obstacle_flag = false
    immediate_stop_flag = false
    new_human_states = human_state[]
    observed_positions = location[]

    if(is_within_range(s.vehicle_x,s.vehicle_y, s.vehicle_goal.x, s.vehicle_goal.y, m.radius_around_vehicle_goal))
        #println("Goal reached")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        vehicle_reached_goal_flag = true
        new_vehicle_speed = clamp(s.vehicle_v + a.delta_speed, 0.0, m.max_vehicle_speed)
        push!(observed_positions, location(-25.0,-25.0))
    elseif( (s.vehicle_x>m.world.length) || (s.vehicle_y>m.world.breadth) || (s.vehicle_x<0.0) || (s.vehicle_y<0.0) )
        #print("Running into wall")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        collision_with_obstacle_flag = true
        new_vehicle_speed = clamp(s.vehicle_v + a.delta_speed, 0.0, m.max_vehicle_speed)
        push!(observed_positions, location(-50.0,-50.0))
    else
        if(a.delta_speed == -10.0)
            immediate_stop_flag = true
        end
        new_vehicle_speed = clamp(s.vehicle_v + a.delta_speed, 0.0, m.max_vehicle_speed)
        vehicle_path::Vector{Tuple{Float64,Float64,Float64}} = update_vehicle_position_pomdp_planning(s.vehicle_x, s.vehicle_y, s.vehicle_theta, s.vehicle_L,
                                                        a.delta_heading_angle, new_vehicle_speed, m.world.length, m.world.breadth, m.one_time_step, m.num_segments_in_one_time_step)
        new_vehicle_position = vehicle_path[end]
        #If vehicle goes out of bounds by taking this action
        if( (new_vehicle_position[1]>m.world.length) || (new_vehicle_position[2]>m.world.breadth) || (new_vehicle_position[1]<0.0) || (new_vehicle_position[2]<0.0) )
            new_vehicle_position = (-100.0, -100.0, -100.0)
            collision_with_obstacle_flag = true
            push!(observed_positions, location(-50.0,-50.0))
        #=
        If vehicle did not go out of bounds by taking this action, then check if there is a collision with
        any human or static obstacle during vehicle's path.
        =#
        else
            # Simulate all the nearby humans
            for human in s.nearby_humans
                if(s.vehicle_v!=0.0)
                    if( find_if_two_circles_intersect(s.vehicle_x, s.vehicle_y, s.vehicle_L, human.x, human.y, m.min_safe_distance_from_human) )
                        new_vehicle_position = (-100.0, -100.0, -100.0)
                        collision_with_human_flag = true
                        new_human_states = human_state[]
                        observed_positions = location[ location(-50.0,-50.0) ]
                        # println("Collision with this human " ,s.nearby_humans[human_index] , " ", time_index )
                        # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", intermediate_human_location )
                        break
                    end
                end
                modified_human_state,observed_location = update_human_position_pomdp_planning(human,m.world.length,m.world.breadth,m.one_time_step,m.observation_discretization_length,rng)
                push!(new_human_states, modified_human_state)
                push!(observed_positions, observed_location)
            end
            if(!collision_with_human_flag)
                #Vehicle is moving
                if(new_vehicle_speed != 0.0)
                    for time_index in 2:m.num_segments_in_one_time_step+1
                        for human_index in 1:length(s.nearby_humans)
                            intermediate_human_location = get_intermediate_point_human_trajectory(s.nearby_humans[human_index].x,s.nearby_humans[human_index].y,
                                                        new_human_states[human_index].x,new_human_states[human_index].y, (1/m.num_segments_in_one_time_step)*(time_index-1) )
                            if( find_if_two_circles_intersect(vehicle_path[time_index][1], vehicle_path[time_index][2], s.vehicle_L,
                                                        intermediate_human_location[1], intermediate_human_location[2], m.min_safe_distance_from_human) )
                                new_vehicle_position = (-100.0, -100.0, -100.0)
                                collision_with_human_flag = true
                                new_human_states = human_state[]
                                observed_positions = location[ location(-50.0,-50.0) ]
                                break
                            end
                        end
                        if( !collision_with_human_flag )
                            #Check if the vehicle intersects with any static obstacle
                            for obstacle in m.world.obstacles
                                if( find_if_two_circles_intersect(vehicle_path[time_index][1], vehicle_path[time_index][2], s.vehicle_L,
                                                                        obstacle.x, obstacle.y, obstacle.r + m.min_safe_distance_from_obstacle) )
                                    new_vehicle_position = (-100.0, -100.0, -100.0)
                                    collision_with_obstacle_flag = true
                                    new_human_states = human_state[]
                                    observed_positions = location[ location(-50.0,-50.0) ]
                                    break
                                end
                            end
                        end
                        if(collision_with_human_flag || collision_with_obstacle_flag)
                            vehicle_reached_goal_flag = false
                            break
                        end
                    end
                #Vehicle is stationary
                else
                    #Check if the vehicle intersects with any static obstacle
                    for obstacle in m.world.obstacles
                        if( find_if_two_circles_intersect(s.vehicle_x, s.vehicle_y, s.vehicle_L,
                                                                obstacle.x,obstacle.y,obstacle.r + m.obstacle_distance_threshold) )
                            new_vehicle_position = (-100.0, -100.0, -100.0)
                            collision_with_obstacle_flag = true
                            new_human_states = human_state[]
                            observed_positions = location[ location(-50.0,-50.0) ]
                            # println("Collision with this obstacle " ,obstacle, " ", time_index )
                            break
                        end
                    end
                end
            end
        end
    end
    # Next POMDP State
    sp = state_extended_space_POMDP_planner(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,s.vehicle_L,s.vehicle_goal,new_human_states)
    # Generated Observation
    o = observed_positions

    # R(s,a): Reward for being at state s and taking action a
    #Penalize if collision with human
    r = human_collision_penalty(collision_with_human_flag, m.min_safe_distance_from_human)
    #println("Reward from collision with human", r)
    #Penalize if collision with obstacle
    r += obstacle_collision_penalty(collision_with_obstacle_flag, m.obstacle_collision_penalty)
    #println("Reward from collision with obstacle", r)
    #Reward if reached goal
    r += vehicle_goal_reached_reward(vehicle_reached_goal_flag, m.goal_reached_reward)
    #println("Reward from goal ", r)
    #Penalize if going slow when it can go fast
    r += low_speed_penalty(s.vehicle_v, m.max_vehicle_speed)
    #println("Reward from not traveling at max_speed ", r)
    #Penalize if had to apply sudden brakes
    r += immediate_stop_penalty(immediate_stop_flag, m.human_collision_penalty)
    #println("Reward if you had to apply immediate brakes", r)
    #Penalize if vehicle's heading angle changes
    r += heading_angle_change_penalty(a.delta_heading_angle)
    #Penalty to avoid long paths
    r += -1.0

    return (sp=sp, o=o, r=r)
end
#=
unit_test_h1 = human_state(10.0,10.0,1.0,location(0.0,0.0),1)
unit_test_h2 = human_state(10.0,10.0,1.0,location(100.0,0.0),2)
unit_test_nearby_humans = human_state[unit_test_h1,unit_test_h2]
unit_test_es_planner_state = state_extended_space_POMDP_planner(2.0,2.0,0.0,1.0,1.0,location(99.0,74.0),unit_test_nearby_humans)
unit_test_es_planner_action = action_extended_space_POMDP_planner(pi/12,1.0)
unit_test_env = experiment_environment(100.0,100.0,obstacle_location[])
unit_test_es_pomdp = extended_space_POMDP_planner(0.99,1.0,-100.0,1.0,-100.0,1.0,100.0,3.0,1.0,10,1.0,unit_test_env)
POMDPs.gen( unit_test_es_pomdp,unit_test_es_planner_state,unit_test_es_planner_action,MersenneTwister(7) )
=#


#************************************************************************************************
#Upper bound value function for DESPOT

function is_collision_state_pomdp_planning(s::state_extended_space_POMDP_planner,m::extended_space_POMDP_planner)
    if((s.vehicle_x>m.world.length) || (s.vehicle_y>m.world.breadth) || (s.vehicle_x<0.0) || (s.vehicle_y<0.0))
        return true
    elseif(s.vehicle_v != 0.0)
        for human in s.nearby_humans
            if( is_within_range(s.vehicle_x,s.vehicle_y,human.x,human.y,m.min_safe_distance_from_human+s.vehicle_L) )
                return true
            end
        end
    end
    for obstacle in m.world.obstacles
        if( is_within_range(s.vehicle_x,s.vehicle_y,obstacle.x,obstacle.y,m.min_safe_distance_from_obstacle+obstacle.r+s.vehicle_L) )
            return true
        end
    end
    return false
end

#This is not accurate for HV or NHV, especially when static obstacles are present. Can we get a better and tighter upper bound?
function time_to_goal_pomdp_planning(s::state_extended_space_POMDP_planner,max_vehicle_speed::Float64)
    vehicle_distance_to_goal = sqrt( (s.vehicle_x-s.vehicle_goal.x)^2 + (s.vehicle_y-s.vehicle_goal.y)^2 )
    # println("Distance is :", vehicle_distance_to_goal)
    # println(s)
    # println("Time is :", floor(vehicle_distance_to_goal/max_vehicle_speed))
    return floor(vehicle_distance_to_goal/max_vehicle_speed)
end

function calculate_upper_bound_pomdp_planning(m::extended_space_POMDP_planner, b)

    #@show lbound(DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning_2D_action_space), max_depth=100, final_value=reward_to_be_awarded_at_max_depth_in_lower_bound_policy_rollout),m , b)
    value_sum = 0.0
    for (s, w) in weighted_particles(b)
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            value_sum += 0.0
        elseif(is_within_range(s.vehicle_x,s.vehicle_y, s.vehicle_goal.x, s.vehicle_goal.y, m.radius_around_vehicle_goal))
            value_sum += w*m.goal_reached_reward
            # println("Upper bound PA is :", value_sum)
        elseif(is_collision_state_pomdp_planning(s,m))
            value_sum += w*m.human_collision_penalty
            # println("Upper bound PB is :", value_sum)
        else
            value_sum += w*((discount(m)^time_to_goal_pomdp_planning(s,m.max_vehicle_speed))*m.goal_reached_reward)
            # println("Upper bound PC is :", value_sum)
        end
    end
    # println("Upper bound is :", value_sum)
    return value_sum
end
#@code_warntype calculate_upper_bound_value(golfcart_pomdp(), initialstate_distribution(golfcart_pomdp()))


#************************************************************************************************
#Lower bound policy function for DESPOT
function calculate_lower_bound_pomdp_planning(m::extended_space_POMDP_planner,b)
    #Implement a reactive controller for your lower bound
    speed_change_to_be_returned = 1.0
    delta_angle = 0.0
    d_far_threshold = 6.0
    d_near_threshold = 4.0
    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true

    for (s, w) in weighted_particles(b)
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            continue
        else
            if(first_execution_flag)
                required_orientation = get_heading_angle( s.vehicle_goal.x, s.vehicle_goal.y, s.vehicle_x, s.vehicle_y)
                delta_angle = required_orientation - s.vehicle_theta
                abs_delta_angle = abs(delta_angle)
                if(abs_delta_angle<=pi)
                    delta_angle = clamp(delta_angle, -pi/4, pi/4)
                else
                    if(delta_angle>=0.0)
                        delta_angle = clamp(delta_angle-2*pi, -pi/4, pi/4)
                    else
                        delta_angle = clamp(delta_angle+2*pi, -pi/4, pi/4)
                    end
                end
                first_execution_flag = false
            else
                dist_to_closest_human = 200.0  #Some really big infeasible number (not Inf because avoid the type mismatch error)
                for human in s.nearby_humans
                    euclidean_distance = sqrt((s.vehicle_x - human.x)^2 + (s.vehicle_y - human.y)^2)
                    if(euclidean_distance < dist_to_closest_human)
                        dist_to_closest_human = euclidean_distance
                    end
                    if(dist_to_closest_human < d_near_threshold)
                        return action_extended_space_POMDP_planner(delta_angle,-1.0)
                    end
                end
                if(dist_to_closest_human > d_far_threshold)
                    chosen_acceleration = 1.0
                else
                    chosen_acceleration = 0.0
                end
                if(chosen_acceleration < speed_change_to_be_returned)
                    speed_change_to_be_returned = chosen_acceleration
                end
            end
        end
    end

    #This condition is true only when all the states in the belief are terminal. In that case, just return (0.0,0.0)
    if(first_execution_flag == true)
        #@show(0.0,0.0)
        return action_extended_space_POMDP_planner(0.0,0.0)
    end

    #This means all humans are away and you can accelerate.
    if(speed_change_to_be_returned == 1.0)
        #@show(0.0,speed_change_to_be_returned)
        return action_extended_space_POMDP_planner(delta_angle,speed_change_to_be_returned)
    end

    #If code has reached this point, then the best action is to maintain your current speed.
    #We have already found the best steering angle to take.
    #@show(best_delta_angle,0.0)
    return action_extended_space_POMDP_planner(delta_angle,0.0)
end

function reward_to_be_awarded_at_max_depth_in_lower_bound_policy_rollout(m,b)
    #print("HI")
    # #sleep(5)
    # print(b.depth)
    value_sum = 0.0
    for (s, w) in weighted_particles(b)
        # cart_distance_to_goal = sqrt( (s.cart.x - s.cart.goal.x)^2 + (s.cart.y - s.cart.goal.y)^2 )
        # if(cart_distance_to_goal > 1.0)
        #     value_sum += w*(1/cart_distance_to_goal)*m.goal_reward
        # end
        value_sum += w*((discount(m)^time_to_goal_pomdp_planning_2D_action_space(s,m.max_vehicle_speed))*m.goal_reached_reward)
    end
    #println("HG rules")
    return value_sum
end


#************************************************************************************************
#Functions for debugging lb>ub error
function debug_is_collision_state_pomdp_planning_2D_action_space(s,m)

    if((s.vehicle.x>100.0) || (s.vehicle.y>100.0) || (s.vehicle.x<0.0) || (s.vehicle.y<0.0))
        println("Stepped Outside")
        return true
    else
        for human in s.nearby_humans
            if(is_within_range(location(s.vehicle.x,s.vehicle.y),location(human.x,human.y),m.min_safe_distance_from_human))
                println("Collision with human ", human)
                return true
            end
        end
        for obstacle in m.world.obstacles
            if(is_within_range(location(s.vehicle.x,s.vehicle.y),location(obstacle.x,obstacle.y),obstacle.r + m.obstacle_distance_threshold))
                println("Collision with obstacle ", obstacle)
                return true
            end
        end
        return false
    end
end

function debug_golfcart_upper_bound_2D_action_space(m,b)

    # lower = lbound(DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning_2D_action_space), max_depth=100),m , b)
    lower = lbound(DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning_2D_action_space), max_depth=100, final_value=reward_to_be_awarded_at_max_depth_in_lower_bound_policy_rollout),m , b)
    #@show(lower)
    value_sum = 0.0
    for (s, w) in weighted_particles(b)
        if (s.cart.x == -100.0 && s.cart.y == -100.0)
            value_sum += 0.0
        elseif (is_within_range(location(s.cart.x,s.cart.y), s.cart.goal, m.cart_goal_reached_distance_threshold))
            value_sum += w*m.goal_reward
        elseif (is_collision_state_pomdp_planning_2D_action_space(s,m))
            value_sum += w*m.human_collision_penalty
        else
            value_sum += w*((discount(m)^time_to_goal_pomdp_planning_2D_action_space(s,m.max_cart_speed))*m.goal_reward)
        end
    end
    #@show("*********************************************************************")
    #@show(value_sum)
    u = (value_sum)/weight_sum(b)
    if lower > u
        push!(bad, (lower,u,b))
        @show("IN DEBUG",lower,u)
    end
    return u
end

#************************************************************************************************
#Action Function for the POMDP
function get_actions(m::extended_space_POMDP_planner,b)
    pomdp_state = first(particles(b))
    required_orientation = get_heading_angle( pomdp_state.vehicle_goal.x, pomdp_state.vehicle_goal.y, pomdp_state.vehicle_x, pomdp_state.vehicle_y)
    delta_angle = required_orientation - pomdp_state.vehicle_theta
    abs_delta_angle = abs(delta_angle)
    if(abs_delta_angle<=pi)
        delta_angle = clamp(delta_angle, -pi/4, pi/4)
    else
        if(delta_angle>=0.0)
            delta_angle = clamp(delta_angle-2*pi, -pi/4, pi/4)
        else
            delta_angle = clamp(delta_angle+2*pi, -pi/4, pi/4)
        end
    end
    if(pomdp_state.vehicle_v == 0.0)
        if(delta_angle==pi/4 || delta_angle==-pi/4)
            return [action_extended_space_POMDP_planner(-pi/4,1.0),action_extended_space_POMDP_planner(-pi/6,1.0),action_extended_space_POMDP_planner(-pi/12,1.0),
            action_extended_space_POMDP_planner(0.0,0.0),action_extended_space_POMDP_planner(0.0,1.0),action_extended_space_POMDP_planner(pi/12,1.0),
            action_extended_space_POMDP_planner(pi/6,1.0),action_extended_space_POMDP_planner(pi/4,1.0)]
            # return [(-pi/4,2.0),(-pi/6,2.0),(-pi/12,2.0),(0.0,0.0),(0.0,2.0),(pi/12,2.0),(pi/6,2.0),(pi/4,2.0)]
        else
            return [action_extended_space_POMDP_planner(delta_angle, 1.0),action_extended_space_POMDP_planner(-pi/4,1.0),action_extended_space_POMDP_planner(-pi/6,1.0),
            action_extended_space_POMDP_planner(-pi/12,1.0),action_extended_space_POMDP_planner(0.0,0.0),action_extended_space_POMDP_planner(0.0,1.0),
            action_extended_space_POMDP_planner(pi/12,1.0),action_extended_space_POMDP_planner(pi/6,1.0),action_extended_space_POMDP_planner(pi/4,1.0)]
            # return [(delta_angle, 2.0),(-pi/4,2.0),(-pi/6,2.0),(-pi/12,2.0),(0.0,0.0),(0.0,2.0),(pi/12,2.0),(pi/6,2.0),(pi/4,2.0)]
        end
    else
        if(delta_angle==pi/4 || delta_angle==-pi/4)
            return [action_extended_space_POMDP_planner(-pi/4,0.0),action_extended_space_POMDP_planner(-pi/6,0.0),action_extended_space_POMDP_planner(-pi/12,0.0),
            action_extended_space_POMDP_planner(0.0,-1.0),action_extended_space_POMDP_planner(0.0,0.0),action_extended_space_POMDP_planner(0.0,1.0),
            action_extended_space_POMDP_planner(pi/12,0.0),action_extended_space_POMDP_planner(pi/6,0.0),action_extended_space_POMDP_planner(pi/4,0.0),
            action_extended_space_POMDP_planner(-10.0,-10.0)]
            # return [(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-2.0),(0.0,0.0),(0.0,2.0),(pi/12,0.0),(pi/6,0.0),(pi/4,0.0),(-10.0,-10.0)]
        else
            return [action_extended_space_POMDP_planner(delta_angle, 0.0),action_extended_space_POMDP_planner(-pi/4,0.0),action_extended_space_POMDP_planner(-pi/6,0.0),
            action_extended_space_POMDP_planner(-pi/12,0.0),action_extended_space_POMDP_planner(0.0,-1.0),action_extended_space_POMDP_planner(0.0,0.0),
            action_extended_space_POMDP_planner(0.0,1.0),action_extended_space_POMDP_planner(pi/12,0.0),action_extended_space_POMDP_planner(pi/6,0.0),
            action_extended_space_POMDP_planner(pi/4,0.0),action_extended_space_POMDP_planner(-10.0,-10.0)]
            # return [(delta_angle, 0.0),(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-2.0),(0.0,0.0),(0.0,2.0),(pi/12,0.0),(pi/6,0.0),(pi/4,0.0),(-10.0,-10.0)]
        end
        # return [(delta_angle, 1.0),(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-1.0),(0.0,0.0),(0.0,1.0),(pi/12,1.0),(pi/6,0.0),(pi/4,0.0),(-10.0,-10.0)]
        # return [(delta_angle, 1.0),(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-1.0),(0.0,0.0),(0.0,1.0),(pi/12,1.0),(pi/6,0.0),(pi/4,0.0),(-10.0,-10.0)]
    end
end
