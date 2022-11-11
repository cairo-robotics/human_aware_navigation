include("pomdp_planning_utils.jl")

#=
Struct for POMDP State
=#
struct StateLimitedSpacePOMDP
    vehicle_x::Float64
    vehicle_y::Float64
    vehicle_theta::Float64
    vehicle_v::Float64
    index_vehicle_controls_sequence::Int64
    nearby_humans::Array{HumanState,1}
end

#=
Struct for POMDP Action
=#
struct ActionLimitedSpacePOMDP
    delta_speed::Float64
end

#=
Struct for Hybrid A* policy
=#
struct HybridAStarPolicy
    controls_sequence::Array{Float64,1}
    len::Int64
end

#=
Struct for POMDP
=#
struct LimitedSpacePOMDP{P} <: POMDPs.POMDP{StateLimitedSpacePOMDP,ActionLimitedSpacePOMDP,Array{Location,1}}
    discount_factor::Float64
    min_safe_distance_from_human::Float64
    human_collision_penalty::Float64
    radius_around_vehicle_goal::Float64
    goal_reached_reward::Float64
    vehicle_L::Float64
    max_vehicle_speed::Float64
    vehicle_goal::Location
    one_time_step::Float64
    num_segments_in_one_time_step::Int64
    observation_discretization_length::Float64
    d_near::Float64
    d_far::Float64
    world::ExperimentEnvironment
    rollout_guide::P
end

function LimitedSpacePOMDP(pomdp_details,exp_details,vehicle_params,rollout_guide)

    return LimitedSpacePOMDP(
        pomdp_details.discount_factor,
        pomdp_details.min_safe_distance_from_human,
        pomdp_details.human_collision_penalty,
        pomdp_details.radius_around_vehicle_goal,
        pomdp_details.goal_reached_reward,
        vehicle_params.L,
        pomdp_details.max_vehicle_speed,
        vehicle_params.goal,
        pomdp_details.one_time_step,
        pomdp_details.num_segments_in_one_time_step,
        pomdp_details.observation_discretization_length,
        pomdp_details.d_near,
        pomdp_details.d_far,
        exp_details.env,
        rollout_guide
        )
end


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
function Base.rand(rng::AbstractRNG, scenario_params::TreeSearchScenarioParameters{VehicleParametersLSPlanner})
    humans = Array{HumanState,1}()
    for i in 1:scenario_params.num_nearby_humans
        sampled_goal = Distributions.rand(rng, SparseCat(scenario_params.human_goals,scenario_params.nearby_humans_belief[i].pdf))
        new_human = HumanState(scenario_params.nearby_humans[i].x,scenario_params.nearby_humans[i].y,scenario_params.nearby_humans[i].v,sampled_goal)
        new_human = update_human_position(new_human,scenario_params.world_length,scenario_params.world_breadth,scenario_params.time_duration,0.0)
        push!(humans, new_human)
    end
    return StateLimitedSpacePOMDP(scenario_params.vehicle_x,scenario_params.vehicle_y,scenario_params.vehicle_theta,
                scenario_params.vehicle_v,scenario_params.vehicle_params.index,humans)
end

#=
************************************************************************************************
Simulate the vehicle one step forward in POMDP planning
=#
function update_vehicle_position(s, m, new_vehicle_speed, num_segments_in_one_time_step)

    current_x, current_y, current_theta = s.vehicle_x, s.vehicle_y, s.vehicle_theta
    if(new_vehicle_speed == 0.0)
        vehicle_path = Tuple{Float64,Float64,Float64}[ (current_x, current_y, current_theta) ]
        vehicle_path = repeat(vehicle_path, num_segments_in_one_time_step+1)
    else
        vehicle_path = Tuple{Float64,Float64,Float64}[ (current_x, current_y, current_theta) ]
        for i in 1:num_segments_in_one_time_step
            steering_angle = m.rollout_guide.controls_sequence[s.index_vehicle_controls_sequence+i-1]
            if(steering_angle == 0.0)
                new_theta = current_theta
                new_x = current_x + new_vehicle_speed*cos(current_theta)*(m.one_time_step/num_segments_in_one_time_step)
                new_y = current_y + new_vehicle_speed*sin(current_theta)*(m.one_time_step/num_segments_in_one_time_step)
            else
                new_theta = current_theta + (new_vehicle_speed * tan(steering_angle) * (m.one_time_step/num_segments_in_one_time_step) / m.vehicle_L)
                new_theta = wrap_between_0_and_2Pi(new_theta)
                new_x = current_x + ((m.vehicle_L / tan(steering_angle)) * (sin(new_theta) - sin(current_theta)))
                new_y = current_y + ((m.vehicle_L / tan(steering_angle)) * (cos(current_theta) - cos(new_theta)))
            end
            push!(vehicle_path,(new_x,new_y,new_theta))
            current_x,current_y,current_theta = new_x,new_y,new_theta
            if(s.index_vehicle_controls_sequence+i > m.rollout_guide.len)
                for j in i+1:num_segments_in_one_time_step
                    push!(vehicle_path,(current_x, current_y, current_theta))
                end
                return vehicle_path
            end
        end
    end
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


#=
************************************************************************************************
POMDP Generative Model
=#

#parent = Dict()
function POMDPs.gen(m::LimitedSpacePOMDP, s, a, rng)

    vehicle_reached_goal = false
    collision_with_human = false
    collision_with_obstacle = false
    immediate_stop = false
    next_human_states = HumanState[]
    observed_positions = Location[]

    #=
        Check if current vehicle position collides with any nearby human.
        Check if current vehicle position is in the goal region.
        Apply given action on the vehicle and get vehicle path.
        Propogate humans and get their paths.
        Check if vehicle path collides with the path of any nearby human and with any static obstacle.
        Generate the new state accordingly.
        Generate corrsponding observation and reward.
    =#

    for human in s.nearby_humans
        if(s.vehicle_v != 0.0)
            if( is_within_range(s.vehicle_x, s.vehicle_y, human.x, human.y, m.min_safe_distance_from_human+m.vehicle_L) )
                # println("Collision with this human " ,s.nearby_humans[human_index] , " ", time_index )
                # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", intermediate_human_location )
                new_vehicle_position = (-100.0, -100.0, -100.0)
                collision_with_human = true
                # next_human_states = human_state[]
                observed_positions = Location[ Location(-50.0,-50.0) ]
                sp = StateLimitedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
                r = human_collision_penalty(collision_with_human, m.human_collision_penalty)
                return (sp=sp, o=observed_positions, r=r)
            end
        end
    end

    if(is_within_range(s.vehicle_x,s.vehicle_y, m.vehicle_goal.x, m.vehicle_goal.y, m.radius_around_vehicle_goal))
        # println("Goal reached")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        vehicle_reached_goal = true
        # next_human_states = human_state[]
        observed_positions = Location[ Location(-50.0,-50.0) ]
        sp = StateLimitedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
        r = vehicle_goal_reached_reward(vehicle_reached_goal, m.goal_reached_reward)
        return (sp=sp, o=observed_positions, r=r)
    end

    if(a.delta_speed == -10.0)
        immediate_stop = true
    end
    new_vehicle_speed = clamp(s.vehicle_v + a.delta_speed, 0.0, m.max_vehicle_speed)
    num_segments_in_one_time_step = Int(new_vehicle_speed/m.speed_change)
    vehicle_path = update_vehicle_position(s, m, new_vehicle_speed, num_segments_in_one_time_step)
    new_vehicle_position = vehicle_path[end]

    for human in s.nearby_humans
        scaling_factor_for_noise = 0.2
        noise = (rand(rng) - 0.5)*human.v*m.one_time_step*scaling_factor_for_noise
        modified_human_state = update_human_position(human,m.world.length,m.world.breadth,m.one_time_step,noise)
        if(new_vehicle_speed!=0.0)
            human_path_x = LinRange(human.x,modified_human_state.x,num_segments_in_one_time_step+1)
            human_path_y = LinRange(human.y,modified_human_state.y,num_segments_in_one_time_step+1)
            for time_index in 2:num_segments_in_one_time_step+1
                if( is_within_range(vehicle_path[time_index][1],vehicle_path[time_index][2],human_path_x[time_index],human_path_y[time_index],m.min_safe_distance_from_human+m.vehicle_L) )
                    # println("Collision with this human " ,human , "at time ", time_index )
                    # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", (human_path_x[time_index],human_path_y[time_index]) )
                    new_vehicle_position = (-100.0, -100.0, -100.0)
                    collision_with_human = true
                    next_human_states = HumanState[]
                    observed_positions = Location[ Location(-50.0,-50.0) ]
                    sp = StateLimitedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,next_human_states)
                    r = human_collision_penalty(collision_with_human, m.human_collision_penalty)
                    return (sp=sp, o=observed_positions, r=r)
                end
            end
        end
        if(modified_human_state.x!=modified_human_state.goal.x && modified_human_state.y!=modified_human_state.goal.y)
            push!(next_human_states, modified_human_state)
            discrete_new_x = floor(modified_human_state.x/m.observation_discretization_length) * m.observation_discretization_length
            discrete_new_y = floor(modified_human_state.y/m.observation_discretization_length) * m.observation_discretization_length
            observed_location = Location(discrete_new_x, discrete_new_y)
            push!(observed_positions, observed_location)
        end
    end

    # if(is_within_range(new_vehicle_position[1],new_vehicle_position[2], s.vehicle_goal.x, s.vehicle_goal.y, m.radius_around_vehicle_goal))
    #     # println("Goal reached")
    #     new_vehicle_position = (-100.0, -100.0, -100.0)
    #     vehicle_reached_goal = true
    #     # next_human_states = human_state[]
    #     observed_positions = location[ location(-50.0,-50.0) ]
    #     sp = state_Limited_space_POMDP_planner(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,s.vehicle_L,s.vehicle_goal,next_human_states)
    #     r = vehicle_goal_reached_reward(vehicle_reached_goal, m.goal_reached_reward)
    #     return (sp=sp, o=observed_positions, r=r)
    # end

    # If the code reaches here, then both s and sp are safe states. Define corresponding new POMDP state.
    sp = StateLimitedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,
                                    s.index_vehicle_controls_sequence+num_segments_in_one_time_step,next_human_states)

    # R(s,a): Reward for being at state s and taking action a
    #Penalize if collision with human
    r = low_speed_penalty(s.vehicle_v, m.max_vehicle_speed)
    #println("Reward from not traveling at max_speed ", r)
    #Penalize if had to apply sudden brakes
    r += immediate_stop_penalty(immediate_stop, m.human_collision_penalty)
    #println("Reward if you had to apply immediate brakes", r)
    #Penalize if vehicle's heading angle changes
    r += heading_angle_change_penalty(a.delta_speed)
    #Penalize unsmooth paths
    r += unsmooth_motion_penalty(a)
    #Penalty to avoid long paths
    r += -1.0

    # if( sp.vehicle_x<0.0+sp.vehicle_L || sp.vehicle_y<0.0+sp.vehicle_L || sp.vehicle_x>m.world.length-sp.vehicle_L || sp.vehicle_y>m.world.breadth-sp.vehicle_L )
    #     println(s)
    #     println(sp)
    #     println(r)
    # end

    return (sp=sp, o=observed_positions, r=r)
end
#=
unit_test_h1 = human_state(10.0,10.0,1.0,location(0.0,0.0),1)
unit_test_h2 = human_state(10.0,10.0,1.0,location(100.0,0.0),2)
unit_test_nearby_humans = human_state[unit_test_h1,unit_test_h2]
unit_test_es_planner_state = state_Limited_space_POMDP_planner(2.0,2.0,0.0,1.0,1.0,location(99.0,74.0),unit_test_nearby_humans)
unit_test_es_planner_action = ActionLimitedSpacePOMDP(pi/12,1.0)
unit_test_env = experiment_environment(100.0,100.0,obstacle_location[])
unit_test_es_pomdp = Limited_space_POMDP_planner(0.99,1.0,-100.0,1.0,-100.0,1.0,100.0,3.0,1.0,10,1.0,unit_test_env)
POMDPs.gen( unit_test_es_pomdp,unit_test_es_planner_state,unit_test_es_planner_action,MersenneTwister(7) )
=#


#=
************************************************************************************************
Upper bound value function for DESPOT
=#

function is_collision_state(s::StateLimitedSpacePOMDP,m::LimitedSpacePOMDP)
    if((s.vehicle_v == 0.0)
        return false
    else
        for human in s.nearby_humans
            if( is_within_range(s.vehicle_x,s.vehicle_y,human.x,human.y,m.min_safe_distance_from_human+m.vehicle_L) )
                return true
            end
        end
    end
    return false
end

#This is not accurate for HV or NHV, especially when static obstacles are present. Can we get a better and tighter upper bound?
function time_to_goal(s::StateLimitedSpacePOMDP,m::LimitedSpacePOMDP)
    remaining_path_length = m.rollout_guide.len - s.index_vehicle_controls_sequence
    return ceil(remaining_path_length/m.max_vehicle_speed)
end

function calculate_upper_bound(m::LimitedSpacePOMDP, b)

    # lower = lbound(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(Limited_space_pomdp, b)),max_depth=100),m,b)
    value_sum = 0.0
    if(b.depth == 100)
        return value_sum
    else
        for (s, w) in weighted_particles(b)
            if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
                value_sum += 0.0
            elseif(is_within_range(s.vehicle_x,s.vehicle_y, m.vehicle_goal.x, m.vehicle_goal.y, m.radius_around_vehicle_goal))
                value_sum += w*m.goal_reached_reward
                # println("Upper bound PA is :", value_sum)
            elseif(is_collision_state(s,m))
                value_sum += w*m.human_collision_penalty
                # println("Upper bound PB is :", value_sum)
            else
                value_sum += w*((discount(m)^time_to_goal(s,m))*m.goal_reached_reward)
                # println("Upper bound PC is :", value_sum)
            end
        end
    end
    # println("Upper bound is :", value_sum)
    # u = (value_sum)/weight_sum(b)
    # if lower > value_sum
    #     push!(bad, (lower,value_sum,b))
    #     @show("While debugging ",lower,value_sum,b.depth)
    # end
    return value_sum
end
#@code_warntype calculate_upper_bound_value(golfcart_pomdp(), initialstate_distribution(golfcart_pomdp()))


#=
************************************************************************************************
Lower bound policy function for DESPOT
=#
function calculate_lower_bound(m::LimitedSpacePOMDP,b)
    #Implement a reactive controller for your lower bound
    delta_speed = m.speed_change

    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true
    for (s, w) in weighted_particles(b)
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            continue
        else
            if(first_execution_flag)
                first_execution_flag = false
            else
                dist_to_closest_human = 20000.0  #Some really big infeasible number (not Inf to avoid the type mismatch error)
                for human in s.nearby_humans
                    euclidean_distance = sqrt((s.vehicle_x - human.x)^2 + (s.vehicle_y - human.y)^2)
                    if(euclidean_distance < dist_to_closest_human)
                        dist_to_closest_human = euclidean_distance
                    end
                    if(dist_to_closest_human < m.d_near)
                        delta_speed = -m.speed_change
                        return ActionLimitedSpacePOMDP(delta_speed)
                    end
                end
                if(dist_to_closest_human > m.d_far)
                    chosen_delta_speed = m.speed_change
                else
                    chosen_delta_speed = 0.0
                end
                if(chosen_delta_speed < delta_speed)
                    delta_speed = chosen_delta_speed
                end
            end
        end
    end

    #This condition is true only when all the states in the belief are terminal. In that case, just return (0.0,0.0)
    if(first_execution_flag)
        return ActionLimitedSpacePOMDP(0.0)
    end
    return ActionLimitedSpacePOMDP(delta_speed)
end

#=
************************************************************************************************
Action Function for the POMDP
=#
function get_actions(m::LimitedSpacePOMDP,b)
        return [ActionLimitedSpacePOMDP(-m.speed_change),
                ActionLimitedSpacePOMDP(0.0),
                ActionLimitedSpacePOMDP(m.speed_change),
                ActionLimitedSpacePOMDP(-10.0)
                ]
end


discount(m::LimitedSpacePOMDP) = m.discount_factor
isterminal(m::LimitedSpacePOMDP, s::StateLimitedSpacePOMDP) = is_terminal_state(s,Location(-100.0,-100.0));
actions(m::LimitedSpacePOMDP,b) = get_actions(m,b)