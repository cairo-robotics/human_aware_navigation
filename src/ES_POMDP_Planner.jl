include("pomdp_planning_utils.jl")

#=
Struct for POMDP State
=#
struct StateExtendedSpacePOMDP
    vehicle_x::Float64
    vehicle_y::Float64
    vehicle_theta::Float64
    vehicle_v::Float64
    nearby_humans::Array{HumanState,1}
end

# struct NewStateExtendedSpacePOMDP{M}
#     vehicle_x::Float64
#     vehicle_y::Float64
#     vehicle_theta::Float64
#     vehicle_v::Float64
#     nearby_humans::SVector{M,HumanState}
# end

#=
Struct for POMDP Action
=#
struct ActionExtendedSpacePOMDP
    steering_angle::Float64
    delta_speed::Float64
end

#=
Struct for HJB policy
=#
struct HJBPolicy{F1<:Function,F2<:Function}
    Dt::Float64
    value_array::Array{Float64,1}
    q_value_array::Array{Array{Float64,1},1}
    get_actions::F1
    get_cost::F2
    env::Environment
    veh::VehicleBody
    state_grid::StateGrid
end

#=
Struct for POMDP
=#
struct ExtendedSpacePOMDP{P} <: POMDPs.POMDP{StateExtendedSpacePOMDP,ActionExtendedSpacePOMDP,Array{Location,1}}
    discount_factor::Float64
    min_safe_distance_from_human::Float64
    human_collision_penalty::Float64
    min_safe_distance_from_obstacle::Float64
    obstacle_collision_penalty::Float64
    radius_around_vehicle_goal::Float64
    goal_reached_reward::Float64
    vehicle_wheelbase::Float64
    vehicle_length::Float64
    vehicle_breadth::Float64
    vehicle_D::Float64
    vehicle_R::Float64
    max_vehicle_speed::Float64
    max_vehicle_steering_angle::Float64
    vehicle_action_max_delta_heading_angle::Float64
    vehicle_action_delta_speed::Float64
    vehicle_goal::Location
    one_time_step::Float64
    num_segments_in_one_time_step::Int64
    observation_discretization_length::Float64
    d_near::Float64
    d_far::Float64
    world::ExperimentEnvironment
    rollout_guide::P
end

function ExtendedSpacePOMDP(pomdp_details,env,vehicle_params,rollout_guide)

    return ExtendedSpacePOMDP(
        pomdp_details.discount_factor,
        pomdp_details.min_safe_distance_from_human,
        pomdp_details.human_collision_penalty,
        pomdp_details.min_safe_distance_from_obstacle,
        pomdp_details.obstacle_collision_penalty,
        pomdp_details.radius_around_vehicle_goal,
        pomdp_details.goal_reached_reward,
        vehicle_params.wheelbase,
        vehicle_params.length,
        vehicle_params.breadth,
        vehicle_params.dist_origin_to_center,
        vehicle_params.radius,
        vehicle_params.max_speed,
        vehicle_params.max_steering_angle,
        pomdp_details.action_max_delta_heading_angle,
        pomdp_details.action_delta_speed,
        vehicle_params.goal,
        pomdp_details.one_time_step,
        pomdp_details.num_segments_in_one_time_step,
        pomdp_details.observation_discretization_length,
        pomdp_details.d_near,
        pomdp_details.d_far,
        env,
        rollout_guide
        )
end

function Base.rand(rng::AbstractRNG, scenario_params::TreeSearchScenarioParameters{VehicleParametersESPlanner})
    humans = Array{HumanState,1}()
    # return StateExtendedSpacePOMDP(scenario_params.vehicle_x,scenario_params.vehicle_y,scenario_params.vehicle_theta,scenario_params.vehicle_v,humans)
    for i in 1:scenario_params.num_nearby_humans
        sampled_goal = Distributions.rand(rng, SparseCat(scenario_params.human_goals,scenario_params.nearby_humans_belief[i].pdf))
        new_human = HumanState(scenario_params.nearby_humans[i].x,scenario_params.nearby_humans[i].y,scenario_params.nearby_humans[i].v,sampled_goal)
        noise = (rand(rng) - 0.5)*new_human.v*scenario_params.time_duration*0.2
        new_human = update_human_position(new_human,scenario_params.world_length,scenario_params.world_breadth,scenario_params.time_duration,noise)
        push!(humans, new_human)
    end
    return StateExtendedSpacePOMDP(scenario_params.vehicle_x,scenario_params.vehicle_y,scenario_params.vehicle_theta,scenario_params.vehicle_v,humans)
end

#=
************************************************************************************************
Simulate the vehicle one step forward in POMDP planning
=#
function update_vehicle_position(s, m::ExtendedSpacePOMDP, steering_angle, new_vehicle_speed)

    current_x, current_y, current_theta = s.vehicle_x, s.vehicle_y, s.vehicle_theta
    if(new_vehicle_speed == 0.0)
        vehicle_path = Tuple{Float64,Float64,Float64}[ (current_x, current_y, current_theta) ]
        vehicle_path = repeat(vehicle_path, m.num_segments_in_one_time_step+1)
    else
        vehicle_path = Tuple{Float64,Float64,Float64}[ (current_x, current_y, current_theta) ]
        # push!(vehicle_path,(Float64(current_x), Float64(current_y), Float64(current_theta)))
        # arc_length = new_vehicle_speed * m.one_time_step
        # steering_angle = atan((m.vehicle_L*steering_angle)/arc_length)
        for i in (1:m.num_segments_in_one_time_step)
            if(steering_angle == 0.0)
                new_theta = current_theta
                new_x = current_x + new_vehicle_speed*cos(current_theta)*(m.one_time_step/m.num_segments_in_one_time_step)
                new_y = current_y + new_vehicle_speed*sin(current_theta)*(m.one_time_step/m.num_segments_in_one_time_step)
            else
                new_theta = current_theta + (new_vehicle_speed * tan(steering_angle) * (m.one_time_step/m.num_segments_in_one_time_step) / m.vehicle_wheelbase)
                new_theta = wrap_between_0_and_2Pi(new_theta)
                new_x = current_x + ((m.vehicle_wheelbase / tan(steering_angle)) * (sin(new_theta) - sin(current_theta)))
                new_y = current_y + ((m.vehicle_wheelbase / tan(steering_angle)) * (cos(current_theta) - cos(new_theta)))
            end
            push!(vehicle_path,(Float64(new_x), Float64(new_y), Float64(new_theta)))
            current_x,current_y,current_theta = new_x,new_y,new_theta
            vehicle_center_x = current_x + m.vehicle_D*cos(current_theta)
            vehicle_center_y = current_y + m.vehicle_D*cos(current_theta)
            if(is_within_range(vehicle_center_x,vehicle_center_y,m.vehicle_goal.x,m.vehicle_goal.y,m.radius_around_vehicle_goal))
                for j in i+1:m.num_segments_in_one_time_step
                    push!(vehicle_path,(current_x, current_y, current_theta))
                end
                return vehicle_path
            end
            if( vehicle_center_x<0.0+m.vehicle_R || vehicle_center_y<0.0+m.vehicle_R || vehicle_center_x>m.world.length-m.vehicle_R || vehicle_center_y>m.world.breadth-m.vehicle_R )
                for j in i+1:m.num_segments_in_one_time_step
                    push!(vehicle_path,(current_x, current_y, current_theta))
                end
                return vehicle_path
            end
        end
    end
    # println(vehicle_x, vehicle_y, vehicle_theta, vehicle_L, steering_angle, new_vehicle_speed)
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

function testUV(s,unit_test_es_pomdp)
    unit_test_h1 = HumanState(10.0,10.0,1.0,Location(0.0,0.0))
    unit_test_h2 = HumanState(10.0,10.0,1.0,Location(100.0,0.0))
    # unit_test_h1 = HumanState(2.0,4.0,1.0,Location(0.0,0.0))
    # unit_test_h2 = HumanState(3.0,3.0,1.0,Location(100.0,0.0))
    # unit_test_nearby_humans = HumanState[unit_test_h1,unit_test_h2]
    # unit_test_nearby_humans = SVector{2,HumanState}(unit_test_h1,unit_test_h2)
    # unit_test_es_planner_state = StateExtendedSpacePOMDP(2.0,2.0,0.0,1.0,unit_test_nearby_humans)
    return update_vehicle_position(s, unit_test_es_pomdp, 0.0, 2.0)
end
=#


#=
************************************************************************************************
POMDP Generative Model
=#
#parent = Dict()

# function POMDPs.gen(m::Tuple{Int64,Int64},s,a,rng)
#     return 1
# end
#
# function WTF(m::Tuple{Int64,Int64},s,a,rng)
#     return 1
# end

function POMDPs.gen(m::ExtendedSpacePOMDP, s, a, rng)

    vehicle_reached_goal = false
    collision_with_human = false
    collision_with_obstacle = false
    immediate_stop = false
    next_human_states = HumanState[]
    observed_positions = Location[]

    # println(s)
    #=
        Check if current vehicle position collides with any nearby human.
        Check if current vehicle position collides with any static obstacle.
        Check if current vehicle position is outside the environment boundary.
        Check if current vehicle position is in the goal region.
        Apply given action on the vehicle and get vehicle path.
        Propogate humans and get their paths.
        Check if vehicle path collides with the path of any nearby human and with any static obstacle.
        Generate the new state accordingly.
        Generate corrsponding observation and reward.
    =#

    # x = SVector(s.vehicle_x, s.vehicle_y, s.vehicle_theta, s.vehicle_v)
    # HJB_val_x = interp_value(x, m.rollout_guide.value_array, m.rollout_guide.state_grid)
    # if(HJB_val_x < 0.0)
    #     new_vehicle_position = (-100.0, -100.0, -100.0)
    #     collision_with_obstacle = true
    #     # next_human_states = human_state[]
    #     observed_positions = Location[ Location(-50.0,-50.0) ]
    #     sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
    #     r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
    #     return (sp=sp, o=observed_positions, r=r)
    # end

    vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
    vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)
    # return
    for human in s.nearby_humans
    # for i in 1:length(s.nearby_humans)
        # human = s.nearby_humans[i]
        # println("jj")
        if(s.vehicle_v != 0.0)
            if( is_within_range(vehicle_center_x, vehicle_center_y, human.x, human.y, m.min_safe_distance_from_human+m.vehicle_R) )
                # println("Collision with this human " ,s.nearby_humans[human_index] , " ", time_index )
                # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", intermediate_human_location )
                new_vehicle_position = (-100.0, -100.0, -100.0)
                collision_with_human = true
                observed_positions = Location[ Location(-50.0,-50.0) ]
                # observed_positions = Svector{1,Location}(Location(-50.0,-50.0))
                # sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
                sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,s.nearby_humans)
                r = human_collision_penalty(collision_with_human, m.human_collision_penalty)
                return (sp=sp, o=observed_positions, r=r)
            end
        end
    end
    # println("Success")
    # return 11
    for obstacle in m.world.obstacles
        if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,obstacle.r+m.min_safe_distance_from_obstacle+m.vehicle_R) )
            # println("Collision with this obstacle " ,obstacle)
            new_vehicle_position = (-100.0, -100.0, -100.0)
            collision_with_obstacle = true
            observed_positions = Location[ Location(-50.0,-50.0) ]
            sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
            r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
            return (sp=sp, o=observed_positions, r=r)
        end
    end

    if( vehicle_center_x<0.0+m.vehicle_R || vehicle_center_y<0.0+m.vehicle_R || vehicle_center_x>m.world.length-m.vehicle_R || vehicle_center_y>m.world.breadth-m.vehicle_R )
        # println("Running into boundary wall")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        collision_with_obstacle = true
        observed_positions = Location[ Location(-50.0,-50.0) ]
        sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
        r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
        # println(s)
        # println(r)
        return (sp=sp, o=observed_positions, r=r)
    end

    if(is_within_range(vehicle_center_x,vehicle_center_y, m.vehicle_goal.x, m.vehicle_goal.y, m.radius_around_vehicle_goal))
        # println("Goal reached")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        vehicle_reached_goal = true
        # next_human_states = human_state[]
        observed_positions = Location[ Location(-50.0,-50.0) ]
        sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
        r = vehicle_goal_reached_reward(vehicle_reached_goal, m.goal_reached_reward)
        # println(r)
        return (sp=sp, o=observed_positions, r=r)
    end

    if(a.delta_speed == -10.0)
        immediate_stop = true
    end
    new_vehicle_speed = clamp(s.vehicle_v + a.delta_speed, 0.0, m.max_vehicle_speed)
    # steering_angle = get_steering_angle(m.vehicle_L, a.delta_heading_angle, new_vehicle_speed, m.one_time_step)
    steering_angle = a.steering_angle
    vehicle_path = update_vehicle_position(s, m, steering_angle, new_vehicle_speed)
    new_vehicle_position = vehicle_path[end]

    for human in s.nearby_humans
        scaling_factor_for_noise = 0.2
        noise = (rand(rng) - 0.5)*human.v*m.one_time_step*scaling_factor_for_noise
        modified_human_state = update_human_position(human,m.world.length,m.world.breadth,m.one_time_step,noise)
        if(new_vehicle_speed!=0.0)
            human_path_x = LinRange(human.x,modified_human_state.x,m.num_segments_in_one_time_step+1)
            human_path_y = LinRange(human.y,modified_human_state.y,m.num_segments_in_one_time_step+1)
            for time_index in 2:m.num_segments_in_one_time_step+1
                vehicle_center_x = vehicle_path[time_index][1] + m.vehicle_D*cos(vehicle_path[time_index][3])
                vehicle_center_y = vehicle_path[time_index][2] + m.vehicle_D*sin(vehicle_path[time_index][3])
                # println(vehicle_center_x,vehicle_center_y)
                if( is_within_range(vehicle_center_x,vehicle_center_y,human_path_x[time_index],human_path_y[time_index],m.min_safe_distance_from_human+m.vehicle_R) )
                    # println("Collision with this human " ,human , "at time ", time_index )
                    # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", (human_path_x[time_index],human_path_y[time_index]) )
                    new_vehicle_position = (-100.0, -100.0, -100.0)
                    collision_with_human = true
                    next_human_states = HumanState[]
                    observed_positions = Location[ Location(-50.0,-50.0) ]
                    sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,next_human_states)
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

    if(new_vehicle_speed!=0.0)
        for time_index in 2:m.num_segments_in_one_time_step+1
            vehicle_center_x = vehicle_path[time_index][1] + m.vehicle_D*cos(vehicle_path[time_index][3])
            vehicle_center_y = vehicle_path[time_index][2] + m.vehicle_D*sin(vehicle_path[time_index][3])
            # println(vehicle_center_x,vehicle_center_y)
            for obstacle in m.world.obstacles
                if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,m.min_safe_distance_from_obstacle+m.vehicle_R+obstacle.r) )
                    # println("Collision with this obstacle " ,obstacle)
                    new_vehicle_position = (-100.0, -100.0, -100.0)
                    collision_with_obstacle = true
                    next_human_states = HumanState[]
                    observed_positions = Location[ Location(-50.0,-50.0) ]
                    sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
                    r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
                    return (sp=sp, o=observed_positions, r=r)
                end
            end
        end
    end

    # if( new_vehicle_position[1]<0.0+s.vehicle_L || new_vehicle_position[2]<0.0+s.vehicle_L || new_vehicle_position[1]>m.world.length-s.vehicle_L || new_vehicle_position[2]>m.world.breadth-s.vehicle_L )
    #     # println("Running into boundary wall")
    #     new_vehicle_position = (-100.0, -100.0, -100.0)
    #     collision_with_obstacle = true
    #     # next_human_states = human_state[]
    #     observed_positions = location[ location(-50.0,-50.0) ]
    #     sp = state_extended_space_POMDP_planner(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,s.vehicle_L,s.vehicle_goal,next_human_states)
    #     r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
    #     # println(s)
    #     # println(r)
    #     return (sp=sp, o=observed_positions, r=r)
    # end
    #
    # if(is_within_range(new_vehicle_position[1],new_vehicle_position[2], s.vehicle_goal.x, s.vehicle_goal.y, m.radius_around_vehicle_goal))
    #     # println("Goal reached")
    #     new_vehicle_position = (-100.0, -100.0, -100.0)
    #     vehicle_reached_goal = true
    #     # next_human_states = human_state[]
    #     observed_positions = location[ location(-50.0,-50.0) ]
    #     sp = state_extended_space_POMDP_planner(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,s.vehicle_L,s.vehicle_goal,next_human_states)
    #     r = vehicle_goal_reached_reward(vehicle_reached_goal, m.goal_reached_reward)
    #     return (sp=sp, o=observed_positions, r=r)
    # end

    # If the code reaches here, then both s and sp are safe states. Define corresponding new POMDP state.
    sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed,next_human_states)

    # R(s,a): Reward for being at state s and taking action a
    #Penalize if collision with human
    r = low_speed_penalty(s.vehicle_v, m.max_vehicle_speed)
    #println("Reward from not traveling at max_speed ", r)
    #Penalize if had to apply sudden brakes
    r += immediate_stop_penalty(immediate_stop, m.human_collision_penalty)
    #println("Reward if you had to apply immediate brakes", r)
    #Penalize if vehicle's heading angle changes
    r += heading_angle_change_penalty(sp.vehicle_v,a.steering_angle)
    #Penalty to avoid long paths
    r += -1.0
    # println(r)

    # if( sp.vehicle_x<0.0+sp.vehicle_L || sp.vehicle_y<0.0+sp.vehicle_L || sp.vehicle_x>m.world.length-sp.vehicle_L || sp.vehicle_y>m.world.breadth-sp.vehicle_L )
    #     println(s)
    #     println(sp)
    #     println(r)
    # end
    # println(s)
    # println(sp)
    # println(r)
    # println("***************************************")
    return (sp=sp, o=observed_positions, r=r)
end

function testgen(unit_test_env,unit_test_es_pomdp,rng)
    unit_test_h1 = HumanState(10.0,10.0,1.0,Location(0.0,0.0))
    unit_test_h2 = HumanState(10.0,10.0,1.0,Location(100.0,0.0))
    # unit_test_h1 = HumanState(2.0,4.0,1.0,Location(0.0,0.0))
    # unit_test_h2 = HumanState(3.0,3.0,1.0,Location(100.0,0.0))
    # unit_test_nearby_humans = HumanState[unit_test_h1,unit_test_h2]
    unit_test_nearby_humans = SVector{2,HumanState}(unit_test_h1,unit_test_h2)
    unit_test_es_planner_state = StateExtendedSpacePOMDP(2.0,2.0,0.0,1.0,unit_test_nearby_humans)
    unit_test_es_planner_action = ActionExtendedSpacePOMDP(pi/12,1.0)
    # unit_test_env = ExperimentEnvironment(100.0,100.0,SVector{0,ObstacleLocation}())
    # unit_test_env = ExperimentEnvironment(100.0,100.0,ObstacleLocation[])
    # unit_test_es_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide)
    return POMDPs.gen( unit_test_es_pomdp,unit_test_es_planner_state,unit_test_es_planner_action,rng )
end



#=
************************************************************************************************
Upper bound value function for DESPOT
=#

function is_collision_state(s::StateExtendedSpacePOMDP,m::ExtendedSpacePOMDP)
    vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
    vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)
    if( vehicle_center_x<0.0+m.vehicle_R || vehicle_center_y<0.0+m.vehicle_R || vehicle_center_x>m.world.length-m.vehicle_R || vehicle_center_y>m.world.breadth-m.vehicle_R )
        return true
    elseif(s.vehicle_v != 0.0)
        for human in s.nearby_humans
            if( is_within_range(vehicle_center_x,vehicle_center_y,human.x,human.y,m.min_safe_distance_from_human+m.vehicle_R) )
                return true
            end
        end
    end
    for obstacle in m.world.obstacles
        if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,m.min_safe_distance_from_obstacle+obstacle.r+m.vehicle_R) )
            return true
        end
    end
    return false
end

#This is not accurate for HV or NHV, especially when static obstacles are present. Can we get a better and tighter upper bound?
function time_to_goal(s::StateExtendedSpacePOMDP,m::ExtendedSpacePOMDP{HJBPolicy})
    vehicle_distance_to_goal = sqrt( (s.vehicle_x-m.vehicle_goal.x)^2 + (s.vehicle_y-m.vehicle_goal.y)^2 )
    # println("Distance is :", vehicle_distance_to_goal)
    # println(s)
    # println("Time is :", floor(vehicle_distance_to_goal/max_vehicle_speed))
    # return floor(vehicle_distance_to_goal/m.max_vehicle_speed/m.one_time_step)
    return floor(vehicle_distance_to_goal/m.max_vehicle_speed)
end

# function time_to_goal(s::StateExtendedSpacePOMDP, m::ExtendedSpacePOMDP{HJBPolicy})
#     # interpolate HJB value for given state
#     x = SVector(s.vehicle_x, s.vehicle_y, s.vehicle_theta, s.vehicle_v)
#     HJB_val_x = interp_value(x, m.rollout_guide.value_array, m.rollout_guide.state_grid)
#     # calculate minimum number of steps to reach goal for given HJB value
#     steps_to_goal = floor(HJB_val_x / m.rollout_guide.Dt)
#     # # apply accumulated discount to undiscounted HJB value
#     # discounted_HJB_time_to_goal = m.discount_factor^steps_to_goal * HJB_val_x
#     return steps_to_goal
# end

# function calculate_upper_bound(m::ExtendedSpacePOMDP{HJBPolicy}, b)
#
#     # lower = lbound(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=100),m,b)
#     # lower = lbound(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(m, b)),max_depth=pomdp_details.tree_search_max_depth),m,b)
#     value_sum = 0.0
#     if(b.depth == 100)
#         return value_sum
#     else
#         for (s, w) in weighted_particles(b)
#             if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
#                 value_sum += 0.0
#             elseif(is_collision_state(s,m))
#                 value_sum += w*m.human_collision_penalty
#                 # println("Upper bound PB is :", value_sum)
#             elseif(is_within_range(s.vehicle_x,s.vehicle_y, m.vehicle_goal.x, m.vehicle_goal.y, m.radius_around_vehicle_goal))
#                 value_sum += w*m.goal_reached_reward
#                 # println("Upper bound PA is :", value_sum)
#             else
#                 value_sum += w*((discount(m)^time_to_goal(s,m))*m.goal_reached_reward)
#                 # println("Upper bound PC is :", value_sum)
#             end
#         end
#     end
#     # println("Upper bound is :", value_sum)
#     # u = (value_sum)/weight_sum(b)
#     # if lower > value_sum
#     #     push!(bad, (lower,value_sum,b))
#     #     @show("While debugging ",lower,value_sum,b.depth)
#     # end
#     return value_sum
# end
# @code_warntype calculate_upper_bound_value(golfcart_pomdp(), initialstate_distribution(golfcart_pomdp()))

# function calculate_upper_bound(m::ExtendedSpacePOMDP{HJBPolicy}, b)
#
#     # lower = lbound(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(m, b)),max_depth=pomdp_details.tree_search_max_depth),m,b)
#     value_sum = 0.0
#     if(b.depth == 100)
#         return value_sum
#     else
#         fp = first(particles(b))
#         x = SVector(fp.vehicle_x, fp.vehicle_y, fp.vehicle_theta, fp.vehicle_v)
#         HJB_val_x = interp_value(x, m.rollout_guide.value_array, m.rollout_guide.state_grid)
#         for (s, w) in weighted_particles(b)
#             if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
#                 value_sum += 0.0
#             elseif(is_collision_state(s,m))
#                 value_sum += w*m.human_collision_penalty
#                 # println("Upper bound PB is :", value_sum)
#             else
#                 value_sum += w*(HJB_val_x)
#                 # println("Upper bound PC is :", value_sum)
#             end
#         end
#     end
#     # println("Upper bound is :", value_sum)
#     # # u = (value_sum)/weight_sum(b)
#     # if(value_sum < 0.0)
#     #     value_sum = -100.0
#     # end
#     # if lower > value_sum
#     #     push!(bad, (lower,value_sum,b))
#     #     @show("While debugging ",lower,value_sum,b.depth)
#     # end
#     return value_sum
# end

function upper_bound_rollout_gen(m::ExtendedSpacePOMDP, x, a)

    vehicle_reached_goal = false
    collision_with_obstacle = false
    immediate_stop = false

    #=
        Check if current vehicle position collides with any static obstacle.
        Check if current vehicle position is outside the environment boundary.
        Check if current vehicle position is in the goal region.
        Apply given action on the vehicle and get vehicle path.
        Check if vehicle path collides with any static obstacle.
        Generate the new state accordingly.
        Generate corresponding reward.
    =#

    vehicle_center_x = x[1] + m.vehicle_D*cos(x[3])
    vehicle_center_y = x[2] + m.vehicle_D*sin(x[3])
    for obstacle in m.world.obstacles
        if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,obstacle.r+m.min_safe_distance_from_obstacle+m.vehicle_R) )
            # println("Collision with this obstacle " ,obstacle)
            # println(x)
            # println(vehicle_center_x, vehicle_center_y)
            new_vehicle_position = (-100.0, -100.0, -100.0)
            collision_with_obstacle = true
            sp = SVector(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],x[4])
            r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
            return (sp=sp,r=r)
        end
    end

    if( vehicle_center_x<0.0+m.vehicle_R || vehicle_center_y<0.0+m.vehicle_R || vehicle_center_x>m.world.length-m.vehicle_R || vehicle_center_y>m.world.breadth-m.vehicle_R )
        # println("Running into boundary wall")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        collision_with_obstacle = true
        sp = SVector(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],x[4])
        r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
        # println(s)
        # println(r)
        return (sp=sp, r=r)
    end

    if(is_within_range(vehicle_center_x,vehicle_center_y, m.vehicle_goal.x, m.vehicle_goal.y, m.radius_around_vehicle_goal))
        # println("Goal reached")
        new_vehicle_position = (-100.0, -100.0, -100.0)
        vehicle_reached_goal = true
        sp = SVector(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],x[4])
        r = vehicle_goal_reached_reward(vehicle_reached_goal, m.goal_reached_reward)
        return (sp=sp, r=r)
    end

    if(a[2] == -10.0)
        immediate_stop = true
    end
    new_vehicle_speed = clamp(x[4] + a[2], 0.0, m.max_vehicle_speed)
    steering_angle = a[1]
    vehicle_path = update_vehicle_position(StateExtendedSpacePOMDP(x[1],x[2],x[3],x[4],HumanState[]), m, steering_angle, new_vehicle_speed)
    new_vehicle_position = vehicle_path[end]

    if(new_vehicle_speed!=0.0)
        for time_index in 2:m.num_segments_in_one_time_step+1
            vehicle_center_x = vehicle_path[time_index][1] + m.vehicle_D*cos(vehicle_path[time_index][3])
            vehicle_center_y = vehicle_path[time_index][2] + m.vehicle_D*sin(vehicle_path[time_index][3])
            for obstacle in m.world.obstacles
                if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,m.min_safe_distance_from_obstacle+m.vehicle_R+obstacle.r) )
                    # println("x " ,x)
                    # println("a " ,a)
                    # println("Collision with this obstacle " ,obstacle)
                    # println("Vehicle path " ,vehicle_path)
                    # println("T " ,time_index)
                    # println("veh_pos " ,vehicle_path[time_index])
                    # println("veh_center ", (vehicle_center_x, vehicle_center_y))
                    new_vehicle_position = (-100.0, -100.0, -100.0)
                    collision_with_obstacle = true
                    sp = SVector(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],x[4])
                    r = obstacle_collision_penalty(collision_with_obstacle, m.obstacle_collision_penalty)
                    return (sp=sp, r=r)
                end
            end
        end
    end

    # If the code reaches here, then both s and sp are safe states. Define corresponding new POMDP state.
    sp = SVector(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],new_vehicle_speed)

    # R(s,a): Reward for being at state s and taking action a
    #Penalize if collision with human
    r = low_speed_penalty(x[4], m.max_vehicle_speed)
    #println("Reward from not traveling at max_speed ", r)
    #Penalize if had to apply sudden brakes
    r += immediate_stop_penalty(immediate_stop, m.human_collision_penalty)
    #println("Reward if you had to apply immediate brakes", r)
    #Penalize if vehicle's heading angle changes
    # r += heading_angle_change_penalty(a.steering_angle)
    #Penalty to avoid long paths
    r += -1.0

    # if( sp.vehicle_x<0.0+sp.vehicle_L || sp.vehicle_y<0.0+sp.vehicle_L || sp.vehicle_x>m.world.length-sp.vehicle_L || sp.vehicle_y>m.world.breadth-sp.vehicle_L )
    #     println(s)
    #     println(sp)
    #     println(r)
    # end
    # println(s)
    # println(sp)
    # println(r)
    # println("***************************************")
    return (sp=sp, r=r)
end

function calculate_upper_bound(m::ExtendedSpacePOMDP, b)
    # lower = lbound(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(m, b)),max_depth=pomdp_details.tree_search_max_depth),m,b)
    # println(lower)
    value_sum = 0.0
    if(b.depth == 100)
        return value_sum
    else
        s = first(particles(b))
        x = SVector(s.vehicle_x, s.vehicle_y, s.vehicle_theta, s.vehicle_v)
        a, q_vals = HJB_policy(SVector(x[1],x[2],wrap_between_negative_pi_to_pi(x[3]),x[4]),0.5,750.0,m.rollout_guide.get_actions,
                    m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,m.rollout_guide.value_array,m.rollout_guide.veh,
                    m.rollout_guide.state_grid)
        i = 0
        # println(x)
        while(x[1]!= -100.0 && b.depth+i <100)
            # println(x)
            # println(a)
            # println(q_vals)
            new_x, r =  upper_bound_rollout_gen(m, x, a)
            value_sum += r
            x = new_x
            a, q_vals = HJB_policy(SVector(x[1],x[2],wrap_between_negative_pi_to_pi(x[3]),x[4]),0.5,750.0,m.rollout_guide.get_actions,
                        m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,m.rollout_guide.value_array,m.rollout_guide.veh,
                        m.rollout_guide.state_grid)

            i+=1
            # println(new_x)
        end
    end
    # if ( lower - value_sum > 10.0 )
    #         push!(bad, (lower,value_sum,b))
    #         println("While debugging ",lower, " ",value_sum, " ",b.depth)
    # end
    # return value_sum+50.0
    return clamp(value_sum,-100.0,Inf)
end

function calculate_upper_bound_x(m::ExtendedSpacePOMDP{HJBPolicy}, x_stop)
    value_sum = 0.0

    # println("UBx: x_stop = ", x_stop)

    x = SVector(x_stop[1], x_stop[2], wrap_between_0_to_2pi(x_stop[3]), 0.0)

    # println("UBx: x = ", x)

    a, q_vals = HJB_policy(SVector(x[1],x[2],wrap_between_negative_pi_to_pi(x[3]),x[4]), 0.5, 750.0, m.rollout_guide.get_actions,
                m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,m.rollout_guide.value_array,m.rollout_guide.veh,
                m.rollout_guide.state_grid)

    i = 0
    while(x[1] != -100.0 && i < 100)
        # generate r and x' for current x-a
        new_x, r = upper_bound_rollout_gen(m, x, a)
        value_sum += r

        # pass x to next time step, get next action from policy
        x = new_x
        a, q_vals = HJB_policy(SVector(x[1],x[2],wrap_between_negative_pi_to_pi(x[3]),x[4]), 0.5, 750.0, m.rollout_guide.get_actions,
                    m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,m.rollout_guide.value_array,m.rollout_guide.veh,
                    m.rollout_guide.state_grid)

        # println("UBx: x = ", x)

        i+=1
    end

    return clamp(value_sum, -100.0, Inf)
end

#=
************************************************************************************************
Lower bound policy function for DESPOT
=#
function calculate_lower_bound(m::ExtendedSpacePOMDP,b)
    # println(typeof(b))
    #Implement a reactive controller for your lower bound
    delta_speed = m.vehicle_action_delta_speed

    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true
    # if(length(b.scenarios)==1)
        # println("Tree Depth : ", b.depth)
    #     println(b.scenarios)
    # end
    debug = false
    if(debug)
        fp = first(particles(b))
        println([fp.vehicle_x,fp.vehicle_y,fp.vehicle_theta,fp.vehicle_v])
    end
    for (s, w) in weighted_particles(b)
        if(s==nothing)
            println(b)
        end
        # println(s.vehicle_x)
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            # println("HG")
            continue
        else
            if(first_execution_flag)
                first_execution_flag = false
            end
            dist_to_closest_human = 20000.0  #Some really big infeasible number (not Inf to avoid the type mismatch error)
            for human in s.nearby_humans
                euclidean_distance = sqrt((s.vehicle_x - human.x)^2 + (s.vehicle_y - human.y)^2)
                if(euclidean_distance < dist_to_closest_human)
                    dist_to_closest_human = euclidean_distance
                end
                if(dist_to_closest_human < m.d_near)
                    delta_speed = -delta_speed
                    safe_value_lim = 750.0
                    a, q_vals = reactive_policy(SVector(s.vehicle_x,s.vehicle_y,wrap_between_negative_pi_to_pi(s.vehicle_theta),s.vehicle_v),delta_speed,
                        safe_value_lim,m.rollout_guide.get_actions,m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
                        m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
                    if(debug)
                        println("Near a Human")
                        println( ActionExtendedSpacePOMDP(a[1],delta_speed) )
                    end
                    return ActionExtendedSpacePOMDP(a[1],delta_speed)
                end
            end
            if(dist_to_closest_human > m.d_far)
                chosen_delta_speed = m.vehicle_action_delta_speed
            else
                chosen_delta_speed = 0.0
            end
            if(chosen_delta_speed < delta_speed)
                delta_speed = chosen_delta_speed
            end
        end
    end

    #This condition is true only when all the states in the belief are terminal. In that case, just return (0.0,0.0)
    if(first_execution_flag)
        if(debug)
            println(ActionExtendedSpacePOMDP(0.0,0.0))
        end
        return ActionExtendedSpacePOMDP(0.0,0.0)
    end
    s = first(particles(b))
    safe_value_lim = 750.0
    a,q_vals = reactive_policy(SVector(s.vehicle_x,s.vehicle_y,wrap_between_negative_pi_to_pi(s.vehicle_theta),s.vehicle_v),delta_speed,
        safe_value_lim,m.rollout_guide.get_actions,m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
        m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
    if(debug)
        @show(delta_speed)
        println( ActionExtendedSpacePOMDP(a[1],a[2]) )
    end
    return ActionExtendedSpacePOMDP(a[1],a[2])
end

#=
************************************************************************************************
Action Function for the POMDP
=#

# ISSUE: need to modify steering angles with +/-Dv in here

function get_actions(m::ExtendedSpacePOMDP,b)
    max_steering_angle = m.max_vehicle_steering_angle
    max_delta_angle = m.vehicle_action_max_delta_heading_angle
    delta_speed = m.vehicle_action_delta_speed
    pomdp_state = first(particles(b))
    if(pomdp_state.vehicle_v == 0.0)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,delta_speed,m.one_time_step),0.0,max_steering_angle)
        return [ActionExtendedSpacePOMDP(-steering_angle,delta_speed),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(2*steering_angle/3,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle,delta_speed)
                ]
    elseif(pomdp_state.vehicle_v == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
        rollout_action, q_vals = reactive_policy(SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),pomdp_state.vehicle_v),
                                delta_speed,750.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
                                m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
        return [
                ActionExtendedSpacePOMDP(-steering_angle,0.0),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(0.0,-delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(steering_angle,0.0),
                ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                ActionExtendedSpacePOMDP(steering_angle,-delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle,-delta_speed),
                ActionExtendedSpacePOMDP(-10.0,-10.0)
                ]
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
        rollout_action, q_vals = reactive_policy(SVector(pomdp_state.vehicle_x,pomdp_state.vehicle_y,wrap_between_negative_pi_to_pi(pomdp_state.vehicle_theta),pomdp_state.vehicle_v),
                                delta_speed,750.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
                                m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
        return [ActionExtendedSpacePOMDP(-steering_angle,0.0),
                ActionExtendedSpacePOMDP(-2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(-steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(0.0,-delta_speed),
                ActionExtendedSpacePOMDP(0.0,0.0),
                ActionExtendedSpacePOMDP(0.0,delta_speed),
                ActionExtendedSpacePOMDP(steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(2*steering_angle/3,0.0),
                ActionExtendedSpacePOMDP(steering_angle,0.0),
                ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2]),
                ActionExtendedSpacePOMDP(steering_angle,-delta_speed),
                ActionExtendedSpacePOMDP(-steering_angle,-delta_speed),
                ActionExtendedSpacePOMDP(-10.0,-10.0)
                ]
    end
end

function default_es_pomdp_action(m,b,ex)
    delta_speed = m.vehicle_action_delta_speed
    # pomdp_state = first(particles(b))
    rollout_action, q_vals = reactive_policy(SVector(b.vehicle_x,b.vehicle_y,wrap_between_negative_pi_to_pi(b.vehicle_theta),b.vehicle_v),
                                delta_speed,750.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,m.rollout_guide.q_value_array,
                                m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
    # println(ex)
    return ActionExtendedSpacePOMDP(rollout_action[1],rollout_action[2])
end

discount(m::ExtendedSpacePOMDP) = m.discount_factor
isterminal(m::ExtendedSpacePOMDP, s::StateExtendedSpacePOMDP) = is_terminal_state(s,Location(-100.0,-100.0));
actions(m::ExtendedSpacePOMDP,b) = get_actions(m,b)


# function get_actions(m::ExtendedSpacePOMDP,b)
#     pomdp_state = first(particles(b))
#     required_orientation = get_heading_angle( m.vehicle_goal.x, m.vehicle_goal.y, pomdp_state.vehicle_x, pomdp_state.vehicle_y)
#     delta_angle = required_orientation - pomdp_state.vehicle_theta
#     abs_delta_angle = abs(delta_angle)
#     if(abs_delta_angle<=pi)
#         delta_angle = clamp(delta_angle, -pi/4, pi/4)
#     else
#         if(delta_angle>=0.0)
#             delta_angle = clamp(delta_angle-2*pi, -pi/4, pi/4)
#         else
#             delta_angle = clamp(delta_angle+2*pi, -pi/4, pi/4)
#         end
#     end
#     if(pomdp_state.vehicle_v == 0.0)
#         if(delta_angle==pi/4 || delta_angle==-pi/4)
#             return [ActionExtendedSpacePOMDP(-pi/4,1.0),ActionExtendedSpacePOMDP(-pi/6,1.0),ActionExtendedSpacePOMDP(-pi/12,1.0),
#             ActionExtendedSpacePOMDP(0.0,0.0),ActionExtendedSpacePOMDP(0.0,1.0),ActionExtendedSpacePOMDP(pi/12,1.0),
#             ActionExtendedSpacePOMDP(pi/6,1.0),ActionExtendedSpacePOMDP(pi/4,1.0)]
#             # return [(-pi/4,2.0),(-pi/6,2.0),(-pi/12,2.0),(0.0,0.0),(0.0,2.0),(pi/12,2.0),(pi/6,2.0),(pi/4,2.0)]
#         else
#             return [ActionExtendedSpacePOMDP(delta_angle, 1.0),ActionExtendedSpacePOMDP(-pi/4,1.0),ActionExtendedSpacePOMDP(-pi/6,1.0),
#             ActionExtendedSpacePOMDP(-pi/12,1.0),ActionExtendedSpacePOMDP(0.0,0.0),ActionExtendedSpacePOMDP(0.0,1.0),
#             ActionExtendedSpacePOMDP(pi/12,1.0),ActionExtendedSpacePOMDP(pi/6,1.0),ActionExtendedSpacePOMDP(pi/4,1.0)]
#             # return [(delta_angle, 2.0),(-pi/4,2.0),(-pi/6,2.0),(-pi/12,2.0),(0.0,0.0),(0.0,2.0),(pi/12,2.0),(pi/6,2.0),(pi/4,2.0)]
#         end
#     else
#         if(delta_angle==pi/4 || delta_angle==-pi/4)
#             return [ActionExtendedSpacePOMDP(-pi/4,0.0),ActionExtendedSpacePOMDP(-pi/6,0.0),ActionExtendedSpacePOMDP(-pi/12,0.0),
#             ActionExtendedSpacePOMDP(0.0,-1.0),ActionExtendedSpacePOMDP(0.0,0.0),ActionExtendedSpacePOMDP(0.0,1.0),
#             ActionExtendedSpacePOMDP(pi/12,0.0),ActionExtendedSpacePOMDP(pi/6,0.0),ActionExtendedSpacePOMDP(pi/4,0.0),
#             ActionExtendedSpacePOMDP(-10.0,-10.0)]
#             # return [ActionExtendedSpacePOMDP(-pi/4,0.0),ActionExtendedSpacePOMDP(-pi/6,0.0),ActionExtendedSpacePOMDP(-pi/12,0.0),
#             # ActionExtendedSpacePOMDP(0.0,-1.0),ActionExtendedSpacePOMDP(0.0,0.0),ActionExtendedSpacePOMDP(0.0,1.0),
#             # ActionExtendedSpacePOMDP(pi/12,0.0),ActionExtendedSpacePOMDP(pi/6,0.0),ActionExtendedSpacePOMDP(pi/4,0.0)]
#         else
#             return [ActionExtendedSpacePOMDP(delta_angle, 0.0),ActionExtendedSpacePOMDP(-pi/4,0.0),ActionExtendedSpacePOMDP(-pi/6,0.0),
#             ActionExtendedSpacePOMDP(-pi/12,0.0),ActionExtendedSpacePOMDP(0.0,-1.0),ActionExtendedSpacePOMDP(0.0,0.0),
#             ActionExtendedSpacePOMDP(0.0,1.0),ActionExtendedSpacePOMDP(pi/12,0.0),ActionExtendedSpacePOMDP(pi/6,0.0),
#             ActionExtendedSpacePOMDP(pi/4,0.0),ActionExtendedSpacePOMDP(-10.0,-10.0)]
#             # return [ActionExtendedSpacePOMDP(delta_angle, 0.0),ActionExtendedSpacePOMDP(-pi/4,0.0),ActionExtendedSpacePOMDP(-pi/6,0.0),
#             # ActionExtendedSpacePOMDP(-pi/12,0.0),ActionExtendedSpacePOMDP(0.0,-1.0),ActionExtendedSpacePOMDP(0.0,0.0),
#             # ActionExtendedSpacePOMDP(0.0,1.0),ActionExtendedSpacePOMDP(pi/12,0.0),ActionExtendedSpacePOMDP(pi/6,0.0),
#             # ActionExtendedSpacePOMDP(pi/4,0.0)]
#         end
#         # return [(delta_angle, 1.0),(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-1.0),(0.0,0.0),(0.0,1.0),(pi/12,1.0),(pi/6,0.0),(pi/4,0.0),(-10.0,-10.0)]
#         # return [(delta_angle, 1.0),(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-1.0),(0.0,0.0),(0.0,1.0),(pi/12,1.0),(pi/6,0.0),(pi/4,0.0),(-10.0,-10.0)]
#     end
# end
