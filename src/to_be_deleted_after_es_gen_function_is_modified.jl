function POMDPs.gen(m::ExtendedSpacePOMDP, s, a, rng)

    vehicle_reached_goal = false
    collision_with_human = false
    collision_with_obstacle = false
    immediate_stop = false
    next_human_states = HumanState[]
    observed_positions = Location[]

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

    # x = [s.vehicle_x,s.vehicle_y,s.vehicle_theta,s.vehicle_v]
    # println(x)
    vehicle_center_x = s.vehicle_x + m.vehicle_D*cos(s.vehicle_theta)
    vehicle_center_y = s.vehicle_y + m.vehicle_D*sin(s.vehicle_theta)

    for human in s.nearby_humans
        if(s.vehicle_v != 0.0)
            if( is_within_range(vehicle_center_x, vehicle_center_y, human.x, human.y, m.min_safe_distance_from_human+m.vehicle_R) )
                # println("Collision with this human " ,s.nearby_humans[human_index] , " ", time_index )
                # println("Vehicle's position is " ,vehicle_path[time_index] , "\nHuman's position is ", intermediate_human_location )
                new_vehicle_position = (-100.0, -100.0, -100.0)
                collision_with_human = true
                # next_human_states = human_state[]
                observed_positions = Location[ Location(-50.0,-50.0) ]
                sp = StateExtendedSpacePOMDP(new_vehicle_position[1],new_vehicle_position[2],new_vehicle_position[3],s.vehicle_v,next_human_states)
                r = human_collision_penalty(collision_with_human, m.human_collision_penalty)
                return (sp=sp, o=observed_positions, r=r)
            end
        end
    end

    for obstacle in m.world.obstacles
        if( is_within_range(vehicle_center_x,vehicle_center_y,obstacle.x,obstacle.y,obstacle.r+m.min_safe_distance_from_obstacle+m.vehicle_R) )
            # println("Collision with this obstacle " ,obstacle)
            new_vehicle_position = (-100.0, -100.0, -100.0)
            collision_with_obstacle = true
            # next_human_states = human_state[]
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
        # next_human_states = human_state[]
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
        scaling_factor_for_noise = 0.0
        noise = (rand(rng) - 0.5)*human.v*m.one_time_step*scaling_factor_for_noise
        modified_human_state = update_human_position(human,m.world.length,m.world.breadth,m.one_time_step,noise)
        if(new_vehicle_speed!=0.0)
            human_path_x = LinRange(human.x,modified_human_state.x,m.num_segments_in_one_time_step+1)
            human_path_y = LinRange(human.y,modified_human_state.y,m.num_segments_in_one_time_step+1)
            for time_index in 2:m.num_segments_in_one_time_step+1
                vehicle_center_x = vehicle_path[time_index][1] + m.vehicle_D*cos(vehicle_path[time_index][3])
                vehicle_center_y = vehicle_path[time_index][2] + m.vehicle_D*sin(vehicle_path[time_index][3])
                println(vehicle_center_x,vehicle_center_y)
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
        if(modified_human_state.x!=modified_human_state.goal.x && modified_human_state.y!=modified_human_state.goal.y)
            push!(next_human_states, modified_human_state)
            discrete_new_x = floor(modified_human_state.x/m.observation_discretization_length) * m.observation_discretization_length
            discrete_new_y = floor(modified_human_state.y/m.observation_discretization_length) * m.observation_discretization_length
            observed_location = Location(discrete_new_x, discrete_new_y)
            push!(observed_positions, observed_location)
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
    return (sp=sp, o=observed_positions, r=r)
end
#=
unit_test_h1 = HumanState(10.0,10.0,1.0,Location(0.0,0.0))
unit_test_h2 = HumanState(10.0,10.0,1.0,Location(100.0,0.0))
unit_test_nearby_humans = HumanState[unit_test_h1,unit_test_h2]
unit_test_es_planner_state = StateExtendedSpacePOMDP(2.0,2.0,0.0,1.0,unit_test_nearby_humans)
unit_test_es_planner_action = ActionExtendedSpacePOMDP(pi/12,1.0)
unit_test_env = experiment_environment(100.0,100.0,obstacle_location[])
unit_test_es_pomdp = extended_space_POMDP_planner(0.99,1.0,-100.0,1.0,-100.0,1.0,100.0,3.0,1.0,10,1.0,unit_test_env)
POMDPs.gen( unit_test_es_pomdp,unit_test_es_planner_state,unit_test_es_planner_action,MersenneTwister(7) )
=#
