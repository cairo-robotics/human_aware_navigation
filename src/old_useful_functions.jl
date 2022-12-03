function calculate_lower_bound(m::ExtendedSpacePOMDP,b)
    #Implement a reactive controller for your lower bound
    speed_change_to_be_returned = 1.0
    delta_angle = 0.0
    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true
    # if(length(b.scenarios)==1)
    #     println("Tree Depth : ", b.depth)
    #     println(b.scenarios)
    # end
    for (s, w) in weighted_particles(b)
        if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
            # println("HG ",b.depth)
            continue
        else
            if(first_execution_flag)
                required_orientation = get_heading_angle( m.vehicle_goal.x, m.vehicle_goal.y, s.vehicle_x, s.vehicle_y)
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
                    if(dist_to_closest_human < m.d_near)
                        return ActionExtendedSpacePOMDP(delta_angle,-1.0)
                    end
                end
                if(dist_to_closest_human > m.d_far)
                    chosen_delta_speed = 1.0
                else
                    chosen_delta_speed = 0.0
                end
                if(chosen_delta_speed < speed_change_to_be_returned)
                    speed_change_to_be_returned = chosen_delta_speed
                end
            end
        end
    end

    #This condition is true only when all the states in the belief are terminal. In that case, just return (0.0,0.0)
    if(first_execution_flag == true)
        #@show(0.0,0.0)
        return ActionExtendedSpacePOMDP(0.0,0.0)
    end

    #This means all humans are away and you can accelerate.
    if(speed_change_to_be_returned == 1.0)
        #@show(0.0,speed_change_to_be_returned)
        return ActionExtendedSpacePOMDP(delta_angle,speed_change_to_be_returned)
    end

    #If code has reached this point, then the best action is to maintain your current speed.
    #We have already found the best steering angle to take.
    #@show(best_delta_angle,0.0)
    return ActionExtendedSpacePOMDP(delta_angle,0.0)
end

function calculate_lower_bound_new(m::ExtendedSpacePOMDP,b)
    #Implement a reactive controller for your lower bound
    speed_change_to_be_returned = 1.0
    delta_angle = 0.0
    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true
    println("Tree Depth : ", b.depth)
    for (s, w) in weighted_particles(b)
        if(is_within_range(s.vehicle_x,s.vehicle_y, s.vehicle_goal.x, s.vehicle_goal.y, m.radius_around_vehicle_goal))
            println("GOAL", b.depth)
        end
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
                    if(dist_to_closest_human < m.d_near)
                        return ActionExtendedSpacePOMDP(delta_angle,-1.0)
                    end
                end
                if(dist_to_closest_human > m.d_far)
                    chosen_delta_speed = 1.0
                else
                    chosen_delta_speed = 0.0
                end
                if(chosen_delta_speed < speed_change_to_be_returned)
                    speed_change_to_be_returned = chosen_delta_speed
                end
            end
        end
    end

    #This condition is true only when all the states in the belief are terminal. In that case, just return (0.0,0.0)
    if(first_execution_flag == true)
        #@show(0.0,0.0)
        return ActionExtendedSpacePOMDP(0.0,0.0)
    end

    #This means all humans are away and you can accelerate.
    if(speed_change_to_be_returned == 1.0)
        #@show(0.0,speed_change_to_be_returned)
        return ActionExtendedSpacePOMDP(delta_angle,speed_change_to_be_returned)
    end

    #If code has reached this point, then the best action is to maintain your current speed.
    #We have already found the best steering angle to take.
    #@show(best_delta_angle,0.0)
    return ActionExtendedSpacePOMDP(delta_angle,0.0)
end


function reward_at_max_depth_lower_bound_policy_rollout(m,b)
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

#=
************************************************************************************************
Functions for debugging lb>ub error
=#
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

function debug_calculate_lower_bound(m::ExtendedSpacePOMDP,s)
    #Implement a reactive controller for your lower bound
    speed_change_to_be_returned = 1.0
    delta_angle = 0.0
    #This bool is also used to check if all the states in the belief are terminal or not.
    first_execution_flag = true

    if(is_within_range(s.vehicle_x,s.vehicle_y, s.vehicle_goal.x, s.vehicle_goal.y, m.radius_around_vehicle_goal))
        println("GOAL")
    end
    if(s.vehicle_x == -100.0 && s.vehicle_y == -100.0)
        println("Terminal State")
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
                if(dist_to_closest_human < m.d_near)
                    return ActionExtendedSpacePOMDP(delta_angle,-1.0)
                end
            end
            if(dist_to_closest_human > m.d_far)
                chosen_delta_speed = 1.0
            else
                chosen_delta_speed = 0.0
            end
            if(chosen_delta_speed < speed_change_to_be_returned)
                speed_change_to_be_returned = chosen_delta_speed
            end
        end
    end


    #This condition is true only when all the states in the belief are terminal. In that case, just return (0.0,0.0)
    if(first_execution_flag == true)
        #@show(0.0,0.0)
        return ActionExtendedSpacePOMDP(0.0,0.0)
    end

    #This means all humans are away and you can accelerate.
    if(speed_change_to_be_returned == 1.0)
        #@show(0.0,speed_change_to_be_returned)
        return ActionExtendedSpacePOMDP(delta_angle,speed_change_to_be_returned)
    end

    #If code has reached this point, then the best action is to maintain your current speed.
    #We have already found the best steering angle to take.
    #@show(best_delta_angle,0.0)
    return ActionExtendedSpacePOMDP(delta_angle,0.0)
end
