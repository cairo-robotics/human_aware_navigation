#Various different miscellaneous functions that are needed by different components and are common to multiple files

function is_within_range(p1_x,p1_y, p2_x, p2_y, threshold_distance)
    euclidean_distance = ((p1_x - p2_x)^2 + (p1_y - p2_y)^2)^0.5
    if(euclidean_distance<=threshold_distance)
        return true
    else
        return false
    end
end

function is_within_range(location1::Location, location2::Location, threshold_distance)
    euclidean_distance = ((location1.x - location2.x)^2 + (location1.y - location2.y)^2)^0.5
    if(euclidean_distance<=threshold_distance)
        return true
    else
        return false
    end
end

function wrap_between_0_and_2Pi(theta)
   return mod(theta,2*pi)
end

function in_obstacle(px,py,obstacle,padding=0.0)
    return is_within_range(px,py,obstacle.x,obstacle.y,obstacle.r+padding)
end

function find_maximum_element(some_array, len)
    max_element::Float64 = -Inf
    index::Int64 = 0
    for i in 1:len
        if(max_element<some_array[i])
            max_element = some_array[i]
            index = i
        end
    end
    return (max_element,index)
end

function get_heading_angle(human_x, human_y, vehicle_x, vehicle_y)

    #First Quadrant
    if(human_x >= vehicle_x && human_y >= vehicle_y)
        if(human_x == vehicle_x)
            heading_angle = pi/2.0
        elseif(human_y == vehicle_y)
            heading_angle = 0.0
        else
            heading_angle = atan((human_y - vehicle_y) / (human_x - vehicle_x))
        end
    #Second Quadrant
    elseif(human_x <= vehicle_x && human_y >= vehicle_y)
        if(human_x == vehicle_x)
            heading_angle = pi/2.0
        elseif(human_y == vehicle_y)
            heading_angle = pi/1.0
        else
            heading_angle = atan((human_y - vehicle_y) / (human_x - vehicle_x)) + pi
        end
    #Third Quadrant
    elseif(human_x <= vehicle_x && human_y <= vehicle_y)
        if(human_x == vehicle_x)
            heading_angle = 3*pi/2.0
        elseif(human_y == vehicle_y)
            heading_angle = pi/1.0
        else
            heading_angle = atan((human_y - vehicle_y) / (human_x - vehicle_x)) + pi
        end
    #Fourth Quadrant
    else(human_x >= vehicle_x && human_y <= vehicle_y)
        if(human_x == vehicle_x)
            heading_angle = 3*pi/2.0
        elseif(human_y == vehicle_y)
            heading_angle = 0.0
        else
            heading_angle = 2.0*pi + atan((human_y - vehicle_y) / (human_x - vehicle_x))
        end
    end

    return heading_angle
end

function get_steering_angle(vehicle_length, delta_angle, vehicle_speed, time_duration)
    if(vehicle_speed==0.0 || delta_angle==0.0)
        return 0.0
    else
        return atan((vehicle_length*delta_angle)/(vehicle_speed*time_duration))
    end
end

function move_vehicle(vehicle_x,vehicle_y,vehicle_theta,vehicle_L,steering_angle,vehicle_speed,time_duration)
    if(vehicle_speed == 0.0)
        return (vehicle_x,vehicle_y,vehicle_theta)
    end
    if(steering_angle == 0.0)
        new_theta = vehicle_theta
        new_x = vehicle_x + vehicle_speed*cos(new_theta)*(time_duration)
        new_y = vehicle_y + vehicle_speed*sin(new_theta)*(time_duration)
    else
        new_theta = vehicle_theta + (vehicle_speed * tan(steering_angle) * (time_duration) / vehicle_L)
        new_theta = wrap_between_0_and_2Pi(new_theta)
        new_x = vehicle_x + ((vehicle_L / tan(steering_angle)) * (sin(new_theta) - sin(vehicle_theta)))
        new_y = vehicle_y + ((vehicle_L / tan(steering_angle)) * (cos(vehicle_theta) - cos(new_theta)))
    end
    return (new_x,new_y,new_theta)
end

function get_vehicle_trajectory(vehicle,vehicle_params,time_value,planning_details,exp_details)

    current_x,current_y,current_theta = vehicle.x,vehicle.y,vehicle.theta
    vehicle_path_x, vehicle_path_y, vehicle_path_theta = Float64[current_x],Float64[current_y],Float64[current_theta]
    current_time_step_start = floor(time_value/exp_details.one_time_step)*planning_details.one_time_step
    current_time_step_end = current_time_step_start + planning_details.one_time_step
    num_steps_first_interval = Int64( (current_time_step_end - time_value)/exp_details.simulator_time_step )
    num_steps_remaining_intervals = Int64(planning_details.one_time_step/exp_details.simulator_time_step)

    for i in 1:num_steps_first_interval
        steering_angle = vehicle_params.controls_sequence[1]
        # steering_angle = get_steering_angle(vehicle_params.L,action,planning_details.veh_path_planning_v,planning_details.one_time_step)
        new_x,new_y,new_theta = get_new_vehicle_position(current_x,current_y,current_theta,vehicle_params.L,steering_angle,planning_details.veh_path_planning_v,exp_details.simulator_time_step)
        push!(vehicle_path_x,new_x)
        push!(vehicle_path_y,new_y)
        push!(vehicle_path_theta,new_theta)
        current_x,current_y,current_theta = new_x,new_y,new_theta
    end

    for steering_angle in vehicle_params.controls_sequence[2:end]
        # steering_angle = get_steering_angle(vehicle_params.L,action,planning_details.veh_path_planning_v,planning_details.one_time_step)
        for i in 1:num_steps_remaining_intervals
            new_x,new_y,new_theta = get_new_vehicle_position(current_x,current_y,current_theta,vehicle_params.L,
                                            steering_angle,planning_details.veh_path_planning_v,exp_details.simulator_time_step)
            push!(vehicle_path_x,new_x)
            push!(vehicle_path_y,new_y)
            push!(vehicle_path_theta,new_theta)
            current_x,current_y,current_theta = new_x,new_y,new_theta
            # println(current_x, " ", current_y, " ", current_theta," ",action)
        end
    end

    return vehicle_path_x,vehicle_path_y,vehicle_path_theta
end

function write_and_print(io::IOStream, string_to_be_written_and_printed::String)
    write(io, string_to_be_written_and_printed * "\n")
    println(string_to_be_written_and_printed)
end

function check_consistency_personal_copy(io, s)
    if s.move_count > 0.01*length(s.rngs) && s.move_warning
        msg = """
             DESPOT's MemorizingSource random number generator had to move the memory locations of the rngs $(s.move_count) times. If this number was large, it may be affecting performance (profiling is the best way to tell).
             To suppress this warning, use MemorizingSource(..., move_warning=false).
             To reduce the number of moves, try using MemorizingSource(..., min_reserve=n) and increase n until the number of moves is low (the final min_reserve was $(s.min_reserve)).
             """
        write_and_print(io, msg )
    end
end

function write_experiment_details_to_file(rand_noise_generator_seed_for_env,rand_noise_generator_seed_for_sim,
        rand_noise_generator_seed_for_solver, all_gif_environments, all_observed_environments, all_generated_beliefs_using_complete_lidar_data, all_generated_beliefs,
        all_generated_trees,all_risky_scenarios,all_actions,all_planners,cart_throughout_path, number_risks, number_of_sudden_stops,
        time_taken_by_cart, cart_reached_goal_flag, cart_ran_into_static_obstacle_flag, cart_ran_into_boundary_wall_flag, experiment_success_flag, filename)

    d = OrderedDict()
    d["rand_noise_generator_seed_for_env"] = rand_noise_generator_seed_for_env
    d["rand_noise_generator_seed_for_sim"] = rand_noise_generator_seed_for_sim
    # d["rand_noise_generator_seed_for_prm"] = rand_noise_generator_seed_for_prm
    d["rand_noise_generator_seed_for_solver"] = rand_noise_generator_seed_for_solver
    d["all_gif_environemnts"] = all_gif_environments
    d["all_observed_environments"] = all_observed_environments
    d["all_generated_beliefs_using_complete_lidar_data"] = all_generated_beliefs_using_complete_lidar_data
    d["all_generated_beliefs"] = all_generated_beliefs
    #d["all_generated_trees"] = all_generated_trees
    d["all_risky_scenarios"] = all_risky_scenarios
    d["all_actions"] = all_actions
    d["all_planners"] = all_planners
    #d["cart_throughout_path"] = cart_throughout_path
    d["number_risks"] = number_risks
    d["number_of_sudden_stops"] = number_of_sudden_stops
    d["time_taken_by_cart"] = time_taken_by_cart
    d["cart_reached_goal_flag"] = cart_reached_goal_flag
    d["cart_ran_into_static_obstacle_flag"] = cart_ran_into_static_obstacle_flag
    d["cart_ran_into_boundary_wall_flag"] = cart_ran_into_boundary_wall_flag
    d["experiment_success_flag"] = experiment_success_flag

    save(filename, d)
end

function calculate_mean_and_variance_from_given_dict(given_dict)
    total_sum = 0.0
    total_valid_entries = 0
    for k in keys(given_dict)
        if(given_dict[k] != nothing)
            total_sum += given_dict[k]
            total_valid_entries += 1
        end
    end

    given_dict_mean = total_sum/total_valid_entries
    given_dict_var = 0.0

    for k in keys(given_dict)
        if(given_dict[k] != nothing)
            given_dict_var += ( (given_dict[k] - given_dict_mean)^2 )
        end
    end

    given_dict_var = given_dict_var/(total_valid_entries-1)

    return given_dict_mean, given_dict_var
end

#************************************************************************************************
#Simulate human towards its goal for one time step

function update_human_position(human::HumanState,world_length::Float64,world_breadth::Float64,time_step::Float64,rand_num::Float64)

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
    new_human_state = HumanState(new_x,new_y,human.v,human.goal)

    return new_human_state
end
#=
unit_test_human = human_state(10.0,10.0,1.0,location(100.0,100.0),7.0)
update_human_position_pomdp_planning(unit_test_human,env.length,env.breadth,1.0,1.0,MersenneTwister(1234))
update_human_position_pomdp_planning(unit_test_human,100.0,100.0,1.0,1.0,MersenneTwister(1234))
@code_warntype update_human_position_pomdp_planning(unit_test_human,100.0,100.0,1.0,1.0,MersenneTwister(1234))
=#
