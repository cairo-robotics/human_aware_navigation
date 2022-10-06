Base.copy(obj::es_vehicle_parameters) = es_vehicle_parameters(obj.L,obj.max_speed,obj.goal)
Base.copy(obj::ls_vehicle_parameters) = ls_vehicle_parameters(obj.L,obj.max_speed,obj.goal,obj.hybrid_astar_path)
Base.copy(obj::Tuple{Array{human_state,1},Array{Int64,1}}) = (copy(obj[1]),copy(obj[2]))
Base.copy(obj::nearby_humans) = nearby_humans(copy(obj.position_data),copy(obj.ids),copy(obj.belief))



struct simulator
    env::experiment_environment
    vehicle::Vehicle
    vehicle_params::Union{es_vehicle_parameters, ls_vehicle_parameters}
    vehicle_sensor_data::vehicle_sensor
    humans::Array{human_state,1}
    humans_params::Array{human_parameters,1}
    one_time_step::Float64
end

#=
Function for moving vehicle in the environment
Returns a vehicle struct object
=#
function propogate_vehicle(vehicle::Vehicle, vehicle_params::es_vehicle_parameters, steering_angle::Float64, speed::Float64, time_duration::Float64)
    if(speed == 0.0)
        return Vehicle(vehicle.x,vehicle.y,vehicle.theta,speed)
    end
    if(steering_angle == 0.0)
        new_theta = vehicle.theta
        new_x = vehicle.x + speed*cos(new_theta)*(time_duration)
        new_y = vehicle.y + speed*sin(new_theta)*(time_duration)
    else
        new_theta = vehicle.theta + (speed * tan(steering_angle) * (time_duration) / vehicle_params.L)
        new_theta = wrap_between_0_and_2Pi(new_theta)
        new_x = vehicle.x + ((vehicle_params.L / tan(steering_angle)) * (sin(new_theta) - sin(vehicle.theta)))
        new_y = vehicle.y + ((vehicle_params.L / tan(steering_angle)) * (cos(vehicle.theta) - cos(new_theta)))
    end
    return Vehicle(new_x,new_y,new_theta,speed)
end

#=
Function for moving human in the environment
Returns a human_state struct object
=#
function propogate_human(human::human_state, world::experiment_environment, one_time_step::Float64, user_defined_rng::AbstractRNG)

    #Random noise in human's motion
    scaling_factor_for_noise = 0.5  #Change this vairable to decide the amount of noise to be introduced
    rand_num = (rand(user_defined_rng) - 0.5)*human.v*one_time_step*scaling_factor_for_noise

    #First Quadrant
    if(human.goal.x >= human.x && human.goal.y >= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y + (human.v)*one_time_step + rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x + (human.v)*one_time_step + rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x + ((human.v)*one_time_step + rand_num)*cos(heading_angle)
            new_y = human.y + ((human.v)*one_time_step + rand_num)*sin(heading_angle)
        end
    #Second Quadrant
    elseif(human.goal.x <= human.x && human.goal.y >= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y + (human.v)*one_time_step + rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x - (human.v)*one_time_step - rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x - ((human.v)*one_time_step + rand_num)*cos(heading_angle)
            new_y = human.y - ((human.v)*one_time_step + rand_num)*sin(heading_angle)
        end
    #Third Quadrant
    elseif(human.goal.x <= human.x && human.goal.y <= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y - (human.v)*one_time_step - rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x - (human.v)*one_time_step - rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x - ((human.v)*one_time_step + rand_num)*cos(heading_angle)
            new_y = human.y - ((human.v)*one_time_step + rand_num)*sin(heading_angle)
        end
    #Fourth Quadrant
    else(human.goal.x >= human.x && human.goal.y <= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y - (human.v)*one_time_step - rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x + (human.v)*one_time_step + rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x + ((human.v)*one_time_step + rand_num)*cos(heading_angle)
            new_y = human.y + ((human.v)*one_time_step + rand_num)*sin(heading_angle)
        end
    end

    new_x = clamp(new_x,0,world.length)
    new_y = clamp(new_y,0,world.breadth)
    #@show(new_x,new_y)
    new_human_state = human_state(new_x, new_y, human.v, human.goal)
    return new_human_state
end

#=
Functions to modify vehicle_params
Returns a vehicle_parameters struct object
=#
function modify_vehicle_params(params::es_vehicle_parameters)
    return es_vehicle_parameters(params.L,params.max_speed,params.goal)
end

function modify_vehicle_params(params::ls_vehicle_parameters)
    return ls_vehicle_parameters(params.L,params.max_speed,params.goal,params.hybrid_astar_path[2:end])
end

function get_lidar_data_and_ids(vehicle,humans,humans_params,lidar_range)
    lidar_data = Array{human_state,1}()
    ids = Array{Int64,1}()
    for i in 1:length(humans)
        human = humans[i]
        id = humans_params[i].id
        if(is_within_range(vehicle.x,vehicle.y,human.x,human.y,lidar_range))
            if(human.x!= human.goal.x || human.y!= human.goal.y)
                push!(lidar_data,human)
                push!(ids,id)
            end
        end
    end
    return lidar_data,ids
end

#=
Function to check if it is time to update the vehicle's belief
Return the updated belief if it is time, else create a copy of passed belief and return it.
=#
function get_belief(old_sensor_data, new_lidar_data, new_ids, human_goal_locations)
    new_belief = update_belief(old_sensor_data,new_lidar_data,new_ids,human_goal_locations)
    return new_belief
end

function get_vehicle_sensor_data(veh,humans,humans_params,old_sensor_data,exp_details,current_time_value)
    if( isinteger(current_time_value/exp_details.update_sensor_data_time_interval) )
        new_lidar_data, new_ids = get_lidar_data_and_ids(veh,humans,humans_params,exp_details.lidar_range)
        new_belief = get_belief(old_sensor_data, new_lidar_data, new_ids, exp_details.human_goal_locations)
        return vehicle_sensor(new_lidar_data,new_ids,new_belief)
    else
        return vehicle_sensor(copy(old_sensor_data.lidar_data),copy(old_sensor_data.ids),copy(old_sensor_data.belief))
    end
end

function get_nearby_humans(sim_obj,num_nearby_humans,min_safe_distance_from_human,cone_half_angle::Float64=pi/3.0)

    nearby_humans_position_data = Array{human_state,1}()
    nearby_humans_id = Array{Int64,1}()
    nearby_humans_belief = Array{belief_over_human_goals,1}()
    priority_queue_nearby_humans = PriorityQueue{Tuple{human_state,Int64,belief_over_human_goals},Float64}(Base.Order.Forward)
    humans = sim_obj.vehicle_sensor_data.lidar_data
    ids = sim_obj.vehicle_sensor_data.ids
    current_belief = sim_obj.vehicle_sensor_data.belief
    vehicle = sim_obj.vehicle

    for i in 1:length(humans)
        human = humans[i]
        human_id = ids[i]
        human_belief = current_belief[i]
        angle_between_vehicle_and_human = get_heading_angle(human.x, human.y, vehicle.x, vehicle.y)
        difference_in_angles = abs(vehicle.theta - angle_between_vehicle_and_human)
        euclidean_distance = sqrt( (vehicle.x - human.x)^2 + (vehicle.y - human.y)^2 )
        if(difference_in_angles <= cone_half_angle)
            priority_queue_nearby_humans[(human,human_id,human_belief)] = euclidean_distance
        elseif((2*pi - difference_in_angles) <= cone_half_angle)
            priority_queue_nearby_humans[(human,human_id,human_belief)] = euclidean_distance
        elseif(euclidean_distance<=min_safe_distance_from_human+sim_obj.vehicle_params.L)
            priority_queue_nearby_humans[(human,human_id,human_belief)] = euclidean_distance
        end
    end

    num_nearby_humans = min(num_nearby_humans, length(priority_queue_nearby_humans))
    for i in 1:num_nearby_humans
        human,id,belief = dequeue!(priority_queue_nearby_humans)
        push!(nearby_humans_position_data, human)
        push!(nearby_humans_id, id)
        push!(nearby_humans_belief, belief)
    end

    return nearby_humans(nearby_humans_position_data,nearby_humans_id,nearby_humans_belief)
end

#=
Function to check if it is time to stop the simulation
Returns true or false
=#
function stop_simulation!(sim_obj,curr_time,exp_details)
    #Check if the vehicle has reached the goal
    if(is_within_range(sim_obj.vehicle.x,sim_obj.vehicle.y,sim_obj.vehicle_params.goal.x,sim_obj.vehicle_params.goal.y,exp_details.radius_around_vehicle_goal))
        exp_details.vehicle_reached_goal = true
        return true
    end
    #Check if the vehicle is colliding with obstacles in the environment
    for obstacle in sim_obj.env.obstacles
        if(in_obstacle(sim_obj.vehicle.x,sim_obj.vehicle.y,obstacle,sim_obj.vehicle.L))
            exp.details.vehicle_ran_into_obstacle = true
            return true
        end
    end
    #Check if the vehicle is colliding with environment's boundary
    if(sim_obj.vehicle.x<0.0+sim_obj.vehicle_params.L || sim_obj.vehicle.y<0.0+sim_obj.vehicle_params.L ||
        sim_obj.vehicle.x>sim_obj.env.length-sim_obj.vehicle_params.L || sim_obj.vehicle.y>sim_obj.env.breadth-sim_obj.vehicle_params.L)
        exp_details.vehicle_ran_into_boundary_wall = true
        return true
    end
    if(curr_time >= exp_details.MAX_TIME_LIMIT)
        return true
    end
    return false
end

#=
Function to simulate vehicle and humans in the environment for given time duration.
Propogate humans for simulator's one time step.
Propogate vehicle for simulator's one time step.
Get new vehicle sensor data.
Create new struct object for vehicle params.
Create new struct object for the vehicle.
Create new struct object for the humans.
=#
function simulate_vehicle_and_humans!(sim::simulator, vehicle_steering_angle::Float64, vehicle_speed::Float64, current_time::Float64, time_duration::Float64, exp_details::experiment_details)

    current_sim_obj = sim
    number_steps_in_sim = Int64(time_duration/current_sim_obj.one_time_step)

    for i in 1:number_steps_in_sim
        CURRENT_TIME_VALUE = current_time + (i*current_sim_obj.one_time_step)
        propogated_humans = Array{human_state,1}(undef,exp_details.num_humans_env)
        for i in 1:exp_details.num_humans_env
            propogated_humans[i] = propogate_human(current_sim_obj.humans[i], current_sim_obj.env, current_sim_obj.one_time_step, exp_details.user_defined_rng)
        end
        #Get new vehicle object and new vehicle_params object
        new_veh_obj = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params, vehicle_steering_angle, vehicle_speed, current_sim_obj.one_time_step)
        new_vehicle_parameters = copy(current_sim_obj.vehicle_params)

        #Get new vehicle sensor data
        new_sensor_data_obj = get_vehicle_sensor_data(new_veh_obj,propogated_humans,current_sim_obj.humans_params,
                            current_sim_obj.vehicle_sensor_data,exp_details,CURRENT_TIME_VALUE)

        #Create a new simulator object
        new_sim_object = simulator(current_sim_obj.env, new_veh_obj, new_vehicle_parameters, new_sensor_data_obj, propogated_humans, current_sim_obj.humans_params, current_sim_obj.one_time_step)

        #Check if this is a risky scenario
        num_risks_this_time_step = get_num_risks(new_veh_obj, new_vehicle_parameters, new_sensor_data_obj.lidar_data, exp_details.max_risk_distance)
        if(num_risks_this_time_step!=0)
            exp_details.number_risky_scenarios += num_risks_this_time_step
            exp_details.risky_scenarios[CURRENT_TIME_VALUE] = new_sim_object
        end

        exp_details.sim_objects[CURRENT_TIME_VALUE] = new_sim_object
        exp_details.nearby_humans[CURRENT_TIME_VALUE] = copy(exp_details.nearby_humans[current_time])
        current_sim_obj = new_sim_object
    end
    return current_sim_obj
end


#Returns the updated belief over humans and number of risks encountered
function simulate_cart_and_pedestrians_and_generate_gif_environments(env_right_now, current_belief,
                                                            all_gif_environments, all_risky_scenarios, time_stamp,
                                                            num_humans_to_care_about_while_pomdp_planning, cone_half_angle,
                                                            lidar_range, closest_ped_dist_threshold, user_defined_rng,
                                                            time_interval, steering_angle)

    number_risks = 0
    env_before_humans_simulated_for_first_half_time_interval = deepcopy(env_right_now)
    num_steps_inside_simulator = 10
    time_step_inside_simulator = time_interval / num_steps_inside_simulator

    #Simulate for 0 to 0.5*time_interval
    initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    for i in 1:Int(num_steps_inside_simulator/2)
        extra_parameters = [env_right_now.cart.v, env_right_now.cart.L, steering_angle]
        x,y,theta = get_intermediate_points(initial_state, time_step_inside_simulator, extra_parameters);
        env_right_now.cart.x, env_right_now.cart.y, env_right_now.cart.theta = last(x), last(y), last(theta)
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,time_step_inside_simulator,user_defined_rng)
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            closest_ped_dist_threshold, cone_half_angle)

        dict_key = "t="*string(time_stamp)*"_"*string(i)
        all_gif_environments[dict_key] =  deepcopy(env_right_now)
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            all_risky_scenarios[dict_key] =  deepcopy(env_right_now)
        end
        initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    end
    #Update your belief after first half of the time_interval
    updated_belief = update_belief_from_old_world_and_new_world(current_belief,env_before_humans_simulated_for_first_half_time_interval, env_right_now)

    #Simulate for 0.5*time_interval to time_interval
    env_before_humans_and_cart_simulated_for_second_half_time_interval = deepcopy(env_right_now)
    initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    for i in Int(num_steps_inside_simulator/2)+1:num_steps_inside_simulator
        extra_parameters = [env_right_now.cart.v, env_right_now.cart.L, steering_angle]
        x,y,theta = get_intermediate_points(initial_state, time_step_inside_simulator, extra_parameters);
        env_right_now.cart.x, env_right_now.cart.y, env_right_now.cart.theta = last(x), last(y), last(theta)
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,time_step_inside_simulator,user_defined_rng)
        if(i==10)
            respawn_humans(env_right_now, user_defined_rng)
        end
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            closest_ped_dist_threshold, cone_half_angle)

        dict_key = "t="*string(time_stamp)*"_"*string(i)
        all_gif_environments[dict_key] =  deepcopy(env_right_now)
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            all_risky_scenarios[dict_key] =  deepcopy(env_right_now)
        end
        initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    end

    #Update your belief after the second half of time_interval
    final_updated_belief = update_belief_from_old_world_and_new_world(updated_belief,
                                                    env_before_humans_and_cart_simulated_for_second_half_time_interval, env_right_now)

    return final_updated_belief, number_risks
end

#Functions for simulating the cart and pedestrians for the 1D planner

#Returns the updated belief over humans and number of risks encountered
function hybrid_astar_1D_pomdp_simulate_pedestrians_and_generate_gif_environments_when_cart_stationary(env_right_now, current_belief,
                                                                        all_gif_environments, all_risky_scenarios, time_stamp,
                                                                        num_humans_to_care_about_while_pomdp_planning, cone_half_angle,
                                                                        lidar_range, closest_ped_dist_threshold, user_defined_rng)

    number_risks = 0
    env_before_humans_and_cart_simulated_for_first_half_second = deepcopy(env_right_now)

    #Simulate for 0 to 0.5 seconds
    for i in 1:5
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,0.1,user_defined_rng)
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            closest_ped_dist_threshold, cone_half_angle)
        dict_key = "t="*string(time_stamp)*"_"*string(i)
        all_gif_environments[dict_key] =  deepcopy(env_right_now)
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            all_risky_scenarios[dict_key] =  deepcopy(env_right_now)
        end
    end

    #Update your belief after first 0.5 seconds
    updated_belief = update_belief_from_old_world_and_new_world(current_belief,
                                                    env_before_humans_and_cart_simulated_for_first_half_second, env_right_now)

    #Simulate for 0.5 to 1 second
    env_before_humans_and_cart_simulated_for_second_half_second = deepcopy(env_right_now)
    for i in 6:10
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,0.1,user_defined_rng)
        if(i==10)
            respawn_humans(env_right_now, user_defined_rng)
        end
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            closest_ped_dist_threshold, cone_half_angle)
        dict_key = "t="*string(time_stamp)*"_"*string(i)
        all_gif_environments[dict_key] =  deepcopy(env_right_now)
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            all_risky_scenarios[dict_key] =  deepcopy(env_right_now)
        end
    end

    #Update your belief after second 0.5 seconds
    final_updated_belief = update_belief_from_old_world_and_new_world(updated_belief,
                                                    env_before_humans_and_cart_simulated_for_second_half_second, env_right_now)

    return final_updated_belief, number_risks
end

#Returns the updated belief over humans and number of risks encountered
function hybrid_astar_1D_pomdp_simulate_cart_and_pedestrians_and_generate_gif_environments_when_cart_moving(env_right_now, current_belief,
                                                            all_gif_environments, all_risky_scenarios, time_stamp,
                                                            num_humans_to_care_about_while_pomdp_planning, cone_half_angle,
                                                            lidar_range, closest_ped_dist_threshold, user_defined_rng)

    #First simulate only the cart and get its path
    goal_reached_in_this_time_step_flag = false
    if(env_right_now.cart.v > length(env_right_now.cart_hybrid_astar_path))
        steering_angles = env_right_now.cart_hybrid_astar_path
        goal_reached_in_this_time_step_flag = true
    else
        steering_angles = env_right_now.cart_hybrid_astar_path[1:Int(env_right_now.cart.v)]
    end
    cart_path_x = Float64[]; cart_path_y = Float64[]; cart_path_theta = Float64[]
    initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    for i in 1:length(steering_angles)
        steering_angle = steering_angles[i]
        extra_parameters = [env_right_now.cart.v, env_right_now.cart.L, steering_angle]
        x,y,theta = get_intermediate_points(initial_state, 1.0/env_right_now.cart.v, extra_parameters, 0.1/env_right_now.cart.v )
        append!(cart_path_x, x[2:end])
        append!(cart_path_y, y[2:end])
        append!(cart_path_theta, theta[2:end])
        initial_state = [last(cart_path_x),last(cart_path_y),last(cart_path_theta)]
    end

    number_risks = 0

    #Simulate for 0 to 0.5 seconds
    env_before_humans_and_cart_simulated_for_first_half_second = deepcopy(env_right_now)
    initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    curr_hybrid_astar_path_index = 0

    for i in 1:5
        cart_path_index = clamp(Int(i*env_right_now.cart.v),1,10*length(steering_angles))
        env_right_now.cart.x, env_right_now.cart.y, env_right_now.cart.theta = cart_path_x[cart_path_index], cart_path_y[cart_path_index], cart_path_theta[cart_path_index]
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,0.1,user_defined_rng)
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            closest_ped_dist_threshold, cone_half_angle)
        if( floor( (0.1*i) / (1/env_right_now.cart.v) ) > curr_hybrid_astar_path_index)
            curr_hybrid_astar_path_index += 1
            env_right_now.cart_hybrid_astar_path = env_right_now.cart_hybrid_astar_path[2 : end]
        end
        dict_key = "t="*string(time_stamp)*"_"*string(i)
        all_gif_environments[dict_key] =  deepcopy(env_right_now)
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            all_risky_scenarios[dict_key] =  deepcopy(env_right_now)
        end
        initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    end

    #Update your belief after first 0.5 seconds
    updated_belief = update_belief_from_old_world_and_new_world(current_belief,
                                                    env_before_humans_and_cart_simulated_for_first_half_second, env_right_now)

    #Simulate for 0.5 to 1 second
    env_before_humans_and_cart_simulated_for_second_half_second = deepcopy(env_right_now)
    initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    for i in 6:10
        cart_path_index = clamp(Int(i*env_right_now.cart.v),1,10*length(steering_angles))
        env_right_now.cart.x, env_right_now.cart.y, env_right_now.cart.theta = cart_path_x[cart_path_index], cart_path_y[cart_path_index], cart_path_theta[cart_path_index]
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,0.1,user_defined_rng)
        if(i==10)
            respawn_humans(env_right_now, user_defined_rng)
        end
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            closest_ped_dist_threshold, cone_half_angle)
        if( floor( (0.1*i) / (1/env_right_now.cart.v) ) > curr_hybrid_astar_path_index)
            curr_hybrid_astar_path_index += 1
            env_right_now.cart_hybrid_astar_path = env_right_now.cart_hybrid_astar_path[2 : end]
        end
        dict_key = "t="*string(time_stamp)*"_"*string(i)
        all_gif_environments[dict_key] =  deepcopy(env_right_now)
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            all_risky_scenarios[dict_key] =  deepcopy(env_right_now)
        end
        initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    end

    #Update your belief after second 0.5 seconds
    final_updated_belief = update_belief_from_old_world_and_new_world(updated_belief,
                                                    env_before_humans_and_cart_simulated_for_second_half_second, env_right_now)

    # if(goal_reached_in_this_time_step_flag)
    #     env_right_now.cart_hybrid_astar_path = []
    # else
    #     env_right_now.cart_hybrid_astar_path = env_right_now.cart_hybrid_astar_path[Int(env_right_now.cart.v)+1:end]
    # end
    return final_updated_belief, number_risks
end
