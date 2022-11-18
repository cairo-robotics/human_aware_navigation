include("simulator_utils.jl")


struct Simulator{P}
    env::ExperimentEnvironment
    vehicle::Vehicle
    vehicle_params::P
    vehicle_sensor_data::VehicleSensor
    humans::Array{HumanState,1}
    humans_params::Array{HumanParameters,1}
    one_time_step::Float64
end


#=
Function for moving vehicle in the environment
Returns a vehicle struct object
=#
function propogate_vehicle(vehicle::Vehicle, vehicle_params::VehicleParametersESPlanner, steering_angle::Float64, speed::Float64, time_duration::Float64)
    new_x,new_y,new_theta =  move_vehicle(vehicle.x,vehicle.y,vehicle.theta,vehicle_params.wheelbase,steering_angle,speed,time_duration)
    return Vehicle(new_x,new_y,new_theta,speed)
end

#=
Function for moving human in the environment
Returns a human_state struct object
=#
function propogate_human(human::HumanState,human_params::HumanParameters)
    # modified_human_state = get_next_human_state()
    modified_human_state = human_params.path[human_params.path_index+1]
    return modified_human_state
end

#=
Functions to modify vehicle_params
Returns a vehicle_parameters struct object
=#
function modify_vehicle_params(params::VehicleParametersESPlanner)
    return VehicleParametersESPlanner(params.wheelbase,params.length,params.breadth,params.dist_origin_to_center,
                params.radius,params.max_speed,params.max_steering_angle,params.goal)
end

function modify_vehicle_params(params::VehicleParametersLSPlanner)
    return VehicleParametersLSPlanner(params.L,params.max_speed,params.goal,params.hybrid_astar_path[2:end])
end


#=
Function to check if it is time to stop the simulation
Returns true or false
=#
function stop_simulation!(sim_obj,curr_time,exp_details,output)
    #Check if the vehicle has reached the goal
    vehicle_center_x = sim_obj.vehicle.x + sim_obj.vehicle_params.dist_origin_to_center*cos(sim_obj.vehicle.theta)
    vehicle_center_y = sim_obj.vehicle.y + sim_obj.vehicle_params.dist_origin_to_center*sin(sim_obj.vehicle.theta)

    if(is_within_range(vehicle_center_x,vehicle_center_y,sim_obj.vehicle_params.goal.x,sim_obj.vehicle_params.goal.y,exp_details.radius_around_vehicle_goal))
        output.vehicle_reached_goal = true
        println("Vehicle has reached the goal")
        return true
    end
    #Check if the vehicle is colliding with obstacles in the environment
    for obstacle in sim_obj.env.obstacles
        if(in_obstacle(vehicle_center_x,vehicle_center_y,obstacle,sim_obj.vehicle_params.radius))
            output.vehicle_ran_into_obstacle = true
            println("Vehicle collided with a static obstacle")
            return true
        end
    end
    #Check if the vehicle is colliding with environment's boundary
    if(vehicle_center_x<0.0+sim_obj.vehicle_params.radius || vehicle_center_y<0.0+sim_obj.vehicle_params.radius ||
        vehicle_center_x>sim_obj.env.length-sim_obj.vehicle_params.radius || vehicle_center_y>sim_obj.env.breadth-sim_obj.vehicle_params.radius)
        output.vehicle_ran_into_boundary_wall = true
        println("Vehicle collided with environment boundary")
        return true
    end
    if(curr_time >= exp_details.MAX_TIME_LIMIT)
        println("Vehicle didn't reach the goal in given time limit")
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
function simulate_vehicle_and_humans!(sim::Simulator, vehicle_steering_angle::Float64, vehicle_speed::Float64, current_time::Float64, time_duration::Float64,
                            exp_details::ExperimentDetails, output::Output)

    current_sim_obj = sim
    number_steps_in_sim = Int64(time_duration/current_sim_obj.one_time_step)

    for i in 1:number_steps_in_sim
        CURRENT_TIME_VALUE = current_time + (i*current_sim_obj.one_time_step)
        propogated_humans = Array{HumanState,1}(undef,exp_details.num_humans_env)
        for i in 1:exp_details.num_humans_env
            # propogated_humans[i] = propogate_human(current_sim_obj.humans[i], current_sim_obj.env, current_sim_obj.one_time_step, exp_details.user_defined_rng)
            propogated_humans[i] = propogate_human(current_sim_obj.humans[i], current_sim_obj.humans_params[i])
            # current_sim_obj.humans_params[i].path_index = clamp(current_sim_obj.humans_params[i].path_index+1,1,length(current_sim_obj.humans_params[i].path))
            current_sim_obj.humans_params[i].path_index += 1
        end
        #Get new vehicle object and new vehicle_params object
        new_veh_obj = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params, vehicle_steering_angle, vehicle_speed, current_sim_obj.one_time_step)
        new_vehicle_parameters = modify_vehicle_params(current_sim_obj.vehicle_params)
        # new_humans,new_humans_params = propogated_humans,copy(current_sim_obj.humans_params)
        new_humans, new_humans_params = respawn_humans(propogated_humans,current_sim_obj.humans_params,new_veh_obj,new_vehicle_parameters,exp_details)

        #Get new vehicle sensor data
        new_sensor_data_obj = get_vehicle_sensor_data(new_veh_obj,new_humans,new_humans_params,current_sim_obj.vehicle_sensor_data,exp_details,CURRENT_TIME_VALUE)

        #Create a new simulator object
        new_sim_object = Simulator(current_sim_obj.env, new_veh_obj, new_vehicle_parameters, new_sensor_data_obj, new_humans, new_humans_params, current_sim_obj.one_time_step)

        #Check if this is a risky scenario
        num_risks_this_time_step = get_num_risks(new_veh_obj, new_vehicle_parameters, new_sensor_data_obj.lidar_data, exp_details.max_risk_distance)
        if(num_risks_this_time_step!=0)
            output.number_risky_scenarios += num_risks_this_time_step
            output.risky_scenarios[CURRENT_TIME_VALUE] = new_sim_object
        end

        output.sim_objects[CURRENT_TIME_VALUE] = new_sim_object
        output.nearby_humans[CURRENT_TIME_VALUE] = copy(output.nearby_humans[current_time])
        current_sim_obj = new_sim_object
    end
    return current_sim_obj
end

function run_experiment!(current_sim_obj, planner, exp_details, pomdp_details, output, output2)
    # initialize variables
    current_time_value = 0.0
    current_vehicle_speed = 0.0
    current_vehicle_steering_angle = 0.0
    current_action = ActionExtendedSpacePOMDP(0.0,0.0)
    output.vehicle_actions[current_time_value] = current_action
    output.sim_objects[current_time_value] = current_sim_obj
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    output.nearby_humans[current_time_value] = nbh
    next_action = nothing
    info = nothing
    predicted_vehicle_pos_in_goal = false
    debug = false
    start_time = time()

    # run simulation
    while(!stop_simulation!(current_sim_obj,current_time_value,exp_details,output))
        # print current execution step
        state_array = [current_sim_obj.vehicle.x, current_sim_obj.vehicle.y, current_sim_obj.vehicle.theta, current_sim_obj.vehicle.v]
        action_array = [current_action.steering_angle, current_action.delta_speed]
        println("\nTime_k = ", current_time_value,
              "\t State_k = ", round.(state_array, digits=3),
              "\t Action_k = ", round.(action_array, digits=3))

        # simulate for (one_time_step - pomdp_planning_time - buffer_time) seconds
        if(debug)
            println("Simulating for one_time_step - pomdp_planning_time - buffer_time seconds : " , exp_details.one_time_step - (pomdp_details.planning_time + exp_details.buffer_time))
            start_time = time()
        end

        time_duration_until_planning_begins = exp_details.one_time_step - (pomdp_details.planning_time + exp_details.buffer_time)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                current_time_value, time_duration_until_planning_begins, exp_details, output)
        current_time_value += time_duration_until_planning_begins

        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end

        #=
        Take the current status of the environment.
        Predict the vehicle's future position using current_action after (pomdp_planning_time + buffer_time) seconds.
        Initialize scenarios with sampled human goals and predicted human positions after (pomdp_planning_time + buffer_time) seconds.
        Solve the POMDP and find the action.
        =#
        time_duration_until_pomdp_action_determined = pomdp_details.planning_time + exp_details.buffer_time
        predicted_vehicle_state = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params,
                                            current_vehicle_steering_angle, current_vehicle_speed, time_duration_until_pomdp_action_determined)
        modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params)

        # ISSUE: not applying POMDP action to sim when shielding is turned off
        #   - still calculates an action on line 219, it just isn't applied to the sim
        #   - is gen function being run?
        #       - think simulate_vehicle_and_humans!() is the function that propagates everything
        #       - looks like it's called on line 166 each loop, so seems like vehicle should be propagating fine

        # check if vehicle in goal
        if(is_within_range(predicted_vehicle_state.x,predicted_vehicle_state.y,modified_vehicle_params.goal.x,modified_vehicle_params.goal.y,exp_details.radius_around_vehicle_goal))
            println("Predicted vehicle state is in goal")
            next_pomdp_action = ActionExtendedSpacePOMDP(0.0,0.0)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = nothing
            output.despot_trees[current_time_value] = nothing
            predicted_vehicle_pos_in_goal = true
        
        # if not, take observation and calculate next action
        else
            if(debug)
                println("Starting POMDP planning")
                start_time = time()
            end

            nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,pomdp_details.min_safe_distance_from_human,
                                                    pomdp_details.cone_half_angle)
            b = TreeSearchScenarioParameters(predicted_vehicle_state.x,predicted_vehicle_state.y,predicted_vehicle_state.theta,predicted_vehicle_state.v,
                                modified_vehicle_params, exp_details.human_goal_locations, length(nbh.position_data), nbh.position_data, nbh.belief,
                                current_sim_obj.env.length,current_sim_obj.env.breadth,time_duration_until_pomdp_action_determined)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = b

            # run DESPOT to calculate action for t_k1
            solver_t0 = time()

            next_pomdp_action, info = action_info(planner, b)

            solver_runtime = time() - solver_t0
            println("solver_runtime = $solver_runtime sec")
            push!(output2.solver_runtimes, solver_runtime)

            if(debug)
                println("Finished POMDP planning. Action selected")
                time_taken = time() - start_time
                println("Time Taken : ", time_taken)
            end

            output.despot_trees[current_time_value] = info
        end

        # simulate for (pomdp_planning_time + buffer_time) seconds
        if(debug)
            println("Simulating for pomdp_planning_time + buffer_time seconds : " , pomdp_details.planning_time + exp_details.buffer_time)
            start_time = time()
        end

        time_duration_until_next_action_is_applied = pomdp_details.planning_time + exp_details.buffer_time
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                    current_time_value, time_duration_until_next_action_is_applied, exp_details, output)

        # SHIELDING ---
        run_shield = false
        output2.shield_enabled = run_shield

        if run_shield == true
            println("POMDP requested action = ", [next_pomdp_action.steering_angle, next_pomdp_action.delta_speed])

            # observes environment at t_k1
            # current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
            #                                         current_time_value, time_duration_until_next_action_is_applied, exp_details, output)
            shielding_nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,pomdp_details.min_safe_distance_from_human,
                                                    pomdp_details.cone_half_angle)
            Dt_obs_to_k1 = 0.0
            
            # checks if vehicle is in the goal
            if predicted_vehicle_pos_in_goal == true
                next_action = next_pomdp_action
                push!(output2.shield_hist, false)

            # checks if tree is empty (root node took default action)
            elseif length(info[:tree].children[1]) == 0
                next_action = next_pomdp_action
                push!(output2.shield_hist, false)

            # else, runs shield and returns best safe action
            else
                next_action, shield_intervened = get_best_shielded_action(predicted_vehicle_state, shielding_nbh.position_data, Dt_obs_to_k1, exp_details.one_time_step,
                    shield_get_actions, veh_body, exp_details.human_goal_locations, planner.pomdp, info[:tree], exp_details.user_defined_rng)
                
                if shield_intervened == true
                    output2.number_shield_interventions += 1
                    push!(output2.shield_hist, true)
                else
                    push!(output2.shield_hist, false)
                end
            end
        else
            next_action = next_pomdp_action
            push!(output2.shield_hist, false)
        end
        # ---

        # pass items to next time step
        current_vehicle_speed = clamp((current_vehicle_speed + next_action.delta_speed), 0.0, pomdp_details.max_vehicle_speed)
        current_vehicle_steering_angle = next_action.steering_angle
        current_action = next_action
        current_time_value += time_duration_until_next_action_is_applied

        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end

        # store relevant values in exp_details
        output.vehicle_actions[current_time_value] = current_action
        if(current_action.delta_speed == -10.0)
            output.number_sudden_stops += 1
        end
    end

    # catch e
    #     println("\n Things failed during the simulation. \n The error message is : \n ")
    #     println(e)
    #     return exp_details
    # end
    output.time_taken = current_time_value
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
