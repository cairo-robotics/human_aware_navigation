move_vehicle(vehicle_x,vehicle_y,vehicle_theta,vehicle_L,steering_angle,vehicle_speed,time_duration)

propogate_vehicle(vehicle::Vehicle, vehicle_params::VehicleParametersESPlanner, steering_angle::Float64, speed::Float64, time_duration::Float64)



function propogate_vehicle(vehicle, vehicle_params, vehicle_speed, time_duration, path_planning_details, exp_details)

    num_steps_on_vehicle_path = Int64((vehicle_speed * time_duration)/(veh_path_planning_v*time_path_planning))
    time_between_steering_angle_changes = time_duration/num_steps_on_vehicle_path
    num_sim_steps = Int64(time_duration/sim_one_time_step)
    simulator_time_duration = time_until_steering_angle_changes/num_sim_steps
    complete_vehicle_path = Vehicle[]
    final_vehicle_path = Vehicle[vehicle]
    current_x,current_y,current_theta = vehicle.x,vehicle.y,vehicle.theta

    for i in 1:num_steps_on_vehicle_path
        steering_angle = path[index+i]
        for j in 1:num_sim_steps
            new_x,new_y,new_theta = move_vehicle(current_x,current_y,current_theta,vehicle_params.wheelbase,steering_angle,vehicle_speed,simulator_time_duration)
            push!(vehicle_path, Vehicle(new_x,new_y,new_theta,vehicle_speed))
            current_x,current_y,current_theta = new_x,new_y,new_theta
        end
    end

    i = num_steps_on_vehicle_path
    while(index<length(complete_vehicle_path))
        push!(final_vehicle_path, complete_vehicle_path[index])
        index += num_steps_on_vehicle_path
    end

    return final_vehicle_path
end



function run_experiment!(current_sim_obj, planner, exp_details, path_planning_details, pomdp_details, output)
    # initialize variables
    current_time_value = 0.0
    current_vehicle_speed = 0.0
    current_action = ActionLimitedSpacePOMDP(0.0)
    output.vehicle_actions[current_time_value] = current_action
    output.sim_objects[current_time_value] = current_sim_obj
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    output.nearby_humans[current_time_value] = nbh
    next_action = nothing
    info = nothing
    predicted_vehicle_pos_in_goal = false
    debug = false
    run_shield = true
    start_time = time()

    #=
    Psuedocode
        while(!stop_condition)
            1) Print vehicle state, action and current time step
            2) Find vehicle trajectory using this action for one_time_step seconds
            3) Simulate humans for (one_time_step - pomdp_planning_time - buffer_time - path_planning_time) seconds
            4) a) Observe current human positions and their belief
               b) Predict future vehicle position
               c) Generate hybrid A* path
               d) Initialize scenarios
               e) Feed scenarios and hybrid A* path to the LS planner to get vehicle action
            5) Simulate humans for (pomdp_planning_time + buffer_time + path_planning_time) seconds
            6) Run shielding
            7) Set parameters for next cycle and store/modify required variables
        end
    =#


    # Run simulation
    while(!stop_simulation!(current_sim_obj,current_time_value,exp_details,output))
        state_array = [current_sim_obj.vehicle.x, current_sim_obj.vehicle.y, current_sim_obj.vehicle.theta, current_sim_obj.vehicle.v]
        action_array = [current_action.steering_angle, current_action.delta_speed]
        println("\nTime_k = ", current_time_value,
              "\t State_k = ", round.(state_array, digits=3),
              "\t Action_k = ", round.(action_array, digits=3))

        # Generate vehicle trajectory using current action
        vehicle_trajectory = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params, current_vehicle_speed, exp_details.one_time_step, path_planning_details, exp_details)

        # Simulate for (one_time_step - pomdp_planning_time - buffer_time) seconds
        if(debug)
            println("Simulating for one_time_step - path_planning_time - pomdp_planning_time - buffer_time seconds : " ,
                                exp_details.one_time_step - (path_planning_details.planning_time + pomdp_details.planning_time + exp_details.buffer_time))
            start_time = time()
        end
        time_duration_until_planning_begins = exp_details.one_time_step - (path_planning_details.planning_time + pomdp_details.planning_time + exp_details.buffer_time)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                current_time_value, time_duration_until_planning_begins, exp_details, output)
        current_time_value += time_duration_until_planning_begins
        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end

        #=
        Planning for the next cycle
        =#
        time_duration_until_pomdp_action_determined = pomdp_details.planning_time + exp_details.buffer_time
        predicted_vehicle_state = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params,
                                            current_vehicle_steering_angle, current_vehicle_speed, time_duration_until_pomdp_action_determined)
        modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params)

        # Check if the predicted vehicle position is in the goal region
        if(is_within_range(predicted_vehicle_state.x,predicted_vehicle_state.y,modified_vehicle_params.goal.x,modified_vehicle_params.goal.y,exp_details.radius_around_vehicle_goal))
            println("Predicted vehicle state is in goal")
            next_pomdp_action = ActionExtendedSpacePOMDP(0.0,0.0)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = nothing
            output.despot_trees[current_time_value] = nothing
            predicted_vehicle_pos_in_goal = true
        # If it is not, then take observation from the environment and plan for the next action
        else
            if(debug)
                println("Starting Hybrid A* path planning")
                start_time = time()
            end
            nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,pomdp_details.min_safe_distance_from_human,
                                                    pomdp_details.cone_half_angle)
            vehicle_steering_controls = hybrid_astar_search(current_sim_obj.env, predicted_vehicle_state, vehicle_params, vehicle_actions, nbh, path_planning_details)
            if(length(vehicle_steering_controls) == 0)
                println("Hybrid A* path not found within the given time limit. Reusing old path")
                #Modify path index or the steering_controls_sequence itself in vehicle_params
            else
                #Modify vehicle params here
            end
            if(debug)
                println("Finished Hybrid A* path planning.")
                time_taken = time() - start_time
                println("Time Taken : ", time_taken)
            end
            if(debug)
                println("Starting POMDP planning")
                start_time = time()
            end
            b = TreeSearchScenarioParameters(predicted_vehicle_state.x,predicted_vehicle_state.y,predicted_vehicle_state.theta,predicted_vehicle_state.v,
                                modified_vehicle_params, exp_details.human_goal_locations, length(nbh.position_data), nbh.position_data, nbh.belief,
                                current_sim_obj.env.length,current_sim_obj.env.breadth,time_duration_until_pomdp_action_determined)
            output.nearby_humans[current_time_value] = nbh
            output.b_root[current_time_value] = b

            # Run DESPOT to calculate next action
            next_pomdp_action, info = action_info(planner, b)

            if(debug)
                println("Finished POMDP planning. Action selected")
                time_taken = time() - start_time
                println("Time Taken : ", time_taken)
            end
            output.despot_trees[current_time_value] = info
        end

        if(debug)
            println("Simulating for path_planning_time + pomdp_planning_time + buffer_time seconds : " , path_planning_details.planning_time + pomdp_details.planning_time + exp_details.buffer_time)
            start_time = time()
        end
        # Simulate for (pomdp_planning_time + buffer_time) seconds
        time_duration_until_next_action_is_applied = path_planning_details.planning_time + pomdp_details.planning_time + exp_details.buffer_time
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                    current_time_value, time_duration_until_next_action_is_applied, exp_details, output)

        # Shielding
        if(run_shield)
            println("POMDP requested action = ", [next_pomdp_action.steering_angle, next_pomdp_action.delta_speed])
            shielding_nbh = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,pomdp_details.min_safe_distance_from_human,
                                                    pomdp_details.cone_half_angle)
            Dt_obs_to_k1 = 0.0
            # Check if the predicted vehicle position is in the goal region
            if(predicted_vehicle_pos_in_goal)
                next_action = next_pomdp_action
            # checks if tree is empty (root node took default action)
            elseif length(info[:tree].children[1]) == 0
                next_action = next_pomdp_action
            # else, runs shield and returns best safe action
            else
                next_action = get_best_shielded_action(predicted_vehicle_state, shielding_nbh.position_data, Dt_obs_to_k1, exp_details.one_time_step,
                    shield_get_actions, veh_body, exp_details.human_goal_locations, planner.pomdp, info[:tree], exp_details.user_defined_rng)
            end
        else
            next_action = next_pomdp_action
        end

        if(debug)
            println("Finished simulation")
            time_taken = time() - start_time
            println("Time Taken : ", time_taken)
        end
        
        # Set parameters for the next cycle
        current_vehicle_speed = clamp((current_vehicle_speed + next_action.delta_speed), 0.0, pomdp_details.max_vehicle_speed)
        current_action = next_action
        current_time_value += time_duration_until_next_action_is_applied

        # Store relevant values in exp_details
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
