Base.copy(obj::es_vehicle_parameters) = es_vehicle_parameters(obj.L,obj.max_speed,obj.goal)
Base.copy(obj::ls_vehicle_parameters) = ls_vehicle_parameters(obj.L,obj.max_speed,obj.goal,obj.hybrid_astar_path)
Base.copy(obj::Tuple{Array{human_state,1},Array{Int64,1}}) = (copy(obj[1]),copy(obj[2]))
Base.copy(obj::nearby_humans) = nearby_humans(copy(obj.position_data),copy(obj.ids),copy(obj.belief))
Base.copy(obj::human_state) = human_state(obj.x,obj.y,obj.v,obj.goal)
Base.copy(obj::human_parameters) = human_parameters(obj.id)

struct simulator
    env::experiment_environment
    vehicle::Vehicle
    vehicle_params::Union{es_vehicle_parameters, ls_vehicle_parameters}
    vehicle_sensor_data::vehicle_sensor
    humans::Array{human_state,1}
    humans_params::Array{human_parameters,1}
    one_time_step::Float64
end

function in_obstacle(px,py,obstacle,padding=0.0)
    return is_within_range(px,py,obstacle.x,obstacle.y,obstacle.r+padding)
end

function in_safe_area(px,py,vehicle_x,vehicle_y,world)
    for obstacle in world.obstacles
        if(in_obstacle(px,py,obstacle))
            return false
        end
    end
    if(is_within_range(px,py,vehicle_x,vehicle_y,4.0))
        return false
    end
    return true
end

function get_human_goals(world)
    g1 = location(0.0, 0.0)
    g2 = location(0.0, world.breadth)
    g3 = location(world.length, world.breadth)
    g4 = location(world.length, 0.0)
    return [g1,g2,g3,g4]
end

function get_human_start_position(world, vehicle_start_location, user_defined_rng)
    human_x, human_y = rand(user_defined_rng,2).*(world.length,world.breadth)
    while(!in_safe_area(human_x,human_y,vehicle_start_location.x,vehicle_start_location.y,world))
        human_x, human_y = rand(user_defined_rng,2).*(world.length,world.breadth)
    end
    return (human_x,human_y)
end


function generate_humans(world,vehicle_start_location,human_start_velocity,all_goals_list,total_num_humans,user_defined_rng)
    humans = Array{human_state,1}()
    params = Array{human_parameters,1}()
    curr_id = 1
    for i in 1:total_num_humans
        human_start_x,human_start_y = get_human_start_position(world, vehicle_start_location, user_defined_rng)
        human_goal =  all_goals_list[Int(ceil(rand(user_defined_rng)*length(all_goals_list)))]
        human = human_state(human_start_x,human_start_y,human_start_velocity,human_goal)
        human_param = human_parameters(curr_id)
        push!(humans, human)
        push!(params, human_param)
        curr_id += 1
    end
    return humans, params
end
#=
unit_test_world = experiment_environment(100.0,100.0,[obstacle_location(30,30,15), obstacle_location(60,70,15)])
unit_test_vehicle_start_location = location(1.0,25.0)
unit_test_all_goals_list = [location(0.0,0.0),location(100.0,0.0),location(100.0,100.0),location(0.0,100.0)]
generate_humans(unit_test_world,unit_test_vehicle_start_location,1.0,unit_test_all_goals_list,100,MersenneTwister(11))
=#

function spawn_new_human(exp_details, start_vehicle, vehicle_params)

    println("************************Spawning new human************************")
    #=
    Choose which side to spawn human from
    Find x and y
    sample goal on the opposite edge
    initialize and return a new human
    =#

    start_vehicle = output.sim_objects[0.0].vehicle

    vehicle_goal = output.sim_objects[0.0].vehicle_params.goal
    vehicle_L = output.sim_objects[0.0].vehicle_params.L
    possible_sides = [ (0.0,Inf), (Inf,exp_details.env.breadth), (exp_details.env.length,Inf), (Inf,0.0) ]
    chosen_x, chosen_y = rand(exp_details.user_defined_rng, possible_sides)

    if(chosen_x!=Inf)
        chosen_y = rand(exp_details.user_defined_rng)*exp_details.env.breadth
        while( !in_safe_area(chosen_x,chosen_y,start_vehicle.x,start_vehicle.y,exp_details.env) ||
                is_within_range(chosen_x,chosen_y,vehicle_goal.x,vehicle_goal.y, exp_details.radius_around_vehicle_goal+vehicle_L))
            chosen_y = rand(exp_details.user_defined_rng)*exp_details.env.breadth
        end
        if(chosen_x == 0.0)
            human_goal_x = exp_details.env.length
            human_goal_y = rand(exp_details.user_defined_rng,[0.0, exp_details.env.breadth])
        else
            human_goal_x = 0.0
            human_goal_y = rand(exp_details.user_defined_rng,[0.0, exp_details.env.breadth])
        end
        new_human = human_state(chosen_x,chosen_y,exp_details.human_start_v,location(human_goal_x,human_goal_y))
        return new_human
    end

    if(chosen_y!=Inf)
        chosen_x = rand(exp_details.user_defined_rng)*exp_details.env.length
        while( !in_safe_area(chosen_x,chosen_y,start_vehicle.x,start_vehicle.y,exp_details.env) ||
                is_within_range(chosen_x,chosen_y,vehicle_goal.x,vehicle_goal.y, exp_details.radius_around_vehicle_goal+vehicle_L))
            chosen_x = rand(exp_details.user_defined_rng)*exp_details.env.breadth
        end
        if(chosen_y == 0.0)
            human_goal_y = exp_details.env.breadth
            human_goal_x = rand(exp_details.user_defined_rng,[0.0, exp_details.env.length])
        else
            human_goal_y = 0.0
            human_goal_x = rand(exp_details.user_defined_rng,[0.0, exp_details.env.length])
        end
        new_human = human_state(chosen_x,chosen_y,exp_details.human_start_v,location(human_goal_x,human_goal_y))
        return new_human
    end
end


function respawn_humans(existing_humans,existing_humans_params,exp_details, output)

    start_vehicle = output.sim_objects[0].vehicle
    vehicle_params = output.sim_objects[0].vehicle_params
    final_humans = Array{human_state,1}()
    final_humans_params = Array{human_parameters,1}()

    for i in 1:exp_details.num_humans_env
        h = existing_humans[i]
        h_params = existing_humans_params[i]
        if( (h.x == h.goal.x) && (h.y == h.goal.y) )
            continue
        else
            new_h = copy(h)
            new_h_params = copy(h_params)
            push!(final_humans, new_h)
            push!(final_humans_params, new_h_params)
        end
    end

    latest_id = existing_humans_params[end].id
    num_respawn_humans = exp_details.num_humans_env - length(final_humans)

    for i in 1:num_respawn_humans
        h = spawn_new_human(exp_details, start_vehicle, vehicle_params)
        h_params = human_parameters(latest_id+i)
        push!(final_humans, h)
        push!(final_humans_params, h_params)
    end

    return final_humans,final_humans_params
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

function is_divisible(a,b)
    return isapprox(round(a/b)*b, a)
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

function get_vehicle_sensor_data(veh,humans,humans_params,old_sensor_data,exp_details,current_time_value)
    if( is_divisible(current_time_value,exp_details.update_sensor_data_time_interval) )
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
function stop_simulation!(sim_obj,curr_time,exp_details,output)
    #Check if the vehicle has reached the goal
    if(is_within_range(sim_obj.vehicle.x,sim_obj.vehicle.y,sim_obj.vehicle_params.goal.x,sim_obj.vehicle_params.goal.y,exp_details.radius_around_vehicle_goal))
        output.vehicle_reached_goal = true
        println("Vehicle has reached the goal")
        return true
    end
    #Check if the vehicle is colliding with obstacles in the environment
    for obstacle in sim_obj.env.obstacles
        if(in_obstacle(sim_obj.vehicle.x,sim_obj.vehicle.y,obstacle,sim_obj.vehicle.L))
            output.vehicle_ran_into_obstacle = true
            println("Vehicle collided with a static obstacle")
            return true
        end
    end
    #Check if the vehicle is colliding with environment's boundary
    if(sim_obj.vehicle.x<0.0+sim_obj.vehicle_params.L || sim_obj.vehicle.y<0.0+sim_obj.vehicle_params.L ||
        sim_obj.vehicle.x>sim_obj.env.length-sim_obj.vehicle_params.L || sim_obj.vehicle.y>sim_obj.env.breadth-sim_obj.vehicle_params.L)
        output.vehicle_ran_into_boundary_wall = true
        println("Vehicle collided with environment boundary")
        return true
    end
    if(curr_time >= exp_details.MAX_TIME_LIMIT)
        println("Vehicle didn't reach the goal in givem time limit")
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
function simulate_vehicle_and_humans!(sim::simulator, vehicle_steering_angle::Float64, vehicle_speed::Float64, current_time::Float64, time_duration::Float64,
                            exp_details::experiment_details, output::Output)

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
        new_humans,new_humans_params = propogated_humans,copy(current_sim_obj.humans_params)
        # new_humans, new_humans_params = respawn_humans(propogated_humans,current_sim_obj.humans_params,exp_details,output)

        #Get new vehicle sensor data
        new_sensor_data_obj = get_vehicle_sensor_data(new_veh_obj,new_humans,new_humans_params,current_sim_obj.vehicle_sensor_data,exp_details,CURRENT_TIME_VALUE)

        #Create a new simulator object
        new_sim_object = simulator(current_sim_obj.env, new_veh_obj, new_vehicle_parameters, new_sensor_data_obj, new_humans, new_humans_params, current_sim_obj.one_time_step)

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

function run_experiment!(current_sim_obj, planner, exp_details, pomdp_details, output)

    current_time_value = 0.0
    current_vehicle_speed = 0.0
    current_vehicle_steering_angle = 0.0
    current_action = action_extended_space_POMDP_planner(0.0,0.0)
    output.vehicle_actions[current_time_value] = current_action
    output.sim_objects[current_time_value] = current_sim_obj
    output.nearby_humans[current_time_value] = nearby_humans(human_state[], Int64[], belief_over_human_goals[])

    # try
    while(!stop_simulation!(current_sim_obj,current_time_value,exp_details,output))

        println("Current time value : ", current_time_value)
        println("Current Vehicle State: ", current_sim_obj.vehicle)
        println("Action to be executed now : ", current_action)
        #=
        Simulate for (one_time_step - pomdp_planning_time - buffer_time) seconds
        =#
        time_duration_until_planning_begins = exp_details.one_time_step - (pomdp_details.planning_time + exp_details.buffer_time)
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                current_time_value, time_duration_until_planning_begins, exp_details, output)
        current_time_value += time_duration_until_planning_begins

        #=
        Take the current status of the environment.
        Predict the vehicle's future position using current_action after (pomdp_planning_time + buffer_time) seconds.
        Initialize scenarios with sampled human goals and predicted human positions after (pomdp_planning_time + buffer_time) seconds.
        Solve the POMDP and find the action.
        =#
        time_duration_until_pomdp_action_determined = pomdp_details.planning_time + exp_details.buffer_time
        predicted_vehicle_state = propogate_vehicle(current_sim_obj.vehicle, current_sim_obj.vehicle_params,
                                            current_vehicle_steering_angle, current_vehicle_speed, time_duration_until_pomdp_action_determined)
        # println(predicted_vehicle_state)
        modified_vehicle_params = modify_vehicle_params(current_sim_obj.vehicle_params)
        nearby_humans = get_nearby_humans(current_sim_obj,pomdp_details.num_nearby_humans,pomdp_details.min_safe_distance_from_human,
                                                pomdp_details.cone_half_angle)
        b = tree_search_scenario_parameters(predicted_vehicle_state.x,predicted_vehicle_state.y,predicted_vehicle_state.theta,predicted_vehicle_state.v,
                            modified_vehicle_params, exp_details.human_goal_locations, nearby_humans.position_data, nearby_humans.belief,
                            current_sim_obj.env.length,current_sim_obj.env.breadth,time_duration_until_pomdp_action_determined)
        next_action, info = action_info(planner, b)
        output.despot_trees[current_time_value] = info
        output.nearby_humans[current_time_value] = nearby_humans
        output.b_root[current_time_value] = b

        #=
        Simulate for (pomdp_planning_time + buffer_time) seconds
        =#
        time_duration_until_next_action_is_applied =  pomdp_details.planning_time + exp_details.buffer_time
        current_sim_obj = simulate_vehicle_and_humans!(current_sim_obj, current_vehicle_steering_angle, current_vehicle_speed,
                                                current_time_value, time_duration_until_next_action_is_applied, exp_details, output)

        current_vehicle_speed = clamp(current_vehicle_speed+next_action.delta_speed, 0.0, pomdp_details.max_vehicle_speed)
        current_vehicle_steering_angle = get_steering_angle(current_sim_obj.vehicle_params.L, next_action.delta_heading_angle, current_vehicle_speed, exp_details.one_time_step)
        current_action = next_action
        current_time_value += time_duration_until_next_action_is_applied
        #=
        Store relevant values in exp_details
        =#
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
