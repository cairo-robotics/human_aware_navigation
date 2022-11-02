Base.copy(obj::VehicleParametersESPlanner) = VehicleParametersESPlanner(obj.L,obj.max_speed,obj.goal)
Base.copy(obj::VehicleParametersLSPlanner) = VehicleParametersLSPlanner(obj.L,obj.max_speed,obj.goal,obj.hybrid_astar_path)
Base.copy(obj::Tuple{Array{HumanState,1},Array{Int64,1}}) = (copy(obj[1]),copy(obj[2]))
Base.copy(obj::NearbyHumans) = NearbyHumans(copy(obj.position_data),copy(obj.ids),copy(obj.belief))
Base.copy(obj::HumanState) = HumanState(obj.x,obj.y,obj.v,obj.goal)
Base.copy(obj::HumanParameters) = HumanParameters(obj.id,obj.path,obj.path_index)

function is_divisible(a,b)
    return isapprox(round(a/b)*b, a)
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
    g1 = Location(0.0, 0.0)
    g2 = Location(0.0, world.breadth)
    g3 = Location(world.length, world.breadth)
    g4 = Location(world.length, 0.0)
    return [g1,g2,g3,g4]
end

function get_human_start_position(world, vehicle_start_location, user_defined_rng)
    human_x, human_y = rand(user_defined_rng,2).*(world.length,world.breadth)
    while(!in_safe_area(human_x,human_y,vehicle_start_location.x,vehicle_start_location.y,world))
        human_x, human_y = rand(user_defined_rng,2).*(world.length,world.breadth)
    end
    return (human_x,human_y)
end

function get_human_path(human, world, sim_one_time_step, user_defined_rng)
    path = HumanState[human]
    scaling_factor_for_noise = 0.5  #Change this vairable to decide the amount of noise to be introduced
    while(human.x!= human.goal.x && human.y!=human.goal.y)
        noise = (rand(user_defined_rng) - 0.5)*human.v*sim_one_time_step*scaling_factor_for_noise
        modified_human_state = update_human_position(human,world.length,world.breadth,sim_one_time_step,noise)
        push!(path,modified_human_state)
        human = modified_human_state
    end
    return path
end

function generate_humans(world,vehicle_start_location,human_start_velocity,all_goals_list,total_num_humans,sim_one_time_step,user_defined_rng)
    humans = Array{HumanState,1}()
    params = Array{HumanParameters,1}()
    curr_id = 1
    for i in 1:total_num_humans
        human_start_x,human_start_y = get_human_start_position(world, vehicle_start_location, user_defined_rng)
        human_goal =  all_goals_list[Int(ceil(rand(user_defined_rng)*length(all_goals_list)))]
        human = HumanState(human_start_x,human_start_y,human_start_velocity,human_goal)
        human_path = get_human_path(human, world, sim_one_time_step, user_defined_rng)
        human_param = HumanParameters(curr_id,human_path,1)
        push!(humans, human)
        push!(params, human_param)
        curr_id += 1
    end
    return humans, params
end
#=
unit_test_world = ExperimentEnvironment(100.0,100.0,[ObstacleLocation(30,30,15), ObstacleLocation(60,70,15)])
unit_test_vehicle_start_location = Location(1.0,25.0)
unit_test_all_goals_list = get_human_goals(unit_test_world)
h,p = generate_humans(unit_test_world,unit_test_vehicle_start_location,1.0,unit_test_all_goals_list,100,1.0,MersenneTwister(11))
=#

function spawn_new_human(exp_details, vehicle, vehicle_params)

    # println("************************Spawning new human************************")
    #=
    Choose which side to spawn human from
    Find x and y
    sample goal on the opposite edge
    initialize and return a new human
    =#

    vehicle_goal = vehicle_params.goal
    vehicle_L = vehicle_params.L
    possible_sides = [ (0.0,Inf), (Inf,exp_details.env.breadth), (exp_details.env.length,Inf), (Inf,0.0) ]
    chosen_x, chosen_y = rand(exp_details.user_defined_rng, possible_sides)

    if(chosen_x!=Inf)
        chosen_y = rand(exp_details.user_defined_rng)*exp_details.env.breadth
        while( !in_safe_area(chosen_x,chosen_y,vehicle.x,vehicle.y,exp_details.env) ||
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
        new_human = HumanState(chosen_x,chosen_y,exp_details.human_start_v,Location(human_goal_x,human_goal_y))
        return new_human
    end

    if(chosen_y!=Inf)
        chosen_x = rand(exp_details.user_defined_rng)*exp_details.env.length
        while( !in_safe_area(chosen_x,chosen_y,vehicle.x,vehicle.y,exp_details.env) ||
                is_within_range(chosen_x,chosen_y,vehicle_goal.x,vehicle_goal.y, exp_details.radius_around_vehicle_goal+vehicle_L))
            chosen_x = rand(exp_details.user_defined_rng)*exp_details.env.length
        end
        if(chosen_y == 0.0)
            human_goal_y = exp_details.env.breadth
            human_goal_x = rand(exp_details.user_defined_rng,[0.0, exp_details.env.length])
        else
            human_goal_y = 0.0
            human_goal_x = rand(exp_details.user_defined_rng,[0.0, exp_details.env.length])
        end
        new_human = HumanState(chosen_x,chosen_y,exp_details.human_start_v,Location(human_goal_x,human_goal_y))
        return new_human
    end
end

function respawn_humans(existing_humans,existing_humans_params,current_vehicle,current_vehicle_params,exp_details)

    final_humans = Array{HumanState,1}()
    final_humans_params = Array{HumanParameters,1}()

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
        h = spawn_new_human(exp_details, current_vehicle, current_vehicle_params)
        h_path = get_human_path(h, exp_details.env, exp_details.simulator_time_step, exp_details.user_defined_rng)
        # println(latest_id)
        h_params = HumanParameters(latest_id+i, h_path, 1)
        push!(final_humans, h)
        push!(final_humans_params, h_params)
        # println(h_params)
    end

    return final_humans,final_humans_params
end



function get_lidar_data_and_ids(vehicle,humans,humans_params,lidar_range)
    lidar_data = Array{HumanState,1}()
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
