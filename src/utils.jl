#Various different miscellaneous functions that are needed by different components and are common to multiple files

function is_within_range(p1_x,p1_y, p2_x, p2_y, threshold_distance)
    euclidean_distance = ((p1_x - p2_x)^2 + (p1_y - p2_y)^2)^0.5
    if(euclidean_distance<=threshold_distance)
        return true
    else
        return false
    end
end

function is_within_range(location1::location, location2::location, threshold_distance)
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

#Given three points, find the center and the radius of the circle they lie on
function find_center_and_radius(x1,y1,x2,y2,x3,y3)
    x12 = x1-x2
    x13 = x1-x3

    y12 = y1-y2
    y13 = y1-y3

    y31 = y3-y1
    y21 = y2-y1

    x31 = x3-x1
    x21 = x2-x1

    sx13 = (x1*x1) - (x3*x3)
    sy13 = (y1*y1) - (y3*y3)
    sx21 = (x2*x2) - (x1*x1)
    sy21 = (y2*y2) - (y1*y1)

    f = ((sx13) * (x12) + (sy13) * (x12) + (sx21) * (x13) + (sy21) * (x13))/(2 * ((y31) * (x12) - (y21) * (x13)))
    g = ((sx13) * (y12)+ (sy13) * (y12)+ (sx21) * (y13)+ (sy21) * (y13))/(2 * ((x31) * (y12) - (x21) * (y13)))
    c = -(x1*x1) - (y1*y1) - (2*g*x1) - (2*f*y1)
    r = sqrt( (g*g) + (f*f) - c );

    return -g, -f, r
end

#Given a circle's center and radius and a line segment, find if they intersect
function find_if_circle_and_line_segment_intersect(cx::Float64,cy::Float64,cr::Float64,
                                    ex::Float64,ey::Float64,lx::Float64,ly::Float64)
    dx = lx-ex
    dy = ly-ey
    fx = ex-cx
    fy = ey-cy

    #Quadratic equation is  t^2 ( d · d ) + 2t ( d · f ) +  (f · f - r^2) = 0
    #Refer to this link if needed - https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    #Standard form is a.t^2 + b.t + c = 0

    a = (dx^2 + dy^2)
    b = 2*(dx*fx + dy*fy)
    c = (fx^2 + fy^2) - (cr^2)
    discriminant = (b^2 - 4*a*c)

    if(discriminant<0)
        return false
    elseif (discriminant == 0)
        t = -b/(2*a)
        if(t>=0 && t<=1)
            return true
        end
    else
        discriminant = sqrt(discriminant)
        t = (-b-discriminant)/(2*a)
        if(t>=0 && t<=1)
            return true
        end
        t = (-b+discriminant)/(2*a)
        if(t>=0 && t<=1)
            return true
        end
    end
    return false
end

# Check if line segment joining (x1,y1) to (x2,y2) and line segment joining (x3,y3)) to (x4,y4) intersect or not
function find_if_two_line_segments_intersect(x1::Float64,y1::Float64,x2::Float64,y2::Float64,
                                        x3::Float64,y3::Float64,x4::Float64,y4::Float64)

    #Refer to this link for the logic
    #http://paulbourke.net/geometry/pointlineplane/

    epsilon = 10^-6
    same_denominator = ( (y4-y3)*(x2-x1) ) - ( (x4-x3)*(y2-y1) )
    numerator_ua = ( (x4-x3)*(y1-y3) ) - ( (y4-y3)*(x1-x3) )
    numerator_ub = ( (x2-x1)*(y1-y3) ) - ( (y2-y1)*(x1-x3) )

    if(abs(same_denominator) < epsilon && abs(numerator_ua) < epsilon
                                        && abs(numerator_ub) < epsilon)
         return true;
    elseif (abs(same_denominator) < epsilon)
        return false
    else
        ua = numerator_ua/same_denominator
        ub = numerator_ub/same_denominator
        if(ua>=0.0 && ua<=1.0 && ub>=0.0 && ub<=1.0)
            return true;
        else
            return false;
        end
    end
end

function find_if_two_circles_intersect(c1x::Float64,c1y::Float64,c1r::Float64,c2x::Float64,c2y::Float64,c2r::Float64)
    dist_c1_c2 = (c1x - c2x)^2 + (c1y - c2y)^2
    if(dist_c1_c2 > (c1r + c2r)^2 )
        return false
    else
        return true
    end
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

function respawn_humans(world, user_defined_rng_for_env)

    total_humans_respawned_so_far = world.num_humans - world.max_num_humans
    num_humans_already_at_goal = 0
    for human in world.humans
        if( (human.x == human.goal.x) && (human.y == human.goal.y) )
            num_humans_already_at_goal += 1
        end
    end

    total_number_of_humans = length(world.humans)
    num_humans_to_be_respawned = num_humans_already_at_goal - total_humans_respawned_so_far
    distance_threshold = 1.0

    while(num_humans_to_be_respawned!=0)
        # println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Respawning Humans in the simulator ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        rand_num = rand(user_defined_rng_for_env)
        rand_num_to_choose_side = floor(4*rand_num)

        #rand_num_to_choose_side == 0 represents the left edge of the square
        if( rand_num_to_choose_side == 0.0 )
            new_human_x = 0.0
            new_human_y = floor(world.breadth*rand_num)
            while(is_within_range_check_with_points(new_human_x,new_human_y,world.cart_start_location.x,
                        world.cart_start_location.y,distance_threshold) && is_within_range_check_with_points(new_human_x,new_human_y,
                        world.cart.goal.x,world.cart.goal.y,distance_threshold) )
                new_human_y = floor(world.breadth*rand(user_defined_rng_for_env))
            end
            #Possible Goal locations for this human are G3(100,100) and G4(100,0)
            new_human_goal = world.goals[Int(ceil(rand(user_defined_rng_for_env)*2) + 2)]
            new_human_state = human_state(new_human_x,new_human_y,1,new_human_goal, Float64(total_number_of_humans + 1))
            total_number_of_humans += 1
        #rand_num_to_choose_side == 1 represents the top edge of the square
        elseif( rand_num_to_choose_side == 1.0 )
            new_human_y = world.breadth
            new_human_x = floor(world.length*rand_num)
            while(is_within_range_check_with_points(new_human_x,new_human_y,world.cart_start_location.x,
                        world.cart_start_location.y,distance_threshold) || is_within_range_check_with_points(new_human_x,new_human_y,
                        world.cart.goal.x,world.cart.goal.y,distance_threshold) )
                new_human_x = floor(world.length*rand(user_defined_rng_for_env))
            end
            #Possible Goal locations for this human are G1(0,0) and G4(100,0)
            if( rand(user_defined_rng_for_env) > 0.5)
                new_human_goal = world.goals[4]
            else
                new_human_goal = world.goals[1]
            end
            new_human_state = human_state(new_human_x,new_human_y,1,new_human_goal, Float64(total_number_of_humans + 1))
            total_number_of_humans += 1
        #rand_num_to_choose_side == 2 represents the right edge of the square
        elseif( rand_num_to_choose_side == 2.0 )
            new_human_x = world.length
            new_human_y = floor(world.breadth*rand_num)
            while(is_within_range_check_with_points(new_human_x,new_human_y,world.cart_start_location.x,
                        world.cart_start_location.y,distance_threshold) || is_within_range_check_with_points(new_human_x,new_human_y,
                        world.cart.goal.x,world.cart.goal.y,distance_threshold) )
                new_human_y = floor(world.breadth*rand(user_defined_rng_for_env))
            end
            #Possible Goal locations for this human are G1(0,0) and G2(0,100)
            new_human_goal = world.goals[Int(ceil(rand(user_defined_rng_for_env)*2))]
            new_human_state = human_state(new_human_x,new_human_y,1,new_human_goal, Float64(total_number_of_humans + 1))
            total_number_of_humans += 1
        #rand_num_to_choose_side == 3 represents the bottom edge of the square
        elseif( rand_num_to_choose_side == 3.0 )
            new_human_y = 0.0
            new_human_x = floor(world.length*rand_num)
            while(is_within_range_check_with_points(new_human_x,new_human_y,world.cart_start_location.x,
                        world.cart_start_location.y,distance_threshold) || is_within_range_check_with_points(new_human_x,new_human_y,
                        world.cart.goal.x,world.cart.goal.y,distance_threshold) )
                new_human_x = floor(world.length*rand(user_defined_rng_for_env))
            end
            #Possible Goal locations for this human are G2(0,100) and G3(100,100)
            new_human_goal = world.goals[Int(ceil(rand(user_defined_rng_for_env)*2) + 1)]
            new_human_state = human_state(new_human_x,new_human_y,1,new_human_goal, Float64(total_number_of_humans + 1))
            total_number_of_humans += 1
        end

        push!(world.humans, new_human_state)
        world.num_humans += 1
        num_humans_to_be_respawned -= 1
    end
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

function move_human_for_one_time_step_in_actual_environment(world,time_step,user_defined_rng)
    moved_human_positions = Array{human_state,1}()
    for human in world.humans
        if(human.x == human.goal.x && human.y==human.goal.y)
            new_human_state = human_state(human.x,human.y,human.v,human.goal,human.id)
        elseif (is_within_range(location(human.x,human.y), human.goal, 1.0))
            new_human_state = human_state(human.goal.x,human.goal.y,human.v,human.goal,human.id)
        else
            new_human_state = get_new_human_position_actual_environemnt(human,world,time_step,user_defined_rng)
        end
        push!(moved_human_positions,new_human_state)
    end
    return moved_human_positions
end

function get_nearest_n_pedestrians_hybrid_astar_search(world,current_belief,n,closest_ped_dist_threshold,cone_half_angle::Float64=pi/3.0)
    nearest_n_pedestrians = Array{Tuple{human_state,human_probability_over_goals},1}()
    priority_queue_nearest_n_pedestrians = PriorityQueue{Tuple{human_state,human_probability_over_goals},Float64}(Base.Order.Forward)
    for i in 1:length(world.cart_lidar_data)
        human = world.cart_lidar_data[i]
        angle_between_cart_and_human = get_heading_angle(human.x, human.y, world.cart.x, world.cart.y)
        difference_in_angles = abs(world.cart.theta - angle_between_cart_and_human)
        euclidean_distance = sqrt( (world.cart.x - human.x)^2 + (world.cart.y - human.y)^2 )
        if(difference_in_angles <= cone_half_angle)
            priority_queue_nearest_n_pedestrians[(human,current_belief[i])] = euclidean_distance
        elseif ( (2*pi - difference_in_angles) <= cone_half_angle )
            priority_queue_nearest_n_pedestrians[(human,current_belief[i])] = euclidean_distance
        elseif (euclidean_distance<=closest_ped_dist_threshold)
            priority_queue_nearest_n_pedestrians[(human,current_belief[i])] = euclidean_distance
        end
    end
    for i in 1:n
        if(length(priority_queue_nearest_n_pedestrians) != 0)
            push!(nearest_n_pedestrians,dequeue!(priority_queue_nearest_n_pedestrians))
        else
            break
        end
    end
    return nearest_n_pedestrians
end

function get_intermediate_point_human_trajectory(start_x, start_y, end_x, end_y, discrete_time)
    tbr_x = start_x + discrete_time*(end_x - start_x)
    tbr_y = start_y + discrete_time*(end_y - start_y)
    return tbr_x,tbr_y
end

function get_num_risks(vehicle,vehicle_params,human_position_data,min_safe_distance_from_human)
    risks = 0
    if(vehicle.v!=0.0)
        for human in human_position_data
            euclidean_distance = sqrt( (human.x - vehicle.x)^2 + (human.y - vehicle.y)^2 )
            if(euclidean_distance<=(vehicle_params.L+min_safe_distance_from_human))
                println( "A risky scenario encountered and the distance is : ", euclidean_distance )
                risks += 1
            end
        end
    else
        return 0;
    end
    return risks;
end

function get_steering_angle(vehicle_length, delta_angle, vehicle_speed, time_duration)
    if(vehicle_speed==0.0 || delta_angle==0.0)
        return 0.0
    else
        return atan((vehicle_length*delta_angle)/(vehicle_speed*time_duration))
    end
end

function write_and_print(io::IOStream, string_to_be_written_and_printed::String)
    write(io, string_to_be_written_and_printed * "\n")
    println(string_to_be_written_and_printed)
end

#=
************************************************************************************************************************************************
=#

function get_nearby_humans(vehicle, vehicle_params, lidar_data, num_nearby_humans, min_safe_distance_from_human, cone_half_angle::Float64=pi/3.0)
    nearby_humans = Array{human_state,1}()
    index_nearby_humans = Array{Int64,1}()
    priority_queue_nearby_humans = PriorityQueue{Tuple{human_state,Int64},Float64}(Base.Order.Forward)
    humans = lidar_data[1]
    for i in 1:length(humans)
        human = humans[i]
        index_human_in_lidar_data = i  #Position of this human in the lidar data array. Need this position to access corresponding belief
        angle_between_vehicle_and_human = get_heading_angle(human.x, human.y, vehicle.x, vehicle.y)
        difference_in_angles = abs(vehicle.theta - angle_between_vehicle_and_human)
        euclidean_distance = sqrt( (vehicle.x - human.x)^2 + (vehicle.y - human.y)^2 )
        if(difference_in_angles <= cone_half_angle)
            priority_queue_nearby_humans[(human,index_human_in_lidar_data)] = euclidean_distance
        elseif((2*pi - difference_in_angles) <= cone_half_angle)
            priority_queue_nearby_humans[(human,index_human_in_lidar_data)] = euclidean_distance
        elseif(euclidean_distance<=min_safe_distance_from_human+vehicle_params.L)
            priority_queue_nearby_humans[(human,index_human_in_lidar_data)] = euclidean_distance
        end
    end

    num_nearby_humans = min(num_nearby_humans, length(priority_queue_nearby_humans))
    for i in 1:num_nearby_humans
        human,index = dequeue!(priority_queue_nearby_humans)
        push!(nearby_humans, human)
        push!(index_nearby_humans, index)
    end

    return (nearby_humans, index_nearby_humans)
end

function get_belief_nearby_humans(sensor_data, nearby_humans, index_nearby_humans)
    belief_nearby_humans = Array{belief_over_human_goals,1}()
    for index in index_nearby_humans
        push!(belief_nearby_humans, sensor_data.belief[index])
    end
    return belief_nearby_humans
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

function is_there_immediate_collision_with_pedestrians(world, pedestrian_distance_threshold)
    for human in world.complete_cart_lidar_data
        if( find_if_two_circles_intersect(world.cart.x, world.cart.y, world.cart.L,human.x, human.y, pedestrian_distance_threshold) )
            return true
        end
    end
    return false
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
