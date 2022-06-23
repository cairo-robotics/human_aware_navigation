include("environment.jl")
include("prm.jl")
include("utils.jl")
include("two_d_action_space_pomdp_for_prm.jl")
include("belief_tracker.jl")
include("simulator.jl")
using DataStructures
using FileIO
using JLD2
using D3Trees

Base.copy(s::cart_state) = cart_state(s.x, s.y,s.theta,s.v,s.L,s.goal)

function run_one_simulation_2D_POMDP_planner_prm(env_right_now, user_defined_rng, m,
                            planner, filename = "output_just_2d_action_space_pomdp_planner_prm.txt")

    time_taken_by_cart = 0
    number_risks = 0
    one_time_step = 0.5
    lidar_range = 30
    num_humans_to_care_about_while_pomdp_planning = 6
    cone_half_angle::Float64 = (1)*pi
    number_of_sudden_stops = 0
    cart_ran_into_boundary_wall_flag = false
    cart_ran_into_static_obstacle_flag = false
    cart_reached_goal_flag = true
    experiment_success_flag = true
    cart_throughout_path = OrderedDict()
    all_gif_environments = OrderedDict()
    all_observed_environments = OrderedDict()
    all_generated_beliefs = OrderedDict()
    all_generated_beliefs_using_complete_lidar_data = OrderedDict()
    all_generated_trees = OrderedDict()
    all_risky_scenarios = OrderedDict()
    all_actions = OrderedDict()
    all_planners = OrderedDict()
    MAX_TIME_LIMIT = 300

    #Sense humans near cart before moving
    #Generate Initial Lidar Data and Belief for humans near cart
    env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
    env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                        env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                        m.pedestrian_distance_threshold, cone_half_angle)

    initial_belief_over_complete_cart_lidar_data = update_belief([],env_right_now.goals,[],env_right_now.complete_cart_lidar_data)
    initial_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(initial_belief_over_complete_cart_lidar_data,
                                                            env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
    #initial_belief = update_belief([],env_right_now.goals,[],env_right_now.complete_cart_lidar_data)

    dict_key = "t="*string(time_taken_by_cart)
    all_gif_environments[dict_key] = deepcopy(env_right_now)
    all_observed_environments[dict_key] = deepcopy(env_right_now)
    all_generated_beliefs_using_complete_lidar_data[dict_key] = initial_belief_over_complete_cart_lidar_data
    all_generated_beliefs[dict_key] = initial_belief
    cart_throughout_path[dict_key] = copy(env_right_now.cart)

    #Simulate for t=0 to t=1
    io = open(filename,"a")
    write_and_print( io, "Simulating for time interval - (" * string(time_taken_by_cart) * " , " * string(time_taken_by_cart+1) * ")" )
    write_and_print( io, "Current cart state = " * string(env_right_now.cart) )

    #Update human positions in environment for two time steps and cart's belief accordingly
    current_belief_over_complete_cart_lidar_data, risks_in_simulation = simulate_pedestrians_and_generate_gif_environments_when_cart_stationary(env_right_now,
                                                        initial_belief_over_complete_cart_lidar_data,all_gif_environments, all_risky_scenarios, time_taken_by_cart,
                                                        num_humans_to_care_about_while_pomdp_planning,cone_half_angle, lidar_range, m.pedestrian_distance_threshold,
                                                        user_defined_rng, io )
    current_belief =  get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                            env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
    number_risks += risks_in_simulation
    time_taken_by_cart += 1
    dict_key = "t="*string(time_taken_by_cart)
    all_observed_environments[dict_key] = deepcopy(env_right_now)
    all_generated_beliefs_using_complete_lidar_data[dict_key] = current_belief_over_complete_cart_lidar_data
    all_generated_beliefs[dict_key] = current_belief
    cart_throughout_path[dict_key] = copy(env_right_now.cart)
    write_and_print( io, "Modified cart state = " * string(env_right_now.cart) )
    close(io)

    # try
        #Start Simulating for t>1
        while(!is_within_range(location(env_right_now.cart.x,env_right_now.cart.y), env_right_now.cart.goal, 1.0))
            #display_env(env_right_now)
            io = open(filename,"a")
            cart_ran_into_boundary_wall_flag = check_if_cart_collided_with_boundary_wall(env_right_now)
            cart_ran_into_static_obstacle_flag = check_if_cart_collided_with_static_obstacles(env_right_now)
            if( !cart_ran_into_boundary_wall_flag && !cart_ran_into_static_obstacle_flag )

                write_and_print( io, "Simulating for time interval - (" * string(time_taken_by_cart) * " , " * string(time_taken_by_cart+1) * ")" )
                write_and_print( io, "Current cart state = " * string(env_right_now.cart) )

                #Solve POMDP to get the best action
                # m = golfcart_2D_action_space_pomdp()
                dict_key = "t="*string(time_taken_by_cart)
                all_planners[dict_key] = deepcopy(planner)
                b = POMDP_2D_action_space_state_distribution_prm(m.world,current_belief)
                a, info = action_info(planner, b)
                check_consistency_personal_copy(io,planner.rs)
                if(is_there_immediate_collision_with_pedestrians(m.world, m.pedestrian_distance_threshold))
                    if(env_right_now.cart.v == 1.0)
                        a = POMDP_2D_action_type_prm(0.0,-1.0,0,0.0,0.0,0,0.0,0.0)
                    elseif(env_right_now.cart.v == m.max_cart_speed)
                        a = POMDP_2D_action_type_prm(-10.0,-10.0,0,0.0,0.0,0,0.0,0.0)
                    elseif(env_right_now.cart.v == 0.0)
                        a = POMDP_2D_action_type_prm(0.0,0.0,0,0.0,0.0,0,0.0,0.0)
                    end
                end
                write_and_print( io, "Action chosen by 2D action space POMDP planner: " * string(a) )
                dict_key = "t="*string(time_taken_by_cart)
                all_generated_trees[dict_key] = deepcopy(info)
                all_actions[dict_key] = a

                if(env_right_now.cart.v!=0 && a.delta_velocity == -10.0)
                    number_of_sudden_stops += 1
                end
                env_right_now.cart.v = clamp(env_right_now.cart.v + a.delta_velocity,0,m.max_cart_speed)

                if(env_right_now.cart.v != 0.0)
                    # That means the cart is not stationary and we now have to simulate both cart and the pedestrians.
                    # steering_angle = atan((env_right_now.cart.L*a[1])/env_right_now.cart.v)
                    if(a.delta_angle != -10.0)
                        current_belief_over_complete_cart_lidar_data, risks_in_simulation = simulate_cart_and_pedestrians_and_generate_gif_environments_when_cart_moving(
                                                                            env_right_now,current_belief_over_complete_cart_lidar_data, all_gif_environments,
                                                                            all_risky_scenarios, time_taken_by_cart,num_humans_to_care_about_while_pomdp_planning,
                                                                            cone_half_angle, lidar_range, m.pedestrian_distance_threshold,
                                                                            user_defined_rng, a.delta_angle, io)
                        m.last_prm_vertex_visited = -1
                        m.next_prm_vertex_to_be_visited = -1
                        m.last_prm_vertex_crossed_flag = false
                    elseif( a.delta_angle == -10.0 )
                        current_belief_over_complete_cart_lidar_data, risks_in_simulation, first_vertex_crossed_flag = simulate_cart_and_pedestrians_and_generate_gif_environments_when_cart_moving_along_prm_path(
                                                                            env_right_now,current_belief_over_complete_cart_lidar_data, all_gif_environments,
                                                                            all_risky_scenarios, time_taken_by_cart,num_humans_to_care_about_while_pomdp_planning,
                                                                            cone_half_angle, lidar_range, m.pedestrian_distance_threshold,
                                                                            user_defined_rng, a.first_prm_vertex_x, a.first_prm_vertex_y,
                                                                            a.second_prm_vertex_x, a.second_prm_vertex_y, io)
                        m.last_prm_vertex_visited = a.first_prm_vertex_num
                        m.next_prm_vertex_to_be_visited = a.second_prm_vertex_num
                        if(first_vertex_crossed_flag)
                            m.last_prm_vertex_crossed_flag = true
                        else
                            m.last_prm_vertex_crossed_flag = false
                        end
                    end
                    current_belief =  get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                                        env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
                    number_risks += risks_in_simulation
                else
                    #That means the cart is stationary and we now just have to simulate the pedestrians.
                    current_belief_over_complete_cart_lidar_data, risks_in_simulation = simulate_pedestrians_and_generate_gif_environments_when_cart_stationary(
                                                                        env_right_now,current_belief_over_complete_cart_lidar_data,all_gif_environments,
                                                                        all_risky_scenarios, time_taken_by_cart,num_humans_to_care_about_while_pomdp_planning,
                                                                        cone_half_angle, lidar_range, m.pedestrian_distance_threshold,
                                                                        user_defined_rng, io )
                    current_belief =  get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                                        env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)


                    number_risks += risks_in_simulation
                end

                time_taken_by_cart += 1
                dict_key = "t="*string(time_taken_by_cart)
                all_observed_environments[dict_key] = deepcopy(env_right_now)
                all_generated_beliefs_using_complete_lidar_data[dict_key] = current_belief_over_complete_cart_lidar_data
                all_generated_beliefs[dict_key] = current_belief
                cart_throughout_path[dict_key] = copy(env_right_now.cart)

                write_and_print( io, "Modified cart state = " * string(env_right_now.cart) )
                write_and_print( io, "************************************************************************" )


                if(time_taken_by_cart>MAX_TIME_LIMIT)
                    cart_reached_goal_flag = false
                    break
                end
            else
                if(cart_ran_into_static_obstacle_flag)
                    write_and_print( io, "Cart ran into a static obstacle in the environment")
                elseif (cart_ran_into_boundary_wall_flag)
                    write_and_print( io, "Cart ran into a boundary wall in the environment")
                end
                cart_reached_goal_flag = false
                break
            end
            close(io)
        end
    # catch e
    #     println("\n Things failed during the simulation. \n The error message is : \n ")
    #     println(e)
    #     experiment_success_flag = false
    #     return all_gif_environments, all_observed_environments, all_generated_beliefs_using_complete_lidar_data, all_generated_beliefs,
    #         all_generated_trees,all_risky_scenarios,all_actions,all_planners,cart_throughout_path, number_risks, number_of_sudden_stops,
    #         time_taken_by_cart, cart_reached_goal_flag, cart_ran_into_static_obstacle_flag, cart_ran_into_boundary_wall_flag, experiment_success_flag
    # end

    io = open(filename,"a")
    if(cart_reached_goal_flag == true)
        write_and_print( io, "Goal Reached! :D" )
        write_and_print( io, "Time Taken by cart to reach goal : " * string(time_taken_by_cart) )
    else
        if(cart_ran_into_boundary_wall_flag)
            write_and_print( io, "Cart ran into a wall :(" )
            write_and_print( io, "Time elapsed before this happened : " * string(time_taken_by_cart) )
        elseif cart_ran_into_static_obstacle_flag
            write_and_print( io, "Cart ran into a static obstacle :(" )
            write_and_print( io, "Time elapsed before this happened : " * string(time_taken_by_cart) )
        else
            write_and_print( io, "Cart ran out of time :(" )
            write_and_print( io, "Time Taken by cart when it didn't reach the goal : " * string(time_taken_by_cart) )
        end
    end
    write_and_print( io, "Number of risky scenarios encountered by the cart : " * string(number_risks) )
    write_and_print( io, "Number of sudden stops taken by the cart : " * string(number_of_sudden_stops) )
    close(io)

    return all_gif_environments, all_observed_environments, all_generated_beliefs_using_complete_lidar_data, all_generated_beliefs,
        all_generated_trees,all_risky_scenarios,all_actions,all_planners,cart_throughout_path, number_risks, number_of_sudden_stops,
        time_taken_by_cart, cart_reached_goal_flag, cart_ran_into_static_obstacle_flag, cart_ran_into_boundary_wall_flag, experiment_success_flag
end

function get_actions_holonomic_prm(m::POMDP_Planner_2D_action_space_prm,b)
    pomdp_state = first(particles(b))
    x_point =  clamp(floor(Int64,pomdp_state.cart.x/ 1.0)+1,1,100)
    y_point =  clamp(floor(Int64,pomdp_state.cart.y/ 1.0)+1,1,100)
    theta_point = clamp(floor(Int64,pomdp_state.cart.theta/(pi/18))+1,1,36)
    # nearest_prm_point = m.lookup_table[x_point,y_point]
    if(m.last_prm_vertex_visited == -1)
        nearest_prm_point = m.lookup_table[x_point,y_point,theta_point]
    elseif(m.last_prm_vertex_crossed_flag)
        next_prm_vertex = m.prm_details[m.next_prm_vertex_to_be_visited]
        if(m.next_prm_vertex_to_be_visited == 2)
            next_to_next_prm_vertex = next_prm_vertex.path_to_goal[1]
        else
            next_to_next_prm_vertex = next_prm_vertex.path_to_goal[2]
        end
        nearest_prm_point = lookup_table_struct(m.next_prm_vertex_to_be_visited,next_prm_vertex.x,next_prm_vertex.x,
                                                        next_to_next_prm_vertex.vertex_num, next_to_next_prm_vertex.x, next_to_next_prm_vertex.y)
    else
        nearest_prm_point = m.lookup_table[x_point,y_point,theta_point]
    end
    if(pomdp_state.cart.v == 0.0)
        if(nearest_prm_point.closest_prm_vertex_num == -1)
            a = [ POMDP_2D_action_type_prm(-pi/4,1.0,0,0.0,0.0,0,0.0,0.0) , POMDP_2D_action_type_prm(-pi/6,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(-pi/12,1.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(0.0,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(0.0,0.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(pi/12,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(pi/6,1.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(pi/4,1.0,0,0.0,0.0,0,0.0,0.0) ]
        else
            a = [ POMDP_2D_action_type_prm(-pi/4,1.0,0,0.0,0.0,0,0.0,0.0) , POMDP_2D_action_type_prm(-pi/6,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(-pi/12,1.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(0.0,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(0.0,0.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(pi/12,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(pi/6,1.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(pi/4,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(-10.0,1.0,nearest_prm_point.closest_prm_vertex_num,nearest_prm_point.closest_prm_vertex_x,
                                            nearest_prm_point.closest_prm_vertex_y,nearest_prm_point.next_prm_vertex_num,
                                            nearest_prm_point.next_prm_vertex_x, nearest_prm_point.next_prm_vertex_y) ]
        end
    # elseif (pomdp_state.cart.v == m.max_cart_speed)
    else
        if(nearest_prm_point.closest_prm_vertex_num == -1)
            # return [(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-1.0),(0.0,0.0),(0.0,1.0),(pi/12,1.0),(pi/6,0.0),(pi/4,0.0),(-10.0,-10.0)]
            #Without immediate stop action
            a = [ POMDP_2D_action_type_prm(-pi/4,0.0,0,0.0,0.0,0,0.0,0.0) , POMDP_2D_action_type_prm(-pi/6,0.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(-pi/12,0.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(0.0,-1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(0.0,0.0,0,0.0,0.0,0,0.0,0.0),POMDP_2D_action_type_prm(0.0,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(pi/12,0.0,0,0.0,0.0,0,0.0,0.0),POMDP_2D_action_type_prm(pi/6,0.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(pi/4,0.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(-10.0,-10.0,0,0.0,0.0,0,0.0,0.0) ]
        else
            # return [(delta_angle, 0.0),(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-1.0),(0.0,0.0),(0.0,1.0),(pi/12,1.0),(pi/6,0.0),(pi/4,0.0),(-10.0,-10.0)]
            #Without immediate stop action
            a = [ POMDP_2D_action_type_prm(-pi/4,0.0,0,0.0,0.0,0,0.0,0.0) , POMDP_2D_action_type_prm(-pi/6,0.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(-pi/12,0.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(0.0,-1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(0.0,0.0,0,0.0,0.0,0,0.0,0.0),POMDP_2D_action_type_prm(0.0,1.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(pi/12,0.0,0,0.0,0.0,0,0.0,0.0),POMDP_2D_action_type_prm(pi/6,0.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(pi/4,0.0,0,0.0,0.0,0,0.0,0.0), POMDP_2D_action_type_prm(-10.0,-10.0,0,0.0,0.0,0,0.0,0.0),
                POMDP_2D_action_type_prm(-10.0,0.0,nearest_prm_point.closest_prm_vertex_num,nearest_prm_point.closest_prm_vertex_x,
                                                                                    nearest_prm_point.closest_prm_vertex_y,nearest_prm_point.next_prm_vertex_num,
                                                                                    nearest_prm_point.next_prm_vertex_x, nearest_prm_point.next_prm_vertex_y) ]
        end
    end
    return a
end


# lookup_table = nothing
gr()
run_simulation_flag = false
write_to_file_flag = false
create_gif_flag = true

if(run_simulation_flag)

    #Set seeds for different random number generators randomly
    rand_noise_generator_seed_for_env = rand(UInt32)
    rand_noise_generator_seed_for_sim = rand(UInt32)
    # rand_noise_generator_seed_for_prm = 11
    # rand_noise_generator_for_env = MersenneTwister(rand_noise_generator_seed_for_env)
    # rand_noise_generator_for_sim = MersenneTwister(rand_noise_generator_seed_for_sim)

    #Set seeds for different random number generators manually
    # rand_noise_generator_seed_for_env = 0x948828c9
    # rand_noise_generator_seed_for_sim = 0x06424ca6
    rand_noise_generator_seed_for_prm = 11
    rand_noise_generator_seed_for_solver = 0x0b8d125b
    rand_noise_generator_for_env = MersenneTwister(rand_noise_generator_seed_for_env)
    rand_noise_generator_for_sim = MersenneTwister(rand_noise_generator_seed_for_sim)
    rand_noise_generator_for_prm = MersenneTwister(rand_noise_generator_seed_for_prm)
    rand_noise_generator_for_solver = MersenneTwister(rand_noise_generator_seed_for_solver)

    #Initialize environment
    # env = generate_environment_no_obstacles(300, rand_noise_generator_for_env)
    # env = generate_environment_small_circular_obstacles(300, rand_noise_generator_for_env)
    # env = generate_environment_large_circular_obstacles(300, rand_noise_generator_for_env)
    env = generate_environment_L_shaped_corridor(300, rand_noise_generator_for_env)


    #Load prm_details and the lookup_table
    prm_info_dict = load("prm_hash_table_scenario4.jld2")
    lookup_table = prm_info_dict["lookup_table"]
    prm_details = prm_info_dict["prm_details"]
    if(lookup_table == nothing)
        #Generate PRM and prm_details array
        graph = generate_prm_vertices(1000, rand_noise_generator_for_prm, env)
        d = generate_prm_edges(env, graph, 10)
        prm_details = Array{prm_info_struct,1}(undef,nv(graph))
        for i in 1:nv(graph)
           st = prm_info_struct(get_prop(graph,i,:x) , get_prop(graph,i,:y), get_prop(graph,i,:dist_to_goal), get_prop(graph,i,:path_to_goal))
           prm_details[i] = st
        end
        lookup_table = generate_prm_points_coordinates_lookup_table_holonomic_using_x_y_theta(env,graph)
    end

    env_right_now = deepcopy(env)

    filename = "output_just_2d_action_space_pomdp_planner_prm.txt"
    io = open(filename,"w")
    write_and_print( io, "RNG seed for generating environemnt -> " * string(rand_noise_generator_seed_for_env))
    write_and_print( io, "RNG seed for simulating pedestrians -> " * string(rand_noise_generator_seed_for_sim))
    write_and_print( io, "RNG seed for Generating PRM -> " * string(rand_noise_generator_seed_for_prm))

    #Create POMDP for env_right_now
    #POMDP_Planner_2D_action_space <: POMDPs.POMDP{POMDP_state_2D_action_space,Int,Array{location,1}}
    # discount_factor::Float64; pedestrian_distance_threshold::Float64; pedestrian_collision_penalty::Float64;
    # obstacle_distance_threshold::Float64; obstacle_collision_penalty::Float64; goal_reward_distance_threshold::Float64;
    # cart_goal_reached_distance_threshold::Float64; goal_reward::Float64; max_cart_speed::Float64; world::experiment_environment;
    # lookup_table:: Matrix
    golfcart_2D_action_space_pomdp_prm = POMDP_Planner_2D_action_space_prm(0.97,1.0,-100.0,2.0,-100.0,0.0,1.0,1000.0,2.0,env_right_now,prm_details,lookup_table,-1,-1,false)
    discount(p::POMDP_Planner_2D_action_space_prm) = p.discount_factor
    isterminal(::POMDP_Planner_2D_action_space_prm, s::POMDP_state_2D_action_space_prm) = is_terminal_state_pomdp_planning_prm(s,location(-100.0,-100.0));
    actions(m::POMDP_Planner_2D_action_space_prm,b) = get_actions_holonomic_prm(m,b)

    # solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound_policy_pomdp_planning_2D_action_space(golfcart_2D_action_space_pomdp, b)),
    #                         max_depth=100),calculate_upper_bound_value_pomdp_planning_2D_action_space, check_terminal=true),K=50,D=100,T_max=Inf,max_trials=50, tree_in_info=true)
    solver_prm = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound_policy_pomdp_planning_2D_action_space_prm(golfcart_2D_action_space_pomdp_prm, b)),
                            max_depth=100),calculate_upper_bound_value_pomdp_planning_2D_action_space_prm, check_terminal=true),K=50,D=100,T_max=0.5,tree_in_info=true,
                            rng = rand_noise_generator_for_solver)
    # solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning_2D_action_space),max_depth=100,
    #                         final_value=reward_to_be_awarded_at_max_depth_in_lower_bound_policy_rollout),
    #     calculate_upper_bound_value_pomdp_planning_2D_action_space, check_terminal=true),K=100,D=100,T_max=0.5, tree_in_info=true, default_action=(-10.0,-10.0))

    write_and_print( io, "RNG seed for Solver -> " * string(solver_prm.rng.seed[1]) * "\n")
    close(io)

    planner = POMDPs.solve(solver_prm, golfcart_2D_action_space_pomdp_prm);

    just_2D_pomdp_all_gif_environments_prm, just_2D_pomdp_all_observed_environments_prm, just_2D_pomdp_all_generated_beliefs_using_complete_lidar_data_prm,
            just_2D_pomdp_all_generated_beliefs_prm, just_2D_pomdp_all_generated_trees_prm, just_2D_pomdp_all_risky_scenarios_prm, just_2D_pomdp_all_actions_prm,
            just_2D_pomdp_all_planners_prm,just_2D_pomdp_cart_throughout_path_prm, just_2D_pomdp_number_risks_prm,just_2D_pomdp_number_of_sudden_stops_prm,
            just_2D_pomdp_time_taken_by_cart_prm,just_2D_pomdp_cart_reached_goal_flag_prm, just_2D_pomdp_cart_ran_into_static_obstacle_flag_prm,
            just_2D_pomdp_cart_ran_into_boundary_wall_flag_prm,just_2D_pomdp_experiment_success_flag_prm = run_one_simulation_2D_POMDP_planner_prm(env_right_now,
                                                                                        rand_noise_generator_for_sim, golfcart_2D_action_space_pomdp_prm, planner)

    if(create_gif_flag)
        anim = @animate for k ∈ keys(just_2D_pomdp_all_observed_environments_prm)
            display_env(just_2D_pomdp_all_observed_environments_prm[k], k);
            #savefig("./plots_just_2d_action_space_pomdp_planner/plot_"*string(i)*".png")
        end
        gif(anim, "just_2D_action_space_pomdp_planner_run_prm.gif", fps = 2)
    end

    if(write_to_file_flag || just_2D_pomdp_number_risks_prm != 0 || just_2D_pomdp_cart_ran_into_boundary_wall_flag_prm
                || just_2D_pomdp_cart_ran_into_static_obstacle_flag_prm || !just_2D_pomdp_experiment_success_flag_prm || !just_2D_pomdp_cart_reached_goal_flag_prm)
        expt_file_name_prm = "expt_details_just_2d_action_space_pomdp_planner_prm.jld2"
        write_experiment_details_to_file_for_prm(rand_noise_generator_seed_for_env,rand_noise_generator_seed_for_sim,rand_noise_generator_seed_for_prm,
                solver_prm.rng.seed[1],just_2D_pomdp_all_gif_environments_prm, just_2D_pomdp_all_observed_environments_prm,
                just_2D_pomdp_all_generated_beliefs_using_complete_lidar_data_prm,just_2D_pomdp_all_generated_beliefs_prm, just_2D_pomdp_all_generated_trees_prm,
                just_2D_pomdp_all_risky_scenarios_prm, just_2D_pomdp_all_actions_prm,just_2D_pomdp_all_planners_prm,just_2D_pomdp_cart_throughout_path_prm,
                just_2D_pomdp_number_risks_prm,just_2D_pomdp_number_of_sudden_stops_prm,just_2D_pomdp_time_taken_by_cart_prm,just_2D_pomdp_cart_reached_goal_flag_prm,
                just_2D_pomdp_cart_ran_into_static_obstacle_flag_prm,just_2D_pomdp_cart_ran_into_boundary_wall_flag_prm,just_2D_pomdp_experiment_success_flag_prm,
                expt_file_name_prm)
    end

end

#=
expt_file_name = "expt_details_just_2d_action_space_pomdp_planner.jld2";
expt_details_dict = load(expt_file_name);
test_time_step = "47";
b = POMDP_2D_action_space_state_distribution(expt_details_dict["all_observed_environments"]["t="*test_time_step],expt_details_dict["all_generated_beliefs"]["t="*test_time_step]);
copy_of_planner = deepcopy(expt_details_dict["all_planners"]["t="*test_time_step]);
supposed_a, supposed_info = action_info(copy_of_planner, b);
supposed_a
=#

#=
test_time_step = "47";
b = POMDP_2D_action_space_state_distribution(just_2D_pomdp_all_observed_environments["t="*test_time_step],just_2D_pomdp_all_generated_beliefs["t="*test_time_step]);
copy_of_planner = deepcopy(just_2D_pomdp_all_planners["t="*test_time_step]);
supposed_a, supposed_info = action_info(copy_of_planner, b);
despot_tree = supposed_info[:tree];
curr_scenario_belief = ARDESPOT.get_belief(despot_tree,1,deepcopy(just_2D_pomdp_all_planners["t="*test_time_step]).rs);
ARDESPOT.lbound(copy_of_planner.bounds.lower, copy_of_planner.pomdp, curr_scenario_belief)
ARDESPOT.ubound(copy_of_planner.bounds.upper, copy_of_planner.pomdp, curr_scenario_belief)
supposed_a[1]*180/pi
# curr_scenarios = supposed_info[:tree].scenarios[1];
# curr_scenario_belief =
ARDESPOT.lbound(copy_of_planner.bounds.lower, copy_of_planner.pomdp, curr_scenario_belief)
ARDESPOT.ubound(copy_of_planner.bounds.upper, copy_of_planner.pomdp, curr_scenario_belief)
test_time_step = "8";
b = POMDP_2D_action_space_state_distribution(just_2D_pomdp_all_observed_environments["t="*test_time_step],just_2D_pomdp_all_generated_beliefs["t="*test_time_step]);
copy_of_planner = deepcopy(just_2D_pomdp_all_planners["t="*test_time_step]);
root_scenarios = [i=>rand(copy_of_planner.rng, b) for i in 1:copy_of_planner.sol.K];
curr_scenario_belief = ScenarioBelief(root_scenarios, copy_of_planner.rs, 0, b);
L_0, U_0 = bounds(copy_of_planner.bounds, copy_of_planner.pomdp, curr_scenario_belief)
lb_policy = DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound_policy_pomdp_planning_2D_action_space_debug(golfcart_2D_action_space_pomdp, b,io)),max_depth=100);
io = open("random_file.txt", "w")
ARDESPOT.lbound(lb_policy, copy_of_planner.pomdp, curr_scenario_belief)
close(io)
ARDESPOT.ubound(copy_of_planner.bounds.upper, copy_of_planner.pomdp, curr_scenario_belief)
calculate_lower_bound_policy_pomdp_planning_2D_action_space_debug(copy_of_planner.pomdp, curr_scenario_belief)
gen_action_debugging = (-0.6491513225430392, 1.0)
curr_pomdp_state = curr_scenario_belief.scenarios[1][2]
POMDPs.gen(golfcart_2D_action_space_pomdp, curr_pomdp_state, gen_action_debugging, MersenneTwister(1234))
new_pomdp_state = POMDP_state_2D_action_space(cart_state(11.254059234304371, 33.770025148236186, 0.9201807793229126, 2.0, 1.0, location(100.0, 75.0)), human_state[human_state(26.540962076986272, 32.66428415194191, 1.0, location(100.0, 100.0), 2.0), human_state(23.34710220661714, 47.10950052256582, 1.0, location(100.0, 100.0), 9.0), human_state(25.81733060754584, 32.05392277150804, 1.0, location(100.0, 100.0), 105.0), human_state(5.80010818880632, 50.37685216243477, 1.0, location(0.0, 100.0), 140.0), human_state(5.853906380814106, 13.429549932455878, 1.0, location(0.0, 0.0), 256.0), human_state(1.0409728300598835, 4.163891320239533, 1.0, location(0.0, 0.0), 281.0)])
new_scenario = [1=>new_pomdp_state]
new_scenario_belief = ScenarioBelief(new_scenario, copy_of_planner.rs, 0, b);
action_according_to_rollout_policy_at_this_belief = calculate_lower_bound_policy_pomdp_planning_2D_action_space_debug(copy_of_planner.pomdp, new_scenario_belief)
println(action_according_to_rollout_policy_at_this_belief)
POMDPs.gen(golfcart_2D_action_space_pomdp, new_pomdp_state, action_according_to_rollout_policy_at_this_belief, MersenneTwister(1234))
x_point =  floor(Int64,new_pomdp_state.cart.x/ 1.0)+1
y_point =  floor(Int64,new_pomdp_state.cart.y/ 1.0)+1
theta_point = clamp(floor(Int64,new_pomdp_state.cart.theta/(pi/18))+1,1,36)
nearest_prm_point = lookup_table[x_point,y_point,theta_point]
=#


#=
anim = @animate for k ∈ keys(just_2D_pomdp_all_gif_environments)
    display_env(just_2D_pomdp_all_gif_environments[k],k);
    #savefig("./plots_just_2d_action_space_pomdp_planner/plot_"*all_gif_environments[i][1]*".png")
end
gif(anim, "just_2D_action_space_pomdp_planner_run.gif", fps = 20)
=#

#=
anim = @animate for k ∈ keys(just_2D_pomdp_all_gif_environments_prm)
    display_env(just_2D_pomdp_all_gif_environments_prm[k],k);
    #savefig("./plots_just_2d_action_space_pomdp_planner/plot_"*all_gif_environments[i][1]*".png")
end
gif(anim, "just_2D_action_space_pomdp_planner_run_prm.gif", fps = 20)
=#

#inchrome(D3Tree(just_2D_pomdp_all_generated_trees[9][:tree]))

#Notes on the simulator!
#=
1) just_2D_pomdp_all_observed_environments, just_2D_pomdp_all_generated_beliefs and just_2D_pomdp_all_generated_trees have data for
        Initial environment at index 1
        Env from time step 0 to the end from index 2 onwards
        Env at index 2 is obtained by just waiting and updating the belief over pedestrians in lidar data
        Also, ith index in just_2D_pomdp_all_observed_environments implies the environment at time stamp (i-1).
        i.e. just_2D_pomdp_all_observed_environments[23] is actually the environment at time stamp 22 seconds.
2) just_2D_pomdp_all_gif_environments has the simulator data for every 0.1 second.
        just_2D_pomdp_all_gif_environments[i][1] gives the simulator time stamp.
        If just_2D_pomdp_all_gif_environments[i][1] is 22_6, then it means that env is for time step 22 sec to 23 seconds and 0.1*6=0.6 seconds after 22 seconds.
        If it is 22_10, then it means that env is for 0.1*10=1 second after 22 seconds. So, essentially env at 23 seconds
        The next one would be 23_1. There is nothing of the format 23_0 (equivalent of that is 22_10). 23_1 means that env is for 0.1 second after 23 seconds.
        So, 22_6 essentially implies that this corresponds to the simulator starting from environment at index 23 in just_2D_pomdp_all_observed_environments.
        i.e. a_b corresponds to environment at index a+1 in just_2D_pomdp_all_observed_environments when b is not equal to 10
             When b = 10, it corresponds to environment at index a+2 in just_2D_pomdp_all_observed_environments
             In short, we can say a_b corresponds to environment at index floor(a+1+(0.1*b)) in just_2D_pomdp_all_observed_environments
        If, just_2D_pomdp_all_gif_environments[i][1] is 22_6, then i is 227. So, a_b is at index (10*a)+b+1
        Also, just_2D_pomdp_all_observed_environments[i] === just_2D_pomdp_all_gif_environments [(i-1)*10 + 1]
        The index for just_2D_pomdp_all_generated_trees is same as the index for just_2D_pomdp_all_observed_environments.
=#