include("environment.jl")
include("utils.jl")
include("two_d_action_space_pomdp.jl")
include("belief_tracker.jl")
include("prm.jl")
using DataStructures

Base.copy(s::cart_state) = cart_state(s.x, s.y,s.theta,s.v,s.L,s.goal)

#Returns the updated belief over humans and number of risks encountered
function simulate_pedestrians_and_generate_gif_environments_when_cart_stationary(env_right_now, current_belief,
                                                                        all_gif_environments, all_risky_scenarios, time_stamp,
                                                                        num_humans_to_care_about_while_pomdp_planning, cone_half_angle,
                                                                        lidar_range, user_defined_rng)

    number_risks = 0
    env_before_humans_simulated_for_first_half_second = deepcopy(env_right_now)

    #Simulate for 0 to 0.5 seconds
    for i in 1:5
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,0.1,user_defined_rng)
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            cone_half_angle)
        push!( all_gif_environments, (string(time_stamp)*"_"*string(i),deepcopy(env_right_now)) )
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            push!(all_risky_scenarios, (string(time_stamp)*"_"*string(i),deepcopy(env_right_now)) )
        end
    end

    #Update your belief after first 0.5 seconds
    updated_belief = update_belief_from_old_world_and_new_world(current_belief,
                                                    env_before_humans_simulated_for_first_half_second, env_right_now)

    #Simulate for 0.5 to 1 second
    env_before_humans_simulated_for_second_half_second = deepcopy(env_right_now)
    for i in 6:10
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,0.1,user_defined_rng)
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            cone_half_angle)
        push!( all_gif_environments, (string(time_stamp)*"_"*string(i),deepcopy(env_right_now)) )
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            push!(all_risky_scenarios, (string(time_stamp)*"_"*string(i),deepcopy(env_right_now)) )
        end
    end

    #Update your belief after second 0.5 seconds
    final_updated_belief = update_belief_from_old_world_and_new_world(updated_belief,
                                                    env_before_humans_simulated_for_second_half_second, env_right_now)

    return final_updated_belief, number_risks
end

#Returns the updated belief over humans and number of risks encountered
function simulate_cart_and_pedestrians_and_generate_gif_environments_when_cart_moving(env_right_now, current_belief,
                                                            all_gif_environments, all_risky_scenarios, time_stamp,
                                                            num_humans_to_care_about_while_pomdp_planning, cone_half_angle,
                                                            lidar_range, user_defined_rng, steering_angle)

    number_risks = 0

    #Simulate for 0 to 0.5 seconds
    env_before_humans_and_cart_simulated_for_first_half_second = deepcopy(env_right_now)
    initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    for i in 1:5
        extra_parameters = [env_right_now.cart.v, env_right_now.cart.L, steering_angle]
        x,y,theta = get_intermediate_points(initial_state, 0.1, extra_parameters);
        env_right_now.cart.x, env_right_now.cart.y, env_right_now.cart.theta = last(x), last(y), last(theta)
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,0.1,user_defined_rng)
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            cone_half_angle)

        push!( all_gif_environments, (string(time_stamp)*"_"*string(i),deepcopy(env_right_now)) )
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            push!(all_risky_scenarios, (string(time_stamp)*"_"*string(i),deepcopy(env_right_now)) )
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
        extra_parameters = [env_right_now.cart.v, env_right_now.cart.L, steering_angle]
        x,y,theta = get_intermediate_points(initial_state, 0.1, extra_parameters);
        env_right_now.cart.x, env_right_now.cart.y, env_right_now.cart.theta = last(x), last(y), last(theta)
        env_right_now.humans = move_human_for_one_time_step_in_actual_environment(env_right_now,0.1,user_defined_rng)
        env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
        env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                            env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                            cone_half_angle)

        push!( all_gif_environments, (string(time_stamp)*"_"*string(i),deepcopy(env_right_now)) )
        if(get_count_number_of_risks(env_right_now) != 0)
            number_risks += get_count_number_of_risks(env_right_now)
            push!(all_risky_scenarios, (string(time_stamp)*"_"*string(i),deepcopy(env_right_now)) )
        end
        initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
    end

    #Update your belief after second 0.5 seconds
    final_updated_belief = update_belief_from_old_world_and_new_world(updated_belief,
                                                    env_before_humans_and_cart_simulated_for_second_half_second, env_right_now)

    return final_updated_belief, number_risks
end

function run_one_simulation(env_right_now, user_defined_rng, m, planner)

    time_taken_by_cart = 0
    number_risks = 0
    one_time_step = 0.5
    lidar_range = 30
    num_humans_to_care_about_while_pomdp_planning = 6
    #cone_half_angle = (pi)/3.0
    #cone_half_angle = (2.0/3.0)*pi
    cone_half_angle = (1.0)*pi
    number_of_sudden_stops = 0
    cart_ran_into_boundary_wall_near_goal_flag = false
    cart_reached_goal_flag = true
    filename = "output_just_2d_action_space_pomdp_planner.txt"
    cart_throughout_path = []
    all_gif_environments = []
    all_observed_environments = []
    all_generated_beliefs = []
    all_generated_beliefs_using_complete_lidar_data = []
    all_generated_trees = []
    all_risky_scenarios = []
    all_actions = []
    reached_goal_flag = false
    current_cart_vertex_num = 1
    parent_cart_vertex_num = 0

    #Sense humans near cart before moving
    #Generate Initial Lidar Data and Belief for humans near cart
    env_right_now.complete_cart_lidar_data = get_lidar_data(env_right_now,lidar_range)
    env_right_now.cart_lidar_data = get_nearest_n_pedestrians_in_cone_pomdp_planning_1D_or_2D_action_space(env_right_now.cart,
                                                        env_right_now.complete_cart_lidar_data, num_humans_to_care_about_while_pomdp_planning,
                                                        cone_half_angle)

    initial_belief_over_complete_cart_lidar_data = update_belief([],env_right_now.goals,[],env_right_now.complete_cart_lidar_data)
    initial_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(initial_belief_over_complete_cart_lidar_data,
                                                            env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
    #initial_belief = update_belief([],env_right_now.goals,[],env_right_now.complete_cart_lidar_data)

    push!(all_gif_environments, ("-1",deepcopy(env_right_now)))
    push!(all_observed_environments,deepcopy(env_right_now))
    push!(all_generated_beliefs_using_complete_lidar_data, initial_belief_over_complete_cart_lidar_data)
    push!(all_generated_beliefs, initial_belief)
    push!(all_generated_trees, nothing)

    #Simulate for t=0 to t=1
    io = open(filename,"w")
    write_and_print( io, "Simulating for time interval - (" * string(time_taken_by_cart) * " , " * string(time_taken_by_cart+1) * ")" )
    write_and_print( io, "Current cart state = " * string(env_right_now.cart) )

    #Update human positions in environment for two time steps and cart's belief accordingly
    current_belief_over_complete_cart_lidar_data, risks_in_simulation = simulate_pedestrians_and_generate_gif_environments_when_cart_stationary(env_right_now,
                                                        initial_belief_over_complete_cart_lidar_data,all_gif_environments, all_risky_scenarios, time_taken_by_cart,
                                                        num_humans_to_care_about_while_pomdp_planning,cone_half_angle, lidar_range,
                                                        MersenneTwister( Int64( floor( 100*rand(user_defined_rng) ) ) ) )
    current_belief =  get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                            env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
    number_risks += risks_in_simulation
    time_taken_by_cart += 1
    push!(all_observed_environments,deepcopy(env_right_now))
    push!(all_generated_beliefs_using_complete_lidar_data, current_belief_over_complete_cart_lidar_data)
    push!(all_generated_beliefs, current_belief)
    #push!(all_generated_trees, nothing)

    write_and_print( io, "Modified cart state = " * string(env_right_now.cart) )
    close(io)

    #Start Simulating for t>1
    while(!is_within_range(location(env_right_now.cart.x,env_right_now.cart.y), env_right_now.cart.goal, 1.0))
        display_env(env_right_now, (current_cart_vertex_num, parent_cart_vertex_num))
        io = open(filename,"a")
        if( (env_right_now.cart.x<=100.0 && env_right_now.cart.y<=100.0 && env_right_now.cart.x>=0.0 && env_right_now.cart.y>=0.0) )


            write_and_print( io, "Simulating for time interval - (" * string(time_taken_by_cart) * " , " * string(time_taken_by_cart+1) * ")" )
            write_and_print( io, "Current cart state = " * string(env_right_now.cart) )

            #Solve POMDP to get the best action
            # m = golfcart_2D_action_space_pomdp()
            @show(m.world.cart.x)
            b = POMDP_2D_action_space_state_distribution(m.world,current_belief,current_cart_vertex_num,parent_cart_vertex_num)
            a, info = action_info(planner, b)
            push!( all_actions, (time_taken_by_cart,a) )
            write_and_print( io, "Action chosen by 2D action space POMDP planner: " * string(a) )

            if(env_right_now.cart.v!=0 && a == 3)
                number_of_sudden_stops += 1
            end

            push!(all_generated_trees, deepcopy(info))

            if(a==3)
                env_right_now.cart.v = 0.0
            else
                env_right_now.cart.v = clamp( get_prop(env_right_now.graph,current_cart_vertex_num,a,:weight) ,0,m.max_cart_speed)
                parent_cart_vertex_num = current_cart_vertex_num
                current_cart_vertex_num = a
            end

            if(env_right_now.cart.v != 0.0)
                #That means the cart is not stationary and we now have to simulate both cart and the pedestrians.
                env_right_now.cart.theta = get_heading_angle( get_prop(env_right_now.graph,a,:x), get_prop(env_right_now.graph,a,:y),
                                                                                        env_right_now.cart.x, env_right_now.cart.y)
                steering_angle = 0.0
                current_belief_over_complete_cart_lidar_data, risks_in_simulation = simulate_cart_and_pedestrians_and_generate_gif_environments_when_cart_moving(
                                                                    env_right_now,current_belief_over_complete_cart_lidar_data, all_gif_environments,
                                                                    all_risky_scenarios, time_taken_by_cart,num_humans_to_care_about_while_pomdp_planning,
                                                                    cone_half_angle, lidar_range,MersenneTwister( Int64( floor( 100*rand(user_defined_rng) ) ) ),
                                                                    steering_angle)
                current_belief =  get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                                    env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
                println(env_right_now.cart)
                env_right_now.cart.x = get_prop(env_right_now.graph, a, :x)
                env_right_now.cart.y = get_prop(env_right_now.graph, a, :y)
                number_risks += risks_in_simulation
                println(env_right_now.cart)

            else
                #That means the cart is stationary and we now just have to simulate the pedestrians.
                current_belief_over_complete_cart_lidar_data, risks_in_simulation = simulate_pedestrians_and_generate_gif_environments_when_cart_stationary(
                                                                    env_right_now,current_belief_over_complete_cart_lidar_data,all_gif_environments,
                                                                    all_risky_scenarios, time_taken_by_cart,num_humans_to_care_about_while_pomdp_planning,
                                                                    cone_half_angle, lidar_range,MersenneTwister( Int64( floor( 100*rand(user_defined_rng) ) ) ) )
                current_belief =  get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                                    env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)


                number_risks += risks_in_simulation
            end

            push!(all_observed_environments,deepcopy(env_right_now))
            push!(all_generated_beliefs_using_complete_lidar_data, current_belief_over_complete_cart_lidar_data)
            push!(all_generated_beliefs, current_belief)

            write_and_print( io, "Modified cart state = " * string(env_right_now.cart) )
            write_and_print( io, "************************************************************************" )
            push!(cart_throughout_path,(copy(env_right_now.cart)))

        else
            write_and_print( io, "Cart ran into Boundary Walls")
            cart_ran_into_boundary_wall_near_goal_flag = true
            cart_reached_goal_flag = false
            break
        end
        close(io)
        time_taken_by_cart += 1
        if(time_taken_by_cart>100)
            cart_reached_goal_flag = false
            break
        end
    end

    io = open(filename,"a")
    if(cart_reached_goal_flag == true)
        write_and_print( io, "Goal Reached! :D" )
        write_and_print( io, "Time Taken by cart to reach goal : " * string(time_taken_by_cart) )
    else
        if(cart_ran_into_boundary_wall_near_goal_flag == true)
            write_and_print( io, "Cart ran into a wall :(" )
            write_and_print( io, "Time Taken by cart to run into a wall : " * string(time_taken_by_cart) )
        else
            write_and_print( io, "Cart ran out of time :(" )
            write_and_print( io, "Time Taken by cart when it didn't reach the goal : " * string(time_taken_by_cart) )
        end
    end
    write_and_print( io, "Number of risky scenarios encountered by the cart : " * string(number_risks) )
    write_and_print( io, "Number of sudden stops taken by the cart : " * string(number_of_sudden_stops) )
    close(io)

    return all_gif_environments, all_observed_environments, all_generated_beliefs_using_complete_lidar_data, all_generated_beliefs,
                all_generated_trees,all_risky_scenarios, all_actions, number_risks, number_of_sudden_stops, time_taken_by_cart,
                cart_reached_goal_flag
end

function get_available_neighbors(m::POMDP_Planner_2D_action_space,b)
    pomdp_state = first(particles(b))
    actions::Array{Int64,1} = filter(x->x!=pomdp_state.parent_vertex_num, neighbors(m.world.graph,pomdp_state.curr_vertex_num))
    #@show(pomdp_state.curr_vertex_num, pomdp_state.parent_vertex_num, actions)
    return actions
end

run_simulation_flag = true
if(run_simulation_flag)
    gr()
    #env = generate_environment_no_obstacle(MersenneTwister(1))
    env = generate_environment_small_circular_obstacles(100,MersenneTwister(15))
    #env = generate_environment_large_circular_obstacles(300,MersenneTwister(15))
    env.graph = generate_prm_vertices(1000,env)
    distance_dict_prm = generate_prm_edges(env,10)
    env_right_now = deepcopy(env)

    #Find max edge weight. This will have to be removed later when PRM is more dense and better.
    max_edge_weight = get_max_weight_edge_from_prm(env.graph)
    cart_max_velocity = ceil(max_edge_weight)

    #Create POMDP for env_right_now
    #POMDP_Planner_2D_action_space <: POMDPs.POMDP{POMDP_state_2D_action_space,Int,Array{location,1}}
    # discount_factor::Float64; pedestrian_distance_threshold::Float64; pedestrian_collision_penalty::Float64;
    # obstacle_distance_threshold::Float64; obstacle_collision_penalty::Float64; goal_reward_distance_threshold::Float64;
    # cart_goal_reached_distance_threshold::Float64; goal_reward::Float64; max_cart_speed::Float64; world::experiment_environment

    golfcart_2D_action_space_pomdp = POMDP_Planner_2D_action_space(0.99,2.0,-100.0,1.0,-100.0,0.0,1.0,100.0,cart_max_velocity,env_right_now)
    discount(p::POMDP_Planner_2D_action_space) = p.discount_factor
    isterminal(::POMDP_Planner_2D_action_space, s::POMDP_state_2D_action_space) = is_terminal_state_pomdp_planning(s,location(-100.0,-100.0));
    #actions(::POMDP_Planner_2D_action_space) = [(-10.0,-10.0),(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-1.0),(0.0,0.0),(0.0,1.0),(pi/12,0.0),(pi/6,0.0),(pi/4,0.0)]
    #actions(m::POMDP_Planner_2D_action_space) = [(-10.0,-10.0),(-pi/4,0.0),(-pi/6,0.0),(-pi/12,0.0),(0.0,-1.0),(0.0,0.0),(0.0,1.0),(pi/12,0.0),(pi/6,0.0),(pi/4,0.0)]
    actions(m::POMDP_Planner_2D_action_space,b) = get_available_neighbors(m,b)

    # solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning_2D_action_space),max_depth=100,
    #                         final_value=reward_to_be_awarded_at_max_depth_in_lower_bound_policy_rollout),
    #                         calculate_upper_bound_value_pomdp_planning_2D_action_space, check_terminal=true),K=100,D=100,T_max=0.5, tree_in_info=true, default_action=3)
    solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound_policy_pomdp_planning_2D_action_space(golfcart_2D_action_space_pomdp, b)),
                            max_depth=100),calculate_upper_bound_value_pomdp_planning_2D_action_space, check_terminal=true),
                            K=100,D=100,T_max=0.5, tree_in_info=true)

    planner = POMDPs.solve(solver, golfcart_2D_action_space_pomdp);

    display_env(golfcart_2D_action_space_pomdp.world)
    just_2D_pomdp_all_gif_environments, just_2D_pomdp_all_observed_environments, just_2D_pomdp_all_generated_beliefs_using_complete_lidar_data,
            just_2D_pomdp_all_generated_beliefs, just_2D_pomdp_all_generated_trees, just_2D_pomdp_all_risky_scenarios,just_2D_pomdp_all_actions,
            just_2D_pomdp_number_risks,just_2D_pomdp_number_of_sudden_stops,just_2D_pomdp_time_taken_by_cart,
            just_2D_pomdp_cart_reached_goal_flag = run_one_simulation(env_right_now, MersenneTwister(111),
                                                                                        golfcart_2D_action_space_pomdp, planner)

    anim = @animate for i ∈ 1:length(just_2D_pomdp_all_observed_environments)
        display_env(just_2D_pomdp_all_observed_environments[i]);
        #savefig("./plots_just_2d_action_space_pomdp_planner/plot_"*string(i)*".png")
    end

    gif(anim, "just_2D_action_space_pomdp_planner_run.gif", fps = 2)
end


# anim = @animate for i ∈ 1:length(just_2D_pomdp_all_gif_environments)
#     display_env(just_2D_pomdp_all_gif_environments[i][2]);
#     #savefig("./plots_just_2d_action_space_pomdp_planner/plot_"*all_gif_environments[i][1]*".png")
# end
# gif(anim, "just_2D_action_space_pomdp_planner_run.gif", fps = 20)

#inchrome(D3Tree(just_2D_pomdp_all_generated_trees[9][:tree]))
