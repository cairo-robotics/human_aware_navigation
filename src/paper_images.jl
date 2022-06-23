function display_all_cart_paths_in_env(env::experiment_environment, all_cart_paths_dict, traj_color)

    #Plot Boundaries
    p = plot([0.0],[0.0],legend=false,grid=false, axis=nothing)
    default(dpi=72)
    plot!([env.length], [env.breadth],legend=false,xaxis=false, yaxis=false)

    #Plot Obstacles
    for i in 1: length(env.obstacles)
        scatter!([env.obstacles[i].x], [env.obstacles[i].y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    annotate!(env.cart_start_location.x+2, env.cart_start_location.y, text("START", :saddlebrown, :center,:bold, 30))
    annotate!(env.cart.goal.x, env.cart.goal.y, text("GOAL", :saddlebrown, :right, 30))

    for k in keys(all_cart_paths_dict)
        cart_path = all_cart_paths_dict[k]
        all_time_steps = collect(keys(cart_path))
        final_cart_pos = cart_path[all_time_steps[end]]
        if(!is_within_range_check_with_points(env.cart.goal.x, env.cart.goal.y, final_cart_pos.x, final_cart_pos.y, 1.0))
            continue
        else
            x_points = []
            y_points = []
            for time_step in all_time_steps
                push!(x_points, cart_path[time_step].x)
                push!(y_points, cart_path[time_step].y)
            end
        end
        plot!(x_points,y_points, color=traj_color)
    end
    plot!(size=(plot_size,plot_size))
    display(p)
    return p
end

#=
display_all_cart_paths_in_env(e , a["cart_path_1D_POMDP_planner_dict"], "orange")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce1_astar.png")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce2_astar.png")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce3_astar.png")

display_all_cart_paths_in_env(e , a["cart_path_2D_POMDP_planner_dict_fmm"], "olive")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce1_fmm.png")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce2_fmm.png")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce3_fmm.png")

display_all_cart_paths_in_env(e , a["cart_path_2D_POMDP_planner_dict_prm"], "steelblue")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce1_prm.png")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce2_prm.png")
savefig("/home/himanshu/Documents/Projects/human_aware_navigation/media/sce3_prm.png")
=#

#Function to display the environment

function generate_small_environment_for_paper_image()

    world_length = 10.0
    world_breadth = 10.0
    g1 = location(0.0,0.0)
    g2 = location(0.0,world_breadth)
    g3 = location(world_length,world_breadth)
    g4 = location(world_length,0.0)
    cart_goal = location(world_length,7.50)
    all_goals_list = [g1,g2,g3,g4]

    o1 = obstacle_location(6.50,4.50,1.5)
    all_obstacle_list = [o1]
    # o1 = obstacle_location(50.0,50.0,20.0)
    # o2 = obstacle_location(50.0,20.0,20.0)
    # o3 = obstacle_location(80.0,50.0,20.0)
    # o4 = obstacle_location(80.0,20.0,20.0)
    # all_obstacle_list = [o1,o2,o3,o4]

    golfcart = cart_state(1.0,2.5,0.0,2.0,1.0,cart_goal)
    initial_cart_lidar_data = Array{human_state,1}()
    initial_complete_cart_lidar_data = Array{human_state,1}()

    number_of_humans = 4
    max_num_humans = number_of_humans
    human_state_start_list = Array{human_state,1}()
    human1 = human_state(4.3,3,1.0,g1,1)
    # human2 = human_state(5,9,1.0,g2,2)
    human3 = human_state(2,7,1.0,g3,3)
    human4 = human_state(7.3,2,1.0,g1,4)
    push!(human_state_start_list,human1)
    # push!(human_state_start_list,human2)
    push!(human_state_start_list,human3)
    push!(human_state_start_list,human4)
    world = experiment_environment(world_length,world_breadth,max_num_humans,number_of_humans,
                    all_goals_list,human_state_start_list,all_obstacle_list,golfcart,initial_cart_lidar_data,
                    initial_complete_cart_lidar_data,Float64[],location(golfcart.x, golfcart.y))


    prob_dist_tuple = [(human1,human_probability_over_goals([1.0,0.0,0.0,0.0])) , (human2,human_probability_over_goals([0.0,1.0,0.0,0.0])),
                            (human3,human_probability_over_goals([0.0,0.0,1.0,0.0])), (human4,human_probability_over_goals([1.0,0.0,0.0,0.0])) ]
    prob_dist_tuple = [(human3,human_probability_over_goals([0.0,0.0,1.0,0.0])), (human4,human_probability_over_goals([1.0,0.0,0.0,0.0])) ]
    world.cart_hybrid_astar_path = hybrid_a_star_search(world.cart.x, world.cart.y,world.cart.theta, world.cart.goal.x, world.cart.goal.y, world,
                                            prob_dist_tuple,100.0);

    return world
end

function display_env_with_all_humans(env::experiment_environment, sce4_flag = false)

    #Plot Boundaries
    default(dpi=72)
    # p = plot([0.0],[0.0],legend=false,grid=false, axis=nothing)
    p = plot([0.0],[0.0],legend=false,grid=false)
    # plot!([env.length], [env.breadth],legend=false, xaxis=false, yaxis=false)
    plot!([env.length], [env.breadth],legend=false)

    #Plot Golfcart
    scatter!([env.cart.x], [env.cart.y], shape=:circle, color="blue", msize= 0.3*plot_size*cart_size/env.length)
    quiver!([env.cart.x],[env.cart.y],quiver=([cos(env.cart.theta)],[sin(env.cart.theta)]), color="blue")

    #Plot Obstacles
    for i in 1: length(env.obstacles)
        scatter!([env.obstacles[i].x], [env.obstacles[i].y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    #Plot all the Humans
    for i in 1: length(env.humans)
        if(sce4_flag)
            if( (env.humans[i].x > env.obstacles[1].x) && ( env.humans[i].y < env.obstacles[1].y + env.obstacles[1].r ) )
                continue
            else
                for obs in env.obstacles
                    if( is_within_range_check_with_points( env.humans[i].x, env.humans[i].y, obs.x, obs.y, obs.r+1))
                        continue
                    else
                        scatter!([env.humans[i].x], [env.humans[i].y],color="red",msize=0.5*plot_size/env.length)
                    end
                end
            end
        else
            display_flag = true
            for obs in env.obstacles
                if( is_within_range_check_with_points( env.humans[i].x, env.humans[i].y, obs.x, obs.y, obs.r+1))
                    display_flag = false
                    break
                end
            end
            if(display_flag)
                    scatter!([env.humans[i].x], [env.humans[i].y],color="red",msize=0.5*plot_size/env.length)
            end
        end
    end


    # annotate!(env.cart_start_location.x+2, env.cart_start_location.y, text("START", :green, :center, 30))
    annotate!(env.cart.goal.x, env.cart.goal.y, text("GOAL", :green, :right, 30))
    plot!(size=(plot_size,plot_size))
    display(p)
end

function display_mean_imporvement_time_plots()

    #=
    a["mean_planning_time_improvement_fmm"]
    a["mean_planning_time_improvement_prm"]

    =#
    x_axis_points = [100,200,300,400]
    mit_fmm_sce1 = [12.98,18.5,22.21,25.82]
    vit_fmm_sce1 = [0.70, 0.99, 1.14, 2.1  ]
    mit_prm_sce1 = [12.96,18.57,22.64,24.54]
    vit_prm_sce1 = [0.67, 0.97, 1.17, 2.12 ]

    mit_fmm_sce2 = [12.92,17.2,20.72,23.81]
    vit_fmm_sce2 = [0.96, 1.01, 1.6, 2.12]
    mit_prm_sce2 = [13.05,17.23,18.93,21.68]
    vit_prm_sce2 = [0.89, 0.99, 1.68, 2.18]

    mit_fmm_sce3 = [8.89,15.34,19.37,26.81]
    vit_fmm_sce3 = [0.88,1.24,1.48,2.39]
    mit_prm_sce3 = [9.8,15.64,19.85,26.86]
    vit_prm_sce3 = [0.91,1.32,1.59,2.74]

    mit_NHV_sce1 = [17.08, 28.75, 33.06, 29.34]
    vit_NHV_sce1 = [1.03, 1.46, 2.21, 2.51]

    y_matrix = zeros(4,7)
    y_matrix[:,1] = mit_fmm_sce1
    y_matrix[:,2] = mit_prm_sce1
    y_matrix[:,3] = mit_fmm_sce2
    y_matrix[:,4] = mit_prm_sce2
    y_matrix[:,5] = mit_fmm_sce3
    y_matrix[:,6] = mit_prm_sce3
    y_matrix[:,7] = mit_NHV_sce1

    ribbon_matrix = zeros(4,7)
    ribbon_matrix[:,1] = vit_fmm_sce1
    ribbon_matrix[:,2] = vit_prm_sce1
    ribbon_matrix[:,3] = vit_fmm_sce2
    ribbon_matrix[:,4] = vit_prm_sce2
    ribbon_matrix[:,5] = vit_fmm_sce3
    ribbon_matrix[:,6] = vit_prm_sce3
    ribbon_matrix[:,7] = vit_NHV_sce1


    p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM (Scenario 1)" "2D-PRM (Scenario 1)" "2D-FMM (Scenario 2)" "2D-PRM (Scenario 2)" "2D-FMM (Scenario 3)" "2D-PRM (Scenario 3)" "2D-NHV (Scenario 1)"],
                                        lw = 3, color = ["green" "red" "blue" "maroon" "orange" "olive" "purple"],
                                        xlabel = "Number of pedestrians in the environment" , ylabel = "Mean Improvement time over 1D-A* (in sec)",
                                        ribbon=ribbon_matrix,fillalpha=.1)

    # p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM (Scenario 1)" "2D-PRM (Scenario 1)" "2D-FMM (Scenario 2)" "2D-PRM (Scenario 2)" "2D-FMM (Scenario 3)" "2D-PRM (Scenario 3)" "2D-NHV (Scenario 1)"],
    #                                     lw = 3, color = ["green" "red" "blue" "maroon" "orange" "olive" "purple"],
    #                                     xlabel = "Number of pedestrians in the environment" , ylabel = "Mean Improvement time over 1D-A* (in sec)",
    #                                     yerror=ribbon_matrix)
    # plot!(x_axis_points, mit_prm_sce1, legend=:bottomleft,grid=false,label="F2", color = "cyan")
    # plot!(x_axis_points, mit_fmm_sce2)
    # plot!(x_axis_points, mit_prm_sce2)
    # plot!(x_axis_points, mit_fmm_sce3)
    # plot!(x_axis_points, mit_prm_sce3)
    default(dpi=300)
    display(p)
end

function get_hybrid_astar_path_points(env)
    current_x, current_y, current_theta = env.cart.x, env.cart.y, env.cart.theta
    path_x, path_y,path_theta = [env.cart.x],[env.cart.y],[env.cart.theta]
    arc_length = 1.0
    time_interval = 1.0
    num_time_intervals = 10
    for delta_angle in env.cart_hybrid_astar_path
        final_orientation_angle = wrap_between_0_and_2Pi(current_theta+delta_angle)
        for j in 1:10
            if(delta_angle == 0.0)
                new_theta = current_theta
                new_x = current_x + arc_length*cos(current_theta)*(1/num_time_intervals)
                new_y = current_y + arc_length*sin(current_theta)*(1/num_time_intervals)
            else
                new_theta = current_theta + (delta_angle * (1/num_time_intervals))
                new_theta = wrap_between_0_and_2Pi(new_theta)
                new_x = current_x + arc_length*cos(final_orientation_angle)*(1/num_time_intervals)
                new_y = current_y + arc_length*sin(final_orientation_angle)*(1/num_time_intervals)
            end
            push!(path_x,new_x)
            push!(path_y,new_y)
            push!(path_theta,new_theta)
            current_x, current_y,current_theta = new_x,new_y,new_theta
        end
    end
    # plot!(path_x,path_y,color="grey")
    return (path_x, path_y)
end

function get_new_human_position_actual_environemnt_for_paper_image(human, world, time_step, user_defined_rng)

    rand_num = (rand(user_defined_rng) - 0.5)*0
    #rand_num = 0.0
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

    new_x = clamp(new_x,0,world.length)
    new_y = clamp(new_y,0,world.breadth)
    #@show(new_x,new_y)
    new_human_state = human_state(new_x, new_y, human.v, human.goal,human.id)
    return new_human_state
end

function display_that_cool_image()

    env = generate_small_environment_for_paper_image()
    possible_delta_theta = [ -2*pi/9, -pi/9, 0.0, pi/9, 2*pi/9 ]

    default(dpi=300)
    p = plot([0.0],[0.0],legend=false,grid=false, axis=nothing)
    # p = plot([0.0],[0.0],legend=false,grid=false)
    # plot!([env.length], [env.breadth],legend=false)
    plot!([env.length], [env.breadth],legend=false, xaxis=false, yaxis=false)

    #Plot Golfcart
    annotate!(env.cart.x, env.cart.y, text("V(t)", :white, :center, 10))
    scatter!([env.cart.x], [env.cart.y], shape=:circle, color="darkgreen", msize= 0.35*plot_size*cart_size/env.length)
    quiver!([env.cart.x],[env.cart.y],quiver=([cos(env.cart.theta)],[sin(env.cart.theta)]), color="black")

    #Plot Obstacles
    for i in 1: length(env.obstacles)
        scatter!([env.obstacles[i].x], [env.obstacles[i].y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    #Plot all the Humans
    for i in 1: length(env.humans)
        annotate!(env.humans[i].x, env.humans[i].y, text("H(t)", :white, :center, 10))
        human_heading_angle = get_heading_angle(env.humans[i].goal.x, env.humans[i].goal.y, env.humans[i].x, env.humans[i].y)
        quiver!([env.humans[i].x], [env.humans[i].y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="black")
        scatter!([env.humans[i].x], [env.humans[i].y],color="darkgoldenrod4",msize=0.35*plot_size/env.length)
        new_human_state = get_new_human_position_actual_environemnt_for_paper_image( env.humans[i], env, 1.0, MersenneTwister(100))
        annotate!(new_human_state.x, new_human_state.y, text("H(t+1)", :white, :center, 10))
        quiver!([new_human_state.x], [new_human_state.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="black")
        scatter!([new_human_state.x], [new_human_state.y],color="goldenrod3",msize=0.35*plot_size/env.length)
        if(i!=1)
            new_new_human_state = get_new_human_position_actual_environemnt_for_paper_image( new_human_state, env, 1.0, MersenneTwister(100))
            annotate!(new_new_human_state.x, new_new_human_state.y, text("H(t+2)", :white, :center, 10))
            quiver!([new_new_human_state.x], [new_new_human_state.y],quiver=([0.5*cos(human_heading_angle)],[0.5*sin(human_heading_angle)]), color="black")
            scatter!([new_new_human_state.x], [new_new_human_state.y],color="darkgoldenrod1",msize=0.35*plot_size/env.length)
        end
    end
    #Plot the Hybrid A* path if it exists
    if(length(env.cart_hybrid_astar_path)!=0)
        px,py = get_hybrid_astar_path_points(env)
        # plot!(px,py,color="grey")
    end

    depth_one_cart_positions = []
    bad_actions = [-pi/9, 0.0, pi/9]
    for delta_angle in possible_delta_theta
        cart_path = update_cart_position_pomdp_planning_2D_action_space(env.cart, delta_angle, 2.0, env.length, env.breadth,1.0, 10)
        new_pos = cart_path[end]
        # plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="darkgreen")
        # annotate!(new_pos[1], new_pos[2], text("V", :green, :center, 30))
        if(delta_angle in bad_actions)
            annotate!(new_pos[1], new_pos[2], text("V(t+1)", :black, :center, 10))
            plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="red")
            scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="red", msize= 0.35*plot_size*cart_size/env.length)
            # quiver!([new_pos[1]],[new_pos[2]],quiver=([cos(new_pos[3])],[sin(new_pos[3])]), color="red")
        else
            plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="lightgrey")
            annotate!(new_pos[1], new_pos[2], text("V(t+1)", :black, :center, 10))
            scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="green2", msize= 0.35*plot_size*cart_size/env.length)
            quiver!([new_pos[1]],[new_pos[2]],quiver=([cos(new_pos[3])],[sin(new_pos[3])]), color="black")
            push!(depth_one_cart_positions,new_pos)
        end
    end

    depth_two_cart_positions = []
    bad_actions = [1,2,5,6,7]
    start_bad_index = 1
    for pos in depth_one_cart_positions
        temp_cs = cart_state(pos[1], pos[2], pos[3], env.cart.v, env.cart.L, env.cart.goal)
        for delta_angle in possible_delta_theta
            cart_path = update_cart_position_pomdp_planning_2D_action_space(temp_cs, delta_angle, 2.0, env.length, env.breadth,1.0, 10)
            new_pos = cart_path[end]
            if(start_bad_index in bad_actions)
                annotate!(new_pos[1], new_pos[2], text("V(t+2)", :black, :center, 10))
                plot!([temp_cs.x, new_pos[1]], [temp_cs.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="red")
                scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="red", msize= 0.35*plot_size*cart_size/env.length)
                # quiver!([new_pos[1]],[new_pos[2]],quiver=([cos(new_pos[3])],[sin(new_pos[3])]), color="red")
            else
                push!(depth_two_cart_positions,new_pos)
                plot!([temp_cs.x, new_pos[1]], [temp_cs.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="lightgrey")
                annotate!(new_pos[1], new_pos[2], text("V(t+2)", :black, :center, 10))
                scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="darkseagreen1", msize= 0.35*plot_size*cart_size/env.length)
                quiver!([new_pos[1]],[new_pos[2]],quiver=([cos(new_pos[3])],[sin(new_pos[3])]), color="black")
            end
            start_bad_index +=1
        end
    end



    for pos in depth_two_cart_positions
        temp_env = deepcopy(env)
        temp_env.cart = cart_state(pos[1], pos[2], pos[3], env.cart.v, env.cart.L, env.cart.goal)
        temp_env.cart_hybrid_astar_path = hybrid_a_star_search(temp_env.cart.x, temp_env.cart.y,temp_env.cart.theta, temp_env.cart.goal.x,
                                            temp_env.cart.goal.y, temp_env, Array{Tuple{human_state,human_probability_over_goals},1}(),100.0);
        px,py = get_hybrid_astar_path_points(temp_env)
        plot!(px,py,color="brown")
    end
    annotate!(env.cart.goal.x, env.cart.goal.y, text("GOAL", :saddlebrown, :right, 30))
    plot!(size=(plot_size,plot_size))
    display(p)
end

function sce1_display_mean_imporvement_time_plots()

    #=
    a["mean_planning_time_improvement_fmm"]
    a["mean_planning_time_improvement_prm"]

    =#

    x_axis_points = [100,200,300,400]
    mit_fmm_sce1 = [12.98,18.5,22.21,25.82]
    vit_fmm_sce1 = [0.70, 0.99, 1.14, 2.1  ]
    mit_prm_sce1 = [12.96,18.57,22.64,24.54]
    vit_prm_sce1 = [0.67, 0.97, 1.17, 2.12 ]

    mit_NHV_sce1 = [17.08, 28.75, 33.06, 29.34]
    vit_NHV_sce1 = [1.03, 1.46, 2.21, 2.51]

    y_matrix = zeros(4,3)
    y_matrix[:,1] = mit_fmm_sce1
    y_matrix[:,2] = mit_prm_sce1
    y_matrix[:,3] = mit_NHV_sce1

    ribbon_matrix = zeros(4,7)
    ribbon_matrix[:,1] = vit_fmm_sce1
    ribbon_matrix[:,2] = vit_prm_sce1
    ribbon_matrix[:,3] = vit_NHV_sce1


    p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM" "2D-PRM" "2D-NHV"],
                                        lw = 3, color = ["green" "blue" "purple"],
                                        xlabel = "Number of pedestrians in the environment" , ylabel = "Mean Improvement time over 1D-A* (in sec)",
                                        ribbon=ribbon_matrix,fillalpha=.1)

    # p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM" "2D-PRM" "2D-NHV"],
    #                                     lw = 3, color = ["green" "red" "blue"],
    #                                     xlabel = "Number of pedestrians in the environment" , ylabel = "Mean Improvement time over 1D-A* (in sec)",
    #                                     yerror=ribbon_matrix)
    default(dpi=300)
    display(p)
end

function sce2_display_mean_imporvement_time_plots()

    x_axis_points = [100,200,300,400]
    mit_fmm_sce2 = [12.92,17.2,20.72,23.81]
    vit_fmm_sce2 = [0.96, 1.01, 1.6, 2.12]
    mit_prm_sce2 = [13.05,17.23,18.93,21.68]
    vit_prm_sce2 = [0.89, 0.99, 1.68, 2.18]

    y_matrix = zeros(4,2)
    y_matrix[:,1] = mit_fmm_sce2
    y_matrix[:,2] = mit_prm_sce2

    ribbon_matrix = zeros(4,2)
    ribbon_matrix[:,1] = vit_fmm_sce2
    ribbon_matrix[:,2] = vit_prm_sce2

    p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM" "2D-PRM"],
                                        lw = 3, color = ["green" "blue" ],
                                        xlabel = "Number of pedestrians in the environment" , ylabel = "Mean Improvement time over 1D-A* (in sec)",
                                        ribbon=ribbon_matrix,fillalpha=.1)

    # p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM" "2D-PRM" "2D-NHV"],
    #                                     lw = 3, color = ["green" "red" "blue"],
    #                                     xlabel = "Number of pedestrians in the environment" , ylabel = "Mean Improvement time over 1D-A* (in sec)",
    #                                     yerror=ribbon_matrix)
    default(dpi=300)
    display(p)
end

function sce3_display_mean_imporvement_time_plots()

    x_axis_points = [100,200,300,400]
    mit_fmm_sce3 = [4.6,15.34,19.37,26.81]
    vit_fmm_sce3 = [0,1.24,1.48,2.39]
    mit_prm_sce3 = [9.2,15.64,19.85,26.86]
    vit_prm_sce3 = [0,1.32,1.59,2.74]

    y_matrix = zeros(4,2)
    y_matrix[:,1] = mit_fmm_sce3
    y_matrix[:,2] = mit_prm_sce3

    ribbon_matrix = zeros(4,2)
    ribbon_matrix[:,1] = vit_fmm_sce3
    ribbon_matrix[:,2] = vit_prm_sce3

    p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM" "2D-PRM"],
                                        lw = 3, color = ["olive" "lightblue" ],
                                        xlabel = "Number of pedestrians in the environment" , ylabel = "Mean Improvement time over 1D-A* (in sec)",
                                        ribbon=ribbon_matrix,fillalpha=.5)

    # p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM" "2D-PRM" "2D-NHV"],
    #                                     lw = 3, color = ["green" "red" "blue"],
    #                                     xlabel = "Number of pedestrians in the environment" , ylabel = "Mean Improvement time over 1D-A* (in sec)",
    #                                     yerror=ribbon_matrix)
    default(dpi=300)
    display(p)
end


#=
c = 0
d = 0
for k in keys(a["total_time_taken_2D_POMDP_planner_dict_fmm"])
   println( a["total_time_taken_2D_POMDP_planner_dict_fmm"][k] , " ", a["total_time_taken_2D_POMDP_planner_dict_prm"][k], " ", a["total_time_taken_1D_POMDP_planner_dict"][k])
   if( a["total_time_taken_2D_POMDP_planner_dict_fmm"][k] <= a["total_time_taken_1D_POMDP_planner_dict"][k] )
       c+=1
   end
   if( a["total_time_taken_2D_POMDP_planner_dict_prm"][k] <= a["total_time_taken_1D_POMDP_planner_dict"][k] )
       d+=1
   end
end

a["mean_time_taken_1D_POMDP_planner"] , sqrt(a["variance_time_taken_1D_POMDP_planner"]/100)
a["mean_sudden_stops_1D_POMDP_planner"] , sqrt(a["variance_sudden_stops_1D_POMDP_planner"]/100)
a["mean_time_taken_2D_POMDP_planner_fmm"] , sqrt(a["variance_time_taken_2D_POMDP_planner_fmm"]/100)
c
a["mean_sudden_stops_2D_POMDP_planner_fmm"] , sqrt(a["variance_sudden_stops_2D_POMDP_planner_fmm"]/100)
a["mean_time_taken_2D_POMDP_planner_prm"] , a["variance_time_taken_2D_POMDP_planner_fmm"]
d
a["mean_sudden_stops_2D_POMDP_planner_prm"] , sqrt(a["variance_sudden_stops_2D_POMDP_planner_prm"]/100)


a["mean_planning_time_improvement_fmm"]
sqrt(a["variance_planning_time_improvement_fmm"])/10
a["mean_planning_time_improvement_prm"]
sqrt(a["variance_planning_time_improvement_prm"])/10

=#


#=
c = 0
for k in keys(a["total_time_taken_2D_POMDP_planner_dict"])
   println( a["total_time_taken_2D_POMDP_planner_dict"][k] , " ", a["total_time_taken_1D_POMDP_planner_dict"][k])
   if( a["total_time_taken_2D_POMDP_planner_dict"][k] <= a["total_time_taken_1D_POMDP_planner_dict"][k] )
       c+=1
   end
end

a["mean_time_taken_1D_POMDP_planner"]
a["mean_sudden_stops_1D_POMDP_planner"]
a["mean_time_taken_2D_POMDP_planner"]
c
a["mean_sudden_stops_2D_POMDP_planner"]
a["mean_planning_time_improvement"]
sqrt(a["variance_planning_time_improvement"])/10
=#




function display_that_cool_image_part2()

    env = generate_small_environment_for_paper_image()
    possible_delta_theta = [ -2*pi/9, -pi/9, 0.0, pi/9, 2*pi/9 ]

    default(dpi=300)
    p = plot([0.0],[0.0],legend=false,grid=false, axis=nothing)
    # p = plot([0.0],[0.0],legend=false,grid=false)
    # plot!([env.length], [env.breadth],legend=false)
    plot!([env.length], [env.breadth],legend=false, xaxis=false, yaxis=false)

    #Plot Golfcart
    annotate!(env.cart.x, env.cart.y, text("V", :white, :center, 20))
    scatter!([env.cart.x], [env.cart.y], shape=:circle, color="darkgreen", msize= 0.35*plot_size*cart_size/env.length)
    quiver!([env.cart.x],[env.cart.y],quiver=([0.5*cos(env.cart.theta)],[0.5*sin(env.cart.theta)]), color="black")

    #Plot Obstacles
    for i in 1: length(env.obstacles)
        scatter!([env.obstacles[i].x], [env.obstacles[i].y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    #Plot all the Humans
    for i in 1: length(env.humans)
        annotate!(env.humans[i].x, env.humans[i].y, text("H", :white, :center, 20))
        human_heading_angle = get_heading_angle(env.humans[i].goal.x, env.humans[i].goal.y, env.humans[i].x, env.humans[i].y)
        quiver!([env.humans[i].x], [env.humans[i].y],quiver=([0.7*cos(human_heading_angle)],[0.7*sin(human_heading_angle)]), color="black")
        scatter!([env.humans[i].x], [env.humans[i].y],color="darkgoldenrod4",msize=0.35*plot_size/env.length)
        new_human_state = get_new_human_position_actual_environemnt_for_paper_image( env.humans[i], env, 1.0, MersenneTwister(100))
        # annotate!(new_human_state.x, new_human_state.y, text("H", :white, :center, 20))
        quiver!([new_human_state.x], [new_human_state.y],quiver=([0.7*cos(human_heading_angle)],[0.7*sin(human_heading_angle)]), color="black")
        scatter!([new_human_state.x], [new_human_state.y],color="goldenrod3",msize=0.35*plot_size/env.length)
        if(i!=1)
            new_new_human_state = get_new_human_position_actual_environemnt_for_paper_image( new_human_state, env, 1.0, MersenneTwister(100))
            # annotate!(new_new_human_state.x, new_new_human_state.y, text("H", :white, :center,20))
            quiver!([new_new_human_state.x], [new_new_human_state.y],quiver=([0.5*cos(human_heading_angle)],[0.5*sin(human_heading_angle)]), color="black")
            scatter!([new_new_human_state.x], [new_new_human_state.y],color="darkgoldenrod1",msize=0.35*plot_size/env.length)
        end
    end
    #Plot the Hybrid A* path if it exists
    if(length(env.cart_hybrid_astar_path)!=0)
        px,py = get_hybrid_astar_path_points(env)
        # plot!(px,py,color="grey")
    end

    depth_one_cart_positions = []
    bad_actions = [-pi/9, 0.0, pi/9]
    for delta_angle in possible_delta_theta
        cart_path = update_cart_position_pomdp_planning_2D_action_space(env.cart, delta_angle, 2.0, env.length, env.breadth,1.0, 10)
        new_pos = cart_path[end]
        quiver!([env.cart.x],[env.cart.y],quiver=([1.5*cos(new_pos[3])],[1.5*sin(new_pos[3])]), color="black",)
        # plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="darkgreen")
        # annotate!(new_pos[1], new_pos[2], text("V", :white, :center, 20))
        if(delta_angle in bad_actions)
            # annotate!(new_pos[1], new_pos[2], text("V(t+1)", :black, :center, 10))
            plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="black",lw = 3)
            scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="red", msize= 0.35*plot_size*cart_size/env.length)
            # quiver!([new_pos[1]],[new_pos[2]],quiver=([cos(new_pos[3])],[sin(new_pos[3])]), color="red")
        else
            plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="black",lw = 3)
            # annotate!(new_pos[1], new_pos[2], text("V", :white, :center, 20))
            scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="green2", msize= 0.35*plot_size*cart_size/env.length)
            quiver!([new_pos[1]],[new_pos[2]],quiver=([0.5*cos(new_pos[3])],[0.5*sin(new_pos[3])]), color="black")
            push!(depth_one_cart_positions,new_pos)
        end
    end

    depth_two_cart_positions = []
    bad_actions = [1,2,5,6,7]
    start_bad_index = 1
    for pos in depth_one_cart_positions
        temp_cs = cart_state(pos[1], pos[2], pos[3], env.cart.v, env.cart.L, env.cart.goal)
        for delta_angle in possible_delta_theta
            cart_path = update_cart_position_pomdp_planning_2D_action_space(temp_cs, delta_angle, 2.0, env.length, env.breadth,1.0, 10)
            new_pos = cart_path[end]
            if(start_bad_index in bad_actions)
                # annotate!(new_pos[1], new_pos[2], text("V", :white, :center, 20))
                plot!([temp_cs.x, new_pos[1]], [temp_cs.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="black",lw = 3)
                scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="red", msize= 0.35*plot_size*cart_size/env.length)
                quiver!([temp_cs.x],[temp_cs.y],quiver=([1.1*cos(pos[3]+delta_angle)],[1.1*sin(pos[3]+delta_angle)]), color="black")
                # quiver!([new_pos[1]],[new_pos[2]],quiver=([cos(new_pos[3])],[sin(new_pos[3])]), color="red")
            else
                push!(depth_two_cart_positions,new_pos)
                plot!([temp_cs.x, new_pos[1]], [temp_cs.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="black",lw = 3)
                # annotate!(new_pos[1], new_pos[2], text("V", :white, :center, 20))
                scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="darkolivegreen2", msize= 0.35*plot_size*cart_size/env.length)
                quiver!([new_pos[1]],[new_pos[2]],quiver=([0.5*cos(new_pos[3])],[0.5*sin(new_pos[3])]), color="black")
                quiver!([temp_cs.x],[temp_cs.y],quiver=([1.1*cos(pos[3]+delta_angle)],[1.1*sin(pos[3]+delta_angle)]), color="black")
            end
            start_bad_index +=1
        end
    end



    for pos in depth_two_cart_positions
        temp_env = deepcopy(env)
        temp_env.cart = cart_state(pos[1], pos[2], pos[3], env.cart.v, env.cart.L, env.cart.goal)
        temp_env.cart_hybrid_astar_path = hybrid_a_star_search(temp_env.cart.x, temp_env.cart.y,temp_env.cart.theta, temp_env.cart.goal.x,
                                            temp_env.cart.goal.y, temp_env, Array{Tuple{human_state,human_probability_over_goals},1}(),100.0);
        px,py = get_hybrid_astar_path_points(temp_env)
        plot!(px,py,color="grey",lw = 5,ls=:dash)
    end

    # for pos in depth_one_cart_positions
    #     temp_env = deepcopy(env)
    #     temp_env.cart = cart_state(pos[1], pos[2], pos[3], env.cart.v, env.cart.L, env.cart.goal)
    #     temp_env.cart_hybrid_astar_path = hybrid_a_star_search(temp_env.cart.x, temp_env.cart.y,temp_env.cart.theta, temp_env.cart.goal.x,
    #                                         temp_env.cart.goal.y, temp_env, Array{Tuple{human_state,human_probability_over_goals},1}(),100.0);
    #     px,py = get_hybrid_astar_path_points(temp_env)
    #     plot!(px,py,color="steelblue",lw = 5)
    # end
    annotate!(env.cart.goal.x, env.cart.goal.y, text("GOAL", :saddlebrown, :right, :italic, 50))
    plot!(size=(plot_size,plot_size))
    display(p)
end


function display_that_cool_image_part2_d1()

    env = generate_small_environment_for_paper_image()
    possible_delta_theta = [ -2*pi/9, -pi/9, 0.0, pi/9, 2*pi/9 ]

    default(dpi=300)
    p = plot([0.0],[0.0],legend=false,grid=false, axis=nothing)
    # p = plot([0.0],[0.0],legend=false,grid=false)
    # plot!([env.length], [env.breadth],legend=false)
    plot!([env.length], [env.breadth],legend=false, xaxis=false, yaxis=false)

    #Plot Golfcart
    annotate!(env.cart.x, env.cart.y, text("V", :white, :center, 10))
    scatter!([env.cart.x], [env.cart.y], shape=:circle, color="darkgreen", msize= 0.35*plot_size*cart_size/env.length)
    quiver!([env.cart.x],[env.cart.y],quiver=([0.5*cos(env.cart.theta)],[0.5*sin(env.cart.theta)]), color="black")

    #Plot Obstacles
    for i in 1: length(env.obstacles)
        scatter!([env.obstacles[i].x], [env.obstacles[i].y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    #Plot all the Humans
    for i in 1: length(env.humans)
        annotate!(env.humans[i].x, env.humans[i].y, text("H", :white, :center, 10))
        human_heading_angle = get_heading_angle(env.humans[i].goal.x, env.humans[i].goal.y, env.humans[i].x, env.humans[i].y)
        quiver!([env.humans[i].x], [env.humans[i].y],quiver=([0.7*cos(human_heading_angle)],[0.7*sin(human_heading_angle)]), color="black")
        scatter!([env.humans[i].x], [env.humans[i].y],color="darkgoldenrod4",msize=0.35*plot_size/env.length)
        new_human_state = get_new_human_position_actual_environemnt_for_paper_image( env.humans[i], env, 1.0, MersenneTwister(100))
        # annotate!(new_human_state.x, new_human_state.y, text("H(t+1)", :white, :center, 10))
        quiver!([new_human_state.x], [new_human_state.y],quiver=([0.7*cos(human_heading_angle)],[0.7*sin(human_heading_angle)]), color="black")
        scatter!([new_human_state.x], [new_human_state.y],color="goldenrod3",msize=0.35*plot_size/env.length)
        # if(i!=1)
        #     new_new_human_state = get_new_human_position_actual_environemnt_for_paper_image( new_human_state, env, 1.0, MersenneTwister(100))
        #     # annotate!(new_new_human_state.x, new_new_human_state.y, text("H(t+2)", :white, :center, 10))
        #     quiver!([new_new_human_state.x], [new_new_human_state.y],quiver=([0.5*cos(human_heading_angle)],[0.5*sin(human_heading_angle)]), color="black")
        #     scatter!([new_new_human_state.x], [new_new_human_state.y],color="darkgoldenrod1",msize=0.35*plot_size/env.length)
        # end
    end
    #Plot the Hybrid A* path if it exists
    if(length(env.cart_hybrid_astar_path)!=0)
        px,py = get_hybrid_astar_path_points(env)
        # plot!(px,py,color="grey")
    end

    depth_one_cart_positions = []
    bad_actions = [-pi/9, 0.0, pi/9]
    for delta_angle in possible_delta_theta
        cart_path = update_cart_position_pomdp_planning_2D_action_space(env.cart, delta_angle, 2.0, env.length, env.breadth,1.0, 10)
        new_pos = cart_path[end]
        quiver!([env.cart.x],[env.cart.y],quiver=([1.5*cos(new_pos[3])],[1.5*sin(new_pos[3])]), color="black",)
        # plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="darkgreen")
        # annotate!(new_pos[1], new_pos[2], text("V", :green, :center, 30))
        if(delta_angle in bad_actions)
            # annotate!(new_pos[1], new_pos[2], text("V(t+1)", :black, :center, 10))
            plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="black",lw = 3)
            scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="red", msize= 0.35*plot_size*cart_size/env.length)
            # quiver!([new_pos[1]],[new_pos[2]],quiver=([cos(new_pos[3])],[sin(new_pos[3])]), color="red")
        else
            plot!([env.cart.x, new_pos[1]], [env.cart.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="black",lw = 3)
            # annotate!(new_pos[1], new_pos[2], text("V(t+1)", :black, :center, 10))
            scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="green2", msize= 0.35*plot_size*cart_size/env.length)
            quiver!([new_pos[1]],[new_pos[2]],quiver=([0.5*cos(new_pos[3])],[0.5*sin(new_pos[3])]), color="black")
            push!(depth_one_cart_positions,new_pos)
        end
    end

    # depth_two_cart_positions = []
    # bad_actions = [1,2,5,6,7]
    # start_bad_index = 1
    # for pos in depth_one_cart_positions
    #     temp_cs = cart_state(pos[1], pos[2], pos[3], env.cart.v, env.cart.L, env.cart.goal)
    #     for delta_angle in possible_delta_theta
    #         cart_path = update_cart_position_pomdp_planning_2D_action_space(temp_cs, delta_angle, 2.0, env.length, env.breadth,1.0, 10)
    #         new_pos = cart_path[end]
    #         if(start_bad_index in bad_actions)
    #             # annotate!(new_pos[1], new_pos[2], text("V(t+2)", :black, :center, 10))
    #             plot!([temp_cs.x, new_pos[1]], [temp_cs.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="black",lw = 3)
    #             scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="red", msize= 0.35*plot_size*cart_size/env.length)
    #             quiver!([temp_cs.x],[temp_cs.y],quiver=([1.1*cos(pos[3]+delta_angle)],[1.1*sin(pos[3]+delta_angle)]), color="black")
    #             # quiver!([new_pos[1]],[new_pos[2]],quiver=([cos(new_pos[3])],[sin(new_pos[3])]), color="red")
    #         else
    #             push!(depth_two_cart_positions,new_pos)
    #             plot!([temp_cs.x, new_pos[1]], [temp_cs.y, new_pos[2]],legend=false, xaxis=false, yaxis=false, color="black",lw = 3)
    #             # annotate!(new_pos[1], new_pos[2], text("V(t+2)", :black, :center, 10))
    #             scatter!([new_pos[1]], [new_pos[2]], shape=:circle, color="darkseagreen1", msize= 0.35*plot_size*cart_size/env.length)
    #             quiver!([new_pos[1]],[new_pos[2]],quiver=([0.5*cos(new_pos[3])],[0.5*sin(new_pos[3])]), color="black")
    #             quiver!([temp_cs.x],[temp_cs.y],quiver=([1.1*cos(pos[3]+delta_angle)],[1.1*sin(pos[3]+delta_angle)]), color="black")
    #         end
    #         start_bad_index +=1
    #     end
    # end
    #
    #
    #
    # for pos in depth_two_cart_positions
    #     temp_env = deepcopy(env)
    #     temp_env.cart = cart_state(pos[1], pos[2], pos[3], env.cart.v, env.cart.L, env.cart.goal)
    #     temp_env.cart_hybrid_astar_path = hybrid_a_star_search(temp_env.cart.x, temp_env.cart.y,temp_env.cart.theta, temp_env.cart.goal.x,
    #                                         temp_env.cart.goal.y, temp_env, Array{Tuple{human_state,human_probability_over_goals},1}(),100.0);
    #     px,py = get_hybrid_astar_path_points(temp_env)
    #     plot!(px,py,color="steelblue",lw = 5)
    # end
    #
    for pos in depth_one_cart_positions
        temp_env = deepcopy(env)
        temp_env.cart = cart_state(pos[1], pos[2], pos[3], env.cart.v, env.cart.L, env.cart.goal)
        temp_env.cart_hybrid_astar_path = hybrid_a_star_search(temp_env.cart.x, temp_env.cart.y,temp_env.cart.theta, temp_env.cart.goal.x,
                                            temp_env.cart.goal.y, temp_env, Array{Tuple{human_state,human_probability_over_goals},1}(),100.0);
        px,py = get_hybrid_astar_path_points(temp_env)
        plot!(px,py,color="steelblue",lw = 5)
    end
    annotate!(env.cart.goal.x, env.cart.goal.y, text("GOAL", :saddlebrown, :right, 25))
    plot!(size=(plot_size,plot_size))
    display(p)
end



#=
c = 0
d = 0
for k in keys(a["total_time_taken_2D_POMDP_planner_dict_fmm"])
   println( a["total_time_taken_2D_POMDP_planner_dict_fmm"][k] , " ", a["total_time_taken_2D_POMDP_planner_dict_prm"][k], " ", a["total_time_taken_1D_POMDP_planner_dict"][k])
   if( a["total_time_taken_2D_POMDP_planner_dict_fmm"][k] <= a["total_time_taken_1D_POMDP_planner_dict"][k] )
       c+=1
   end
   if( a["total_time_taken_2D_POMDP_planner_dict_prm"][k] <= a["total_time_taken_1D_POMDP_planner_dict"][k] )
       d+=1
   end
end

a["mean_time_taken_1D_POMDP_planner"] , sqrt(a["variance_time_taken_1D_POMDP_planner"]/100)
a["mean_sudden_stops_1D_POMDP_planner"] , sqrt(a["variance_sudden_stops_1D_POMDP_planner"]/100)

a["mean_time_taken_2D_POMDP_planner_fmm"] , sqrt(a["variance_time_taken_2D_POMDP_planner_fmm"]/100)
c
a["mean_sudden_stops_2D_POMDP_planner_fmm"] , sqrt(a["variance_sudden_stops_2D_POMDP_planner_fmm"]/100)

a["mean_time_taken_2D_POMDP_planner_prm"] , sqrt(a["variance_time_taken_2D_POMDP_planner_prm"]/100)
d
a["mean_sudden_stops_2D_POMDP_planner_prm"] , sqrt(a["variance_sudden_stops_2D_POMDP_planner_prm"]/100)

=#


#=

c = 0
for k in keys(a["total_time_taken_2D_POMDP_planner_dict"])
   println( a["total_time_taken_2D_POMDP_planner_dict"][k] , " ", a["total_time_taken_1D_POMDP_planner_dict"][k])
   if( a["total_time_taken_2D_POMDP_planner_dict"][k] <= a["total_time_taken_1D_POMDP_planner_dict"][k] )
       c+=1
   end
end

a["mean_time_taken_1D_POMDP_planner"] , sqrt(a["variance_time_taken_1D_POMDP_planner"]/100)
a["mean_sudden_stops_1D_POMDP_planner"] , sqrt(a["variance_sudden_stops_1D_POMDP_planner"]/100)

a["mean_time_taken_2D_POMDP_planner"] , sqrt(a["variance_time_taken_2D_POMDP_planner"]/100)
c
a["mean_sudden_stops_2D_POMDP_planner"] , sqrt(a["variance_sudden_stops_2D_POMDP_planner"]/100)

=#

#=
************************** Generate the percentage of imporvement time plot **************************
=#

function display_percentage_imporvement_time_plots()

    sce1_files = ["/home/himanshu/Desktop/RS_Sep13/scenario1_combined_humans_100_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep13/scenario1_combined_humans_200_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep13/scenario1_combined_humans_300_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep11/scenario1_combined_humans_400_experiments_100.jld2"
                ]

    sce2_files = ["/home/himanshu/Desktop/RS_Sep13/scenario2_combined_humans_100_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep13/scenario2_combined_humans_200_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep13/scenario2_combined_humans_300_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep13/scenario2_combined_humans_400_experiments_100.jld2"
                ]

    sce3_files = ["/home/himanshu/Desktop/RS_Sep13/A3/scenario4_combined_humans_100_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep13/scenario4_combined_humans_200_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep13/scenario4_combined_humans_300_experiments_100.jld2",
                "/home/himanshu/Desktop/RS_Sep13/scenario4_combined_humans_400_experiments_100.jld2"
                ]

    nhv_files = ["/home/himanshu/Desktop/human_aware_navigation/src/RESULTS/scenario1_humans_100_experiments_100.jld2",
                 "/home/himanshu/Desktop/human_aware_navigation/src/RESULTS/scenario1_humans_200_experiments_100.jld2",
                 "/home/himanshu/Desktop/human_aware_navigation/src/RESULTS/scenario1_humans_300_experiments_100.jld2",
                 "/home/himanshu/Desktop/human_aware_navigation/src/RESULTS/scenario1_humans_400_experiments_100_attempt1.jld2"
                ]

    y_matrix = zeros(4,8)
    ribbon_matrix = zeros(4,8)

    #SCE1
    fmm_mean_list = []
    fmm_var_list = []
    prm_mean_list = []
    prm_var_list = []
    for fn in sce1_files
        a = load(fn)
        ratio_time_fmm_dict = OrderedDict()
        ratio_time_prm_dict = OrderedDict()
        for k in keys(a["total_time_taken_1D_POMDP_planner_dict"])
            rt_fmm = a["total_time_taken_2D_POMDP_planner_dict_fmm"][k] / a["total_time_taken_1D_POMDP_planner_dict"][k]
            ratio_time_fmm_dict[k] = rt_fmm
            rt_prm = a["total_time_taken_2D_POMDP_planner_dict_prm"][k] / a["total_time_taken_1D_POMDP_planner_dict"][k]
            ratio_time_prm_dict[k] = rt_prm
        end

        fmm_ratio_time_mean, fmm_ratio_time_var = calculate_mean_and_variance_from_given_dict(ratio_time_fmm_dict)
        prm_ratio_time_mean, prm_ratio_time_var = calculate_mean_and_variance_from_given_dict(ratio_time_prm_dict)
        push!(fmm_mean_list, fmm_ratio_time_mean)
        push!(fmm_var_list, sqrt(fmm_ratio_time_var)/10)
        push!(prm_mean_list, prm_ratio_time_mean)
        push!(prm_var_list, sqrt(prm_ratio_time_var)/10)
    end
    y_matrix[:,1] = fmm_mean_list
    y_matrix[:,2] = prm_mean_list
    ribbon_matrix[:,1] = fmm_var_list
    ribbon_matrix[:,2] = prm_var_list

    # SCE 2
    fmm_mean_list = []
    fmm_var_list = []
    prm_mean_list = []
    prm_var_list = []
    for fn in sce2_files
        a = load(fn)
        ratio_time_fmm_dict = OrderedDict()
        ratio_time_prm_dict = OrderedDict()
        for k in keys(a["total_time_taken_1D_POMDP_planner_dict"])
            rt_fmm = a["total_time_taken_2D_POMDP_planner_dict_fmm"][k] / a["total_time_taken_1D_POMDP_planner_dict"][k]
            ratio_time_fmm_dict[k] = rt_fmm
            rt_prm = a["total_time_taken_2D_POMDP_planner_dict_prm"][k] / a["total_time_taken_1D_POMDP_planner_dict"][k]
            ratio_time_prm_dict[k] = rt_prm
        end

        fmm_ratio_time_mean, fmm_ratio_time_var = calculate_mean_and_variance_from_given_dict(ratio_time_fmm_dict)
        prm_ratio_time_mean, prm_ratio_time_var = calculate_mean_and_variance_from_given_dict(ratio_time_prm_dict)
        push!(fmm_mean_list, fmm_ratio_time_mean)
        push!(fmm_var_list, sqrt(fmm_ratio_time_var)/10)
        push!(prm_mean_list, prm_ratio_time_mean)
        push!(prm_var_list, sqrt(prm_ratio_time_var)/10)
    end
    y_matrix[:,3] = fmm_mean_list
    y_matrix[:,4] = prm_mean_list
    ribbon_matrix[:,3] = fmm_var_list
    ribbon_matrix[:,4] = prm_var_list


    #SCE 3
    fmm_mean_list = []
    fmm_var_list = []
    prm_mean_list = []
    prm_var_list = []
    for fn in sce3_files
        a = load(fn)
        ratio_time_fmm_dict = OrderedDict()
        ratio_time_prm_dict = OrderedDict()
        for k in keys(a["total_time_taken_1D_POMDP_planner_dict"])
            rt_fmm = a["total_time_taken_2D_POMDP_planner_dict_fmm"][k] / a["total_time_taken_1D_POMDP_planner_dict"][k]
            ratio_time_fmm_dict[k] = rt_fmm
            rt_prm = a["total_time_taken_2D_POMDP_planner_dict_prm"][k] / a["total_time_taken_1D_POMDP_planner_dict"][k]
            ratio_time_prm_dict[k] = rt_prm
        end

        fmm_ratio_time_mean, fmm_ratio_time_var = calculate_mean_and_variance_from_given_dict(ratio_time_fmm_dict)
        prm_ratio_time_mean, prm_ratio_time_var = calculate_mean_and_variance_from_given_dict(ratio_time_prm_dict)
        push!(fmm_mean_list, fmm_ratio_time_mean)
        push!(fmm_var_list, sqrt(fmm_ratio_time_var)/10)
        push!(prm_mean_list, prm_ratio_time_mean)
        push!(prm_var_list, sqrt(prm_ratio_time_var)/10)
    end
    y_matrix[:,5] = fmm_mean_list
    y_matrix[:,6] = prm_mean_list
    ribbon_matrix[:,5] = fmm_var_list
    ribbon_matrix[:,6] = prm_var_list


    #SCE NHV
    mean_list = []
    var_list = []
    for fn in nhv_files
        a = load(fn)
        ratio_time_dict = OrderedDict()
        for k in keys(a["total_time_taken_1D_POMDP_planner_dict"])
            rt = a["total_time_taken_2D_POMDP_planner_dict"][k] / a["total_time_taken_1D_POMDP_planner_dict"][k]
            ratio_time_dict[k] = rt
        end

        ratio_time_mean, ratio_time_var = calculate_mean_and_variance_from_given_dict(ratio_time_dict)
        push!(mean_list, ratio_time_mean)
        push!(var_list, sqrt(ratio_time_var)/10)
    end
    y_matrix[:,7] = mean_list
    ribbon_matrix[:,7] = var_list

    y_matrix[:,8] = [0.55,0.55,0.55,0.55]
    ribbon_matrix[:,8] = [0,0,0,0]

    for i in 1:7
        println(y_matrix[:,i]')
    end

    x_axis_points = [100,200,300,400]
    p = plot(x_axis_points,y_matrix,legend=:bottomright,grid=false,label = ["2D-FMM (Scenario 1)" "2D-PRM (Scenario 1)" "2D-FMM (Scenario 2)" "2D-PRM (Scenario 2)" "2D-FMM (Scenario 3)" "2D-PRM (Scenario 3)" "2D-NHV" ""],
                                        lw = 3, color = ["green" "red" "blue" "black" "cyan" "olive" "purple" "white"],
                                        xlabel = "Number of pedestrians in the environment" , ylabel = "Travel Time as Ratio of the LS solution",
                                        ribbon=ribbon_matrix,fillalpha=.1)
    default(dpi=300)
    display(p)
end
