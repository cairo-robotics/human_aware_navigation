using Plots
using Random

#Various different Struct definitions
struct location
    x::Float64
    y::Float64
end

mutable struct human_state
    x::Float64
    y::Float64
    v::Float64
    goal::location
    id::Float64
end

struct obstacle_location
    x::Float64
    y::Float64
    r::Float64 #Radius of the obstacle which is assumed to be a circle
end

mutable struct cart_state
    x::Float64
    y::Float64
    theta::Float64
    v::Float64
    L::Float64
    goal::location
end

struct human_probability_over_goals
    distribution::Array{Float64,1}
end

mutable struct experiment_environment
    length::Float64
    breadth::Float64
    max_num_humans::Float64
    num_humans::Int64
    goals::Array{location,1}
    humans::Array{human_state,1}
    obstacles::Array{obstacle_location,1}
    cart::cart_state
    cart_lidar_data::Array{human_state,1}
    complete_cart_lidar_data::Array{human_state,1}
    cart_hybrid_astar_path::Array{Float64,1}
    cart_start_location::location
end

function circleShape(h,k,r)
    theta = LinRange(0,2*pi,100)
    h .+ r*sin.(theta), k .+ r*cos.(theta)
end


#Define the Environment
function generate_ASPEN_environment_no_obstacles(number_of_humans, user_defined_rng)

    world_length =  5.518
    world_breadth = 11.036
    g1 = location(0.0,0.0)
    g2 = location(0.0,world_breadth)
    g3 = location(world_length,world_breadth)
    g4 = location(world_length,0.0)
    cart_goal = location(2.75,world_breadth)
    all_goals_list = [g1,g2,g3,g4]
    all_obstacle_list = obstacle_location[]
    max_num_humans = number_of_humans

    golfcart = cart_state(2.75,0.5,0.0,0.0,0.3,cart_goal)
    initial_cart_lidar_data = Array{human_state,1}()
    initial_complete_cart_lidar_data = Array{human_state,1}()

    human_state_start_list = Array{human_state,1}()
    for i in 1:max_num_humans
        human =  human_state(floor(world_length*rand(user_defined_rng)), floor(world_breadth*rand(user_defined_rng)) , 1.0
                                                , all_goals_list[Int(ceil(rand(user_defined_rng)*4))] , float(i))
        while(is_within_range_check_with_points(human.x,human.y, golfcart.x, golfcart.y, 5.0))
            human =  human_state(floor(world_length*rand(user_defined_rng)), floor(world_breadth*rand(user_defined_rng)) , 1.0
                                                    , all_goals_list[Int(ceil(rand(user_defined_rng)*4))] , float(i))
        end
        push!(human_state_start_list,human)
    end

    world = experiment_environment(world_length,world_breadth,max_num_humans,number_of_humans,
                    all_goals_list,human_state_start_list,all_obstacle_list,golfcart,initial_cart_lidar_data,
                    initial_complete_cart_lidar_data,Float64[],location(golfcart.x, golfcart.y))

    return world
end

#Function to display the environment
function display_env(env::experiment_environment, time_index=nothing, gif_env_num=nothing)

    plot_size = 1000; #number of pixels
    PADDING_AROUND_HUMAN = 0.2

    #Plot Boundaries
    # p = plot([0.0],[0.0],legend=false,grid=false)
    p = plot([0.0],[0.0],legend=false,grid=false,axis=([], false))
    # plot!([env.length], [env.breadth],legend=false)

    #Plot the rectangular environment
    plot!([0.0, env.length],[0.0,0.0], color="grey", lw=2)
    plot!([env.length, env.length],[0.0,env.breadth], color="grey", lw=2)
    plot!([0.0, env.length],[env.breadth,env.breadth], color="grey", lw=2)
    plot!([0.0, 0.0],[0.0,env.breadth], color="grey", lw=2)

    #Plot Humans in the cart lidar data
    for i in 1: length(env.cart_lidar_data)
        # scatter!([env.cart_lidar_data[i].x], [env.cart_lidar_data[i].y],color="green",msize=0.1*plot_size/env.length)
        scatter!([env.cart_lidar_data[i].x], [env.cart_lidar_data[i].y],color="red")
        plot!(circleShape(env.cart_lidar_data[i].x, env.cart_lidar_data[i].y,PADDING_AROUND_HUMAN), lw=0.5, linecolor = :black,
                                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :red, seriestype = [:shape,])
    end

    #Plot humans from cart_lidar_data
    for i in 1: length(env.complete_cart_lidar_data)
        in_lidar_data_flag = false
        for green_human in env.cart_lidar_data
            if(env.complete_cart_lidar_data[i].id == green_human.id)
                in_lidar_data_flag = true
                break
            end
        end
        if(!in_lidar_data_flag)
            scatter!([env.complete_cart_lidar_data[i].x], [env.complete_cart_lidar_data[i].y],color="blue")
            plot!(circleShape(env.complete_cart_lidar_data[i].x, env.complete_cart_lidar_data[i].y,PADDING_AROUND_HUMAN), lw=0.5, linecolor = :black,
                                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :blue, seriestype = [:shape,])
        end
    end

    #Plot Rest of the Humans
    # for i in 1: length(env.humans)
    #     in_lidar_data_flag = false
    #     for green_human in env.cart_lidar_data
    #         if(env.humans[i].id == green_human.id)
    #             in_lidar_data_flag = true
    #             break
    #         end
    #     end
    #     if(!in_lidar_data_flag)
    #         scatter!([env.humans[i].x], [env.humans[i].y],color="red",msize=0.5*plot_size/env.length)
    #     end
    # end

    #Plot Obstacles
    for i in 1: length(env.obstacles)
        scatter!([env.obstacles[i].x], [env.obstacles[i].y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    #Plot Golfcart
    scatter!([env.cart.x], [env.cart.y], shape=:circle, color="grey")
    plot!(circleShape(env.cart.x, env.cart.y,env.cart.L), lw=0.5, linecolor = :black,
                                                        legend=false, fillalpha=0.2, aspect_ratio=1,c= :grey, seriestype = [:shape,])
    quiver!([env.cart.x],[env.cart.y],quiver=([cos(env.cart.theta)],[sin(env.cart.theta)]), color="grey")

    #Plot the Hybrid A* path if it exists
    if(length(env.cart_hybrid_astar_path)!=0)
        #Plotting for normal environments that are 1 sec apart
        if(gif_env_num==nothing)
            initial_state = [env.cart.x,env.cart.y,env.cart.theta]
            path_x, path_y = [env.cart.x],[env.cart.y]
            for steering_angle in env.cart_hybrid_astar_path
                extra_parameters = [1.0, env.cart.L, steering_angle]
                x,y,theta = get_intermediate_points(initial_state, 1.0, extra_parameters);
                for pos_x in 2:length(x)
                    push!(path_x,x[pos_x])
                end
                for pos_y in 2:length(y)
                    push!(path_y,y[pos_y])
                end
                initial_state = [last(x),last(y),last(theta)]
            end
            plot!(path_x,path_y,color="grey",line=(:dot,4))
        #Plotting for gif environments that are 0.1 sec apart
        else
            current_gif_env_time_index = parse(Int, split(gif_env_num,"_")[2])
            current_time_stamp = 0.1*current_gif_env_time_index
            upper_cap_on_time = ceil(current_time_stamp/(1/env.cart.v))
            time_remaining_for_current_steering_angle = ((1/env.cart.v)*upper_cap_on_time) - current_time_stamp
            if(time_remaining_for_current_steering_angle!=0.0 && env.cart.v!=0.0)
                initial_state = [env.cart.x,env.cart.y,env.cart.theta]
                path_x, path_y = [env.cart.x],[env.cart.y]
                steering_angle = env.cart_hybrid_astar_path[1]
                extra_parameters = [env.cart.v, env.cart.L, steering_angle]
                x,y,theta = get_intermediate_points(initial_state,time_remaining_for_current_steering_angle, extra_parameters);
                for pos_x in 2:length(x)
                    push!(path_x,x[pos_x])
                end
                for pos_y in 2:length(y)
                    push!(path_y,y[pos_y])
                end
                initial_state = [last(path_x),last(path_y),last(theta)]
                start_index = 2
            else
                initial_state = [env.cart.x,env.cart.y,env.cart.theta]
                path_x, path_y = [env.cart.x],[env.cart.y]
                start_index = 1
            end
            for steering_angle in env.cart_hybrid_astar_path[start_index:end]
                extra_parameters = [1.0, env.cart.L, steering_angle]
                x,y,theta = get_intermediate_points(initial_state, 1.0, extra_parameters);
                for pos_x in 2:length(x)
                    push!(path_x,x[pos_x])
                end
                for pos_y in 2:length(y)
                    push!(path_y,y[pos_y])
                end
                initial_state = [last(x),last(y),last(theta)]
            end
            plot!(path_x,path_y,color="grey",line=(:dot,4))
        end
    end

    #annotate!(1.0, 25.0, text("S", :purple, :right, 20))
    annotate!(env.cart.goal.x, env.cart.goal.y, text("G", :darkgreen, :right, 20))
    # scatter!([env.cart.goal.x], [env.cart.goal.y],color="purple",shape=:circle,msize=20, opacity=0.5)
    plot!(circleShape(env.cart.goal.x, env.cart.goal.y,GLOBAL_RADIUS_AROUND_GOAL), lw=0.5, linecolor = :black, legend=false, fillalpha=0.2, aspect_ratio=1,c= :darkgreen, seriestype = [:shape,])
    # plot!(circleShape(env.cart.goal.x, env.cart.goal.y,GLOBAL_RADIUS_AROUND_GOAL), lw=0.5, linecolor = :black, legend=false, fillalpha=0.2, c= :blue, seriestype = [:shape,])
    if(time_index != nothing)
        annotate!(env.cart.goal.x+0.5, env.breadth+1, text(string(time_index), :purple, :right, 20))
    end
    plot!(size=(plot_size,plot_size))
    display(p)
end
