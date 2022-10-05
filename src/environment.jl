using Plots
using Random
using DataStructures


#Define the Environment
function generate_environment(l, b, obstacles)
    return experiment_environment(l,b,obstacles)
end

#
#     number_of_humans, user_defined_rng)
#
#     world_length =  5.518
#     world_breadth = 11.036
#
#
#     g1 = location(0.0,0.0)
#     g2 = location(0.0,world_breadth)
#     g3 = location(world_length,world_breadth)
#     g4 = location(world_length,0.0)
#     cart_goal = location(2.75,world_breadth)
#     all_goals_list = [g1,g2,g3,g4]
#     all_obstacle_list = obstacle_location[]
#     max_num_humans = number_of_humans
#
#     golfcart = cart_state(2.75,0.5,0.0,0.0,0.3,cart_goal)
#     initial_cart_lidar_data = Array{human_state,1}()
#     initial_complete_cart_lidar_data = Array{human_state,1}()
#
#     human_state_start_list = Array{human_state,1}()
#     for i in 1:max_num_humans
#         human =  human_state(floor(world_length*rand(user_defined_rng)), floor(world_breadth*rand(user_defined_rng)) , 1.0
#                                                 , all_goals_list[Int(ceil(rand(user_defined_rng)*4))] , float(i))
#         while(is_within_range_check_with_points(human.x,human.y, golfcart.x, golfcart.y, 5.0))
#             human =  human_state(floor(world_length*rand(user_defined_rng)), floor(world_breadth*rand(user_defined_rng)) , 1.0
#                                                     , all_goals_list[Int(ceil(rand(user_defined_rng)*4))] , float(i))
#         end
#         push!(human_state_start_list,human)
#     end
#
#     world = experiment_environment(world_length,world_breadth,max_num_humans,number_of_humans,
#                     all_goals_list,human_state_start_list,all_obstacle_list,golfcart,initial_cart_lidar_data,
#                     initial_complete_cart_lidar_data,Float64[],location(golfcart.x, golfcart.y))
#
#     return world
# end

#Function to display a circle
function circleShape(h,k,r)
    theta = LinRange(0,2*pi,100)
    h .+ r*sin.(theta), k .+ r*cos.(theta)
end

#Function to display the environment
function display_env(exp_details,time_value)

    sim_object = exp_details.sim_objects[time_value]
    plot_size = 1000; #number of pixels
    PADDING_AROUND_HUMAN = 0.2

    #Plot Boundaries
    # p = plot([0.0],[0.0],legend=false,grid=false)
    p = plot(legend=false,grid=false,axis=([], false))
    # plot!([env.length], [env.breadth],legend=false)

    #Plot the rectangular environment
    plot!([0.0, sim_object.env.length],[0.0,0.0], color="grey", lw=2)
    plot!([sim_object.env.length, sim_object.env.length],[0.0,sim_object.env.breadth], color="grey", lw=2)
    plot!([0.0, sim_object.env.length],[sim_object.env.breadth,sim_object.env.breadth], color="grey", lw=2)
    plot!([0.0, 0.0],[0.0,sim_object.env.breadth], color="grey", lw=2)

    #Plot nearby humans in red
    for human in exp_details.nearby_humans[time_value].position_data
        # scatter!([env.cart_lidar_data[i].x], [env.cart_lidar_data[i].y],color="green",msize=0.1*plot_size/env.length)
        scatter!([human.x],[human.y],color="red")
        plot!(circleShape(human.x,human.y,exp_details.min_safe_distance_from_human), lw=0.5, linecolor = :black,
                                            legend=false, fillalpha=0.2, aspect_ratio=1,c= :red, seriestype = [:shape,])
        human_heading_angle = get_heading_angle(human.goal.x, human.goal.y,human.x,human.y)
        quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="red")
    end

    #Plot remaining humans in lidar data in green
    for i in 1:length(sim_object.vehicle_sensor_data.lidar_data)
        green_human = sim_object.vehicle_sensor_data.lidar_data[i]
        green_human_id = sim_object.vehicle_sensor_data.ids[i]
        is_nearby_human_flag = !(length(findall(x->x==green_human_id, exp_details.nearby_humans[time_value].ids)) == 0)
        if(!is_nearby_human_flag)
            scatter!([green_human.x],[green_human.y],color="green")
            plot!(circleShape(green_human.x,green_human.y,exp_details.min_safe_distance_from_human), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :green, seriestype = [:shape,])
            human_heading_angle = get_heading_angle(green_human.goal.x, green_human.goal.y,green_human.x,green_human.y)
            quiver!([green_human.x],[green_human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="green")
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
    for obs in sim_object.env.obstacles
        scatter!([obs.x],[obs.y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    #Plot Vehicle
    scatter!([sim_object.vehicle.x], [sim_object.vehicle.y], shape=:circle, color="grey")
    plot!(circleShape(sim_object.vehicle.x,sim_object.vehicle.y,sim_object.vehicle_params.L), lw=0.5, linecolor = :black,
                            legend=false, fillalpha=0.2, aspect_ratio=1,c= :grey, seriestype = [:shape,])
    quiver!([sim_object.vehicle.x],[sim_object.vehicle.y],quiver=([cos(sim_object.vehicle.theta)],[sin(sim_object.vehicle.theta)]), color="grey")

    annotate!(sim_object.vehicle_params.goal.x, sim_object.vehicle_params.goal.y, text("G", :darkgreen, :right, 10))
    plot!(circleShape(sim_object.vehicle_params.goal.x, sim_object.vehicle_params.goal.y,exp_details.radius_around_vehicle_goal), lw=0.5, linecolor = :black,
                                        legend=false, fillalpha=0.2, aspect_ratio=1,c= :darkgreen, seriestype = [:shape,])
    # # plot!(circleShape(env.cart.goal.x, env.cart.goal.y,GLOBAL_RADIUS_AROUND_GOAL), lw=0.5, linecolor = :black, legend=false, fillalpha=0.2, c= :blue, seriestype = [:shape,])

    annotate!(sim_object.vehicle_params.goal.x, sim_object.env.breadth+2, text(string(round(time_value,digits=1)), :purple, :right, 20))
    plot!(size=(plot_size,plot_size))
    display(p)
end
