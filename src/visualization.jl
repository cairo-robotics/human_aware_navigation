using Plots
using D3Trees

#Function to display a circle
function circleShape(h,k,r)
    theta = LinRange(0,2*pi,100)
    h .+ r*sin.(theta), k .+ r*cos.(theta)
end

#Function to display the environment
function observe(output,exp_details,time_value)

    sim_object = output.sim_objects[time_value]
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

    for i in 1:length(exp_details.human_goal_locations)
        goal_name = "G"*string(i)
        annotate!(exp_details.human_goal_locations[i].x, exp_details.human_goal_locations[i].y,text(goal_name, :purple, :center, 15))
    end

    #Plot nearby humans in red
    for human_index in 1:length(output.nearby_humans[time_value].position_data)
        # scatter!([env.cart_lidar_data[i].x], [env.cart_lidar_data[i].y],color="green",msize=0.1*plot_size/env.length)
        human = output.nearby_humans[time_value].position_data[human_index]
        human_id = output.nearby_humans[time_value].ids[human_index]
        scatter!([human.x],[human.y],color="red")
        # annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
        plot!(circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                            legend=false, fillalpha=0.2, aspect_ratio=1,c= :red, seriestype = [:shape,])
        human_heading_angle = get_heading_angle(human.goal.x, human.goal.y,human.x,human.y)
        # quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="red")
    end

    #Plot remaining humans in lidar data in green
    for i in 1:length(sim_object.vehicle_sensor_data.lidar_data)
        green_human = sim_object.vehicle_sensor_data.lidar_data[i]
        green_human_id = sim_object.vehicle_sensor_data.ids[i]
        is_nearby_human = !(length(findall(x->x==green_human_id, output.nearby_humans[time_value].ids)) == 0)
        human_reached_goal = (green_human.x == green_human.goal.x) && (green_human.y == green_human.goal.y)
        if(!is_nearby_human && !human_reached_goal)
            scatter!([green_human.x],[green_human.y],color="green")
            plot!(circleShape(green_human.x,green_human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :green, seriestype = [:shape,])
            human_heading_angle = get_heading_angle(green_human.goal.x, green_human.goal.y,green_human.x,green_human.y)
            # quiver!([green_human.x],[green_human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="green")
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

    annotate!(sim_object.vehicle_params.goal.x, sim_object.env.breadth+1, text(string(round(time_value,digits=1)), :purple, :right, 20))
    plot!(size=(plot_size,plot_size))
    display(p)
end
