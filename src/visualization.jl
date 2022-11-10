using Plots
using D3Trees

#Function to display a circle
function circleShape(h,k,r)
    theta = LinRange(0,2*pi,100)
    h .+ r*sin.(theta), k .+ r*cos.(theta)
end

function get_plot(env,vehicle,vehicle_params,nearby_humans,sensor_data,time_value,exp_details)

    plot_size = 1000; #number of pixels
    boundary_padding = 0.5
    # p = plot(legend=false,grid=false)
    p = plot(legend=false,grid=false,axis=([], false))

    #Plot the rectangular environment
    plot!([0.0, env.length],[0.0,0.0], color="grey", lw=2)
    plot!([env.length, env.length],[0.0,env.breadth], color="grey", lw=2)
    plot!([0.0, env.length],[env.breadth,env.breadth], color="grey", lw=2)
    plot!([0.0, 0.0],[0.0,env.breadth], color="grey", lw=2)
    scatter!([0.0-boundary_padding],[0.0-boundary_padding], color="white",markersize = 0)
    scatter!([env.length+boundary_padding],[0.0-boundary_padding], color="white",markersize = 0)
    scatter!([env.length+boundary_padding],[env.breadth+boundary_padding], color="white",markersize = 0)
    scatter!([0.0-boundary_padding],[env.breadth+boundary_padding], color="white",markersize = 0)

    #Plot Human Goals
    human_goals = get_human_goals(env)
    for i in 1:length(human_goals)
        goal_name = "G"*string(i)
        annotate!(human_goals[i].x, human_goals[i].y,text(goal_name, :purple, :center, 15))
    end

    #Plot Obstacles
    for obs in env.obstacles
        plot!(circleShape(obs.x,obs.y,obs.r), lw=0.5, linecolor = :black,
                                            legend=false, fillalpha=1.0, aspect_ratio=1,c= :black, seriestype = [:shape,])
        # scatter!([obs.x],[obs.y],color="black",shape=:circle,msize=plot_size*obs.r/env.length)
    end

    #Plot humans in lidar data (nearby humans in red, rest in blue)
    for i in 1:length(sensor_data.lidar_data)
        human = sensor_data.lidar_data[i]
        human_id = sensor_data.ids[i]
        is_nearby_human = !(length(findall(x->x==human_id, nearby_humans.ids)) == 0)
        human_reached_goal = (human.x == human.goal.x) && (human.y == human.goal.y)
        if(is_nearby_human)
            scatter!([human.x],[human.y],color="red")
            annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
            plot!(circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :red, seriestype = [:shape,])
            human_heading_angle = get_heading_angle(human.goal.x, human.goal.y,human.x,human.y)
            # quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="red")
        elseif(!human_reached_goal)
            scatter!([human.x],[human.y],color="lightblue")
            # annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
            plot!(circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :lightblue, seriestype = [:shape,])
            human_heading_angle = get_heading_angle(human.goal.x, human.goal.y,human.x,human.y)
            # quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="green")
        end
    end

    #Plot Vehicle
    # scatter!([vehicle.x], [vehicle.y], shape=:circle, color="grey")
    vehicle_center_x = vehicle.x + vehicle_params.dist_origin_to_center*cos(vehicle.theta)
    vehicle_center_y = vehicle.y + vehicle_params.dist_origin_to_center*sin(vehicle.theta)
    plot!(circleShape(vehicle_center_x,vehicle_center_y,vehicle_params.radius), lw=0.5, linecolor = :black,
                            legend=false, fillalpha=0.2, aspect_ratio=1,c= :grey, seriestype = [:shape,])
    quiver!([vehicle_center_x],[vehicle_center_y],quiver=([cos(vehicle.theta)],[sin(vehicle.theta)]), color="grey")

    #Plot Vehicle Goal
    annotate!(vehicle_params.goal.x, vehicle_params.goal.y, text("G", :darkgreen, :right, 10))
    plot!(circleShape(vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal), lw=0.5, linecolor = :black,
                                        legend=false, fillalpha=0.2, aspect_ratio=1,c= :darkgreen, seriestype = [:shape,])

    #Add the time value to the plot
    annotate!(env.length/2, env.breadth+0.5, text(string(round(time_value,digits=1)), :purple, :right, 20))
    plot!(size=(plot_size,plot_size))
    # display(p)
    return p
end
#=
t = 11
p = get_plot( output.sim_objects[t].env, output.sim_objects[t].vehicle, output.sim_objects[t].vehicle_params, output.nearby_humans[t], output.sim_objects[t].vehicle_sensor_data, t, exp_details)
display(p)
=#

#Function to display the environment, vehicle and humans
function observe(output,path_planning_details,exp_details,time_value)
    e = output.sim_objects[time_value].env
    v = output.sim_objects[time_value].vehicle
    vp = output.sim_objects[time_value].vehicle_params
    nbh = output.nearby_humans[time_value]
    sd = output.sim_objects[time_value].vehicle_sensor_data
    p = get_plot(e,v,vp,nbh,sd,time_value,exp_details)
    if(hasfield(typeof(vp),:controls_sequence))
        vehicle_path_x, vehicle_path_y, vehicle_path_theta  = get_vehicle_trajectory(v,vp,time_value,path_planning_details,exp_details)
        plot!(vehicle_path_x,vehicle_path_y,"grey")
    end
    # annotate!(env.length/2, env.breadth/2, text("HG", :purple, :right, 20))
    display(p)
end
