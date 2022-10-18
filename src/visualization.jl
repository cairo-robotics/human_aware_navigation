using Plots
using D3Trees

#Function to display a circle
function circleShape(h,k,r)
    theta = LinRange(0,2*pi,100)
    h .+ r*sin.(theta), k .+ r*cos.(theta)
end

function display_sim(env,vehicle,vehicle_params,nearby_humans,sensor_data,time_value,exp_details)

    plot_size = 1000; #number of pixels
    p = plot(legend=false,grid=false,axis=([], false))

    #Plot the rectangular environment
    plot!([0.0, env.length],[0.0,0.0], color="grey", lw=2)
    plot!([env.length, env.length],[0.0,env.breadth], color="grey", lw=2)
    plot!([0.0, env.length],[env.breadth,env.breadth], color="grey", lw=2)
    plot!([0.0, 0.0],[0.0,env.breadth], color="grey", lw=2)

    #Plot Human Goals
    human_goals = get_human_goals(env)
    for i in 1:length(human_goals)
        goal_name = "G"*string(i)
        annotate!(human_goals[i].x, human_goals[i].y,text(goal_name, :purple, :center, 15))
    end

    #Plot Obstacles
    for obs in env.obstacles
        scatter!([obs.x],[obs.y],color="black",shape=:circle,msize=plot_size*env.obstacles[i].r/env.length)
    end

    #Plot nearby humans in red
    for human_index in 1:length(nearby_humans.position_data)
        human = nearby_humans.position_data[human_index]
        human_id = nearby_humans.ids[human_index]
        scatter!([human.x],[human.y],color="red")
        # annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
        plot!(circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                            legend=false, fillalpha=0.2, aspect_ratio=1,c= :red, seriestype = [:shape,])
        human_heading_angle = get_heading_angle(human.goal.x, human.goal.y,human.x,human.y)
        # quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="red")
    end

    #Plot remaining humans in lidar data in green
    for i in 1:length(sensor_data.lidar_data)
        green_human = sensor_data.lidar_data[i]
        green_human_id = sensor_data.ids[i]
        is_nearby_human = !(length(findall(x->x==green_human_id, nearby_humans.ids)) == 0)
        human_reached_goal = (green_human.x == green_human.goal.x) && (green_human.y == green_human.goal.y)
        if(!is_nearby_human && !human_reached_goal)
            scatter!([green_human.x],[green_human.y],color="green")
            plot!(circleShape(green_human.x,green_human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :green, seriestype = [:shape,])
            human_heading_angle = get_heading_angle(green_human.goal.x, green_human.goal.y,green_human.x,green_human.y)
            # quiver!([green_human.x],[green_human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="green")
        end
    end

    #Plot Vehicle
    scatter!([vehicle.x], [vehicle.y], shape=:circle, color="grey")
    plot!(circleShape(vehicle.x,vehicle.y,vehicle_params.L), lw=0.5, linecolor = :black,
                            legend=false, fillalpha=0.2, aspect_ratio=1,c= :grey, seriestype = [:shape,])
    quiver!([vehicle.x],[vehicle.y],quiver=([cos(vehicle.theta)],[sin(vehicle.theta)]), color="grey")

    #Plot Vehicle Goal
    annotate!(vehicle_params.goal.x, vehicle_params.goal.y, text("G", :darkgreen, :right, 10))
    plot!(circleShape(vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal), lw=0.5, linecolor = :black,
                                        legend=false, fillalpha=0.2, aspect_ratio=1,c= :darkgreen, seriestype = [:shape,])

    #Add the time value to the plot
    annotate!(env.length/2, env.breadth+0.5, text(string(round(time_value,digits=1)), :purple, :right, 20))
    plot!(size=(plot_size,plot_size))
    display(p)
end
#=
t = 11
display_sim( output.sim_objects[t].env, output.sim_objects[t].vehicle, output.sim_objects[t].vehicle_params, output.nearby_humans[t], output.sim_objects[t].vehicle_sensor_data, t, exp_details)
=#


#Function to display the environment
function observe(output,exp_details,time_value)
    e = output.sim_objects[time_value].env
    v = output.sim_objects[time_value].vehicle
    vp = output.sim_objects[time_value].vehicle_params
    nbh = output.nearby_humans[time_value]
    sd = output.sim_objects[time_value].vehicle_sensor_data
    display_sim(e,v,vp,nbh,sd,time_value,exp_details)
end
