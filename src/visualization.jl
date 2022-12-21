using Plots
using UnicodePlots
using D3Trees

#Function to display a circle
function circleShape(h,k,r)
    theta = LinRange(0,2*pi,100)
    h .+ r*sin.(theta), k .+ r*cos.(theta)
end

function get_plot(env,vehicle,vehicle_params,humans,humans_params,sensor_data,nearby_humans,time_value,exp_details)

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
        Plots.annotate!(human_goals[i].x, human_goals[i].y,text(goal_name, :purple, :center, 15))
    end

    #Plot Obstacles
    for obs in env.obstacles
        plot!(circleShape(obs.x,obs.y,obs.r), lw=0.5, linecolor = :black,
                                            legend=false, fillalpha=1.0, aspect_ratio=1,c= :black, seriestype = [:shape,])
        # scatter!([obs.x],[obs.y],color="black",shape=:circle,msize=plot_size*obs.r/env.length)
    end

    #=
    Plot humans!
    Go through all the humans in the current sim object
    If that human is a nearby human, display it in red
    If that human is in sensor data, display it in blue
    If neither of that is true, it is outside the lidar range, ignore it!
    =#
    for i in 1:length(humans)
        human = humans[i]
        human_id = humans_params[i].id
        human_reached_goal = (human.x == human.goal.x) && (human.y == human.goal.y)
        if(human_reached_goal)
            continue
        end
        is_nearby_human = !(length(findall(x->x==human_id, nearby_humans.ids)) == 0)
        if(is_nearby_human)
            scatter!([human.x],[human.y],color="red")
            Plots.annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
            plot!(circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :red, seriestype = [:shape,])
            # human_heading_angle = get_heading_angle(human.goal.x,human.goal.y,human.x,human.y)
            # quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="red")
            continue
        end
        is_sensor_data_human = !(length(findall(x->x==human_id, sensor_data.ids)) == 0)
        if(is_sensor_data_human)
            scatter!([human.x],[human.y],color="lightblue")
            # annotate!(human.x, human.y, text(string(human_id), :purple, :right, 15))
            plot!(circleShape(human.x,human.y,exp_details.max_risk_distance), lw=0.5, linecolor = :black,
                                                legend=false, fillalpha=0.2, aspect_ratio=1,c= :lightblue, seriestype = [:shape,])
            # human_heading_angle = get_heading_angle(human.goal.x,human.goal.y,human.x,human.y)
            # quiver!([human.x],[human.y],quiver=([cos(human_heading_angle)],[sin(human_heading_angle)]), color="green")
            continue
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
    Plots.annotate!(vehicle_params.goal.x, vehicle_params.goal.y, text("G", :darkgreen, :right, 10))
    plot!(circleShape(vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal), lw=0.5, linecolor = :black,
                                        legend=false, fillalpha=0.2, aspect_ratio=1,c= :darkgreen, seriestype = [:shape,])

    #Add the time value to the plot
    Plots.annotate!(env.length/2, env.breadth+0.5, text(string(round(time_value,digits=1)), :purple, :right, 20))
    plot!(size=(plot_size,plot_size))
    # display(p)
    return p
end
#=
t = 11
p = get_plot( output.sim_objects[t].env, output.sim_objects[t].vehicle, output.sim_objects[t].vehicle_params, output.nearby_humans[t], output.sim_objects[t].vehicle_sensor_data, t, exp_details)
display(p)
=#


function get_plot_will_version(env, vehicle, vehicle_params, nearby_humans, sensor_data, time_value, exp_details, x_subpath)
    # NOTE: hard-coding vehicle body in for now, didn't want to modify observe() arguments
    wheelbase = 0.75
    body_dims = [1.0, 0.5]
    origin_to_cent = [0.375, 0.0]
    veh = define_vehicle(wheelbase, body_dims, origin_to_cent, 0.0, 0.0)

    p_sim = plot(aspect_ratio=:equal,
        size=(800,800), dpi=300,
        xticks=0:4:20, yticks=0:4:20,
        xlabel="x-axis [m]", ylabel="y-axis [m]",
        # legend=:bottom,
        legend=false)

    # workspace
    plot!(p_sim, [0.0, env.length], [0.0,0.0], linecolor=:black, linewidth=2, label="")
    plot!(p_sim, [env.length, env.length], [0.0,env.breadth], linecolor=:black, linewidth=2, label="")
    plot!(p_sim, [0.0, env.length], [env.breadth,env.breadth], linecolor=:black, linewidth=2, label="")
    plot!(p_sim, [0.0, 0.0], [0.0,env.breadth], linecolor=:black, linewidth=2, label="")

    # external anchors
    anchor_dist = 0.55
    scatter!(p_sim,
        [0.0 - anchor_dist, env.length + anchor_dist, env.length + anchor_dist, 0.0 - anchor_dist],
        [0.0 - anchor_dist, 0.0 - anchor_dist, env.breadth + anchor_dist, env.breadth + anchor_dist],
        markeralpha=0.0, linealpha=0.0, fillalpha=0.0,
        label="")

    # vehicle goal
    plot!(p_sim, circleShape(vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal),
        color=:green, fillalpha=0.125,
        linecolor=:green, linewidth=2.0,
        label="Vehicle Goal", seriestype = [:shape,])

    # human goals
    human_goals = get_human_goals(env)
    offset = 0.625
    for i in 1:length(human_goals)
        goal_name = "G"*string(i)

        human_goals[i].x > 1/2*env.length ? x_dir = -1 : x_dir = 1
        human_goals[i].y > 1/2*env.breadth ? y_dir = -1 : y_dir = 1

        Plots.annotate!(human_goals[i].x + x_dir*offset, human_goals[i].y + y_dir*offset,
            text(goal_name, :black, :center, 15))
    end

    # obstacles
    for obs_index in 1:length(env.obstacles)
        obs = env.obstacles[obs_index]

        obs_index == 1 ? lbl = "Obstacle" : lbl = ""

        plot!(p_sim, circleShape(obs.x, obs.y, obs.r),
            color=:red, fillalpha=0.125,
            linecolor=:red, linewidth=2.0,
            label=lbl, seriestype = [:shape,])
    end

    # vehicle path
    linez_clim = 2.5
    linez_velocity = zeros(length(x_subpath))
    for kk in 1:(length(x_subpath)-1)
        linez_velocity[kk] = x_subpath[kk+1][4]
    end

    plot!(p_sim, getindex.(x_subpath, 1), getindex.(x_subpath, 2),
        linez=linez_velocity, clim=(0,linez_clim), colorbar_title="Velocity [m/s]",
        linewidth=2,
        label="")

    # vehicle body
    x = [vehicle.x, vehicle.y, vehicle.theta, vehicle.v]
    scatter!(p_sim, [x[1]], [x[2]],
        markershape=:circle, markersize=3, markerstrokewidth=0, markercolor=:black,
        label="")

    veh_body = state_to_body(x, veh)
    plot!(p_sim, veh_body,
        color=:black, alpha=0.125,
        linecolor=:black, linewidth=2.0, linealpha=1.0,
        label="Vehicle")

    # humans
    first_near_flag = true
    first_far_flag = true

    for i in 1:length(sensor_data.lidar_data)
        human = sensor_data.lidar_data[i]
        human_id = sensor_data.ids[i]
        is_nearby_human = !(length(findall(x->x==human_id, nearby_humans.ids)) == 0)
        human_reached_goal = (human.x == human.goal.x) && (human.y == human.goal.y)

        if(is_nearby_human)
            first_near_flag == true ? lbl = "Nearby Human" : lbl = ""
            first_near_flag = false

            scatter!(p_sim, [human.x], [human.y], color=:purple, label=lbl)
            plot!(p_sim, circleShape(human.x, human.y, exp_details.max_risk_distance),
                color=:purple, fillalpha=0.125,
                linecolor=:purple, linewidth=2.0,
                label="", seriestype = [:shape,])

        elseif(!human_reached_goal)
            first_far_flag == true ? lbl = "Far Human" : lbl = ""
            first_far_flag = false

            scatter!(p_sim, [human.x], [human.y], color=:grey, label=lbl)
            plot!(p_sim, circleShape(human.x, human.y, exp_details.max_risk_distance),
                color=:grey, fillalpha=0.125,
                linecolor=:grey, linewidth=2.0,
                label="", seriestype = [:shape,])
        end
    end

    # annotate time value
    t_round = round(time_value, digits=1)
    Plots.annotate!(p_sim, 0.403*env.length, env.breadth+0.5, text("t = $t_round sec", :black, :left, 14))

    return p_sim
end



#Function to display the environment, vehicle and humans
function observe(output,path_planning_details,exp_details,time_value, x_subpath)
    e = output.sim_objects[time_value].env
    v = output.sim_objects[time_value].vehicle
    vp = output.sim_objects[time_value].vehicle_params
    h = output.sim_objects[time_value].humans
    hp = output.sim_objects[time_value].humans_params
    sd = output.sim_objects[time_value].vehicle_sensor_data
    nbh = output.nearby_humans[time_value]

    x_k = [v.x, v.y, wrap_between_negative_pi_to_pi(v.theta), v.v]
    push!(x_subpath, x_k)

    # p = get_plot_will_version(e, v, vp, nbh, sd, time_value, exp_details, x_subpath)
    p = get_plot(e,v,vp,h,hp,sd,nbh,time_value,exp_details)

    # if(hasfield(typeof(vp),:controls_sequence))
    #     vehicle_path_x, vehicle_path_y, vehicle_path_theta  = get_vehicle_trajectory(v,vp,time_value,path_planning_details,exp_details)
    #     plot!(vehicle_path_x,vehicle_path_y,"grey")
    # end
    # annotate!(env.length/2, env.breadth/2, text("HG", :purple, :right, 20))
    display(p)
end
