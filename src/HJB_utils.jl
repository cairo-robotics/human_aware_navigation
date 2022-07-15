function get_HJB_env_vehicle_actions(world, max_vehicle_speed)

    # define workspace
    W = [[0.0 0.0];
        [world.length 0.0];
        [world.length world.breadth];
        [0.0 world.breadth]]

    # define target set
    target_x = world.cart.goal.x - 1.0
    target_y = world.cart.goal.y - 1.0
    T_xy = circle_to_polygon([target_x,target_y,2.0])
    T_theta = [[-pi, pi]]

    # define obstacles --> circular obstacles defined as [x, y, r], converted to polygon overapproximation
    O_vec = []
    for obs in world.obstacles
        o = circle_to_polygon([obs.x,obs.y,obs.r])
        push!(O_vec, o)
    end

    # max_steering_angle = 0.1
    max_steering_angle = 0.0846204431870001
    wheelbase = 0.324    # wheelbase [m]
    vehicle_length = 0.5207      # length [m]
    vehicle_width = 0.2762      # width [m]
    b2a = 0.0889    # rear bumber to rear axle [m]
    HJB_vehicle = Vehicle(max_vehicle_speed, 0.2*max_vehicle_speed, max_steering_angle, wheelbase, vehicle_length, vehicle_width, b2a)
    Am = [[a_v,a_phi] for a_v in [-HJB_vehicle.c_vb, HJB_vehicle.c_vf], a_phi in [-HJB_vehicle.c_phi, 0.0, HJB_vehicle.c_phi]]
    HJB_actions = reshape(Am, (length(Am),1))
    sort!(HJB_actions, dims=1)

    # initialize state grid
    h_xy = 1.0
    h_theta = deg2rad(15)
    HJB_env = Environment(h_xy,
                    h_theta,
                    minimum(W[:,1]) : h_xy : maximum(W[:,1]),
                    minimum(W[:,2]) : h_xy : maximum(W[:,2]),
                    -pi : h_theta : pi,
                    W,
                    T_xy,
                    T_theta,
                    O_vec)

    return HJB_env, HJB_vehicle, HJB_actions
end

function get_HJB_value_function(env,vehicle,actions,run_HJB_flag)
    if run_HJB_flag == true
        du_tol = 0.01
        max_steps = 5000
        anim_bool = false
        U_HJB, T, O = solve_HJB_PDE(actions, du_tol, max_steps, env, vehicle, anim_bool)
        N_grid = size(env.x_grid,1)*size(env.y_grid,1)*size(env.theta_grid,1)
        HJB_dict = Dict(:U_HJB => U_HJB, :T => T, :O => O, :env => env, :veh => vehicle)
        bson("./bson/HJB_dict.bson", HJB_dict)
        println("total grid nodes = ", N_grid)
        return U_HJB,T,O
    else
        HJB_dict = BSON.load("./bson/HJB_dict.bson")
        return HJB_dict[:U_HJB],HJB_dict[:T],HJB_dict[:O]
    end
end

#PLOTS
function plot_HJB_results(HJB_env, HJB_vehicle, start_pose, x_path)
    # plot U as heat map
    anim = @animate for k_plot in 1:size(HJB_env.theta_grid,1)
        p_k = heatmap(HJB_env.x_grid, HJB_env.y_grid,
                    transpose(U_HJB[:,:,k_plot]), clim=(0,100),
                    aspect_ratio=:equal,
                    size=(1000,1100),
                    xlabel="x-axis [m]", ylabel="y-axis [m]",
                    # title="HJB Value Function",
                    titlefontsize = 20,
                    colorbar_title = "time-to-target [s]",
                    legend=false, colorbar=false,
                    # legend=:topright,
                    legend_font_pointsize = 11,
                    top_margin = -30*Plots.mm,
                    left_margin = -8*Plots.mm,
                    bottom_margin = 8*Plots.mm)


        plot_polygon(p_k, HJB_env.W, 2, :black, "Workspace")
        plot_polygon(p_k, HJB_env.T_xy, 2, :green, "Target Set")
        for O in HJB_env.O_vec
            plot_polygon(p_k, O, 2, :red, "")
        end

        # vehicle figure
        x_pos = 110
        y_pos = 45

        x_max = x_pos + sqrt((HJB_vehicle.l-HJB_vehicle.b2a)^2 + (HJB_vehicle.w/2)^2)
        y_min = y_pos - sqrt((HJB_vehicle.l-HJB_vehicle.b2a)^2 + (HJB_vehicle.w/2)^2)

        x = [x_pos, y_pos, HJB_env.theta_grid[k_plot]]

        V_c = pose_to_edges(x, HJB_vehicle)
        V = [[V_c[1][1] V_c[1][2]];
            [V_c[2][1] V_c[2][2]];
            [V_c[3][1] V_c[3][2]];
            [V_c[4][1] V_c[4][2]]]

        plot!(p_k, [x_max], [y_pos], markercolor=:white, markershape=:circle, markersize=3, markerstrokewidth=0, label="")
        plot!(p_k, [x_pos], [y_pos], markercolor=:blue, markershape=:circle, markersize=3, markerstrokewidth=0, label="")
        plot_polygon(p_k, V, 2, :blue, "Vehicle Orientation")

        theta_deg = round(rad2deg(x[3]), digits=1)
        annotate!(x_pos, y_pos+10, text("theta [deg]:\n$theta_deg", 14))

        display(p_k)
    end

    # plot path
    p_path = plot(aspect_ratio=:equal, size=(1400,1300),
                xlabel="x-axis [m]", ylabel="y-axis [m]",
                titlefontsize = 20,
                legend_font_pointsize = 11,
                legend=false,
                top_margin = -4*Plots.mm,
                left_margin = 8*Plots.mm)

    plot_polygon(p_path, HJB_env.W, 2, :black, "Workspace")
    plot_polygon(p_path, HJB_env.T_xy, 2, :green, "Target Set")
    for O in HJB_env.O_vec
        plot_polygon(p_path, O, 2, :red, "")
    end

    plot!(p_path, getindex.(x_path,1), getindex.(x_path,2),
        linewidth = 2,
        label="Optimal Path")

    plot!(p_path, [x_path[1][1]], [x_path[1][2]],
        markercolor=:blue, markershape=:circle, markersize=3, markerstrokewidth=0,
        label="")

    V_c = pose_to_edges(start_pose, HJB_vehicle)
    V = [[V_c[1][1] V_c[1][2]];
        [V_c[2][1] V_c[2][2]];
        [V_c[3][1] V_c[3][2]];
        [V_c[4][1] V_c[4][2]]]

    plot_polygon(p_path, V, 2, :blue, "")

    plot!(p_path, [x_path[end][1]], [x_path[end][2]],
        markercolor=:blue, markershape=:circle, markersize=3, markerstrokewidth=0,
        label="")

    V_c = pose_to_edges(x_path[end], HJB_vehicle)
    V = [[V_c[1][1] V_c[1][2]];
        [V_c[2][1] V_c[2][2]];
        [V_c[3][1] V_c[3][2]];
        [V_c[4][1] V_c[4][2]]]

    plot_polygon(p_path, V, 2, :blue, "")

    display(p_path)
end
