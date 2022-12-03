#=
Function to plot a scenario
=#

function visualize_given_scenario(b)

    s = rand(b)
    root_scenarios = [i=>s for i in 1:1];
    belief = ScenarioBelief(root_scenarios,pomdp_planner.rs, 1, missing);

    current_vehicle = Vehicle(s.vehicle_x,vehicle_y,vehicle_theta,vehicle_v)

    sim_time_step = 0.1
    one_time_step = 0.5
    num_sim_steps = Int64(one_time_step/sim_time_step)
    num_time_steps = 1

    vehicle_path = Vehicle[]
    humans_path = Array{Array{HumanState,1},1}(humans)
    sensor_data = []
    nearby_humans = []

    while(!is_within_range(current_vehicle.x, current_vehicle.y, vehicle_params.goal.x, vehicle_params.goal.y, exp_details.radius_around_vehicle_goal)
        #=
        Find vehicle action using lower bound rollout policy.
        Execute that action for one time step on the vehicle.
        Move humans for one time step.
        Generate plot for that situation.
        =#
        vehicle_action = calculate_lower_bound(m,belief)
        steering_angle = vehicle_action.steering_angle
        delta_speed = vehicle_action.delta_speed
        new_vehicle_speed = clamp(current_vehicle.v+delta_speed, 0.0, m.max_vehicle_speed)
        for i in 1:num_sim_steps
            new_vehicle = propogate_vehicle(current_vehicle, vehicle_params, steering_angle, new_vehicle_speed, sim_time_step)
            push!(vehicle_path,new_vehicle)
            new_humans = HumanState[]
            for j in 1:length(humans)
                nh = propogate_human(humans[j],humans_params[j])
                humans_params[j].path_index += 1
                push!(new_humans, nh)
            end
            push!(humans_path, new_humans)
        end
        current_vehicle = new_vehicle
        humans = new_humans

        get_plot(env,vehicle,vehicle_params,humans,humans_params,sensor_data,nearby_humans,time_value,exp_details)
