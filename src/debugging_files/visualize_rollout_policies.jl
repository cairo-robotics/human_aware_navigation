function generate_rollout(planner,m,b)
    S = statetype(m)
    scenario_trajectory = OrderedDict{Int,Any}()
    tree_depth = 0
    for (k,s) in b.scenarios
        p = (ps = nothing, at=nothing, cs=s ,rew=0.0)
        t = OrderedDict{Any,Any}(tree_depth=>p)
        scenario_trajectory[k] = t
    end
    lb = planner.bounds.lower
    rsum = multiple_particles_rollout(m, lb.policy, b, lb.max_depth-b.depth, lb.final_value, scenario_trajectory, 1)
    return scenario_trajectory
end

function multiple_particles_rollout(pomdp::POMDP, policy::Policy, b::ScenarioBelief, steps::Integer, fval, scenario_trajectory, search_depth)
    S = statetype(pomdp)
    O = obstype(pomdp)
    odict = Dict{O, Vector{Pair{Int, S}}}()

    if steps <= 0
        return length(b.scenarios)*fval(pomdp, b)
    end

    a = action(policy, b)

    r_sum = 0.0
    for (k, s) in b.scenarios
        if !isterminal(pomdp, s)
            rng = ARDESPOT.get_rng(b.random_source, k, b.depth)
            sp, o, r = @gen(:sp, :o, :r)(pomdp, s, a, rng)

            if haskey(odict, o)
                push!(odict[o], k=>sp)
            else
                odict[o] = [k=>sp]
            end

            p = (ps = s, at=a, cs=sp ,rew=r)
            scenario_trajectory[k][search_depth] = p

            r_sum += r
        end
    end

    next_r = 0.0
    for (o, scenarios) in odict
        bp = ScenarioBelief(scenarios, b.random_source, b.depth+1, o)
        if length(scenarios) == 1
            next_r += single_particle_rollout(pomdp, policy, bp, steps-1, fval, scenario_trajectory, search_depth+1)
        else
            next_r += multiple_particles_rollout(pomdp, policy, bp, steps-1, fval, scenario_trajectory, search_depth+1)
        end
    end

    return r_sum + discount(pomdp)*next_r
end

# once there is only one scenario left, just run a rollout
function single_particle_rollout(pomdp::POMDP, policy::Policy, b0::ScenarioBelief, steps::Integer, fval, scenario_trajectory, search_depth)
    @assert length(b0.scenarios) == 1
    disc = 1.0
    r_total = 0.0
    scenario_mem = copy(b0.scenarios)
    (k, s) = first(b0.scenarios)
    b = ScenarioBelief(scenario_mem, b0.random_source, b0.depth, b0._obs)

    while !isterminal(pomdp, s) && steps > 0
        a = action(policy, b)

        rng = ARDESPOT.get_rng(b.random_source, k, b.depth)
        sp, o, r = @gen(:sp, :o, :r)(pomdp, s, a, rng)

        r_total += disc*r

        p = (ps = s, at=a, cs=sp ,rew=r)
        scenario_trajectory[k][search_depth] = p

        s = sp
        scenario_mem[1] = k=>s
        b = ScenarioBelief(scenario_mem, b.random_source, b.depth+1, o)

        disc *= discount(pomdp)
        steps -= 1
        search_depth += 1
    end

    if steps == 0 && !isterminal(pomdp, s)
        r_total += disc*fval(pomdp, b)
    end

    return r_total
end

#
# e = output.sim_objects[time_value].env
# v = output.sim_objects[time_value].vehicle
# vp = output.sim_objects[time_value].vehicle_params
# vb = output.vehicle_body_at_origin
# h = output.sim_objects[time_value].humans
# hp = output.sim_objects[time_value].humans_params
# sd = output.sim_objects[time_value].vehicle_sensor_data
# nbh = output.nearby_humans[time_value]
# push!(vehicle_executed_trajectory, v)
#
# p = get_plot(e, v, vb, vp, nbh, sd, time_value, exp_details, vehicle_executed_trajectory)
#
# env::ExperimentEnvironment
# vehicle::Vehicle
# vehicle_params::P
# vehicle_sensor_data::VehicleSensor
# humans::Array{HumanState,1}
# humans_params::Array{HumanParameters,1}
# one_time_step::Float64


function get_vehicle_trajectory_from_rollout(scenarios_trajectory, scenario_num)
    required_trajectory = scenarios_trajectory[scenario_num]
    path_x, path_y, path_theta = [],[],[]
    for k in keys(required_trajectory)
        curr_pos = required_trajectory[k][:cs]
        if( curr_pos.vehicle_x == -100.0 && curr_pos.vehicle_y == -100.0 )
            break
        else
            push!(path_x, curr_pos.vehicle_x)
            push!(path_y, curr_pos.vehicle_y)
            push!(path_theta, curr_pos.vehicle_theta)
        end
    end
    return path_x,path_y,path_theta
end


function get_human_trajectory_from_rollout(scenarios_trajectory, scenario_num, pomdp)
    required_trajectory = scenarios_trajectory[scenario_num]
    num_humans = length(required_trajectory[0][:cs].nearby_humans)

    human_paths = Dict{Any,Any}()
    for i in 1:num_humans
        human_paths[i] = ([],[])
    end
    active_humans = [i for i in 1:num_humans]

    for k in keys(required_trajectory)
        if( required_trajectory[k][:ps] == nothing )
            humans = required_trajectory[k][:cs].nearby_humans
            active_human_index = 1
            for i in 1:length(humans)
                active_human_num = active_humans[active_human_index]
                push!(human_paths[active_human_num][1] , humans[i].x)
                push!(human_paths[active_human_num][2] , humans[i].y)
                active_human_index += 1
            end
        else
            ps_humans = required_trajectory[k][:ps].nearby_humans
            cs_humans = required_trajectory[k][:cs].nearby_humans
            if(length(ps_humans) == length(cs_humans))
                humans = cs_humans
                active_human_index = 1
                for i in 1:length(humans)
                    active_human_num = active_humans[active_human_index]
                    push!(human_paths[active_human_num][1] , humans[i].x)
                    push!(human_paths[active_human_num][2] , humans[i].y)
                    active_human_index += 1
                end
            else
                for i in 1:length(ps_humans)
                    h = ps_humans[i]
                    if(is_within_range(h.x,h.y,h.goal.x,h.goal.y,h.v*pomdp.one_time_step))
                        new_human = HumanState(h.goal.x,h.goal.y,h.v,h.goal)
                        active_human_num = active_humans[i]
                        push!(human_paths[active_human_num][1] , new_human.x)
                        push!(human_paths[active_human_num][2] , new_human.y)
                        active_humans[i] = 0
                    end
                end
                deleteat!(active_humans, active_humans .== 0)
                humans = cs_humans
                active_human_index = 1
                for i in 1:length(humans)
                    active_human_num = active_humans[active_human_index]
                    push!(human_paths[active_human_num][1] , humans[i].x)
                    push!(human_paths[active_human_num][2] , humans[i].y)
                    active_human_index += 1
                end
            end
        end
    end
    return human_paths
end


function convert_scenario_trajectory_to_sim_objects(scenarios_trajectory, scenario_num, e, vp, )
    required_trajectory = scenarios_trajectory[scenario_num]
end


function visualize_rollout(scenarios_trajectory, pomdp)
    p = observe(output, exp_details,1.5, []);
    for k in keys(scenarios_trajectory)
        vx,vy,vt = get_vehicle_trajectory_from_rollout(scenarios_trajectory, k)
        plot!(p,vx,vy,line=(3,:dot))
        hp = get_human_trajectory_from_rollout(scenarios_trajectory, k, pomdp)
        for k in keys(hp)
            plot!(p,hp[k][1],hp[k][2],line=(3,:dot))
        end
    end
    display(p)
    return p
end


#=
ts = 1.5
scenario_num_from_tree = 46
bel = ScenarioBelief(output.despot_trees[ts][:tree].scenarios[scenario_num_from_tree], pomdp_planner.rs, 0, nothing)
st = generate_rollout(pomdp_planner, extended_space_pomdp, bel)
visualize_rollout(st, extended_space_pomdp)
=#
