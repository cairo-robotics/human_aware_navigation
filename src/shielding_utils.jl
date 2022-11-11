include("/home/himanshu/Documents/Research/marmot-algs/tree-shielding/shield_functions.jl")
include("/home/himanshu/Documents/Research/marmot-algs/tree-shielding/shield_utils.jl")

function get_q_value(D, ba)
    #=
    Same as ba_l in pomdps_glue.jl in ARDESPOT
    =#
    q_value = 0.0
    for bnode in D.ba_children[ba]
        q_value += D.l[bnode]
    end
    q_value += D.ba_rho[ba]
    return q_value
end

function best_shield_action(veh, nearby_humans, Dt_obs_to_k1, Dt_plan, get_actions::Function, veh_body, human_goal_positions, pomdp, despot, user_defined_rng)
    x_k1 = SVector(veh.x,veh.y,wrap_between_negative_pi_to_pi(veh.theta),veh.v)
    nbh_pos = []
    for h in nearby_humans
      push!(nbh_pos, [h.x,h.y])
    end
    shield_actions, shield_actions_index = shield_action_set(x_k1, nbh_pos, Dt_obs_to_k1, Dt_plan, get_actions, veh_body, human_goal_positions, pomdp)
    best_q_value = -Inf
    best_action = []
    for i in 1:length(shield_actions_index)
        l = get_q_value(despot, shield_actions_index[i])
        if( l > best_q_value)
            best_q_value = l
            best_action = [shield_actions[i]]
        elseif l == best_q_value
            push!(best_action,shield_actions[i])
        end
    end

    a = rand(user_defined_rng, best_action)

    println("best safe action = ", a)
    println("len(ia_safe_set) = ", length(shield_actions), ", ia_safe_set = ", shield_actions_index)

    return ActionExtendedSpacePOMDP(a[1],a[2])
end

function get_shield_actions(x, Dt, veh, m)
    # set change in velocity (Dv) limit
    delta_speed = 0.5
    # set steering angle (phi) limit
    Dtheta_lim = deg2rad(45)
    phi_lim = 0.475
    if(x[4] == 0.0)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,Dtheta_lim,delta_speed,Dt),0.0,phi_lim)
        a =  SVector{8, SVector{2, Float64}}(
                (-steering_angle,delta_speed),
                (-2*steering_angle/3,delta_speed),
                (-steering_angle/3,delta_speed),
                (0.0,delta_speed),
                (0.0,0.0),
                (steering_angle/3,delta_speed),
                (2*steering_angle/3,delta_speed),
                (steering_angle,delta_speed)
                )
        return a,collect(1:length(a)),Int64[]
    elseif(x[4] == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,Dtheta_lim,x[4],m.one_time_step),0.0,phi_lim)
        rollout_action, q_vals = reactive_policy(x,delta_speed,800.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,
                            m.rollout_guide.q_value_array,m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
        a =  SVector{11, SVector{2, Float64}}(
                (-steering_angle,0.0),
                (-2*steering_angle/3,0.0),
                (-steering_angle/3,0.0),
                (0.0,-delta_speed),
                (0.0,0.0),
                (steering_angle/3,0.0),
                (2*steering_angle/3,0.0),
                (steering_angle,0.0),
                (rollout_action[1],rollout_action[2]),
                (steering_angle,-delta_speed),
                (-steering_angle,-delta_speed)
                # ActionExtendedSpacePOMDP(-10.0,-10.0)
                )
        return a,collect(1:length(a)),Int64[4,10,11]
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,Dtheta_lim,x[4],m.one_time_step),0.0,phi_lim)
        rollout_action, q_vals = reactive_policy(x,delta_speed,800.0,m.rollout_guide.get_actions, m.rollout_guide.get_cost,m.one_time_step,
                            m.rollout_guide.q_value_array,m.rollout_guide.value_array,m.rollout_guide.veh,m.rollout_guide.state_grid)
        a =  SVector{12, SVector{2, Float64}}(
                (-steering_angle,0.0),
                (-2*steering_angle/3,0.0),
                (-steering_angle/3,0.0),
                (0.0,-delta_speed),
                (0.0,0.0),
                (0.0,delta_speed),
                (steering_angle/3,0.0),
                (2*steering_angle/3,0.0),
                (steering_angle,0.0),
                (rollout_action[1],rollout_action[2]),
                (steering_angle,-delta_speed),
                (-steering_angle,-delta_speed)
                # ActionExtendedSpacePOMDP(-10.0,-10.0)
                )
        return a,collect(1:length(a)),Int64[4,11,12]
    end
end


# t = 3.0;
# sv = output.sim_objects[t].vehicle;
# snbh = output.nearby_humans[t].position_data;
# s_Dt_plan = exp_details.one_time_step;
# s_veh_body = veh_body;
# sm = extended_space_pomdp;
# s_despot = output.despot_trees[t][:tree];
# s_rng = MersenneTwister(11);
#
# best_shield_action(sv,snbh,0.0,s_Dt_plan,get_shield_actions,s_veh_body,exp_details.human_goal_locations,sm,s_despot,s_rng)
