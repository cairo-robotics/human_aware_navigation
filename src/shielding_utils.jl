
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



function get_shield_action(veh, nearby_humans, Dt_obs_to_k1, Dt_plan, get_actions::Function, veh_body, despot, user_defined_rng)
    x_k1 = SVector(veh.x,veh.y,wrap_between_negative_pi_to_pi(veh.theta),veh.v)
    nbh_pos = []
    for h in nearby_humans
      push!(nbh_pos, [h.x,h.y])
    end
    shield_actions, shield_actions_index = shield_action_set(x_k1, nbh_pos, Dt_obs_to_k1, Dt_plan, get_actions, veh_body)
    best_q_value = -Inf
    best_action = []
    for i in 1:length(shield_actions_index)
        l = get_q_value(despot, shield_actions_index[i])
        if( l > best_q_value)
            best_q_value = l
            best_action = [shield_actions[i]]
        else if l == best_q_value
            push!(best_action,shield_actions[i])
        end
    end
    return rand(user_defined_rng, best_action)
end


function get_shield_actions(x, Dt, veh, m)
    # set change in velocity (Dv) limit
    delta_speed = 0.5
    # set steering angle (phi) limit
    Dtheta_lim = deg2rad(45)
    phi_lim = 0.475
    if(x[4] == 0.0)
        steering_angle = clamp(get_steering_angle(veh.wheelbase,Dtheta_lim,delta_speed,Dt),0.0,phi_lim)
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
        return a,collect(1:length(a))
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase,max_delta_angle,pomdp_state.vehicle_v,m.one_time_step),0.0,max_steering_angle)
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
        return a,collect(1:length(a))
end
