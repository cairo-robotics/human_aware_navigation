# shield_wrappers.jl

# main function for POMDP to call to return best safe action
function get_best_shielded_action(veh, nearby_humans, Dt_obs_to_k1, Dt_plan, get_actions::Function, veh_body, human_goal_positions, pomdp, despot, user_defined_rng)
    # reformat vehicle state
    x_k1 = SVector(veh.x, veh.y, wrap_between_negative_pi_to_pi(veh.theta), veh.v)

    # reformat human positions
    nbh_pos = []
    for h in nearby_humans
      push!(nbh_pos, [h.x, h.y])
    end

    # run shielding algorithm to produce shielded action set
    v_human_max = 1.25
    safe_HJB_value_lim = 200.0
    shield_actions, shield_action_indices = shield_action_set(x_k1, nbh_pos, Dt_obs_to_k1, Dt_plan, get_actions, veh_body, human_goal_positions, v_human_max, safe_HJB_value_lim, pomdp)
    println("len(ia_safe_set) = ", length(shield_action_indices))

    # TEST ONLY ---
    if length(shield_action_indices) == 0
        println("ISSUE: no safe actions returned")
        println("x_k1 = ", x_k1)
    end
    # ---

    # ISSUE: think had step where no safe actions were returned
    #   - x_k = [18.652, 15.116, 0.778, 1.0]
    #   - a_k = [0.475, 0.5]
    #   - x_k1 = [19.2, 15.5, 1.3, 1.5] (approx)
    #   - probably due to current inaccuracies with time step, MinkSum, substeps, etc.
    #       -  if shield is designed perfectly (lol), this issue should theoretically never happen

    # find action with best POMDP Q-value in shielded action set 
    best_q_value = -Inf
    best_action = []

    for i in axes(shield_action_indices, 1)
        l = get_q_value(despot, shield_action_indices[i])

        if l > best_q_value
            best_q_value = l
            best_action = [shield_actions[i]]
        elseif l == best_q_value
            push!(best_action, shield_actions[i])
        end
    end

    # draw random if multiple actions have same value
    best_action = rand(user_defined_rng, best_action)

    println("best safe action = ", best_action)

    return ActionExtendedSpacePOMDP(best_action[1], best_action[2])
end

# same as ba_l in pomdps_glue.jl in ARDESPOT
function get_q_value(D, ba)
    # sum value from each child node
    q_value = 0.0
    for bnode in D.ba_children[ba]
        q_value += D.l[bnode]
    end
    q_value += D.ba_rho[ba]

    return q_value
end

function shield_get_actions(x, Dt, veh, m)
    # set change in velocity (Dv) limit
    delta_speed = 0.5

    # set steering angle (phi) limit
    Dtheta_lim = deg2rad(45)
    phi_lim = 0.475

    # set HJB safe value limit
    safe_value_lim = 800.0

    # vehicle at min velocity
    if(x[4] == 0.0)
        steering_angle_p = clamp(get_steering_angle(m.vehicle_wheelbase, Dtheta_lim, (x[4]+delta_speed), Dt), 0.0, phi_lim)
       
        actions = SVector{8, SVector{2, Float64}}(
            (-steering_angle_p, delta_speed),
            (-2/3*steering_angle_p, delta_speed),
            (-1/3*steering_angle_p, delta_speed),
            (0.0, delta_speed),
            (0.0, 0.0),
            (1/3*steering_angle_p, delta_speed),
            (2/3*steering_angle_p, delta_speed),
            (steering_angle_p, delta_speed)
            )

        return actions, collect(1:length(actions)), Int64[]

    # vehicle at max velocity
    elseif(x[4] == m.max_vehicle_speed)
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase, Dtheta_lim, (x[4]), Dt), 0.0, phi_lim)
        steering_angle_n = clamp(get_steering_angle(m.vehicle_wheelbase, Dtheta_lim, (x[4]-delta_speed), Dt), 0.0, phi_lim)

        rollout_action, q_vals = reactive_policy(x, delta_speed, safe_value_lim, m.rollout_guide.get_actions, m.rollout_guide.get_cost, 
            m.one_time_step, m.rollout_guide.q_value_array, m.rollout_guide.value_array, m.rollout_guide.veh, m.rollout_guide.state_grid)
        
        actions = SVector{11, SVector{2, Float64}}(
            (-steering_angle, 0.0),
            (-2/3*steering_angle, 0.0),
            (-1/3*steering_angle, 0.0),
            (0.0, -delta_speed),
            (0.0, 0.0),
            (1/3*steering_angle, 0.0),
            (2/3*steering_angle, 0.0),
            (steering_angle, 0.0),
            (rollout_action[1], rollout_action[2]),
            (steering_angle_n, -delta_speed),
            (-steering_angle_n, -delta_speed)
            # (-10.0, -10.0)
            )

        return actions, collect(1:length(actions)), Int64[4, 10, 11]

    # vehicle between max/min velocity
    else
        steering_angle = clamp(get_steering_angle(m.vehicle_wheelbase, Dtheta_lim, (x[4]), Dt), 0.0, phi_lim)
        steering_angle_n = clamp(get_steering_angle(m.vehicle_wheelbase, Dtheta_lim, (x[4]-delta_speed), Dt), 0.0, phi_lim)

        rollout_action, q_vals = reactive_policy(x, delta_speed, safe_value_lim, m.rollout_guide.get_actions, m.rollout_guide.get_cost, 
            m.one_time_step, m.rollout_guide.q_value_array, m.rollout_guide.value_array, m.rollout_guide.veh, m.rollout_guide.state_grid)

        actions = SVector{12, SVector{2, Float64}}(
            (-steering_angle, 0.0),
            (-2/3*steering_angle, 0.0),
            (-1/3*steering_angle, 0.0),
            (0.0, -delta_speed),
            (0.0, 0.0),
            (0.0, delta_speed),
            (1/3*steering_angle, 0.0),
            (2/3*steering_angle, 0.0),
            (steering_angle, 0.0),
            (rollout_action[1], rollout_action[2]),
            (steering_angle_n, -delta_speed),
            (-steering_angle_n, -delta_speed)
            # (-10.0, -10.0)
            )

        return actions, collect(1:length(actions)), Int64[4, 11, 12]
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