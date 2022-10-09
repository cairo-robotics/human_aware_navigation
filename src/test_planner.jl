time_value = 3.5
e,p = get_details_from_input_parameters(aspen_input)
cso = exp_details.sim_objects[time_value]
a = exp_details.vehicle_actions[time_value]
cvspeed = clamp(cso.vehicle.v+a.delta_speed, 0.0, p.max_vehicle_speed)
cv_sa = get_steering_angle(cso.vehicle_params.L, a.delta_heading_angle, cvspeed, exp_details.one_time_step)
td= exp_details.buffer_time+p.planning_time
pvs = propogate_vehicle(cso.vehicle, cso.vehicle_params, cv_sa, cvspeed, td)
nbh = exp_details.nearby_humans[time_value]
b = tree_search_scenario_parameters(pvs.x,pvs.y,pvs.theta,pvs.v,cso.vehicle_params, exp_details.human_goal_locations, nbh.position_data, nbh.belief,
                    cso.env.length,cso.env.breadth,td)
next_action, info = action_info(pomdp_planner, b)
