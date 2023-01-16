t = 7.5
na, info = action_info(pomdp_planner, output.b_root[t]); println(na)

s = rand(MersenneTwister(1), output.b_root[t])
sp,o,r = POMDPs.gen(extended_space_pomdp, s, debug_calculate_lower_bound(extended_space_pomdp,s), MersenneTwister(1))

s = rand(MersenneTwister(1), output.b_root[t])
p=[]
for i in 1:10
    println(i)
    println("Current vehicle pos: ", (s.vehicle_x,s.vehicle_y,s.vehicle_theta*180/pi,s.vehicle_v) )
    a = debug_calculate_lower_bound(extended_space_pomdp,s)
    sp, o,r = genfn(extended_space_pomdp, s, a, MersenneTwister(1))
    println("Action: ", (a.delta_heading_angle*180/pi,a.delta_speed))
    println("New Vehicle pos: ", (sp.vehicle_x,sp.vehicle_y,sp.vehicle_theta*180/pi,sp.vehicle_v))
    println("Reward: ",r)
    println("****************************")
    push!(p,(s,a,sp))
    s = sp
end


# root_scenarios = [i=>rand(MersenneTwister(1), output.b_root[t]) for i in 1:5]
s = rand(MersenneTwister(1), output.b_root[t]);
root_scenarios = [i=>s for i in 1:100];
belief = ScenarioBelief(root_scenarios,pomdp_planner.rs, 1, missing);
p=[]
for i in 1:30
    println(i)
    println("Current vehicle pos: ", (s.vehicle_x,s.vehicle_y,s.vehicle_theta*180/pi,s.vehicle_v) )
    root_scenarios = [i=>s for i in 1:5]
    belief = ScenarioBelief(root_scenarios,pomdp_planner.rs, 1, missing)
    a = calculate_lower_bound(extended_space_pomdp,belief)
    sp, o,r = genfn(extended_space_pomdp, s, a, MersenneTwister(1))
    println("Action: ", (a.delta_heading_angle*180/pi,a.delta_speed))
    println("New Vehicle pos: ", (sp.vehicle_x,sp.vehicle_y,sp.vehicle_theta*180/pi,sp.vehicle_v))
    println("Reward: ",r)
    println("****************************")
    push!(p,(s,a,sp))
    s = sp
end


t = 0.0
s = rand(output.b_root[t]);
new_vehicle_speed = 0.5
num_segments_in_one_time_step = Int64(new_vehicle_speed/0.5)
pt = OrderedDict()
pt[t] = Vehicle[Vehicle(s.vehicle_x,s.vehicle_y,s.vehicle_theta,new_vehicle_speed)]
for i in 1:36
    path = update_vehicle_position(s, planning_pomdp, new_vehicle_speed, num_segments_in_one_time_step)
    # println(i, " ", path[end])
    new_s = StateLimitedSpacePOMDP(path[end][1], path[end][2], path[end][3], new_vehicle_speed, s.index_vehicle_controls_sequence+num_segments_in_one_time_step, s.nearby_humans)
    s = new_s
    pt[ round(i*planning_pomdp.one_time_step, digits=1) ] =  Vehicle(s.vehicle_x,s.vehicle_y,s.vehicle_theta,new_vehicle_speed)
end
