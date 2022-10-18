include("struct_definition.jl")
include("environment.jl")
include("utils.jl")
include("pomdp_planning.jl")
include("belief_tracker.jl")
include("simulator.jl")
include("parser.jl")
include("visualization.jl")
include("aspen_inputs.jl")
include("no_obstacles_small.jl")
include("no_obstacles_big.jl")
using Revise

#Initialization
input = scenario1_small
input = scenario1_big
# input = aspen
exp_details, pomdp_details, output = get_details_from_input_parameters(input)
env = generate_environment(input.env_length,input.env_breadth,input.obstacles)
exp_details.env = env
exp_details.human_goal_locations = get_human_goals(env)
veh = Vehicle(input.veh_start_x, input.veh_start_y, input.veh_start_theta, input.veh_start_v)
veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
veh_goal = Location(input.veh_goal_x,input.veh_goal_y)
veh_params = VehicleParametersESPlanner(input.veh_L,input.veh_max_speed,veh_goal)
env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,exp_details.user_defined_rng)
initial_sim_obj = Simulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

#=
Define POMDP, POMDP Solver and POMDP Planner
=#
extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details.discount_factor,pomdp_details.min_safe_distance_from_human,
            pomdp_details.human_collision_penalty,pomdp_details.min_safe_distance_from_obstacle,pomdp_details.obstacle_collision_penalty,
            pomdp_details.radius_around_vehicle_goal,pomdp_details.goal_reached_reward,pomdp_details.max_vehicle_speed,pomdp_details.one_time_step,
            pomdp_details.num_segments_in_one_time_step,pomdp_details.observation_discretization_length,pomdp_details.d_near,
            pomdp_details.d_far,env)

pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,T_max=pomdp_details.planning_time,tree_in_info=true)
pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);

#Run the experiment
run_experiment!(initial_sim_obj, pomdp_planner, exp_details, pomdp_details, output)

#=
Print useful values from the experiment
=#


#Create Gif
create_gif = true
if(create_gif)
    anim = @animate for k ∈ keys(output.sim_objects)
        observe(output, exp_details, k);
    end
    gif(anim, "es_planner.gif", fps = 10)
end
