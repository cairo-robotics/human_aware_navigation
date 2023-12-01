# using Pkg
# user = "Himanshu"
# if user == "Himanshu"
#     Pkg.activate("/home/himanshu/Documents/Research/BellmanPDEs.jl/")
# elseif user == "Will"
#     Pkg.activate("/Users/willpope/.julia/dev/BellmanPDEs")
# end
using BellmanPDEs
using JLD2
using ProfileView
using Revise
include("struct_definition.jl")
include("environment.jl")
include("utils.jl")
include("ES_POMDP_Planner.jl")
include("belief_tracker.jl")
include("simulator.jl")
include("simulator_utils.jl")
include("parser.jl")
include("visualization.jl")
include("HJB_wrappers.jl")
include("shielding/shield.jl")
include("shielding/shield_wrappers.jl")

# include("configs/aspen_inputs.jl")
# include("configs/aspen_inputs2.jl")
include("configs/small_obstacles_20x20.jl")
# include("configs/no_obstacles_big.jl")

#=
Initialization
=#
# input_config = scenario1_big
# input_config = aspen2
input_config = small_obstacles_20x20
# input_config = aspen

#=
Define experiment details and POMDP planning details
=#
pomdp_details = POMPDPlanningDetails(input_config)
exp_details = ExperimentDetails(input_config)
output = OutputObj()
# path_planning_details =

#=
Define environment
=#
env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
exp_details.env = env
exp_details.human_goal_locations = get_human_goals(env)

#=
Define Vehicle
=#
veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
r = sqrt( (0.5*input_config.veh_length)^2 + (0.5*input_config.veh_breadth)^2 )
veh_params = VehicleParametersESPlanner(input_config.veh_wheelbase,input_config.veh_length,
                input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal)
veh_body_origin = get_vehicle_body_origin(veh_params.dist_origin_to_center,0.0,veh_params.length,veh_params.breadth)
output.vehicle_body_at_origin = veh_body_origin

#=
Define Humans
=#
env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                        exp_details.simulator_time_step, exp_details.user_defined_rng)

#=
Create sim object
=#
initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

#=
Solve HJB equation for the given environment and vehicle
=#
Dt = 0.5
max_solve_steps = 200
Dval_tol = 0.1
HJB_planning_details = HJBPlanningDetails(Dt, max_solve_steps, Dval_tol, veh_params.max_steering_angle, veh_params.max_speed)
policy_path = "/home/himanshu/Documents/Research/human_aware_navigation/src"
solve_HJB = true
solve_HJB = false
if(solve_HJB)
    println("Solving HJB equation for given environment ....")
    rollout_guide = HJBPolicy(HJB_planning_details, exp_details, veh_params)
    d = Dict("rollout_guide"=>rollout_guide)
    save("./src/HJB_rollout_guide.jld2",d)
else
    s = load("./src/HJB_rollout_guide.jld2")
    rollout_guide = s["rollout_guide"]
end

#=
Define POMDP, POMDP Solver and POMDP Planner
=#
extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide)
pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    max_trials = 20,tree_in_info=true)#,default_action=default_es_pomdp_action)
pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);

#=
Run the experiment
=#
run_experiment!(initial_sim_obj, pomdp_planner, pomdp_details, exp_details, output)

#=
Print useful values from the experiment
=#

#=
Create Gif
=#
create_gif = true
# create_gif = false
if(create_gif)
    vehicle_executed_trajectory = []
    anim = @animate for k ∈ keys(output.sim_objects)
        observe(output, exp_details, k, vehicle_executed_trajectory);
    end
    gif(anim, "es_planner.gif", fps = 10)
end


# nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
# vsd = VehicleSensor(HumanState[], Int64[], HumanGoalsBelief[])
# get_plot(env, veh, veh_params, nbh, vsd, 0.0, exp_details)
