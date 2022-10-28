Pkg.activate("/home/himanshu/Documents/Research/BellmanPDEs.jl/")
using BellmanPDEs
using Revise
include("struct_definition.jl")
include("environment.jl")
include("utils.jl")
include("pomdp_planning.jl")
include("belief_tracker.jl")
include("simulator.jl")
include("parser.jl")
include("visualization.jl")
include("HJB_wrappers.jl")
include("aspen_inputs.jl")
include("aspen_inputs2.jl")
include("no_obstacles_small.jl")
include("no_obstacles_big.jl")

#Initialization
input_config = scenario1_small
input_config = scenario1_big
input_config = aspen
input_config = aspen2
pomdp_details = POMPDPlanningDetails(input_config)
exp_details = ExperimentDetails(input_config)
output = OutputObj()
env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
exp_details.env = env
exp_details.human_goal_locations = get_human_goals(env)
veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
veh_params = VehicleParametersESPlanner(input_config.veh_L,input_config.veh_max_speed,veh_goal)
env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,exp_details.user_defined_rng)
initial_sim_obj = Simulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

#=
Solve HJB equation for the given environment and vehicle
=#
# HJB_planning_details = HJBPlanningDetails(0.5,100,0.1,0.475,veh_params.max_speed)
# rollout_guide = HJBPolicy(HJB_planning_details, exp_details, veh_params)

#=
Define POMDP, POMDP Solver and POMDP Planner
=#
extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,exp_details,veh_params,rollout_guide)
pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->HJB_calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    T_max=pomdp_details.planning_time,tree_in_info=true)
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
        # observe(output, path_planning_details, exp_details, k);
        observe(output, exp_details, exp_details, k);
    end
    gif(anim, "es_planner.gif", fps = 10)
end
