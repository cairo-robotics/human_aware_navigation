using Pkg
user = "Himanshu"
if user == "Himanshu"
    Pkg.activate("/home/himanshu/Documents/Research/BellmanPDEs.jl/")
elseif user == "Will"
    Pkg.activate("/Users/willpope/.julia/dev/BellmanPDEs")
end
using BellmanPDEs
using BSON: @save, @load
using ProfileView
using Revise
include("struct_definition.jl")
include("environment.jl")
include("utils.jl")
include("hybrid_astar.jl")
include("LS_POMDP_Planner.jl")
include("belief_tracker.jl")
include("simulator.jl")
include("parser.jl")
include("visualization.jl")
include("aspen_inputs.jl")
include("aspen_inputs2.jl")
include("small_obstacles_20x20.jl")
include("no_obstacles_big.jl")

#Initialization
input_config = scenario1_big
input_config = aspen
input_config = aspen2
input_config = small_obstacles_20x20

#=
Define experiment details and POMDP planning details
=#
pomdp_details = POMPDPlanningDetails(input_config)
exp_details = ExperimentDetails(input_config)
output = OutputObj()

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
veh_params = VehicleParametersLSPlanner(input_config.veh_L,input_config.veh_max_speed,veh_goal,Float64[],0)

#=
Define Humans
=#
env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                        exp_details.simulator_time_step, exp_details.user_defined_rng)

#=
Create sim object
=#
initial_sim_obj = Simulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

#=
Find hybrid A* path for the given environment and vehicle.
=#
nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
vsd = VehicleSensor(HumanState[], Int64[], HumanGoalsBelief[])
va = get_vehicle_actions(45,15)
pd = PathPlanningDetails(0,20.0,1.0,100.0,Location[],1.0,1.0,0.99,0.5,1.0)
vehicle_controls_sequence = hybrid_astar_search(env,veh,veh_params,va,nbh,pd);
new_vp = VehicleParametersLSPlanner(vp.L,vp.max_speed,vp.goal,vehicle_controls_sequence,1)
rollout_guide = HybridAStarPolicy(vehicle_controls_sequence,length(vehicle_controls_sequence))

#=
Define POMDP, POMDP Solver and POMDP Planner
=#
planning_pomdp = LimitedSpacePOMDP(pomdp_details,exp_details,veh_params,rollout_guide)
pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(planning_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                    calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                    T_max=pomdp_details.planning_time,tree_in_info=true)
pomdp_planner = POMDPs.solve(pomdp_solver, planning_pomdp);

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
