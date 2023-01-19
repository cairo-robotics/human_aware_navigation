using Pkg
user = "Himanshu"
if user == "Himanshu"
    Pkg.activate("/home/himanshu/Documents/Research/BellmanPDEs.jl/")
elseif user == "Will"
    Pkg.activate("/Users/willpope/.julia/dev/BellmanPDEs")
end
using BellmanPDEs
using LazySets
using JLD2
using ProfileView
using Revise
include("struct_definition.jl")
include("environment.jl")
include("utils.jl")
include("hybrid_astar.jl")
include("LS_POMDP_Planner.jl")
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

input_config = small_obstacles_20x20


function run_extended_space_planner_experiment(input_config, rollout_guide)
    #Define experiment details and POMDP planning details
    pomdp_details = POMPDPlanningDetails(input_config)
    exp_details = ExperimentDetails(input_config)
    output = OutputObj()
    #Define environment
    env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
    exp_details.env = env
    exp_details.human_goal_locations = get_human_goals(env)
    #Define Vehicle
    veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
    veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
    veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
    r = sqrt( (0.5*input_config.veh_length)^2 + (0.5*input_config.veh_breadth)^2 )
    veh_params = VehicleParametersESPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal)
    veh_body_origin = get_vehicle_body_origin(veh_params.dist_origin_to_center,0.0,veh_params.length,veh_params.breadth)
    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)
    #Define POMDP, POMDP Solver and POMDP Planner
    extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide)
    pomdp_solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),max_depth=pomdp_details.tree_search_max_depth),
                calculate_upper_bound,check_terminal=true,consistency_fix_thresh=1e-5),K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                T_max=pomdp_details.planning_time*1,tree_in_info=true)#,default_action=default_es_pomdp_action)
    pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)
    run_experiment!(initial_sim_obj, pomdp_planner, pomdp_details, exp_details, output)

    #Create Gif
    # create_gif = true
    create_gif = false
    if(create_gif)
        # vehicle_executed_trajectory = []
        # anim = @animate for k ∈ keys(output.sim_objects)
        #     observe(output, exp_details, k, veh_body_origin, vehicle_executed_trajectory);
        # end
        # gif(anim, "es_planner.gif", fps = 10)
        generate_gif(output, exp_details, veh_body_origin)
    end
    return output
end


function run_limited_space_planner_experiment(input_config)
    #Define experiment details and POMDP planning details
    pomdp_details = POMPDPlanningDetails(input_config)
    pomdp_details.planning_time = input_config.LS_pomdp_planning_time
    exp_details = ExperimentDetails(input_config)
    output = OutputObj()
    #Define environment
    env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
    exp_details.env = env
    exp_details.human_goal_locations = get_human_goals(env)
    #Define path planning details
    path_planning_details = PathPlanningDetails(input_config, env)
    #Define Vehicle
    veh = Vehicle(input_config.veh_start_x, input_config.veh_start_y, input_config.veh_start_theta, input_config.veh_start_v)
    veh_sensor_data = VehicleSensor(HumanState[],Int64[],HumanGoalsBelief[])
    veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
    r = sqrt( (0.5*input_config.veh_length)^2 + (0.5*input_config.veh_breadth)^2 )
    temp_veh_params = VehicleParametersLSPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal,Float64[])
    veh_body_origin = get_vehicle_body_origin(temp_veh_params.dist_origin_to_center, 0.0,temp_veh_params.length, temp_veh_params.breadth)
    #Find hybrid A* path for the given environment and vehicle.
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    vehicle_delta_angle_actions = get_vehicle_actions(45,5)
    vehicle_controls_sequence = hybrid_astar_search(env,veh,temp_veh_params,vehicle_delta_angle_actions,nbh,path_planning_details);
    veh_params = VehicleParametersLSPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal,vehicle_controls_sequence)
    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)
    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)
    #Run the experiment
    run_experiment!(initial_sim_obj, path_planning_details, pomdp_details, exp_details, output)
    #Create Gif
    # create_gif = true
    create_gif = false
    if(create_gif)
        generate_gif(output, exp_details, veh_body_origin)
        # vehicle_executed_trajectory = []
        # anim = @animate for k ∈ keys(output.sim_objects)
        #     observe(output, exp_details, k, veh_body_origin, vehicle_executed_trajectory);
        # end
        # gif(anim, "es_planner.gif", fps = 10)
    end
    return output
end
