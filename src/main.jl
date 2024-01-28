using BellmanPDEs
using JLD2
# using ProfileView
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
include("shielding/lsp_shield_wrappers.jl")


function run_extended_space_planner_experiment_HJB_rollout(input_config, rollout_guide,
                                        sudden_break_flag, run_shield_flag, print_logs=true, create_gif=false)

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
    vehicle_body = get_vehicle_body(veh_params)
    output.vehicle_body = vehicle_body
    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)
    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

    #Define POMDP, POMDP Solver and POMDP Planner
    sol_rng = MersenneTwister(19)
    extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide,sudden_break_flag)
    lower_bound_func = DefaultPolicyLB(
                            FunctionPolicy(b->calculate_lower_bound(extended_space_pomdp, b)),
                            max_depth=pomdp_details.tree_search_max_depth)
    upper_bound_func = old_calculate_upper_bound
    pomdp_solver = DESPOTSolver(
                        bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                        K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                        tree_in_info=true,
                        T_max=0.4,
                        max_trials=100,
                        default_action=get_default_action,
                        rng=sol_rng
                        )
    pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
    #Run the experiment
    try
        run_experiment!(initial_sim_obj, pomdp_planner, lower_bound_func, upper_bound_func,
                        pomdp_details, exp_details, output, run_shield_flag, print_logs)
        #Create Gif
        if(create_gif)
            generate_gif(output, exp_details)
        end
    catch ex
        println("Experiment Failed. Coding Error")
        # println(ex)
    end
    return output
end


function run_limited_space_planner_experiment(input_config,
                    sudden_break_flag, run_shield_flag, print_logs=true, create_gif=false)

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

    #Find hybrid A* path for the given environment and vehicle.
    nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
    vehicle_delta_angle_actions = get_vehicle_actions(45,5)
    given_planning_time = path_planning_details.planning_time
    path_planning_details.planning_time = 10.0
    vehicle_controls_sequence = hybrid_astar_search(env,veh,temp_veh_params,vehicle_delta_angle_actions,nbh,path_planning_details);
    path_planning_details.planning_time = given_planning_time
    veh_params = VehicleParametersLSPlanner(input_config.veh_wheelbase,input_config.veh_length,
                    input_config.veh_breadth,input_config.veh_dist_origin_to_center, r,
                    input_config.veh_max_speed,input_config.veh_max_steering_angle,veh_goal,vehicle_controls_sequence)
    vehicle_body = get_vehicle_body(veh_params)
    output.vehicle_body = vehicle_body

    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)

    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)
    #Run the experiment
    try
        run_experiment!(initial_sim_obj, path_planning_details, pomdp_details, exp_details, output,
                            sudden_break_flag, run_shield_flag, print_logs)
        #Create Gif
        if(create_gif)
            generate_gif(output, exp_details)
        end
    catch ex
        println("Experiment Failed. Coding Error")
        println(ex)
    end
    return output
end

function run_extended_space_planner_experiment_random_rollout(input_config, rollout_guide,
                                    sudden_break_flag, run_shield_flag, print_logs=true, create_gif=false)

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
    vehicle_body = get_vehicle_body(veh_params)
    output.vehicle_body = vehicle_body
    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)
    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

    #Define POMDP, POMDP Solver and POMDP Planner
    rollout_guide = Tuple(true)
    extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide,sudden_break_flag)
    sol_rng = MersenneTwister(19)
    lower_bound_func = DefaultPolicyLB(RandomPolicy(extended_space_pomdp));
    upper_bound_func = old_calculate_upper_bound
    pomdp_solver = DESPOTSolver(
                        bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                        K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                        tree_in_info=true,
                        T_max=0.4,
                        max_trials=100,
                        # default_action=default_es_pomdp_action,
                        rng=sol_rng
                        )
    pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
    #Run the experiment
    try
        run_experiment!(initial_sim_obj, pomdp_planner, lower_bound_func, upper_bound_func,
                        pomdp_details, exp_details, output, run_shield_flag, print_logs)
        #Create Gif
        if(create_gif)
            generate_gif(output, exp_details)
        end
    catch ex
        println("Experiment Failed. Coding Error")
        println(ex)
    end
    return output
end

function run_extended_space_planner_experiment_straight_line_rollout(input_config, rollout_guide,
                                            sudden_break_flag, run_shield_flag, print_logs=true, create_gif=false)

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
    vehicle_body = get_vehicle_body(veh_params)
    output.vehicle_body = vehicle_body
    #Define Humans
    env_humans, env_humans_params = generate_humans(env,veh,exp_details.human_start_v,exp_details.human_goal_locations,exp_details.num_humans_env,
                                            exp_details.simulator_time_step, exp_details.user_defined_rng)
    #Create sim object
    initial_sim_obj = NavigationSimulator(env,veh,veh_params,veh_sensor_data,env_humans,env_humans_params,exp_details.simulator_time_step)

    #Define POMDP, POMDP Solver and POMDP Planner
    rollout_guide = (true,true)
    extended_space_pomdp = ExtendedSpacePOMDP(pomdp_details,env,veh_params,rollout_guide,sudden_break_flag)
    sol_rng = MersenneTwister(19)
    lower_bound_func = DefaultPolicyLB(
                            FunctionPolicy(b->calculate_lower_bound_straight_line(extended_space_pomdp, b)),
                            max_depth=pomdp_details.tree_search_max_depth)
    upper_bound_func = old_calculate_upper_bound
    pomdp_solver = DESPOTSolver(
                        bounds=IndependentBounds(lower_bound_func,upper_bound_func,check_terminal=true,consistency_fix_thresh=1e-5),
                        K=pomdp_details.num_scenarios,D=pomdp_details.tree_search_max_depth,
                        tree_in_info=true,
                        T_max=0.4,
                        max_trials=100,
                        # default_action=default_es_pomdp_action,
                        rng=sol_rng
                        )
    pomdp_planner = POMDPs.solve(pomdp_solver, extended_space_pomdp);
    #Run the experiment
    try
        run_experiment!(initial_sim_obj, pomdp_planner, lower_bound_func, upper_bound_func,
                        pomdp_details, exp_details, output, run_shield_flag, print_logs)
        #Create Gifs
        if(create_gif)
            generate_gif(output, exp_details)
        end
    catch ex
        println("Experiment Failed. Coding Error")
        println(ex)
    end
    return output
end


function run_esp_experiment_HJB_rollout_without_SB_no_shielding(input_config, rollout_guide, create_gif=false)
    sudden_break_flag = false
    run_shield_flag = false
    return run_extended_space_planner_experiment_HJB_rollout(input_config, rollout_guide,
                                            sudden_break_flag, run_shield_flag, create_gif)
end

function run_esp_experiment_HJB_rollout_with_SB_no_shielding(input_config, rollout_guide)
    sudden_break_flag = true
    run_shield_flag = false
    return run_extended_space_planner_experiment_HJB_rollout(input_config, rollout_guide,
                                            sudden_break_flag, run_shield_flag, create_gif)
end

function run_esp_experiment_HJB_rollout_with_shielding(input_config, rollout_guide)
    #This by default means no sudden break action
    sudden_break_flag = false
    run_shield_flag = true
    return run_extended_space_planner_experiment_HJB_rollout(input_config, rollout_guide,
                                            sudden_break_flag, run_shield_flag, create_gif)
end


function run_lsp_experiment_without_SB_no_shielding(input_config, create_gif=false)
    sudden_break_flag = false
    run_shield_flag = false
    return run_limited_space_planner_experiment(input_config,sudden_break_flag,run_shield_flag,create_gif)
end

function run_lsp_experiment_with_SB_no_shielding(input_config,create_gif=false)
    sudden_break_flag = true
    run_shield_flag = false
    return run_limited_space_planner_experiment(input_config,sudden_break_flag,run_shield_flag,create_gif)
end

function run_lsp_experiment_with_shielding(input_config, rollout_guide)
    #This by default means no sudden break action
    sudden_break_flag = false
    run_shield_flag = true
    return run_limited_space_planner_experiment(input_config,sudden_break_flag,run_shield_flag,create_gif)
end



function old_run_pipeline(num_exp, seeds, input_config, rollout_guide)
    lsp_outputs = []
    esp_outputs_random = []
    esp_outputs_straight_line = []
    esp_outputs_HJB = []

    baseline_flag = true
    random_flag = false
    straight_line_flag = false
    HJB_flag = true


    try
        for i in 1:num_exp
            seed = seeds[i][2]

            if(baseline_flag)
                input_config.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with LS Planner ************")
                l = run_lsp_experiment_without_SB_no_shielding(input_config);
                # push!(lsp_outputs,l)
                push!(lsp_outputs,(i=> (l.vehicle_reached_goal,l.number_risky_scenarios,l.time_taken,l.number_sudden_stops)))
            end

            if(random_flag)
                input_config.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - Random Rollouts ************")
                e = run_extended_space_planner_experiment_random_rollout(input_config, rollout_guide);
                # push!(esp_outputs,e)
                push!(esp_outputs_random,(i=> (e.vehicle_reached_goal,e.number_risky_scenarios,e.time_taken,e.number_sudden_stops)))
            end

            if(straight_line_flag)
                input_config.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - Straight Line Rollouts ************")
                e = run_extended_space_planner_experiment_straight_line_rollout(input_config, rollout_guide);
                # push!(esp_outputs,e)
                push!(esp_outputs_straight_line,(i=> (e.vehicle_reached_goal,e.number_risky_scenarios,e.time_taken,e.number_sudden_stops)))
            end

            if(HJB_flag)
                input_config.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - HJB Rollouts ************")
                e = run_esp_experiment_HJB_rollout_without_SB_no_shielding(input_config, rollout_guide);
                # push!(esp_outputs,e)
                push!(esp_outputs_HJB,(i=> (e.vehicle_reached_goal,e.number_risky_scenarios,e.time_taken,e.number_sudden_stops)))
            end

            GC.gc()
        end
    catch ex
        println("The experiment pipeline failed :(")
        println(ex)
    end

    return seeds,lsp_outputs,esp_outputs_random,esp_outputs_straight_line,esp_outputs_HJB
end


function get_results(experimental_data)

    num_experiments = length(experimental_data)

    #Filter out the results where the vehicle reached the goal
    successful_trials = filter(x->x[2][1] == true, experimental_data)
    num_successful_trials = length(successful_trials)
    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)

    #Filter out the results where the vehicle had no collissions and reached the goal
    collission_free_trials = filter(x->x[2][2] == 0, successful_trials)
    num_collission_free_trials = length(collission_free_trials)
    println("Number of successful and collission free trials : ", num_collission_free_trials)

    #Calculate mean and std error on time
    mean_time = mean(d[2][3] for d in collission_free_trials)
    std_error_time = std(d[2][3] for d in collission_free_trials)/sqrt(num_experiments)
    println("Trajectory Time (in seconds) : ", mean_time, " +/- ", std_error_time)

    #Calculate mean and std error on number of SB actions
    mean_SB = mean(d[2][4] for d in collission_free_trials)
    std_error_SB = std(d[2][4] for d in collission_free_trials)/sqrt(num_experiments)
    println("Number of Sudden Break actions : ", mean_SB, " +/- ", std_error_SB)

    return(mean_time,std_error_time,mean_SB,std_error_SB)
end

function get_num_outperformed(lsp_data,esp_data)
    num_experiments = length(esp_data)
    num_outperformed = 0
    for i in 1:num_experiments
        if( lsp_data[i][2][1] && lsp_data[i][2][2]==0 && esp_data[i][2][1] && esp_data[i][2][2]==0)
            if(esp_data[i][2][3] < lsp_data[i][2][3] )
                num_outperformed += 1
            end
        end
    end
    println("Number of experiments in which proposed approach did better : ", num_outperformed)
    return num_outperformed
end

#=
Code to run things!

lsp_output = run_limited_space_planner_experiment(input_config);
s = load("./src/HJB_rollout_guide.jld2")
rollout_guide = s["rollout_guide"];
esp_output = run_extended_space_planner_experiment(input_config, rollout_guide);

=#


#=
Pipeline code

environment_name = "no_obstacles_25x25"
environment_name = "small_obstacles_25x25"
environment_name = "big_obstacle_25x25"
environment_name = "L_shape_25x25"
environment_name = "no_obstacles_50x50"
environment_name = "small_obstacles_50x50"
environment_name = "big_obstacle_50x50"
environment_name = "L_shape_50x50"

filename = "src/configs/"*environment_name*".jl"
include(filename)
input_config = small_obstacles_50x50
rollout_guide_filename = "./src/rollout_guides/HJB_rollout_guide_"*environment_name*".jld2"
s = load(rollout_guide_filename)
rollout_guide = s["rollout_guide"];

num_experiments = 1
seeds = [i=>rand(UInt32) for i in 1:num_experiments]
S,L,ER,ESL,EHJB = run_pipeline(num_experiments,seeds,input_config,rollout_guide);

get_results(L)
get_results(ER)
get_results(ESL)
get_results(EHJB)
get_num_outperformed(L,ER)
get_num_outperformed(L,ESL)
get_num_outperformed(L,EHJB)

#RESULTS!

#= *************************** Mean Time *************************** =#
Standard Error in Mean Time

mean(i[2][3] for i in L)
std(i[2][3] for i in L)/sqrt(num_experiments)
mean(i[2][3] for i in E)
std(i[2][3] for i in E)/sqrt(num_experiments)


#= *************************** Mean #SB Actions *************************** =#
Standard Error in #SB Actions

mean(i[2][4] for i in L)
std(i[2][4] for i in L)/sqrt(num_experiments)
mean(i[2][4] for i in E)
std(i[2][4] for i in E)/sqrt(num_experiments)

#= *************************** #Outperformed *************************** =#

num_outperformed = 0
for i in 1:num_experiments
    if( L[i][2][1] && E[i][2][1] && E[i][2][3] < L[i][2][3] )
        num_outperformed += 1
    end
end
num_outperformed

num_unsafe = 0
for i in 1:num_experiments
    if(E[i][2][2] != 0)
        num_unsafe += 1
        println(i)
    end
end
num_unsafe

num_unfinished = 0
for i in 1:num_experiments
    if(E[i][2][1] == false)
        num_unfinished += 1
        println(i)
    end
end
num_unfinished

=#
