include("main.jl")

mutable struct PipelineIndividualOutput
    number_sudden_stops::Int64
    number_risky_scenarios::Int64
    time_taken::Float64
    vehicle_body::VehicleBody
    vehicle_ran_into_boundary_wall::Bool
    vehicle_ran_into_obstacle::Bool
    vehicle_reached_goal::Bool
    vehicle_expected_trajectory::OrderedDict
    nearby_humans::OrderedDict
    vehicle_actions::OrderedDict
    sim_objects::OrderedDict
    risky_scenarios::OrderedDict
end

function PipelineIndividualOutput(output)
    return PipelineIndividualOutput(
        output.number_sudden_stops,#::Int64
        output.number_risky_scenarios,#::Int64
        output.time_taken,#::Float64
        output.vehicle_body,#::VehicleBody
        output.vehicle_ran_into_boundary_wall,#::Bool
        output.vehicle_ran_into_obstacle,#::Bool
        output.vehicle_reached_goal,#::Bool
        output.vehicle_expected_trajectory,#::OrderedDict
        output.nearby_humans,#::OrderedDict
        output.vehicle_actions,#::OrderedDict
        output.sim_objects,#::OrderedDict
        output.risky_scenarios,#::OrderedDict
    )
end

struct PipelineOutput{I,R}
    environment_name::String
    num_experiments::Int64
    num_humans::Int64
    sudden_break_flag::Bool
    run_shield_flag::Bool
    seeds::Array{UInt32,1}
    input_config::I
    rollout_guide::R
    baseline::Array{PipelineIndividualOutput,1}
    esp_random::Array{PipelineIndividualOutput,1}
    esp_sl::Array{PipelineIndividualOutput,1}
    esp_hjb::Array{PipelineIndividualOutput,1}
end

function PipelineOutput(env_name, num_experiments, num_humans, sudden_break, run_shield,
                        config, rollout_guide, seeds=nothing)

    if(isnothing(seeds))
        seeds = [rand(UInt32) for i in 1:num_experiments]
    else
        @assert length(seeds) == num_experiments
    end
    baseline = Array{PipelineIndividualOutput,1}(undef,num_experiments)
    esp_random = Array{PipelineIndividualOutput,1}(undef,num_experiments)
    esp_sl = Array{PipelineIndividualOutput,1}(undef,num_experiments)
    esp_hjb = Array{PipelineIndividualOutput,1}(undef,num_experiments)

    return PipelineOutput(
        env_name,
        num_experiments,
        num_humans,
        sudden_break,
        run_shield,
        seeds,
        config,
        rollout_guide,
        baseline,
        esp_random,
        esp_sl,
        esp_hjb
    )
end

function run_pipeline!(output_obj)

    print_logs = false

    (;num_experiments, num_humans, sudden_break_flag, run_shield_flag, seeds,
    input_config,rollout_guide,baseline,esp_random,esp_sl,esp_hjb) = output_obj
    input_config.num_humans_env = num_humans

    num_planners = 4
    planner_names = (:lsp,:esp_r,:esp_sl,:esp_hjb)
    planner_functions = (
    lsp=run_extended_space_planner_experiment_random_rollout,
    esp_r=run_extended_space_planner_experiment_random_rollout,
    esp_sl=run_extended_space_planner_experiment_straight_line_rollout,
    esp_hjb=run_extended_space_planner_experiment_HJB_rollout
    )
    run_planner_flags = (lsp=true,esp_r=true,esp_sl=true,esp_hjb=true)

    try
        # Threads.@threads for i in 1:num_experiments
        for i in 1:num_experiments
            seed = seeds[i]

            Threads.@threads for planner_id in 1:num_planners
                planner = different_planners[planner_id]
                flag = run_planner_flags[planner]
                if(flag)
                    IC = deepcopy(input_config) #To ensure the rng doesn't get modified between experiments when multi-threading
                    IC.rng = MersenneTwister(seed)
                    println("************ Running Experiment Number : ", i, " with LS Planner ************")
                    planner_func = planner_functions[planner]
                    o = run_limited_space_planner_experiment(IC,sudden_break_flag,run_shield_flag,print_logs);
                    # push!(baseline,l)
                    baseline_result = PipelineIndividualOutput(l);
                    # push!(baseline,baseline_result)
                    baseline[i] = baseline_result
                end


            if(random_flag)
                IC = deepcopy(input_config) #To ensure the rng doesn't get modified between experiments when multi-threading
                IC.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - Random Rollouts ************")
                e = run_extended_space_planner_experiment_random_rollout(IC, rollout_guide,sudden_break_flag,
                                                            run_shield_flag,print_logs);
                # push!(esp_outputs,e)
                esp_random_result = PipelineIndividualOutput(e)
                # push!(esp_random,esp_random_result)
                esp_random[i] = esp_random_result
            end


            if(straight_line_flag)
                IC = deepcopy(input_config) #To ensure the rng doesn't get modified between experiments when multi-threading
                IC.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - Straight Line Rollouts ************")
                e = run_extended_space_planner_experiment_straight_line_rollout(IC,rollout_guide,sudden_break_flag,
                                                            run_shield_flag,print_logs);
                # push!(esp_outputs,e)
                esp_sl_result = PipelineIndividualOutput(e)
                # push!(esp_sl,esp_sl_result)
                esp_sl[i] = esp_sl_result
            end


            if(HJB_flag)
                IC = deepcopy(input_config)
                IC.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - HJB Rollouts ************")
                e = run_extended_space_planner_experiment_HJB_rollout(IC,rollout_guide,sudden_break_flag,
                                                            run_shield_flag,print_logs)
                # push!(esp_outputs,e)
                esp_hjb_result = PipelineIndividualOutput(e)
                # push!(esp_hjb,esp_hjb_result)
                esp_hjb[i] = esp_hjb_result
            end

            GC.gc()
        end
    catch ex
        println("The experiment pipeline failed :(")
        println(ex)
    end
end


function get_time_results(experimental_data)

    # (;num_experiments,baseline,esp_random,esp_sl,esp_hjb) = output_obj

    #Filter out the results where the vehicle reached the goal
    successful_trials = filter(x->x.vehicle_reached_goal == true, experimental_data)
    num_successful_trials = length(successful_trials)
    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)

    #Filter out the results where the vehicle had no collisions and reached the goal
    collision_free_trials = filter(x->x.number_risky_scenarios == 0, successful_trials)
    num_collision_free_trials = length(collision_free_trials)
    println("Number of successful and collision free trials : ", num_collision_free_trials)

    #Calculate mean and std error on time
    mean_time = mean(d.time_taken for d in collision_free_trials)
    std_error_time = std(d.time_taken for d in collision_free_trials)/sqrt(num_collision_free_trials)
    println("Trajectory Time (in seconds) : ", mean_time, " +/- ", std_error_time)

    return(mean_time,std_error_time)
end


function get_sudden_break_results(experimental_data)

    # (;num_experiments,baseline,esp_random,esp_sl,esp_hjb) = output_obj

    #Filter out the results where the vehicle reached the goal
    successful_trials = filter(x->x.vehicle_reached_goal == true, experimental_data)
    num_successful_trials = length(successful_trials)
    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)

    #Filter out the results where the vehicle had no collisions and reached the goal
    collision_free_trials = filter(x->x.number_risky_scenarios == 0, successful_trials)
    num_collision_free_trials = length(collision_free_trials)
    println("Number of successful and collision free trials : ", num_collision_free_trials)

    #Calculate mean and std error on number of SB actions
    mean_SB = mean(d.number_sudden_stops for d in collision_free_trials)
    std_error_SB = std(d.number_sudden_stops for d in collision_free_trials)/sqrt(num_collision_free_trials)
    println("Number of Sudden Break actions : ", mean_SB, " +/- ", std_error_SB)

    return(mean_SB,std_error_SB)
end


function get_num_collisions(experimental_data, min_dist)

    #Filter out the results where the vehicle reached the goal
    successful_trials = filter(x->x.vehicle_reached_goal == true, experimental_data)
    num_successful_trials = length(successful_trials)
    println("Number of Successful trials where the vehicle reached the goal : ", num_successful_trials)

    collision_per_successful_experiment = [count_num_collisions(d.risky_scenarios,min_dist) for d in successful_trials]

    num_unsafe_experiments = 0
    for collision_num in collision_per_successful_experiment
        if(collision_num!=0)
            num_unsafe_experiments+=1
        end
    end

    println("Number of Experiments where collisions happened : ", num_unsafe_experiments)

    #Calculate mean and std error on number of collisions
    mean_collisions = mean(collision_per_successful_experiment)
    std_error_collisions = std(collision_per_successful_experiment)/sqrt(num_successful_trials)
    println("Number of collisions : ", mean_collisions, " +/- ", std_error_collisions)

    return(mean_collisions,std_error_collisions)
end

function count_num_collisions(risky_scenarios, min_safe_distance_from_human)

    distinct_human_ids = Set{Int}()
    num_distinct_collisions = 0

    for (time_stamp,scenario) in risky_scenarios
        vehicle = scenario.vehicle
        lidar_data = scenario.vehicle_sensor_data.lidar_data
        ids = scenario.vehicle_sensor_data.ids

        vehicle_center_x = vehicle.x # + vehicle_params.dist_origin_to_center*cos(vehicle.theta)
        vehicle_center_y = vehicle.y #+ vehicle_params.dist_origin_to_center*sin(vehicle.theta)

        for (index,human) in enumerate(lidar_data)
            euclidean_distance = sqrt( (human.x - vehicle_center_x)^2 + (human.y - vehicle_center_y)^2 )
            # if(euclidean_distance<=(vehicle_params.radius+min_safe_distance_from_human))
            if(euclidean_distance<=(min_safe_distance_from_human))
                colliding_human_id = ids[index]
                if( !(colliding_human_id in distinct_human_ids))
                    num_distinct_collisions += 1
                    push!(distinct_human_ids,colliding_human_id)
                end
            end
        end
    end

    @assert length(distinct_human_ids) == num_distinct_collisions
    return num_distinct_collisions
end


function get_num_outperformed(baseline, proposed_planner)

    num_experiments = length(baseline)
    num_outperformed = 0

    for i in 1:num_experiments
        if( baseline[i].vehicle_reached_goal && proposed_planner[i].vehicle_reached_goal &&
            baseline[i].number_risky_scenarios == 0 && proposed_planner[i].number_risky_scenarios == 0
            )
            if(baseline[i].time_taken > proposed_planner[i].time_taken)
                num_outperformed += 1
            end
        end
    end

    println("Number of Experiments where Proposed Approach outperformed the baseline : ", num_outperformed)
    return num_outperformed
end

function extract_one_executed_path(output)

    path_x = Float64[]
    path_y = Float64[]
    for (i,sim_obj) in output.sim_objects
        vehicle = sim_obj.vehicle
        push!(path_x, vehicle.x)
        push!(path_y, vehicle.y)
    end
    return path_x,path_y
end

function extract_all_executed_paths(results)

    all_executed_paths = Vector{Pair{Int64, Tuple{Vector{Float64}, Vector{Float64}}}}()
    num_experiments = length(results)

    for i in 1:num_experiments
        if(results[i].vehicle_reached_goal)
            px,py = extract_one_executed_path(results[i])
            push!( all_executed_paths, i=> (px,py) )
        end
    end

    return all_executed_paths
end


function visualize_env(env, vehicle_goal, exp_details)

    l = env.length
    b = env.breadth
    snapshot = plot(aspect_ratio=:equal,size=(1000,1000), dpi=300,
        axis=([], false),
        # xticks=0:2:l, yticks=0:2:b,
        # xlabel="x-axis [m]", ylabel="y-axis [m]",
        # legend=:bottom,
        # legend=false
        )

    #Plot Workspace
    turf_color = :white
    plot!(snapshot, rectangleShape(0.0,0.0,env.length,env.breadth),opacity=0.1,color=turf_color,linewidth=2.0,linecolor=:black,label="")


    #Plot External Anchors
    anchor_dist = 0.5
    scatter!(snapshot,
        [0.0 - anchor_dist, env.length + anchor_dist, env.length + anchor_dist, 0.0 - anchor_dist],
        [0.0 - anchor_dist, 0.0 - anchor_dist, env.breadth + anchor_dist, env.breadth + anchor_dist],
        markeralpha=0.0, fillalpha=0.0, label="")

    #Plot Obstacles
    obstacle_color = :black
    for obs in env.obstacles
        plot!(snapshot, circleShape(obs.x,obs.y,obs.r), color=obstacle_color, linewidth=2.0, linecolor = obstacle_color,
            legend=false, fillalpha=0.4, aspect_ratio=1, label="", seriestype = [:shape,])
        # Plots.annotate!(snapshot,obs.x, obs.y, text("Obs", obstacle_color, :center, 10))
    end

    #Plot Vehicle Goal
    vehicle_goal_color = :yellow
    Plots.annotate!(snapshot,vehicle_goal.x, vehicle_goal.y, text("GOAL", :brown, :center, 30))

    return snapshot
end


function visualize_all_executed_paths( input_config, all_executed_paths, col)
    exp_details = ExperimentDetails(input_config)
    env = generate_environment(input_config.env_length,input_config.env_breadth,input_config.obstacles)
    veh_goal = Location(input_config.veh_goal_x,input_config.veh_goal_y)
    snapshot = visualize_env(env, veh_goal, exp_details)
    Plots.annotate!(snapshot,input_config.veh_start_x, input_config.veh_start_y,
                                    text("START", :brown, :center, 30))
    for (i,path) in all_executed_paths
        px,py = path[1],path[2]
        plot!(snapshot, px, py, linewidth=2,label="",color=col)
    end
    display(snapshot)
    return snapshot
end


function run_experiment(environment_name, num_experiments, num_humans, sudden_break, run_shield )
    filename = "src/configs/"*environment_name*".jl"
    include(filename)
    rollout_guide_filename = "./src/rollout_guides/HJB_rollout_guide_"*environment_name*".jld2"
    s = load(rollout_guide_filename)
    rollout_guide = s["rollout_guide"];
    data = PipelineOutput(environment_name,num_experiments,num_humans,sudden_break,
                            run_shield,input_config,rollout_guide);
    run_pipeline!(data)
    return data
end


#=
Pipeline code

include("src/experiment_pipeline.jl")

environment_name = "no_obstacles_25x25"
environment_name = "small_obstacles_25x25"
environment_name = "big_obstacle_25x25"
environment_name = "L_shape_25x25"
environment_name = "no_obstacles_50x50"
environment_name = "small_obstacles_50x50"
environment_name = "big_obstacle_50x50"
environment_name = "L_shape_50x50"

environment_name = "small_obstacles_50x50"

filename = "src/configs/"*environment_name*".jl"
include(filename)
input_config = small_obstacles_50x50
rollout_guide_filename = "./src/rollout_guides/HJB_rollout_guide_"*environment_name*".jld2"
s = load(rollout_guide_filename)
rollout_guide = s["rollout_guide"];
sudden_break = false
run_shield = false
num_experiments = 1


data_sb_no_shield_no = PipelineOutput(environment_name,num_experiments,false,false,
                            input_config,rollout_guide);
run_pipeline!(data_sb_no_shield_no)

data_sb_yes_shield_no = PipelineOutput(environment_name,num_experiments,true,false,
                            input_config,rollout_guide);
run_pipeline!(data_sb_yes_shield_no)

data_sb_no_shield_yes = PipelineOutput(environment_name,num_experiments,false,true,
                            input_config,rollout_guide);
run_pipeline!(data_sb_no_shield_yes)


data = data_sb_no_shield_no;
data = data_sb_yes_shield_no;

data = s["data"];

get_time_results(data.baseline)
get_time_results(data.esp_hjb)
get_time_results(data.esp_random)
get_time_results(data.esp_sl)

get_num_outperformed(data.baseline, data.esp_hjb)
get_num_outperformed(data.baseline, data.esp_random)
get_num_outperformed(data.baseline, data.esp_sl)

get_num_collisions(data.baseline,1.0)
get_num_collisions(data.esp_hjb,1.0)
get_num_collisions(data.esp_random,1.0)
get_num_collisions(data.esp_sl,1.0)

get_sudden_break_results(data.baseline)
get_sudden_break_results(data.esp_hjb)
get_sudden_break_results(data.esp_random)
get_sudden_break_results(data.esp_sl)



config_fn = "./src/configs/"*env_name*".jl"
include(config_fn)

all_paths_hjb = extract_all_executed_paths(data.esp_hjb);
p = visualize_all_executed_paths(input_config,all_paths_hjb, :olive);
esp_fn = "esp_"*env_name*"_paths.svg"
Plots.savefig(p,esp_fn)

all_paths_baseline = extract_all_executed_paths(data.baseline);
p = visualize_all_executed_paths(input_config, all_paths_baseline, :steelblue);
lsp_fn = "lsp_"*env_name*"_paths.svg"
Plots.savefig(p,lsp_fn)


num_experiments = 100
environment_name = "small_obstacles_25x25"

data_10_humans_no_sb_no_shield = run_experiment(environment_name, num_experiments, 10, false, false )
data_20_humans_no_sb_no_shield = run_experiment(environment_name, num_experiments, 20, false, false )
data_30_humans_no_sb_no_shield = run_experiment(environment_name, num_experiments, 30, false, false )
data_40_humans_no_sb_no_shield = run_experiment(environment_name, num_experiments, 40, false, false )

data_10_humans_yes_sb_no_shield = run_experiment(environment_name, num_experiments, 10, true, false )
data_20_humans_yes_sb_no_shield = run_experiment(environment_name, num_experiments, 20, true, false )
data_30_humans_yes_sb_no_shield = run_experiment(environment_name, num_experiments, 30, true, false )
data_40_humans_yes_sb_no_shield = run_experiment(environment_name, num_experiments, 40, true, false )

data_10_humans_no_sb_yes_shield = run_experiment(environment_name, num_experiments, 10, false, true )
data_20_humans_no_sb_yes_shield = run_experiment(environment_name, num_experiments, 20, false, true )
data_30_humans_no_sb_yes_shield = run_experiment(environment_name, num_experiments, 30, false, true )
data_40_humans_no_sb_yes_shield = run_experiment(environment_name, num_experiments, 40, false, true )

num_experiments = 100
outdoor_environment_names = ("no_obstacles_50x50", "small_obstacles_50x50", "L_shape_50x50")
for env_name in outdoor_environment_names
    for num_humans in (50,100,200)
        for sudden_break in (false, true)
            for run_shield in (false)
                data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
                datafile_name = env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
                data_dict = Dict("data"=>data);
                save(datafile_name, data_dict)
            end
        end
    end
end

num_experiments = 100
indoor_environment_names = ("no_obstacles_25x25", "small_obstacles_25x25", "L_shape_25x25")
for env_name in indoor_environment_names
    for num_humans in (10,20,30,40)
        for sudden_break in (false, true)
             for run_shield in (false)
                data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
                datafile_name = string(num_experiments)*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
                data_dict = Dict("data"=>data);
                save(datafile_name, data_dict)
            end
        end
    end
end



num_experiments = 25
new_environment_names = ("many_small_obstacles_50x50", "big_obstacle_50x50")
for env_name in new_environment_names
    for num_humans in (50,100,200)
        for sudden_break in (false, true)
             for run_shield in (false)
                data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
                datafile_name = string(num_experiments)*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
                data_dict = Dict("data"=>data);
                save(datafile_name, data_dict)
                data = ""
                GC.gc()
            end
        end
    end
end


Threads.@theads for entry in tuples
    data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
    datafile_name = env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
    data_dict = Dict("data"=>data);
    save(datafile_name, data_dict)
end




num_experiments = 25
new_environment_names = ("no_obstacles_50x50", "small_obstacles_50x50", "L_shape_50x50","many_small_obstacles_50x50", "big_obstacle_50x50")
for env_name in new_environment_names
    for num_humans in (50,100,200)
        for sudden_break in (false, true)
             for run_shield in (false)
                data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
                datafile_name = string(num_experiments)*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
                data_dict = Dict("data"=>data);
                save(datafile_name, data_dict)
                data_dict = ""
                data = ""
                GC.gc()
            end
        end
    end
end

all_environment_names = ("no_obstacles_50x50", "small_obstacles_50x50", "L_shape_50x50","many_small_obstacles_50x50", "big_obstacle_50x50")


num_experiments = 10
all_environment_names = ("many_small_obstacles_50x50", "big_obstacle_50x50")
for env_name in all_environment_names
    for num_humans in (50,100,200)
        for (sudden_break,run_shield) in ( (false,false), (true,false), (false,true) )
            data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
            datafile_name = "./RESULTS/"*string(num_experiments)*"_"*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
            data_dict = Dict("data"=>data);
            save(datafile_name, data_dict)
            data_dict = ""
            data = ""
            GC.gc()
        end
    end
end


num_experiments = 10
all_environment_names = ("many_small_obstacles_50x50", "big_obstacle_50x50")
for env_name in all_environment_names
    for num_humans in (50,100,200)
        for (sudden_break,run_shield) in [ (false,true) ]
            data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
            datafile_name = "./RESULTS/"*string(num_experiments)*"_"*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
            data_dict = Dict("data"=>data);
            save(datafile_name, data_dict)
            data_dict = ""
            data = ""
            GC.gc()
        end
    end
end




num_experiments = 100
env_name = "many_small_obstacles_50x50"
num_humans = 200


for (sudden_break,run_shield) in [ (false,false) ]
    data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
    datafile_name = "./RESULTS/"*string(num_experiments)*"_"*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
    data_dict = Dict("data"=>data);
    save(datafile_name, data_dict)
    data_dict = ""
    data = ""
end

GC.gc();

for (sudden_break,run_shield) in [ (true,false) ]
    data = run_experiment(env_name, num_experiments, num_humans, sudden_break, run_shield )
    datafile_name = "./RESULTS/"*string(num_experiments)*"_"*env_name*"_humans_"*string(num_humans)*"_suddenbreak_"*string(sudden_break)*"_runshield_"*string(run_shield)*".jld2"
    data_dict = Dict("data"=>data);
    save(datafile_name, data_dict)
    data_dict = ""
    data = ""
end

=#
