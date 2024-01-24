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

function PipelineOutput(env_name, num_experiments, sudden_break, run_shield,
                        config, rollout_guide, seeds=nothing)

    if(isnothing(seeds))
        seeds = [rand(UInt32) for i in 1:num_experiments]
    else
        @assert length(seeds) == num_experiments
    end
    baseline = Array{PipelineIndividualOutput,1}()
    esp_random = Array{PipelineIndividualOutput,1}()
    esp_sl = Array{PipelineIndividualOutput,1}()
    esp_hjb = Array{PipelineIndividualOutput,1}()

    return PipelineOutput(
        env_name,
        num_experiments,
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

    baseline_flag = true
    random_flag = true
    straight_line_flag = true
    HJB_flag = true

    (;num_experiments, sudden_break_flag, run_shield_flag, seeds,
    input_config,rollout_guide,baseline,esp_random,esp_sl,esp_hjb) = output_obj

    try
        for i in 1:num_experiments
            seed = seeds[i]

            if(baseline_flag)
                input_config.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with LS Planner ************")
                l = run_limited_space_planner_experiment(input_config,sudden_break_flag,run_shield_flag);
                baseline_result = PipelineIndividualOutput(l);
                # push!(baseline,l)
                push!(baseline,baseline_result)
            end

            GC.gc()

            if(random_flag)
                input_config.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - Random Rollouts ************")
                e = run_extended_space_planner_experiment_random_rollout(input_config, rollout_guide,sudden_break_flag,run_shield_flag);
                # push!(esp_outputs,e)
                esp_random_result = PipelineIndividualOutput(e)
                push!(esp_random,esp_random_result)
            end

            GC.gc()

            if(straight_line_flag)
                input_config.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - Straight Line Rollouts ************")
                e = run_extended_space_planner_experiment_straight_line_rollout(input_config,rollout_guide,sudden_break_flag,run_shield_flag);
                # push!(esp_outputs,e)
                esp_sl_result = PipelineIndividualOutput(e)
                push!(esp_sl,esp_sl_result)
            end

            GC.gc()

            if(HJB_flag)
                input_config.rng = MersenneTwister(seed)
                println("************ Running Experiment Number : ", i, " with ES Planner - HJB Rollouts ************")
                e = run_extended_space_planner_experiment_HJB_rollout(input_config,rollout_guide,sudden_break_flag,run_shield_flag)
                # push!(esp_outputs,e)
                esp_hjb_result = PipelineIndividualOutput(e)
                push!(esp_hjb,esp_hjb_result)
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

get_time_results(data.baseline)
get_time_results(data.esp_hjb)
get_time_results(data.esp_random)
get_time_results(data.esp_sl)

get_sudden_break_results(data.baseline)
get_sudden_break_results(data.esp_hjb)
get_sudden_break_results(data.esp_random)
get_sudden_break_results(data.esp_sl)

get_num_collisions(data.baseline,1.0)
get_num_collisions(data.esp_hjb,1.0)
get_num_collisions(data.esp_random,1.0)
get_num_collisions(data.esp_sl,1.0)
=#
