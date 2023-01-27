include("../main.jl")
include("../configs/small_obstacles_20x20.jl")
input_config = small_obstacles_20x20
s = load("./src/HJB_rollout_guide.jld2")
rollout_guide = s["rollout_guide"];

function run_experiment_pipeline(num_humans,num_experiments)

    ESP_outputs = OrderedDict()
    LSP_outputs = OrderedDict()
    experiment_seeds = OrderedDict()
    input_config.num_humans_env = num_humans

    for i in 1:num_experiments
        rand_experiment_seed = rand(UInt32)
        experiment_seeds[i] = rand_experiment_seed
        input_config.rng = MersenneTwister(rand_experiment_seed)
        println("Running Experiment #",i," for ES planner")
        esp_output = run_extended_space_planner_experiment(input_config, rollout_guide)
        ESP_outputs[i] = esp_output
        input_config.rng = MersenneTwister(rand_experiment_seed)
        println("Running Experiment #",i," for LS planner")
        lsp_output = run_limited_space_planner_experiment(input_config);
        LSP_outputs[i] = lsp_output
    end

    return experiment_seeds , ESP_outputs, LSP_outputs
end

# num_humans = parse(Int,ARGS[1])
# num_experiments = parse(Int,ARGS[2])


#=
Code to run this
s,e,l = run_experiment_pipeline(25,5);
0x343b2563
=#
