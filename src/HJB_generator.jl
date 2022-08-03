# Hamilton-Jacobi-Bellman demonstration
using Plots
using BSON
using Debugger
using BenchmarkTools
using ProfileView
using Random

include("HJB_generator_functions.jl")
include("HJB_planner_functions.jl")
include("HJB_utils.jl")

run_HJB_flag = false
dt = 0.1
max_plan_steps = 10

actual_env = generate_environment_small_circular_obstacles(300, MersenneTwister(100))
# actual_env = generate_environment_no_obstacles(300, MersenneTwister(100))
mvs = 4.0  #max_vehicle_speed
mdoa = pi/4
HJB_env, HJB_vehicle, HJB_actions = get_HJB_env_vehicle_actions(actual_env,mvs,mdoa)
U_HJB,T,O = get_HJB_value_function(HJB_env,HJB_vehicle,HJB_actions,run_HJB_flag)

# generate optimal path
start_pose = [25, 80.0, 3*pi/2]
# start_pose = [26.491692934024023, 76.39873468778308, -0.7853981633974478]
start_pose = [4.346092283518907, 2.0907808788783315, 3.718524340685583]
# start_pose = [15.756686126183512, 18.642342489554505, 6.195687001120754]
start_pose = [54.95130762957608, 41.41436061505876, 1.596976265574817]
println(start_pose)
x_path, u_path, num_steps = @btime HJB_planner(start_pose, U_HJB, dt, max_plan_steps, HJB_actions, O, HJB_env, HJB_vehicle)
path_time = num_steps*dt
println("path execution time: ", path_time, " sec")
plot_HJB_results(HJB_env, HJB_vehicle, start_pose, x_path)

# HJB_action(start_pose, U_HJB, HJB_actions, O, HJB_env, HJB_vehicle)

# # vehicle parameters
# marmot = Vehicle(4.0, 2.0, 0.475, 0.324, 0.5207, 0.2762, 0.0889)
# # unit_car = Vehicle(1.0, 0.5, 0.5, 0.5, 0.75, 0.375, 0.125)
# HJB_vehicle = marmot
#
# Am = [[a_v,a_phi] for a_v in [-HJB_vehicle.c_vb, HJB_vehicle.c_vf], a_phi in [-HJB_vehicle.c_phi, 0.0, HJB_vehicle.c_phi]]
# HJB_actions = reshape(Am, (length(Am),1))
# sort!(HJB_actions, dims=1)
#
# # define workspace
# W = [[0.0 0.0];
#     [100.0 0.0];
#     [100.0 100.0];
#     [0.0 100.0]]
#
# # define target set
# # T_xy = [[97.0 74.0];
# #         [100.0 74.0];
# #         [100.0 76.0];
# #         [97.0 76.0]]
# T_xy = circle_to_polygon([99.0,74.0,2.0])
# T_theta = [[-pi, pi]]
#
# # define obstacles --> circular obstacles defined as [x, y, r], converted to polygon overapproximation
# OC1 = circle_to_polygon([65.0, 35.0, 30.0])
# OC2 = circle_to_polygon([35.0, 70.0, 7.0])
# O_vec = [OC1, OC2]
#
#
# # initialize state grid
# h_xy = 1.0
# h_theta = deg2rad(15)
# HJB_env = Environment(h_xy,
#                 h_theta,
#                 minimum(W[:,1]) : h_xy : maximum(W[:,1]),
#                 minimum(W[:,2]) : h_xy : maximum(W[:,2]),
#                 -pi : h_theta : pi,
#                 W,
#                 T_xy,
#                 T_theta,
#                 O_vec)
#
# println("\nstart --- --- ---")
#
# algs_path_nuc = "/adcl/..."
# algs_path = algs_path_mac
