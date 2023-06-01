#Load Required Packages
using DifferentialEquations
using DataStructures
using Random
using LinearAlgebra
include("utils.jl")

struct NodeBin
    discrete_x::Float64
    discrete_y::Float64
    discrete_theta::Float64
end

struct GraphNode
    x::Float64
    y::Float64
    theta::Float64
    actual_cost::Float64
    heuristic_cost::Float64
    action_taken_to_reach_here::Float64
    discrete_x::Float64
    discrete_y::Float64
    discrete_theta::Float64
    parent::Union{GraphNode,Nothing}
    time_stamp::Float64
end

function heuristic_cost(vehicle, goal, world)
    euclidean_distance =  sqrt( (vehicle.x - goal.x)^2 + (vehicle_state.y - goal.y)^2 )
    direct_line_to_goal_slope = wrap_between_0_and_2Pi(atan(goal.y-node.y,goal.x-node.x))
    orientation_cost = 10* dot( (cos(direct_line_to_goal_slope), sin(direct_line_to_goal_slope)) ,
                                (cos(node_theta), sin(node_theta)) )
    return euclidean_distance - orientation_cost
end

function get_path(current_node::GraphNode)
    controls_sequence = Float64[]
    # @show(current_node.actual_cost)
    while(current_node.parent!=nothing)
        # println(current_node.x," ",current_node.y," ",current_node.theta," ",current_node.action_taken_to_reach_here)
        push!(controls_sequence, current_node.action_taken_to_reach_here)
        current_node = current_node.parent
    end
    # println(current_node.x," ",current_node.y," ",current_node.theta," ",current_node.action_taken_to_reach_here)
    return reverse(controls_sequence)
end

function get_vehicle_actions(max_delta_angle, min_delta_angle_difference)
    set_of_delta_angles = Float64[0.0]
    half_num_actions = Int( floor(max_delta_angle/min_delta_angle_difference) )
    for i in 1:half_num_actions
        neg_angle = -min_delta_angle_difference*i*pi/180
        pos_angle = min_delta_angle_difference*i*pi/180
        if(wrap_between_0_and_2Pi(neg_angle) != wrap_between_0_and_2Pi(pos_angle))
            push!(set_of_delta_angles, neg_angle)
            push!(set_of_delta_angles, pos_angle)
        else
            push!(set_of_delta_angles, pos_angle)
        end
    end
    return set_of_delta_angles
end

# function get_new_vehicle_position(vehicle_x,vehicle_y,vehicle_theta,vehicle_L,steering_angle,vehicle_speed,time_duration)
#     new_x,new_y,new_theta = move_vehicle(vehicle_x,vehicle_y,vehicle_theta,vehicle_L,steering_angle,vehicle_speed,time_duration)
#     return (new_x,new_y,new_theta)
# end

function get_next_vehicle_state(current_vehicle_state,vehicle_params,delta_angle,vehicle_speed,time_duration)
    new_vehicle_state = vehicle_params.vehicle_dynamics(current_vehicle_state,vehicle_params,delta_angle,vehicle_speed,time_duration)
    return new_vehicle_state
end

function get_NodeBin(vehicle_state,world,bin_width)
    max_num_bins_x = ceil(world.length/bin_width)
    discrete_x = clamp(ceil(vehicle_state.x/bin_width),1.0,max_num_bins_x)
    max_num_bins_y = ceil(world.breadth/bin_width)
    discrete_y = clamp(ceil(vehicle_state.y/bin_width),1.0,max_num_bins_y)
    discrete_theta = ceil(vehicle_state.theta*180/pi)
    return NodeBin(discrete_x,discrete_y,discrete_theta)
end

function predict_human_state(human,world,time_step)
    noise = 0.0
    new_human_state = update_human_position(human,world.length,world.breadth,time_step,noise)
    return new_human_state
end

function node_cost(world,nearby_humans,vehicle_state,vehicle_params,action,time_stamp,planning_details)

    total_cost = 0.0
    vehicle_x = vehicle_state.x
    vehicle_y = vehicle_state.y
    vehicle_L = vehicle_params.wheelbase

    #Cost from going out of bounds
    if(vehicle_x>world.length-vehicle_L || vehicle_y>world.breadth-vehicle_L || vehicle_x<0.0+vehicle_L || vehicle_y<0.0+vehicle_L)
        return Inf
    end

    #Cost from collision with obstacles
    for obstacle in world.obstacles
        if(in_obstacle(vehicle_x,vehicle_y,obstacle,vehicle_L))
            return Inf
        else
            continue
        end
    end

    #Cost from potential collision with nearby humans
    num_nearby_humans = length(nearby_humans.position_data)
    for human_index in 1:num_nearby_humans
        human = nearby_humans.position_data[human_index]
        belief = nearby_humans.belief[human_index].pdf
        maximum_element, maximum_element_index = find_maximum_element(nearby_humans.belief[human_index].pdf,planning_details.num_human_goals)
        if(maximum_element < 0.5)
            euclidean_distance = sqrt( (vehicle_x - human.x)^2 + (vehicle_y - human.y)^2)
            if(euclidean_distance >= planning_details.radius_around_uncertain_human)
                continue
            elseif(euclidean_distance <= planning_details.min_safe_distance_from_human)
                return Inf
            else
                total_cost += planning_details.human_collision_cost * (1/euclidean_distance)
            end
        else
            inferred_human_goal = planning_details.human_goals[maximum_element_index]
            inferred_human_state = HumanState(human.x,human.y,human.v,inferred_human_goal)
            predicted_human_position = predict_human_state(inferred_human_state,world,time_stamp)
            euclidean_distance::Float64 = sqrt( (vehicle_x - predicted_human_position.x)^2 + (vehicle_y - predicted_human_position.y)^2 )
            if(euclidean_distance >= planning_details.radius_around_uncertain_human)
                continue
            elseif(euclidean_distance <= planning_details.min_safe_distance_from_human)
                return Inf
            else
                #total_cost += pedestrian_cost * (1/distance_of_cart_from_expected_human_position)
                continue
            end
        end
    end

    #Cost from no change in heading angle
    if(action == 0.0)
       total_cost += -1.0
    end

    #Cost from Long Paths
    total_cost += 1

    return total_cost
end

function hybrid_astar_search(world, vehicle_state, vehicle_params, vehicle_actions, nearby_humans, planning_details)

    vehicle_speed = path_planning_details.veh_path_planning_v
    one_time_step = planning_details.one_time_step
    discretization_bin_width = vehicle_speed*one_time_step
    path = Array{Float64,1}()
    open = PriorityQueue{NodeBin,Float64}(Base.Order.Forward)
    closed = Dict{NodeBin,GraphNode}()
    dict_of_nodes = Dict{NodeBin,GraphNode}()
    λ = planning_details.lambda
    time_stamp = 0.0        #Variable to keep track of path length

    node_key  = get_NodeBin(vehicle_state.x,vehicle_state.y,wrap_between_0_and_2Pi(vehicle_state.theta),world,discretization_bin_width)
    start_node = GraphNode(vehicle_state.x,vehicle_state.y,vehicle_state.theta,0.0,heuristic_cost(vehicle_state,vehicle_params.goal,world),-100.0,
                                                node_key.discrete_x,node_key.discrete_y,node_key.discrete_theta,nothing,time_stamp)
    open[node_key] = ((λ^start_node.time_stamp)*start_node.actual_cost) + start_node.heuristic_cost
    dict_of_nodes[node_key] = start_node
    start_time = time()

    while(length(open) != 0)
        current_node = dict_of_nodes[dequeue!(open)]
        if(is_within_range(current_node.x,current_node.y,vehicle_params.goal.x,vehicle_params.goal.y,planning_details.radius_around_vehicle_goal))
            println("Time taken to find the Hybrid A* path: ", time()- start_time)
            return get_path(current_node)
        end

        node_key = NodeBin(current_node.discrete_x,current_node.discrete_y,current_node.discrete_theta)
        closed[node_key] = current_node
        current_vehicle_state = VehicleState(current_node.x, current_node.y, current_node.theta, vehicle_speed)
        for control in vehicle_actions
            # steering_angle = get_steering_angle(vehicle_params.wheelbase,delta_heading_angle,planning_details.veh_path_planning_v,planning_details.one_time_step)
            # steering_angle = delta_heading_angle
            # new_x,new_y,new_theta = get_new_vehicle_position(current_node.x,current_node.y,current_node.theta,vehicle_params.wheelbase,steering_angle,planning_details.veh_path_planning_v,planning_details.one_time_step)

            new_vehicle_state = get_next_vehicle_state(current_vehicle_state,vehicle_params,control,vehicle_speed,one_time_step)
            node_key = NodeBin(discrete_x,discrete_y,discrete_theta)

            if(haskey(closed,node_key))
                continue
            end
            #Calculate actual cost of the new node
            new_time_stamp = current_node.time_stamp+one_time_step
            g = current_node.actual_cost + (λ^(new_time_stamp)) * node_cost(world,nearby_humans,new_vehicle_state,vehicle_params,control,new_time_stamp,planning_details)
            #Calculate heuristic cost of the new node
            h = heuristic_cost(new_vehicle_state,vehicle_params.goal,world)
            #Define new graph node
            new_node = GraphNode(new_vehicle_state.x,new_vehicle_state.y,new_vehicle_state.theta,g,h,control,node_key.discrete_x,node_key.discrete_y,
                                        node_key.discrete_theta,current_node,new_time_stamp)
            if(new_node.actual_cost == Inf)
                closed[node_key] = new_node
                continue
            end
            if(haskey(open,node_key))
                if(dict_of_nodes[node_key].actual_cost > new_node.actual_cost)
                    dict_of_nodes[node_key] = new_node
                    open[node_key] = new_node.heuristic_cost + new_node.actual_cost
                end
            else
                dict_of_nodes[node_key] = new_node
                open[node_key] = new_node.heuristic_cost + new_node.actual_cost
            end
        end
        if(time()- start_time >= planning_details.planning_time)
            println("Time exceeded")
            return path
        end
    end
    println("Path not found")
    return path
end

#=
mutable struct PathPlanningDetails
    num_nearby_humans::Int64
    radius_around_uncertain_human::Float64
    min_safe_distance_from_human::Float64
    human_collision_cost::Float64
    human_goals::Array{Location,1}
    veh_path_planning_v::Float64
    radius_around_vehicle_goal::Float64
    lambda::Float64
    one_time_step::Float64
    planning_time::Float64
end


w = generate_environment(100.0,100.0,ObstacleLocation[ObstacleLocation(30.0,30.0,15.0),ObstacleLocation(70.0,60.0,15.0)])
pd = PathPlanningDetails(0,20.0,1.0,100.0,Location[],1.0,1.0,0.99,0.5,10.0)
v = Vehicle(5.0,10.0,0.0,pd.veh_path_planning_v)
vp = VehicleParametersLSPlanner(0.3,3.0,Location(98.0,80.0),Float64[])
nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
vsd = VehicleSensor(HumanState[], Int64[], HumanGoalsBelief[])
va = get_vehicle_actions(45,5)
cs = hybrid_astar_search(w, v, vp, va, nbh, pd);
new_vp = VehicleParametersLSPlanner(vp.L,vp.max_speed,vp.goal,cs)


w = generate_environment(6.0,12.0,ObstacleLocation[ObstacleLocation(3.0,6.0,1.0)])
pd = PathPlanningDetails(0,20.0,1.0,100.0,Location[],1.0,1.0,0.99,0.5,1.0)
v = Vehicle(0.5,0.5,0.0,pd.veh_path_planning_v)
vp = VehicleParametersLSPlanner(0.3,3.0,Location(3.0,11.5),Float64[])
nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
vsd = VehicleSensor(HumanState[], Int64[], HumanGoalsBelief[])
va = get_vehicle_actions(45,15)
cs = hybrid_astar_search(w, v, vp, va, nbh, pd);
new_vp = VehicleParametersLSPlanner(vp.L,vp.max_speed,vp.goal,cs)

va = get_vehicle_actions(170,10)
nbh = NearbyHumans(HumanState[], Int64[], HumanGoalsBelief[])
p = hybrid_astar_search(env, veh, veh_params, va, nbh, path_planning_details)
new_params = modify_vehicle_params(veh_params, p)
px,py,pt = get_hybrid_astar_trajectory(veh,new_params,1,path_planning_details,exp_details)
=#
