function update_human_position_pomdp_planning(human::human_state,world_length::Float64,world_breadth::Float64,time_step::Float64,discretization_step_length::Float64,rng::AbstractRNG)
    return true
    rand_num = (rand(rng) - 0.5)*0.2
    #rand_num = 0.0
    #First Quadrant
    if(human.goal.x >= human.x && human.goal.y >= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y + (human.v)*time_step + rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x + (human.v)*time_step + rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x + ((human.v)*time_step + rand_num)*cos(heading_angle)
            new_y = human.y + ((human.v)*time_step + rand_num)*sin(heading_angle)
        end
    #Second Quadrant
    elseif(human.goal.x <= human.x && human.goal.y >= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y + (human.v)*time_step + rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x - (human.v)*time_step - rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x - ((human.v)*time_step + rand_num)*cos(heading_angle)
            new_y = human.y - ((human.v)*time_step + rand_num)*sin(heading_angle)
        end
    #Third Quadrant
    elseif(human.goal.x <= human.x && human.goal.y <= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y - (human.v)*time_step - rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x - (human.v)*time_step - rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x - ((human.v)*time_step + rand_num)*cos(heading_angle)
            new_y = human.y - ((human.v)*time_step + rand_num)*sin(heading_angle)
        end
    #Fourth Quadrant
    else(human.goal.x >= human.x && human.goal.y <= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y - (human.v)*time_step - rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x + (human.v)*time_step + rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x + ((human.v)*time_step + rand_num)*cos(heading_angle)
            new_y = human.y + ((human.v)*time_step + rand_num)*sin(heading_angle)
        end
    end

    new_x = clamp(new_x,0.0,world_length)
    new_y = clamp(new_y,0.0,world_breadth)
    discrete_new_x = floor(new_x/discretization_step_length) * discretization_step_length
    discrete_new_y = floor(new_y/discretization_step_length) * discretization_step_length
    new_human_state = human_state(new_x,new_y,human.v,human.goal,human.id)
    observed_location = location(discrete_new_x, discrete_new_y)

    return new_human_state,observed_location
end
#=
unit_test_human = human_state(10.0,10.0,1.0,location(100.0,100.0),7.0)
update_human_position_pomdp_planning(unit_test_human,env.length,env.breadth,1.0,1.0,MersenneTwister(1234))
update_human_position_pomdp_planning(unit_test_human,100.0,100.0,1.0,1.0,MersenneTwister(1234))
@code_warntype update_human_position_pomdp_planning(unit_test_human,100.0,100.0,1.0,1.0,MersenneTwister(1234))
=#



function update_human_position_pomdp_planning2(human::human_state,m::extended_space_POMDP_planner,rng::AbstractRNG)
    return true
    rand_num = (rand(rng) - 0.5)*0.2
    #rand_num = 0.0
    #First Quadrant
    if(human.goal.x >= human.x && human.goal.y >= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y + (human.v)*m.one_time_step + rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x + (human.v)*m.one_time_step + rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x + ((human.v)*m.one_time_step + rand_num)*cos(heading_angle)
            new_y = human.y + ((human.v)*m.one_time_step + rand_num)*sin(heading_angle)
        end
    #Second Quadrant
    elseif(human.goal.x <= human.x && human.goal.y >= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y + (human.v)*m.one_time_step + rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x - (human.v)*m.one_time_step - rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x - ((human.v)*m.one_time_step + rand_num)*cos(heading_angle)
            new_y = human.y - ((human.v)*m.one_time_step + rand_num)*sin(heading_angle)
        end
    #Third Quadrant
    elseif(human.goal.x <= human.x && human.goal.y <= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y - (human.v)*m.one_time_step - rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x - (human.v)*m.one_time_step - rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x - ((human.v)*m.one_time_step + rand_num)*cos(heading_angle)
            new_y = human.y - ((human.v)*m.one_time_step + rand_num)*sin(heading_angle)
        end
    #Fourth Quadrant
    else(human.goal.x >= human.x && human.goal.y <= human.y)
        if(human.goal.x == human.x)
            new_x = human.x
            new_y = human.y - (human.v)*m.one_time_step - rand_num
        elseif(human.goal.y == human.y)
            new_x = human.x + (human.v)*m.one_time_step + rand_num
            new_y = human.y
        else
            heading_angle = atan((human.goal.y - human.y) / (human.goal.x - human.x))
            new_x = human.x + ((human.v)*m.one_time_step + rand_num)*cos(heading_angle)
            new_y = human.y + ((human.v)*m.one_time_step + rand_num)*sin(heading_angle)
        end
    end

    new_x = clamp(new_x,0.0,m.world.length)
    new_y = clamp(new_y,0.0,m.world.breadth)
    discrete_new_x = floor(new_x/m.observation_discretization_length) * m.observation_discretization_length
    discrete_new_y = floor(new_y/m.observation_discretization_length) * m.observation_discretization_length
    new_human_state = human_state(new_x,new_y,human.v,human.goal,human.id)
    observed_location = location(discrete_new_x, discrete_new_y)
    return new_human_state,observed_location
end
#=
unit_test_human = human_state(10.0,10.0,1.0,location(100.0,100.0),7.0)
unit_test_env = experiment_environment(100.0,100.0,obstacle_location[])
unit_test_es_pomdp = extended_space_POMDP_planner(0.99,1.0,-100.0,1.0,-100.0,1.0,100.0,3.0,1.0,10,1.0,unit_test_env)
update_human_position_pomdp_planning2(unit_test_human,unit_test_es_pomdp,MersenneTwister(1234))
@code_warntype update_human_position_pomdp_planning(unit_test_human,100.0,100.0,1.0,1.0,MersenneTwister(1234))
=#


# struct hg
#     v::Float64
# end
#
# function t1(o::hg)
#     if(o.v < 10.0)
#         return true
#     else
#         return false
#     end
# end
#
# function t2(v::Float64)
#     if(v < 10.0)
#         return true
#     else
#         return false
#     end
# end
#
# obj = hg(2.0)
# @btime t1(obj)
# @btime t2(obj.v)

function l1()
    a::Int64 = 1
    a = a*1.2
    return a*a
end

function l2()
    a = 1
    return a*a
end
