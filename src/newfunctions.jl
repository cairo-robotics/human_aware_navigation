struct NNewStateExtendedSpacePOMDP{V,M,N}
    vehicle::V
    nearby_humans::SVector{M,N}
end

struct TempVehicleState
    x::Float64
    y::Float64
    θ::Float64
    v::Float64
end

function move(vehicle::TempVehicleState,a::Tuple{Float64,Float64},time::Float64)::Tuple{Float64, Float64, Float64}
    current_x,current_y,current_theta = vehicle.x,vehicle.y,vehicle.θ
    steering_angle,speed = a
    vehicle_wheelbase = 1.0

    if(steering_angle == 0.0)
        new_theta = current_theta
        new_x = current_x + speed*cos(current_theta)*time
        new_y = current_y + speed*sin(current_theta)*time
    else
        new_theta = current_theta + (speed * tan(steering_angle) * (time) / vehicle_wheelbase)
        new_theta = wrap_between_0_and_2Pi(new_theta)
        new_x = current_x + ((vehicle_wheelbase / tan(steering_angle)) * (sin(new_theta) - sin(current_theta)))
        new_y = current_y + ((vehicle_wheelbase / tan(steering_angle)) * (cos(current_theta) - cos(new_theta)))
    end
    return (new_x,new_y,new_theta)
end

function new_update_vehicle_position(s::TempVehicleState, one_time_step::Float64, steering_angle::Float64, new_vehicle_speed::Float64, ::Val{N}) where N

    current_x, current_y, current_theta = s.x, s.y, s.θ
    g= Location(10,10)
    vehicle_path = MVector{N,Tuple{Float64,Float64,Float64}}(undef)

    # return SVector{N,Tuple{Float64,Float64,Float64}}(vehicle_path)
    # return SVector(vehicle_path)

    if(new_vehicle_speed == 0.0)
        vs = (current_x, current_y, current_theta)
        for i in 1:N
            vehicle_path[i] = vs
        end
    else
        c = 1
        for i in 1:N
            v = TempVehicleState(current_x, current_y, current_theta,new_vehicle_speed)
            new_x,new_y,new_theta = move(v,(steering_angle,new_vehicle_speed),one_time_step/N)
            vehicle_path[i] = (new_x,new_y,new_theta)
            current_x,current_y,current_theta = new_x,new_y,new_theta
            vehicle_center_x = current_x + 1*cos(current_theta)
            vehicle_center_y = current_y + 1*cos(current_theta)
            c+=1
            if(is_within_range(vehicle_center_x,vehicle_center_y,g.x,g.y,1.0) ||
                vehicle_center_x<0.0+0.5 || vehicle_center_y<0.0+0.5 ||
                vehicle_center_x>99.5 || vehicle_center_y>99.5 )
                for j in i+1:10
                    vehicle_path[j] = (current_x, current_y, current_theta)
                end
                return SVector{N,Tuple{Float64,Float64,Float64}}(vehicle_path)
            end
        end
    end
    # println(vehicle_x, vehicle_y, vehicle_theta, vehicle_L, steering_angle, new_vehicle_speed)
    # println(vehicle_path)
    return SVector{N,Tuple{Float64,Float64,Float64}}(vehicle_path)
end
#=
v= TempVehicleState()
@btime new_update_vehicle_position($v,1.0,0.0,0.0,Val(10))
=#

move(s::Tuple{Float64,Float64,Float64}, a::Tuple{Float64,Float64}) = (s[1]+a[1],s[2]+a[2],s[1]*s[2])

function lala(s,a,::Val{N}) where N
    m = MVector{N,Tuple{Float64,Float64,Float64}}(undef)
    cs = s
    m[1] = cs
    for i in 2:N
        ns = move(cs,a)
        m[i] = ns
        cs = ns
        if(cs[3] > 10.2)
            v = 9
            for j in 1:N
                v+=1
            end
            return SVector(m)
        end
    end
    return SVector(m)
end
