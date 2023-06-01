struct NavigationSimulator{P}
    env::ExperimentEnvironment
    vehicle_params::P
    one_time_step::Float64
end

struct NavigationSimulatorObject{ST}
    vehicle_state::ST
    vehicle_sensor_data::VehicleSensor
    humans::Array{HumanState,1}
    humans_params::Array{HumanParameters,1}
end

function move_human()

end

function simulate(sim_obj, vehicle_action, vehicle_dynamics, )

end
