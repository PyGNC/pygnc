using Pkg
Pkg.activate(@__DIR__)
import SatellitePlayground
SP = SatellitePlayground
import SatelliteDynamics
SD = SatelliteDynamics
using LinearAlgebra

include("models.jl")
include("data_packing.jl")

struct ScenarioConfig
    batch_length_s::Int # number of seconds of sensor data to provide
    batch_gps_sample_period_s::Int # batch sample period for gps measurements
    batch_sensor_sample_period_s::Int # batch sample period for other sensor data
    batch_ranging_sample_period_s::Int
    sequential_length_s::Int # number of seconds to run simulation for after batch data generated
    sequential_gps_sample_period_s::Real
    sequential_sensor_sample_period_s::Real
    sequential_ranging_sample_period_s::Real
    integrator_timestep_s::Real
    environment_config::SP.EnvironmentConfig # Satellite environment configuration

    function ScenarioConfig(;
        batch_length_s::Int=1 * 60 * 60,
        batch_gps_sample_period_s::Int=25,
        batch_sensor_sample_period_s::Int=5,
        batch_ranging_sample_period_s::Int=60,
        sequential_length_s::Int=60,
        sequential_gps_sample_period_s::Real=1.0,
        sequential_sensor_sample_period_s::Real=1.0,
        sequential_ranging_sample_period_s::Real=1.0,
        integrator_timestep_s::Real=0.1,
        environment_config::SP.EnvironmentConfig=SP.default_environment_config,
    )
        return new(
            batch_length_s,
            batch_gps_sample_period_s,
            batch_sensor_sample_period_s,
            batch_ranging_sample_period_s,
            sequential_length_s,
            sequential_gps_sample_period_s,
            sequential_sensor_sample_period_s,
            sequential_ranging_sample_period_s,
            integrator_timestep_s,
            environment_config,)
    end
end


function epoch_to_unix_time(time)
    unix_start = SD.Epoch("1970-01-01")
    return time - unix_start
end


function simulate_scenario(;
    scenario_config::ScenarioConfig=ScenarioConfig(),
    initial_osc_state::Vector{<:Real}=[550e3 + SD.R_EARTH, 0.0, deg2rad(98.7), deg2rad(-0.1), 0.0, deg2rad(45)], # initial state of the satellite in osculating orbital elements
    initial_attitude::Vector{<:Real}=[1.0, 0.0, 0.0, 0.0], # initial attitude of the satellite, body to inertial quaternion
    initial_angular_velocity::Vector{<:Real}=deg2rad(1) * [1.0, 0.0, 0.0], # initial angular velocity of the satellite in body frame
    satellite_model::SP.Model=py4_model,
    sensor_model::SatelliteSensors=py4_sensor_model_full
)

    # set up sensor models
    gyro_std_dev_matrix_deg_s = I(3) .* sensor_model.imu_model.gyro_standard_deviation_deg_s
    mag_std_dev_matrix_uT = I(3) .* sensor_model.imu_model.mag_standard_deviation_uT
    sun_std_dev_matrix = I(3) .* sensor_model.sun_sensor_model.standard_deviation
    position_std_dev_matrix = I(3) .* sensor_model.gps_model.position_standard_deviation_m
    velocity_std_dev_matrix = I(3) .* sensor_model.gps_model.velocity_standard_deviation_m
    R_IMU_body = sensor_model.imu_model.R_IMU_body

    mag_bias_uT = sensor_model.imu_model.mag_zero_B_offset_bound_uT * (2 .* rand(3) .- 1.0)
    gyro_bias_deg_s = sensor_model.imu_model.gyro_zero_rate_offset_bound_deg_s * (2 .* rand(3) .- 1.0)

    function measure(state, env)
        sun_vector_eci = SatelliteDynamics.sun_position(env.time)
        unix_time_s = epoch_to_unix_time(env.time)

        ᵇQⁿ = SP.quaternionToMatrix(state.attitude)'

        # since the sun vector is normalized and in the body frame,
        # each component is the cos() of the angle between the sun and that satellite face
        sun_vector_body = ᵇQⁿ * normalize(state.position .- sun_vector_eci) + sun_std_dev_matrix * randn(3)
        earth_vector_body = ᵇQⁿ * normalize(-state.position) + sun_std_dev_matrix * randn(3)
        # combine sun and earth albedo lux
        lux_vector_body = (sensor_model.sun_sensor_model.solar_lux * sun_vector_body +
                           sensor_model.sun_sensor_model.earth_albedo_lux * earth_vector_body)
        dark_measurement = sensor_model.sun_sensor_model.dark_condition_lux * rand(3)
        sun_sensors_positive_faces = max.(dark_measurement, lux_vector_body)
        dark_measurement = sensor_model.sun_sensor_model.dark_condition_lux * rand(3)
        sun_sensors_negative_faces = -min.(dark_measurement, lux_vector_body)
        sun_sensors = [sun_sensors_positive_faces; sun_sensors_negative_faces]

        mag_T = R_IMU_body * env.b
        mag_measurement = 1e6 * mag_T .+ mag_std_dev_matrix_uT * randn(3) .+ mag_bias_uT

        angular_veloctity_deg_s = R_IMU_body * rad2deg.(state.angular_velocity)
        gyro_measurement = angular_veloctity_deg_s .+ gyro_std_dev_matrix_deg_s * randn(3) .+ gyro_bias_deg_s

        state_ECEF = SD.sECItoECEF(env.time, [state.position; state.velocity])
        gps_position = state_ECEF[1:3] .+ position_std_dev_matrix * randn(3)
        gps_velocity = state_ECEF[4:6] .+ velocity_std_dev_matrix * randn(3)

        return (
            timestamp_s=unix_time_s,
            gyro_measurement=gyro_measurement,
            mag_measurement=mag_measurement,
            sun_sensors=sun_sensors,
            gps_position=gps_position,
            gps_velocity=gps_velocity,
            range_true_position=state_ECEF[1:3],
        )
    end

    measurement_history = Vector{NamedTuple}()

    measurement_dt = min(
        scenario_config.batch_gps_sample_period_s,
        scenario_config.batch_sensor_sample_period_s,
        scenario_config.sequential_gps_sample_period_s,
        scenario_config.sequential_sensor_sample_period_s)

    last_measurement = 0
    function callback(measure)
        if measure.timestamp_s - last_measurement >= measurement_dt
            push!(measurement_history, measure)
            last_measurement = measure.timestamp_s
        end
        return zero(SP.Control)
    end

    simulation_length_s = scenario_config.batch_length_s + scenario_config.sequential_length_s

    x0 = SP.state_from_osc(initial_osc_state, initial_attitude, initial_angular_velocity)
    env = copy(SP.default_environment)
    env.config = scenario_config.environment_config
    (state_hist, time) = SP.simulate(
        callback,
        max_iterations=Int(floor(simulation_length_s / scenario_config.integrator_timestep_s)),
        measure=measure,
        dt=scenario_config.integrator_timestep_s,
        initial_condition=x0,
        model=satellite_model,
        environment=env)

    return state_hist, time, measurement_history
end

function measurements_to_batch_file(
    measurement_history::Vector{<:NamedTuple},
    batch_length_s::Int, # number of seconds of sensor data to provide
    batch_gps_sample_period_s::Int, # batch sample period for gps measurements
    batch_sensor_sample_period_s::Int, # batch sample period for other sensor data
    scenario_directory_path::AbstractString;
    satellite_name::AbstractString="",
)
    start_time = measurement_history[1].timestamp_s
    sensor_next_timestep = start_time
    measurement_counter = 0

    sensor_samples_per_gps_samples = batch_gps_sample_period_s // batch_sensor_sample_period_s

    measurement_batch_packet = Vector{UInt8}()
    sensor_sample_counter = 0

    if !isdir(scenario_directory_path)
        mkdir(scenario_directory_path)
    end
    batch_scenario_file = open(joinpath(scenario_directory_path, "batch_sensor_gps_data$(satellite_name).bin"), "w")

    try
        for m in measurement_history
            measurement_counter += 1
            if m.timestamp_s >= sensor_next_timestep
                append!(measurement_batch_packet,
                    pack_batch_sensor_message(
                        m.timestamp_s,
                        m.mag_measurement,
                        m.gyro_measurement,
                        m.sun_sensors
                    )
                )
                sensor_sample_counter += 1

                if sensor_sample_counter >= sensor_samples_per_gps_samples
                    # time for a GPS sample
                    append!(measurement_batch_packet,
                        pack_batch_gps_message(m.timestamp_s, m.gps_position, m.gps_velocity)
                    )
                    # measurement batch packet is complete, write it to file
                    @assert length(measurement_batch_packet) == (99 + 5 * 30)
                    write(batch_scenario_file, measurement_batch_packet)
                    write(batch_scenario_file, "\r\n")
                    sensor_sample_counter = 0
                    measurement_batch_packet = Vector{UInt8}()
                end
                sensor_next_timestep = m.timestamp_s + batch_sensor_sample_period_s
            end
            if m.timestamp_s >= start_time + batch_length_s
                break
            end
        end
    finally
        close(batch_scenario_file)
    end

    return measurement_counter # index of last measurement
end

function measurements_to_sequential_file()
end

function measurements_to_formation_batch_file(
    measurement_histories::Vector{<:Vector{<:NamedTuple}},
    batch_length_s::Int, # number of seconds of sensor data to provide
    batch_ranging_sample_period_s::Int, # batch sample period for range measurements
    sensor_models::Vector{SatelliteSensors},
    scenario_directory_path::AbstractString,
)

    if !isdir(scenario_directory_path)
        mkdir(scenario_directory_path)
    end
    for requester_idx in eachindex(measurement_histories)
        range_batch_scenario_file = open(joinpath(scenario_directory_path, "batch_range_data_$(requester_idx).csv"), "w")
        write(range_batch_scenario_file, "range_counter, time, receiver_id, requester_position[1], requester_position[2], requester_position[3], receiver_position[1], receiver_position[2], receiver_position[3], range, true_range\n")
        requester_measurement_history = measurement_histories[requester_idx]
        measurement_counter = 0
        range_next_timestep = requester_measurement_history[1].timestamp_s
        end_time = batch_length_s + requester_measurement_history[1].timestamp_s
        range_counter = 0
        requester_model = sensor_models[requester_idx].range_sensor_model
        for m in requester_measurement_history
            measurement_counter += 1
            if m.timestamp_s > end_time
                break
            end
            if m.timestamp_s >= range_next_timestep
                for receiver_idx in eachindex(measurement_histories)
                    if requester_idx == receiver_idx
                        continue
                    end
                    # compute measurements and save to batch file
                    m_rx = measurement_histories[receiver_idx][measurement_counter]
                    true_range = norm(m_rx.range_true_position .- m.range_true_position)
                    range = true_range .+ randn() * requester_model.standard_deviation
                    write(range_batch_scenario_file, "$range_counter, $(m.timestamp_s), $receiver_idx, $(m.range_true_position[1]), $(m.range_true_position[2]), $(m.range_true_position[3]), $(m_rx.range_true_position[1]), $(m_rx.range_true_position[2]), $(m_rx.range_true_position[3]), $(range), $(true_range)\n")
                end
                range_counter += 1
                range_next_timestep += batch_ranging_sample_period_s
            end
        end
        close(range_batch_scenario_file)
    end
end

function generate_single_scenario(
    scenario_directory_path::AbstractString;
    scenario_config::ScenarioConfig=ScenarioConfig(),
    initial_osc_state::Vector{<:Real}=[550e3 + SD.R_EARTH, 0.0, deg2rad(98.7), deg2rad(-0.1), 0.0, deg2rad(45)], # initial state of the satellite in osculating orbital elements
    initial_attitude::Vector{<:Real}=[1.0, 0.0, 0.0, 0.0], # initial attitude of the satellite, body to inertial quaternion
    initial_angular_velocity::Vector{<:Real}=deg2rad(1) * [1.0, 0.0, 0.0], # initial angular velocity of the satellite in body frame
    satellite_model::SP.Model=py4_model,
    sensor_model::SatelliteSensors=py4_sensor_model_full,
    satellite_name::AbstractString=""
)

    state_hist, time, measurement_history = simulate_scenario(;
        scenario_config,
        initial_osc_state,
        initial_attitude,
        initial_angular_velocity,
        satellite_model,
        sensor_model,
    )

    last_batch_measurement_index = measurements_to_batch_file(
        measurement_history,
        scenario_config.batch_length_s,
        scenario_config.batch_gps_sample_period_s,
        scenario_config.batch_sensor_sample_period_s,
        scenario_directory_path;
        satellite_name=satellite_name,
    )

    return state_hist, time, measurement_history
end

function generate_formation_scenario(
    scenario_directory_path::AbstractString;
    scenario_config::ScenarioConfig=ScenarioConfig(),
    initial_osc_states::Vector{<:Vector{<:Real}}=[[550e3 + SD.R_EARTH, 0.0, deg2rad(98.7), deg2rad(-0.1), 0.0, deg2rad(45)]], # initial state of the satellite in osculating orbital elements
    initial_attitudes::Vector{<:Vector{<:Real}}=[[1.0, 0.0, 0.0, 0.0]], # initial attitude of the satellite, body to inertial quaternion
    initial_angular_velocities::Vector{<:Vector{<:Real}}=[deg2rad(1) * [1.0, 0.0, 0.0]], # initial angular velocity of the satellite in body frame
    satellite_models::Vector{SP.Model}=[py4_model],
    sensor_models::Vector{SatelliteSensors}=[py4_sensor_model_full]
)
    measurement_histories = Vector{Vector{NamedTuple}}()
    for satellite_idx in eachindex(initial_osc_states)
        initial_osc_state = initial_osc_states[satellite_idx]
        initial_attitude = initial_attitudes[satellite_idx]
        initial_angular_velocity = initial_angular_velocities[satellite_idx]
        satellite_model = satellite_models[satellite_idx]
        sensor_model = sensor_models[satellite_idx]

        _, _, measurement_history = generate_single_scenario(
            scenario_directory_path;
            scenario_config,
            initial_osc_state,
            initial_attitude,
            initial_angular_velocity,
            satellite_model,
            sensor_model,
            satellite_name="_$satellite_idx",
        )
        push!(measurement_histories, measurement_history)
    end

    measurements_to_formation_batch_file(
        measurement_histories,
        scenario_config.batch_length_s,
        scenario_config.batch_ranging_sample_period_s,
        sensor_models,
        scenario_directory_path,
    )
end

function deployment_initial_conditions(
    num_sats::Int,
    osc_state::Vector{<:Real}=[550e3 + SD.R_EARTH, 0.0, deg2rad(98.7), deg2rad(-0.1), 0.0, deg2rad(45)],
    attitude::Vector{<:Real}=[1.0, 0.0, 0.0, 0.0],
    angular_velocity::Vector{<:Real}=deg2rad(1) * [1.0, 0.0, 0.0],
    deployment_velocity::Vector{<:Real}=[1.0, 0.0, 0.0],
    position_std::Real=1.0,
    velocity_std::Real=0.1,
    attitude_std::Real=0.1,
    angular_velocity_std::Real=0.01)

    osc_states = Vector{Vector{<:Real}}()
    attitudes = Vector{Vector{<:Real}}()
    angular_velocities = Vector{Vector{<:Real}}()

    eci_state = SD.sOSCtoCART(osc_state)
    for i = 1:num_sats
        Δp = randn(3) .* position_std
        Δv = deployment_velocity + velocity_std .* randn(3)
        push!(osc_states, SD.sCARTtoOSC(eci_state .+ [Δp; Δv]))
        push!(attitudes, SP.L(SP.H * randn(3) .* attitude_std) * attitude)
        push!(angular_velocities, angular_velocity .+ randn(3) .* angular_velocity_std)
    end
    return osc_states, attitudes, angular_velocities
end

# state_hist, time_hist, measurement_history = simulate_scenario()
# measurements_to_batch_file(
#     measurement_history,
#     1 * 60 * 60, # number of seconds of sensor data to provide
#     25, # batch sample period for gps measurements
#     5, # batch sample period for other sensor data
#     joinpath("..", "scenarios", "default_scenario"),
# )

initial_osc_states, initial_attitudes, initial_angular_velocities = deployment_initial_conditions(4)
generate_formation_scenario(joinpath("..", "scenarios", "default_formation_scenario"),
    initial_osc_states=initial_osc_states,
    initial_attitudes=initial_attitudes,
    initial_angular_velocities=initial_angular_velocities,
    satellite_models=[py4_model for i = 1:4],
    sensor_models=[py4_sensor_model_full for i = 1:4]
)