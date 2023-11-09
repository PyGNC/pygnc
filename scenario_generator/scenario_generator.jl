import SatellitePlayground
SP = SatellitePlayground
import SatelliteDynamics
SD = SatelliteDynamics
using LinearAlgebra

include("models.jl")
include("data_packing.jl")


function epoch_to_unix_time(time)
    unix_start = SD.Epoch("1970-01-01")
    return time - unix_start
end


function simulate_scenario(
    batch_length_s::Int=1 * 60 * 60, # number of seconds of sensor data to provide
    batch_gps_sample_period_s::Int=25, # batch sample period for gps measurements
    batch_sensor_sample_period_s::Int=5, # batch sample period for other sensor data
    sequential_length_s::Int=60, # number of seconds to run simulation for after batch data generated
    sequential_gps_sample_period_s::Real=1.0,
    sequential_sensor_sample_period_s::Real=1.0,
    integrator_timestep_s::Real=0.1,
    initial_osc_state::Vector{<:Real}=[550e3 + SD.R_EARTH, 0.0, deg2rad(98.7), deg2rad(-0.1), 0.0, deg2rad(45)], # initial state of the satellite in osculating orbital elements
    initial_attitude::Vector{<:Real}=[1.0, 0.0, 0.0, 0.0], # initial attitude of the satellite, body to inertial quaternion
    initial_angular_velocity::Vector{<:Real}=deg2rad(1) * [1.0, 0.0, 0.0], # initial angular velocity of the satellite in body frame
    environment_config::SP.EnvironmentConfig=SP.default_environment_config, # Satellite environment configuration
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
            gps_velocity=gps_velocity
        )
    end

    measurement_history = []

    measurement_dt = min(batch_gps_sample_period_s, batch_sensor_sample_period_s, sequential_gps_sample_period_s, sequential_sensor_sample_period_s)
    last_measurement = 0
    function callback(measure)
        if measure.timestamp_s - last_measurement >= measurement_dt
            push!(measurement_history, measure)
            last_measurement = measure.timestamp_s
        end
        return zero(SP.Control)
    end

    simulation_length_s = batch_length_s + sequential_length_s

    x0 = SP.state_from_osc(initial_osc_state, initial_attitude, initial_angular_velocity)
    env = copy(SP.default_environment)
    env.config = environment_config
    (state_hist, time) = SP.simulate(
        callback,
        max_iterations=Int(floor(simulation_length_s / integrator_timestep_s)),
        measure=measure,
        dt=integrator_timestep_s,
        initial_condition=x0,
        model=satellite_model,
        environment=env)

    return state_hist, time, measurement_history
end

function measurements_to_batch_file(
    measurement_history,
    batch_length_s::Int, # number of seconds of sensor data to provide
    batch_gps_sample_period_s::Int, # batch sample period for gps measurements
    batch_sensor_sample_period_s::Int, # batch sample period for other sensor data
    scenario_directory_path::AbstractString,
)
    start_time = measurement_history[1].timestamp_s
    sensor_next_timestep = start_time
    measurement_counter = 0

    sensor_samples_per_gps_samples = batch_gps_sample_period_s // batch_sensor_sample_period_s

    measurement_batch_packet = Vector{UInt8}()
    sensor_sample_counter = 0

    batch_scenario_file = open(joinpath(scenario_directory_path, "batch_gps_sensor_data.bin"), "w")
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

function generate_scenario(
    scenario_directory_path::AbstractString;
    batch_length_s::Int=1 * 60 * 60, # number of seconds of sensor data to provide
    batch_gps_sample_period_s::Int=25, # batch sample period for gps measurements
    batch_sensor_sample_period_s::Int=5, # batch sample period for other sensor data
    sequential_length_s::Int=60, # number of seconds to run simulation for after batch data generated
    sequential_gps_sample_period_s::Real=1.0,
    sequential_sensor_sample_period_s::Real=1.0,
    integrator_timestep_s::Real=0.1,
    initial_osc_state::Vector{<:Real}=[550e3 + SD.R_EARTH, 0.0, deg2rad(98.7), deg2rad(-0.1), 0.0, deg2rad(45)], # initial state of the satellite in osculating orbital elements
    initial_attitude::Vector{<:Real}=[1.0, 0.0, 0.0, 0.0], # initial attitude of the satellite, body to inertial quaternion
    initial_angular_velocity::Vector{<:Real}=deg2rad(1) * [1.0, 0.0, 0.0], # initial angular velocity of the satellite in body frame
    environment_config::SP.EnvironmentConfig=SP.default_environment_config, # Satellite environment configuration
    satellite_model::SP.Model=py4_model,
    sensor_model::SatelliteSensors=py4_sensor_model_full
)

    state_hist, time, measurement_history = simulate_scenario(
        batch_length_s,
        batch_gps_sample_period_s,
        batch_sensor_sample_period_s,
        sequential_length_s,
        sequential_gps_sample_period_s,
        sequential_sensor_sample_period_s,
        integrator_timestep_s,
        initial_osc_state,
        initial_attitude,
        initial_angular_velocity,
        environment_config,
        satellite_model,
        sensor_model,
    )

    last_batch_measurement_index = measurements_to_batch_file(
        measurement_history,
        batch_length_s,
        batch_gps_sample_period_s,
        batch_sensor_sample_period_s,
        scenario_directory_path,
    )

end

state_hist, time_hist, measurement_history = simulate_scenario()
measurements_to_batch_file(
    measurement_history,
    1 * 60 * 60, # number of seconds of sensor data to provide
    25, # batch sample period for gps measurements
    5, # batch sample period for other sensor data
    joinpath("..", "scenarios", "default_scenario"),
)