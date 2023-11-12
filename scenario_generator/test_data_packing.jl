using Test
include("data_packing.jl")

@testset "gps" begin
    unix_time_in = 1.699452212825686e9
    position_in = [1e6, 2e6, 3e6]
    velocity_in = [17e2, 18e2, 19e2]

    # generally relying on default inputs
    msg = pack_batch_gps_message(
        unix_time_in, # unix_timestamp
        position_in, # position
        velocity_in; # velocity
    )

    @test length(msg) == 99

    gps_data = unpack_batch_gps_message(msg)

    (spacecraft_time,
        gps_time_status,
        gps_week,
        gps_milliseconds,
        position_status,
        position_x,
        position_y,
        position_z,
        position_sig_x,
        position_sig_y,
        position_sig_z,
        velocity_status,
        velocity_x,
        velocity_y,
        velocity_z,
        velocity_sig_x,
        velocity_sig_y,
        velocity_sig_z,
        v_latency,
        differential_age,
        solution_age,
        sats_tracked,
        sats_in_solution) = gps_data

    gps_millis_in, gps_week_in = unix_seconds_to_gps_time(unix_time_in)

    @test spacecraft_time == Int(floor(unix_time_in))
    @test gps_time_status == 160
    @test gps_week == gps_week_in
    @test gps_milliseconds == gps_millis_in
    @test position_status == 1
    @test position_in ≈ [position_x, position_y, position_z]
    @test [0.0, 0.0, 0.0] ≈ [position_sig_x, position_sig_y, position_sig_z]
    @test velocity_status == 1
    @test velocity_in ≈ [velocity_x, velocity_y, velocity_z]
    @test [0.0, 0.0, 0.0] ≈ [velocity_sig_x, velocity_sig_y, velocity_sig_z]
    @test v_latency == 0.0
    @test differential_age == 0.0
    @test solution_age == 0.0
    @test sats_tracked == 10
    @test sats_in_solution == 8
end

@testset "lux" begin
    reading = lux_to_opt3001_reading(6000)
    @test (reading & 0xF000) == 0x8000 # correct exponent
    @test (reading & 0xFFFF0000) == 0 # 16 bit number
    @test (reading & 0x0FFF) > 0 # positive mantissa

    # maximum reading
    reading = lux_to_opt3001_reading(1e6)
    @test ((reading & 0xF000) >> 12) == 0b1011 # maximum exponent
    @test (reading & 0xFFFF0000) == 0 # 16 bit number
    @test (reading & 0x0FFF) == 0xFFF # mantissa is maxxed out

    # small reading
    reading = lux_to_opt3001_reading(0.19)
    @test (reading & 0xF000) == 0 # minimum exponent
    @test (reading & 0xFFFF0000) == 0 # 16 bit number
    @test (reading & 0x0FFF) == 19 # mantissa is correct
end

@testset "sensors" begin

    unix_timestamp_in = 1.699452212825686e9
    mag_measurement_in = [9.87, 65.43, 3.21]
    gyro_measurement_in = [11.22, 3.33, 0.4433]
    sun_sensors_in = [12.1567, 560, 32000.4, 984.0, 0.001, 99e9]
    sun_sensors_in_sensor_binned = [12.15, 560.0, 32000.0, 984.0, 0.0, 83865.6]
    msg = pack_batch_sensor_message(
        unix_timestamp_in,
        mag_measurement_in,
        gyro_measurement_in,
        sun_sensors_in,
    )

    @test length(msg) == 30

    (spacecraft_time,
        mag_measurement_out,
        raw_hall_out,
        gyro_measurement_out,
        sun_sensors_out) = unpack_batch_sensor_message(msg)

    @test spacecraft_time == Int(floor(unix_timestamp_in))
    @test mag_measurement_in ≈ mag_measurement_out atol = 0.1
    @test raw_hall_out == 0
    @test gyro_measurement_in ≈ gyro_measurement_out atol = 0.1
    @test sun_sensors_in_sensor_binned ≈ sun_sensors_out
end