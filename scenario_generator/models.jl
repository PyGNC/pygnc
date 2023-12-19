import SatellitePlayground
SP = SatellitePlayground



py4_faces = [
    SP.SatelliteFace(0.1^2, [0.0, 0.0, 1.0], [0.0, 0.0, 0.15 / 2]) # Top +Z
    SP.SatelliteFace(0.1^2, [0.0, 0.0, -1.0], [0.0, 0.0, -0.15 / 2]) # Bottom -Z
    SP.SatelliteFace(0.1 * 0.15, [0.0, 1.0, 0.0], [0.0, 0.05, 0.0]) # Side +Y
    SP.SatelliteFace(0.1 * 0.15, [0.0, -1.0, 0.0], [0.0, -0.05, 0.0]) # Side -Y
    SP.SatelliteFace(0.1 * 0.15, [1.0, 0.0, 0.0], [0.05, 0.0, 0.0]) # Side +X
    SP.SatelliteFace(0.1 * 0.15, [-1.0, 0.0, 0.0], [-0.05, 0.0, 0.0]) # Side -X
    SP.SatelliteFace(0.2 * 0.15, [0.0, 1.0, 0.0], [0.15, 0.05, 0.0]) # Solar 1 +Y face
    SP.SatelliteFace(0.2 * 0.15, [0.0, -1.0, 0.0], [0.15, 0.05, 0.0]) # Solar 1 -Y face
    SP.SatelliteFace(0.2 * 0.15, [0.0, 1.0, 0.0], [-0.15, -0.05, 0.0]) # Solar 2 +Y face
    SP.SatelliteFace(0.2 * 0.15, [0.0, -1.0, 0.0], [-0.15, -0.05, 0.0]) # Solar 2 -Y face
]

py4_dipole_magnitude = let
    py4_voltage = 5.06

    py4_N_turns = [40, 28 * 2, 28]
    py4_coil_R = [6.1 * 2, 8.4 * 2, 7.2 * 2] # Ohms
    py4_coil_I = py4_voltage ./ py4_coil_R # Amps
    py4_coil_A = [4218e-6, 3150e-6, 7091e-6] # m^2

    [SP.sat_dipole_magnitude(py4_N_turns[i], py4_coil_I[i], py4_coil_A[i]) for i = 1:3]
end

py4_mass_measured = 1.58 # kg
py4_inertia_matrix = let
    inertia_CAD = [0.0043 -0.0003 0; -0.0003 0.0049 0; 0 0 0.0035] # kg*m^2, determined from CAD
    mass_CAD = 1.504 # kg
    py4_mass_measured .* inertia_CAD ./ mass_CAD
end

py4_model = SP.Model(
    py4_mass_measured, # mass
    py4_inertia_matrix, # inertia
    Val(:dipole),
    [0.0, 0.0, 0.0], # center of mass offset from geometric center
    py4_faces, # faces for comuting drag
    2.2, # coefficient of drag
    py4_dipole_magnitude,
)

py4_model_nodrag = copy(py4_model)
py4_model_nodrag.drag_coefficient = 0.0

struct IMUModel
    gyro_standard_deviation_deg_s::Real
    gyro_bias_stability_deg_h::Real
    gyro_zero_rate_offset_bound_deg_s::Real
    mag_zero_B_offset_bound_uT::Real
    mag_standard_deviation_uT::Real
    R_IMU_body::AbstractMatrix{<:Real}

    function IMUModel(;
        gyro_standard_deviation_deg_s::Real=0.0,
        gyro_bias_stability_deg_h::Real=0.0,
        gyro_zero_rate_offset_bound_deg_s::Real=0.0,
        mag_zero_B_offset_bound_uT::Real=0.0,
        mag_standard_deviation_uT::Real=0.0,
        R_IMU_body::AbstractMatrix{<:Real}=[1.0 0 0; 0 1.0 0; 0 0 1.0]
    )
        return new(
            gyro_standard_deviation_deg_s,
            gyro_bias_stability_deg_h,
            gyro_zero_rate_offset_bound_deg_s,
            mag_zero_B_offset_bound_uT,
            mag_standard_deviation_uT,
            R_IMU_body
        )
    end
end

function Base.copy(m::IMUModel)
    return IMUModel(;
        gyro_standard_deviation_deg_s=m.gyro_standard_deviation_deg_s,
        gyro_bias_stability_deg_h=m.gyro_bias_stability_deg_h,
        gyro_zero_rate_offset_bound_deg_s=m.gyro_zero_rate_offset_bound_deg_s,
        mag_zero_B_offset_bound_uT=m.mag_zero_B_offset_bound_uT,
        mag_standard_deviation_uT=m.mag_standard_deviation_uT
    )
end

struct GPSModel
    position_standard_deviation_m
    velocity_standard_deviation_m

    function GPSModel(;
        position_standard_deviation_m::Real=0.0,
        velocity_standard_deviation_m::Real=0.0
    )
        return new(
            position_standard_deviation_m,
            velocity_standard_deviation_m,
        )
    end
end

function Base.copy(m::GPSModel)
    return GPSModel(
        position_standard_deviation_m=m.position_standard_deviation_m,
        velocity_standard_deviation_m=m.velocity_standard_deviation_m,
    )
end

struct SunSensorModel
    # TODO make this a real model
    standard_deviation::Real
    solar_lux::Real # lux for a surface perpendicular to the sun vector
    earth_albedo_lux::Real
    dark_condition_lux::Real # maximum lux reading for sensor with no illumination

    function SunSensorModel(;
        standard_deviation::Real=0.01,
        solar_lux::Real=(1361 * 98), # based on https://space.stackexchange.com/questions/29869/what-is-the-lux-value-of-the-sun-and-earth-in-low-earth-orbit-altitude-400-km
        earth_albedo_lux::Real=0.4 * (1361 * 98), # approximate, based on SME table 22-11, pg 688
        dark_condition_lux::Real=0.0
    )
        return new(
            standard_deviation,
            solar_lux,
            earth_albedo_lux,
            dark_condition_lux
        )
    end
end

function Base.copy(m::SunSensorModel)
    return SunSensorModel(
        standard_deviation=m.standard_deviation
    )
end

struct SatelliteSensors
    imu_model::IMUModel
    gps_model::GPSModel
    sun_sensor_model::SunSensorModel
end

function Base.copy(s::SatelliteSensors)
    return SatelliteSensors(
        copy(s.imu_model),
        copy(s.gps_model),
        copy(s.sun_sensor_model),
    )
end

py4_sensor_model_full = SatelliteSensors(
    IMUModel(;
        gyro_standard_deviation_deg_s=0.025, # determined from gyro data
        gyro_bias_stability_deg_h=3, # from bmx160 datasheet
        gyro_zero_rate_offset_bound_deg_s=3, # from bmx160 datasheet
        mag_zero_B_offset_bound_uT=40, # from bmx160 datasheet
        mag_standard_deviation_uT=0.6, # from bmx160 datasheet
        R_IMU_body=[0.0 1.0 0.0; -1.0 0.0 0.0; 0.0 0.0 -1.0]' 
        #since it is transposed takes us from body -> imu coordinates
        
    ),
    GPSModel(;
        position_standard_deviation_m=10.0, # from what Zac told us
        velocity_standard_deviation_m=1e-2 # from what Zac told us
    ),
    SunSensorModel(;
        standard_deviation=0.01,
        solar_lux=(1361 * 98), # based on https://space.stackexchange.com/questions/29869/what-is-the-lux-value-of-the-sun-and-earth-in-low-earth-orbit-altitude-400-km
        earth_albedo_lux=0.4 * (1361 * 98), # approximate, based on SME table 22-11, pg 688
        dark_condition_lux=0.0
    ),
)