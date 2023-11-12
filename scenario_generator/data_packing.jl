using PyCall
using Infiltrator
import SatelliteDynamics
SD = SatelliteDynamics
pystruct = pyimport("struct")

function unix_seconds_to_gps_time(timestamp)
    gps_epoch = SD.Epoch("1980-01-06")
    unix_start = SD.Epoch("1970-01-01")
    gps_unix_diff_seconds = gps_epoch - unix_start
    gps_seconds_since_epoch = timestamp - gps_unix_diff_seconds
    seconds_in_week = (7 * 24 * 60 * 60)
    gps_week = Int(floor(gps_seconds_since_epoch / seconds_in_week))
    gps_seconds = gps_seconds_since_epoch % seconds_in_week
    gps_milliseconds = Int(floor(gps_seconds * 1e3))
    return gps_milliseconds, gps_week
end

""" pack_batch_gps_message

    Pack a gps message for the batch file
    
    Spacecraft Time (s)
    GPS Time Sol Status
    GPS Week
    GPS Time (s)
    Pos Sol Status
    ECEF-X (m)
    ECEF-Y (m)
    ECEF-Z (m)
    ECEF-Xsig (m)
    ECEF-Ysig (m)
    ECEF-Zsig (m)
    Vel Sol Status
    ECEF-XV (m/s)
    ECEF-YV (m/s)
    ECEF-ZV (m/s)
    ECEF-XVsig (m/s)
    ECEF-YVsig (m/s)
    ECEF-ZVsig (m/s)
    V-latency
    Differential Age (s)
    Solution Age (s)
    Sats Tracked
    Sats in Solution
"""
function pack_batch_gps_message(
    unix_timestamp::Real,
    position::Vector{<:Real},
    velocity::Vector{<:Real};
    time_status::Int=160, # FINE, see pg 53, Table 13
    position_status::Int=1, # FIX, see pg 555, Table 93
    position_sig::Vector{<:Real}=zeros(3),
    velocity_status::Int=1, # FIX, see pg 555, Table 93
    velocity_sig::Vector{<:Real}=zeros(3),
    v_latency::Real=0.0,
    differential_age::Real=0.0,
    solution_age::Real=0.0,
    sats_tracked::Int=10,
    sats_in_solution::Int=8
)
    spacecraft_time = Int(floor(unix_timestamp))
    spacecraft_time_bytes = py"($spacecraft_time).to_bytes(length=4, byteorder='big')"
    gps_time_status_bytes = py"($time_status).to_bytes(length=1, byteorder='big')"
    gps_milliseconds, gps_week = unix_seconds_to_gps_time(unix_timestamp)
    gps_milliseconds_bytes = py"($gps_milliseconds).to_bytes(length=4, byteorder='little')"
    gps_week_bytes = py"($gps_week).to_bytes(length=2, byteorder='little')"
    position_status_bytes = py"($position_status).to_bytes(length=1, byteorder='big')"
    ecef_packet = pystruct.pack("<dddfffBdddffffffBB",
        position[1],
        position[2],
        position[3],
        position_sig[1],
        position_sig[2],
        position_sig[3],
        velocity_status,
        velocity[1],
        velocity[2],
        velocity[3],
        velocity_sig[1],
        velocity_sig[2],
        velocity_sig[3],
        v_latency,
        differential_age,
        solution_age,
        sats_tracked,
        sats_in_solution
    )

    gps_packet = Vector{UInt8}(spacecraft_time_bytes
                               * gps_time_status_bytes
                               * gps_week_bytes
                               * gps_milliseconds_bytes
                               * position_status_bytes
                               * ecef_packet)

    return gps_packet
end

function unpack_batch_gps_message(gps_packet)

    spacecraft_time = py"int.from_bytes($(gps_packet[1:4]), 'big')"
    gps_frame = gps_packet[5:end]
    ecef = pystruct.unpack("<dddfffBdddffffffBB", gps_frame[9:end])
    gps_data =
        (
            spacecraft_time,  # approx spacecraft time at gps frame (within 0.8s or less)
            gps_frame[1],  # GPS time sol status
            py"int.from_bytes($(gps_frame[2:3]), 'little')",  # GPS week
            py"int.from_bytes($(gps_frame[4:7]), 'little')",  # time of week (milliseconds)
            gps_frame[8],  # pos sol status
            ecef...
        )

    return gps_data
end

# taken from table 8, pg 20 of opt3001 datasheet
exponent_level_lut = [
    (exponent=UInt16(0b0000), lux_max=40.95, lsb_size=0.01),
    (exponent=UInt16(0b0001), lux_max=81.90, lsb_size=0.02),
    (exponent=UInt16(0b0010), lux_max=163.80, lsb_size=0.04),
    (exponent=UInt16(0b0011), lux_max=327.60, lsb_size=0.08),
    (exponent=UInt16(0b0100), lux_max=655.20, lsb_size=0.16),
    (exponent=UInt16(0b0101), lux_max=1310.40, lsb_size=0.32),
    (exponent=UInt16(0b0110), lux_max=2620.80, lsb_size=0.64),
    (exponent=UInt16(0b0111), lux_max=5241.60, lsb_size=1.28),
    (exponent=UInt16(0b1000), lux_max=10483.20, lsb_size=2.56),
    (exponent=UInt16(0b1001), lux_max=20966.40, lsb_size=5.12),
    (exponent=UInt16(0b1010), lux_max=41932.80, lsb_size=10.24),
    (exponent=UInt16(0b1011), lux_max=83865.60, lsb_size=20.48),
]

function lux_to_opt3001_raw(lux)
    @assert lux >= 0
    for level in exponent_level_lut
        if lux <= level.lux_max
            # quantize lux, multiply and divide by 100 to avoid fractional represenations of lsb_size
            lux = (100 * lux - ((100 * lux) % Int(100 * level.lsb_size))) / 100
            # get fractional part
            fractional = Int(floor(lux / level.lsb_size)) & 0xFFF
            exponent = level.exponent
            return pack_opt3001_raw(exponent, fractional)
        end
    end
    # lux is saturated
    fractional = 0xFFF
    exponent = exponent_level_lut[end].exponent
    return pack_opt3001_raw(exponent, fractional)
end

function pack_opt3001_raw(exponent, fractional)
    #  15     12  11          0
    # [ exponent | fractional  ]
    raw = (exponent << 12) + (fractional & 0x0FFF)
    raw &= 0xFFFF # ensure 16 bit

    return raw
end

function opt3001_raw_to_lux(raw)
    exponent = (raw & 0xF000) >> 12
    fractional = raw & 0x0FFF
    lux = (0.01 * fractional) * (2^exponent)
    return lux
end

""" pack_batch_sensor_message()

    Spacecraft Time (s)
    Raw Mag X
    Raw Mag Y
    Raw Mag Z
    Raw Hall
    Raw GYR X
    Raw GYR Y
    Raw GYR Z
    Lux +X
    Lux +Y
    Lux +Z
    Lux -X
    Lux -Y
    Lux -Z
"""
function pack_batch_sensor_message(
    unix_timestamp::Real,
    mag_measurement::Vector{<:Real},
    gyro_measurement::Vector{<:Real},
    sun_sensors::Vector{<:Real};
    raw_hall::Int=0,
    mag_scalar::Real=(1 / 16),
    gyro_scalar::Real=(1 / 65.6)
)
    spacecraft_time = Int(floor(unix_timestamp))
    spacecraft_time_bytes = py"($spacecraft_time).to_bytes(length=4, byteorder='big')"
    mag_raw = Int.(floor.(mag_measurement / mag_scalar))
    gyro_raw = Int.(floor.(gyro_measurement / gyro_scalar))
    imu_bytes = pystruct.pack("<hhhhhhh",
        mag_raw[1],
        mag_raw[2],
        mag_raw[3],
        raw_hall,
        gyro_raw[1],
        gyro_raw[2],
        gyro_raw[3],
    )

    sun_sensor_bytes = pystruct.pack(">HHHHHH",
        lux_to_opt3001_raw(sun_sensors[1]),
        lux_to_opt3001_raw(sun_sensors[2]),
        lux_to_opt3001_raw(sun_sensors[3]),
        lux_to_opt3001_raw(sun_sensors[4]),
        lux_to_opt3001_raw(sun_sensors[5]),
        lux_to_opt3001_raw(sun_sensors[6])
    )

    sensor_bytes = Vector{UInt8}(spacecraft_time_bytes * imu_bytes * sun_sensor_bytes)

    return sensor_bytes
end

function unpack_batch_sensor_message(sensor_packet;
    mag_scalar::Real=(1 / 16.0),
    gyro_scalar::Real=(1 / 65.6)
)
    spacecraft_time = py"int.from_bytes($(sensor_packet[1:4]), 'big')"
    imu_data = pystruct.unpack("<hhhhhhh", sensor_packet[5:18])
    mag_measurement = [imu_data[1:3]...] * mag_scalar
    raw_hall = imu_data[4]
    gyro_measurement = [imu_data[5:7]...] * gyro_scalar
    sun_raw = pystruct.unpack(">HHHHHH", sensor_packet[19:30])
    sun_lux = [opt3001_raw_to_lux(sun_raw_i) for sun_raw_i in sun_raw]

    return (
        spacecraft_time,
        mag_measurement,
        raw_hall,
        gyro_measurement,
        sun_lux
    )
end