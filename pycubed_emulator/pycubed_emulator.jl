using Pkg
Pkg.activate(@__DIR__)

using LinearAlgebra
using LibSerialPort

include("src/shell_actions.jl")

this_tty = readchomp(open(`tty`, "r", stdout)) # this allows us to directly write to the current terminal

function pycubed_emulator(;
    serial_port::AbstractString=this_tty,
    serial_baud::Int=115200, # serial port baud rate
    username::AbstractString="pi", # username on remote device
    scenario_path::AbstractString=joinpath(@__DIR__, "..", "scenarios", "example_scenario"), # directory for scenario data
    output_batch_sensor_data_file::AbstractString="~/sense.bin", # filepath on rpi to place sensor data in
    real_time::Bool=true, # output data in real time (true) or as fast as possible given min_delay (false)
    minimum_delay::Real=0.1 # minimum delay between subsequent data messages
)

    dev = connect_to_serial_device(serial_port, serial_baud)
    println("Connected to $(serial_port) at $(serial_baud)")
    try
        proper_login, response = check_login(dev, username)
        if !proper_login
            @error "Device at $(serial_port) is not properly logged into user $(username), response was $(response)"
            return
        end
        println("Changing directory to root")
        change_directory(dev, "~")

        res = command_response(dev, "cat ~/pygnc_version.info", all_lines=true)
        println("pygnc version: $res")

        batch_scenario_path = joinpath(scenario_path, "batch_gps_sensor_data.bin")
        transfer_file_to(dev, batch_scenario_path, output_batch_sensor_data_file)



    finally
        close(dev)
    end
end

pi_tty = "/dev/cu.usbserial-AB7JTG00"
pycubed_emulator(serial_port=pi_tty)
# pycubed_emulator(serial_port=this_tty)