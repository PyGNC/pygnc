using Infiltrator
using Base64
""" check_login
Return true if the terminal is properly logged in
"""
function check_login(dev, username)

    response = command_response(dev, "whoami")
    return username == response, response
end

""" command_response
Send a shell command and read a one line response
"""
function command_response(dev, command; all_lines=false)
    command_no_response(dev, command)
    sleep(0.5)
    if bytesavailable(dev) > 0
        println("$(bytesavailable(dev)) bytes available, reading lines")
        raw_response = read(dev, String)
        lines = split(raw_response, "\n")
        if all_lines
            # don't return first line (command that was typed)
            # and don't return last line (new shell prompt)
            response = [clean_line(line) for line in lines[2:end-1]]
        else
            response = clean_line(lines[2])
        end
    else
        @warn "No response received for command $(command)"
        response = nothing
    end
    return response
end

""" clean_line()
Clean up a string returned by the shell
"""
function clean_line(str)
    # remove ANSI shell escape sequences
    noansi = replace(str, r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])" => "")
    # remove non-printable characters
    ret = replace(noansi, r"[^[:print:]]" => "")
    return ret
end


function command_no_response(dev, command)
    write(dev, "$command\r")
    sleep(1)
end

function change_directory(dev, path)
    command_no_response(dev, "cd $path")
end

function check_directory(dev, path)
    response = command_response(dev, "pwd")
    return response == path
end

function transfer_file_to(dev, local_path, dev_path)

    println("Transferring $local_path to $dev_path")
    command_no_response(dev, "base64 -d > $dev_path")
    sleep(1.1)
    f = open(local_path)
    try
        # convert data to base64 and transfer 
        indata = read(f)
        encoded = base64encode(indata)
        println("writing\n$(String(indata))\n\nas\n\n$encoded")
        write(dev, encoded)
        sleep(0.5)
        EOT = b"\x04" # end of transmission
        write(dev, EOT)
        write(dev, EOT)
    finally
        close(f)
    end
end

function connect_to_serial_device(serial_port::AbstractString, serial_baud::Int)
    serial_port_list = get_port_list()
    this_tty = readchomp(open(`tty`, "r", stdout))
    if serial_port in serial_port_list
        println("Opening serial port: $serial_port at baud $serial_baud")
        dev = open(serial_port, serial_baud)
    elseif serial_port == this_tty
        # this allows us to write data to the terminal this script is running in
        println("Opening local tty device $serial_port")
        dev = open(serial_port, "r+")
    else
        @warn "Invalid port, should be $this_tty or one of: \n$serial_port_list"
    end

    return dev
end