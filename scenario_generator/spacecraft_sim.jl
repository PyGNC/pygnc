#Spacecraft Orbit Simulator 
#Newton Euler Dynamics

using LinearAlgebra
using ForwardDiff
#using Plots
using SatelliteDynamics
using DelimitedFiles

#accurate dynamics

#epc is the current epoch (current time), and x is the state

#accurate dynamics. considers gravity, J2, drag, solar radiation
#pressure, and the effect of other bodies (sun/moon)

#ẋ = f(x,t)
function ground_truth_sat_dynamics(x, epc)
    
    r = x[1:3] #satellite position in inertial frame
    v = x[4:6] #satellite velocity in inertial frame
     
    #look up this term. seems to give a rotation matrix
    PN = bias_precession_nutation(epc)
    Earth_r = earth_rotation(epc)
    rpm  = polar_motion(epc) 

    R = rpm*Earth_r*PN
    
    #compute the 3x3 rotation matrix from ECI to ECEF 
    R = rECItoECEF(epc)
    
    #Compute the sun and moon positions in ECI frame
    r_sun = sun_position(epc)
    r_moon = moon_position(epc)
    
    #define the acceleration variable
    a = zeros(eltype(x), 3)
    
    #compute acceleration caused by Earth gravity (includes J2)
    #modeled by a spherical harmonic gravity field
    n_grav = 10
    m_grav = 10
    #main contribution in acceleration
    a+= accel_gravity(x, R, n_grav, m_grav)

    #########custom gravity with J2 instead of using package
    #μ = 3.986004418e14 #m3/s2
    #J2 = 1.08264e-3 
        
    #a_2bp = (-μ*r)/(norm(r))^3
    
    #Iz = [0,0,1]
    
    #a_J2 = ((3*μ*J2*R_EARTH^2)/(2*norm(r)^5))*((((5*dot(r, Iz)^2)/norm(r)^2)-1)*r - 2*dot(r,Iz)*Iz)     

    #a_grav = a_2bp + a_J2
    
    #a+= a_grav
    ####################

    #atmospheric drag
    #compute the atmospheric density from density harris priester model
    ρ = density_harris_priester(epc,r)
    
    #computes acceleration due to drag in inertial directions
    
    cd = 2.0 #drag coefficient
    area_drag = 0.1 #in m2 #area normal to the velocity direction
    m = 1.0
    
    a += accel_drag(x, ρ, m, area_drag, cd, Array{Real,2}(PN))
    
    a_drag = accel_drag(x, ρ, m, area_drag, cd, Array{Real,2}(PN))

    #Solar Radiation Pressure
    #nu = eclipse_conical(x, r_sun) #what is this for
    area_srp = 1.0
    coef_srp = 1.8
    #COMMENTED FOR TESTING
    a += accel_srp(x, r_sun, m, area_srp, coef_srp)
    a_srp = accel_srp(x, r_sun, m, area_srp, coef_srp)
    #acceleration due to external bodies
    
    #COMMENTED FOR TESTING
    a+= accel_thirdbody_sun(x, r_sun)
    a_sun = accel_thirdbody_sun(x, r_sun)
    
    #COMMENTED FOR TESTING
    a+= accel_thirdbody_moon(x, r_moon)
    a_moon = accel_thirdbody_moon(x, r_moon)
    
    a_unmodeled = a_srp + a_sun + a_moon
            
    xdot = x[4:6]
    vdot = a
    
    x_dot = [xdot; vdot]
    
    return x_dot
    
end

function RK4_accurate_model(x, t, h)
    
    f1 = ground_truth_sat_dynamics(x, t) 
    f2 = ground_truth_sat_dynamics(x+0.5*h*f1, t+h/2)
    f3 = ground_truth_sat_dynamics(x+0.5*h*f2, t+h/2)
    f4 = ground_truth_sat_dynamics(x+h*f3, t+h)
    
    xnext = x+(h/6.0)*(f1+2*f2+2*f3+f4)
        
    return xnext
    
end


# initial time for sim
epc0 = Epoch(2012, 11, 8, 12, 0, 0, 0.0)

#Test of some of the SatelliteDynamics functions
#Orbit we want (around ISS orbit conditions)
# iss1 = [6871e3, 0.00064, 51.6450, 1.0804, 27.7899, 190]; 
iss1 = [6928.1e3, 0.0, 98.7, -0.1, 0.0, 190]; 

# Convert osculating elements to Cartesean state
# returns position and velocity (m, m/s). This is the intial position
eci0_1 = sOSCtoCART(iss1, use_degrees=true)

#find the period of the orbit (seconds). only dependent on semi major axis
#T = orbit_period(iss1[1])

T = 60*60*3

#final time
epcf = epc0 + T

h = 0.1 #10 Hz 

#initial pose
x_0 = eci0_1
initial_orbital_elements = SatelliteDynamics.sCARTtoOSC(x_0)

orbitperiod = orbit_period(initial_orbital_elements[1])

orbit_num = 1 #only simulating 1 orbit at 10 Hz

Tf = orbitperiod*orbit_num

t = Array(range(0,Tf, step=h)) #create a range to final time at constant time step
    
all_x = zeros(length(x_0), length(t)) #variable to store all x
    
all_x[:,1] = x_0 #set the initial state
    
for k=1:(length(t) - 1)
        
    current_t = epc0+t[k+1] #calculate the current time
        
    all_x[:,k+1] .= RK4_accurate_model(all_x[:,k], current_t, h) #calculate the next state
        
end

x_hist = all_x

#plot(x_hist[1,:], x_hist[2,:], x_hist[3,:])


# conversion from cartesian coordinates to spherical coordinates
# function cartesian_to_spherical(x)
#     r = sqrt(x[1:3]'*x[1:3])
#     θ = atan(x[3],sqrt(x[1:2]'*x[1:2]))
#     ϕ = atan(x[2],x[1])
    
#     return [r; θ; ϕ]
# end


#Implement this function in spherical coordinates
# function gravitational_potential_new(s)

#     # input: position in spherical coordinates 
#     # s = [r, θ, ϕ]
#     # output: gravitational potential
    
#     #J2 = mu (in km) * radius of Earth^2 (km2)* J2 term
#     #Constants
#     μ = 3.986004418e14 #m3/s2
#     J2 = 1.08264e-3 
    
#     # unpack input
#     r = s[1]
#     θ = s[2]
    
#     m = 1.0 #added in
    
#     #only a function of the latitude
#     U = (μ/r)*(1+((J2*R_EARTH^2)/(2*r^2))*(1-3*(sin(θ))^2))
    
#     return U

# end

# function gravitational_acceleration(x)
#     # input: position in cartesian coordiantes 
#     # output: acceleration in cartesian coordiantes 
    
#     #position
#     q = x[1:3]
    
#     #velocity
#     v = x[4:6]
    
#     c_d = 2.0 #drag coefficient (dimensionless)
    
#     #cross sectional area in m2
#     A = 0.1 

#     ρ = 5e-12
    
#     #rotation of the earth (rad/s)
#     ω_earth = [0,0, OMEGA_EARTH]
    
#     v_rel = v - cross(ω_earth, q)
    
#     f_drag = -0.5*c_d*(A)*ρ*norm(v_rel)*v_rel
    
#     a = (ForwardDiff.gradient(_x -> gravitational_potential_new(cartesian_to_spherical(_x)), q))+ f_drag
    
#     return a 
# end

# function orbit_dynamics(x)
    
#     q = x[1:3]
#     v = x[4:6]
    
#     a = gravitational_acceleration(x) #obtain the gravitational acceleration given the position q
    
#     ẋ = [v; a] #x dot is velocity and acceleration
    
#     return ẋ
# end

# function RK4_satellite_potential(x)

#     h = 1.0 #time step
#     f1 = orbit_dynamics(x)
#     f2 = orbit_dynamics(x+0.5*h*f1)
#     f3 = orbit_dynamics(x+0.5*h*f2)
#     f4 = orbit_dynamics(x+h*f3)
#     xnext = x+(h/6.0)*(f1+2*f2+2*f3+f4)
#     return xnext
    
# end

#use the simplified dynamics model and integrate with RK4

# all_x_2 =  zeros(length(x_0), length(t)) #variable to store all x

# all_x_2[:,1] = x_0

# for k=1:(length(t) - 1)
                
#     all_x_2[:,k+1] .= RK4_satellite_potential(all_x[:,k]) #calculate the next state
        
# end


#plot(all_x_2[1,:], all_x_2[2,:], all_x_2[3,:])

function hat(ω)
    return [0    -ω[3]  ω[2];
            ω[3]  0    -ω[1];
           -ω[2]  ω[1]  0   ]
end

function L(q)
    [q[1] -q[2:4]'; q[2:4] q[1]*I + hat(q[2:4])]
end

function R(q)
    [q[1] -q[2:4]'; q[2:4] q[1]*I - hat(q[2:4])]
end

const H = [zeros(1,3); I];

#momoent of inertia matrix
#J = Diagonal([1.0; 1.0; 1.0])

#from kevin's notebook
# J = [   1.959e-4    2016.333e-9     269.176e-9;
#      2016.333e-9       1.999e-4    2318.659e-9;
#       269.176e-9    2318.659e-9       1.064e-4]

J = [0.0043 -0.0003 0; -0.0003 0.0049 0; 0 0 0.0035]


#Eulers equations
function dynamics_ω(ω)
    
    # solution
    ω_dot= -J\(hat(ω)*(J*ω))

    return ω_dot
end


function dynamics_q(x)
    
    # TODO: implement attitude dynamics with quaternion 
    ẋ = zeros(length(x))
    
    #quaternion
    q = x[1:4]

    #body angular velocity
    ω = x[5:7]
    
    #quaternion kinematics
    q̇ = 0.5*L(q)*H*ω

    #xdot 
    ẋ = [q̇; dynamics_ω(ω)]
    
    return ẋ
end


#using ODE package
# function dynamics_q!(ẋ,x,p,t)
#     # make the ODE in-place so it works with OrdinaryDiffEq
#     ẋ .= dynamics_q(x)
# end

# using OrdinaryDiffEq
# Tf = 60.0
# tspan = (0.0, N)
# prob_q = ODEProblem(dynamics_q!, x0q, tspan)
# sol_q = solve(prob_q,Tsit5());

# sol_q.t
# norm(sol_q.u[end][1:4])
# sol_q.u[end]

function RK4_quat(x)

    f1 = dynamics_q(x)
    f2 = dynamics_q(x+0.5*h*f1)
    f3 = dynamics_q(x+0.5*h*f2)
    f4 = dynamics_q(x+h*f3)
    xnext = x+(h/6.0)*(f1+2*f2+2*f3+f4)

    #renormalizing the quaternion at each step
    xnext[1:4] .= xnext[1:4]/norm(xnext[1:4])
    return xnext
    
end


function Expq(ϕ)
    
    # quaternion exponential map ϕ → q 

    θ = norm(ϕ) #angle
    
    s = cos(θ/2)
    
    v = (ϕ/2)*sinc(θ/(2*pi))
    
    q = [s; v]
    
    q_norm = q/norm(q)
    
    return q_norm

    #return q
end

function lie_midpoint_step_q(xk,h)
    
    qk = xk[1:4]
    ωk = xk[5:7]
    
    xn = zeros(length(xk))


    ωm = ωk+0.5*h*dynamics_ω(ωk)
    ωn = ωk+h*dynamics_ω(ωm)
    
    qn = L(qk)*Expq(h*ωm)
    
    xn = [qn; ωn]
    
    return xn
end

q0 = [1.0; 0; 0; 0]
#ω0 = [0; 2*pi; 0] + 1e-2*randn(3) #small perturbation from intermediate-axis spin


#ω0 = [1*pi/180; 0.5*pi/180; 0] #this the the resulting angular acceleration from the detumbling controller

ω0 = (1*pi/180)*[1.0, 0.0, 0.0]

#get a Lie step integrator

x0q = [q0; ω0]

N = size(t)[1]
xtrajq = zeros(7,N)
xtrajq[:,1] .= x0q


xtrajq_lie = zeros(7,N)
xtrajq_lie[:,1] .= x0q

for k = 1:(N-1)
    #for k=1:2
        xtrajq[:,k+1] = RK4_quat(xtrajq[:,k])
end

xtrajq

for k = 1:(N-1)
#for k=1:2
    xtrajq_lie[:,k+1] = lie_midpoint_step_q(xtrajq_lie[:,k], h)
end

all_quaternions = xtrajq[1:4, :]

xtrajq_lie

xtrajq

all_quaternions_lie = xtrajq_lie[1:4, :]

#lie_midpoint_step_q(xtrajq[:,14], h)

all_quaternions
all_quaternions_lie

xtrajq_lie
normz = zeros(N)
for k=1:N
    normz[k] = norm(all_quaternions[:,k])
end

all_quaternions

all_quaternions[:,21]
all_omega = xtrajq[5:7,:] 

all_omega[:,1]

all_omega[:,end]

writedlm("sim_attitude_omega_v2_new_orbit.txt", xtrajq_lie , ", ")
writedlm("sim_poses_v2_new_orbit.txt", x_hist , ", ")

x_hist