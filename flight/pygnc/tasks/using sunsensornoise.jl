using LinearAlgebra
using DelimitedFiles

normalizing_factor = 186729.2

#x1 = [1000, 2000, 3000, 4000, 5000, 6000]/normalizing_factor
#x2 = [1000.01, 2000.01, 3000.01, 4000.01, 5000.01, 6000.01]/normalizing_factor

x1 = [0, 11860, 0, 52900, 0, 83870]/normalizing_factor

x2 = [0.01, 11860.01, 0.01, 52900.01, 0.01, 83870.01]/normalizing_factor

B = [1.0*Matrix(I,3,3); -1.0*Matrix(I,3,3)]

#B.T@B, B.T@sun_sensors_normalized[:,None])

vec1 = B'*B\B'*x1

vec2 = B'*B\B'*x2

diff = vec2 - vec1

#read the state history from the file
state_history = readdlm("/home/fausto/pygnc/scenario_generator/state_history.txt", ',', Float64)