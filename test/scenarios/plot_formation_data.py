import pandas
import matplotlib.pyplot as plt
import os

def main():
    range_data_A = pandas.read_csv("../../scenarios/default_formation_scenario/batch_range_data_1.csv", skipinitialspace=True)

    ABi = range_data_A["receiver_id"] == 2
    ACi = range_data_A["receiver_id"] == 3
    ADi = range_data_A["receiver_id"] == 4

    AB_ranges = range_data_A["true_range"][ABi]
    AC_ranges = range_data_A["true_range"][ACi]
    AD_ranges = range_data_A["true_range"][ADi]

    range_data_A["plot time"] = (range_data_A["time"] - range_data_A["time"][0]) / 60

    AB_times = range_data_A["plot time"][ABi]
    AC_times = range_data_A["plot time"][ACi]
    AD_times = range_data_A["plot time"][ADi]

    plt.figure()
    plt.plot(AB_times.to_numpy(), AB_ranges.to_numpy(), label="A to B")
    plt.plot(AC_times.to_numpy(), AC_ranges.to_numpy(), label="A to C")
    plt.plot(AD_times.to_numpy(), AD_ranges.to_numpy(), label="A to D")
    plt.xlabel("Time (min)")
    plt.ylabel("Range (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig("/Users/jbwillis/courses/slam_16833/project/relativerange.pdf")

    # Create a 3D plot of relative positions
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the four lines with legend names
    ax.plot3D(range_data_A["receiver_position[1]"][ABi].to_numpy()/1000 - range_data_A["requester_position[1]"][ABi].to_numpy()/1000, range_data_A["receiver_position[2]"][ABi].to_numpy() / 1000 - range_data_A["requester_position[2]"][ABi].to_numpy()/1000, range_data_A["receiver_position[3]"][ABi].to_numpy() / 1000 - range_data_A["requester_position[3]"][ABi].to_numpy()/1000, label='A to B')
    ax.plot3D(range_data_A["receiver_position[1]"][ACi].to_numpy()/1000 - range_data_A["requester_position[1]"][ACi].to_numpy()/1000, range_data_A["receiver_position[2]"][ACi].to_numpy() / 1000 - range_data_A["requester_position[2]"][ACi].to_numpy()/1000, range_data_A["receiver_position[3]"][ACi].to_numpy() / 1000 - range_data_A["requester_position[3]"][ACi].to_numpy()/1000, label='A to C')
    ax.plot3D(range_data_A["receiver_position[1]"][ADi].to_numpy()/1000 - range_data_A["requester_position[1]"][ADi].to_numpy()/1000, range_data_A["receiver_position[2]"][ADi].to_numpy() / 1000 - range_data_A["requester_position[2]"][ADi].to_numpy()/1000, range_data_A["receiver_position[3]"][ADi].to_numpy() / 1000 - range_data_A["requester_position[3]"][ADi].to_numpy()/1000, label='A to D')

    # Add legend
    ax.legend()
    plt.tight_layout()
    plt.savefig("/Users/jbwillis/courses/slam_16833/project/relative3d.pdf")

    # Create a 3D plot of orbit
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the four lines with legend names
    ax.plot3D(range_data_A["requester_position[1]"][ABi].to_numpy()/1000, range_data_A["requester_position[2]"][ABi].to_numpy() / 1000, range_data_A["requester_position[3]"][ABi].to_numpy() / 1000, label='A')
    ax.plot3D(range_data_A["receiver_position[1]"][ABi].to_numpy()/1000, range_data_A["receiver_position[2]"][ABi].to_numpy() / 1000, range_data_A["receiver_position[3]"][ABi].to_numpy() / 1000, label='B')
    ax.plot3D(range_data_A["receiver_position[1]"][ACi].to_numpy()/1000, range_data_A["receiver_position[2]"][ACi].to_numpy() / 1000, range_data_A["receiver_position[3]"][ACi].to_numpy() / 1000, label='C')
    ax.plot3D(range_data_A["receiver_position[1]"][ADi].to_numpy()/1000, range_data_A["receiver_position[2]"][ADi].to_numpy() / 1000, range_data_A["receiver_position[3]"][ADi].to_numpy() / 1000, label='D')
    plt.tight_layout()
    plt.savefig("/Users/jbwillis/courses/slam_16833/project/orbits3d.pdf")

    # Add legend
    ax.legend()

    plt.show()

main()