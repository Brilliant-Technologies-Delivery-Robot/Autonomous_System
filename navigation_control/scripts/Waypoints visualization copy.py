import matplotlib.pyplot as plt

def plot_waypoints(waypoints):
    # Extract x and y coordinates from waypoints
    x_coords = [point[0] for point in waypoints]
    y_coords = [point[1] for point in waypoints]
    
    # Create a plot
    plt.figure(figsize=(10, 6))
    
    # Plot waypoints
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b')
    
    # Add titles and labels
    plt.title('Waypoints Plot')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    
    # Add grid
    plt.grid(True)
    
    # Show the plot
    plt.show()

# Example usage with your waypoints
waypoints = [
[2.275, 2.6250000000000013, 0.0],
[2.275, 2.5750000000000006, 0.0],
[2.275, 2.525, 0.0],
[2.275, 2.475000000000001, 0.0],
[2.275, 2.4250000000000003, 0.0],
[2.275, 2.3750000000000013, 0.0],
[2.275, 2.3250000000000006, 0.0],
[2.3250000000000006, 2.275, 0.0],
[2.3250000000000006, 2.225000000000001, 0.0],
[2.3250000000000006, 2.1750000000000003, 0.0],
[2.3250000000000006, 2.1250000000000013, 0.0],
[2.3250000000000006, 2.0750000000000006, 0.0],
[2.3250000000000006, 2.025, 0.0],
[2.3250000000000006, 1.975000000000001, 0.0],
[2.3250000000000006, 1.9250000000000003, 0.0],
[2.3250000000000006, 1.8750000000000013, 0.0],
[2.3250000000000006, 1.8250000000000006, 0.0],
[2.3250000000000006, 1.775, 0.0],
[2.3250000000000006, 1.725000000000001, 0.0],
[2.3250000000000006, 1.6750000000000003, 0.0],
[2.3250000000000006, 1.6250000000000013, 0.0],
[2.3250000000000006, 1.5750000000000006, 0.0],
[2.3250000000000006, 1.525, 0.0],
[2.3250000000000006, 1.475000000000001, 0.0],
[2.3250000000000006, 1.4250000000000003, 0.0],
[2.3250000000000006, 1.3750000000000013, 0.0],
[2.3250000000000006, 1.3250000000000006, 0.0],
[2.275, 1.275, 0.0],
[2.225000000000001, 1.225000000000001, 0.0],
[2.1750000000000003, 1.1750000000000003, 0.0],
[2.1250000000000013, 1.1250000000000013, 0.0],
[2.0750000000000006, 1.0750000000000006, 0.0],
[2.025, 1.025, 0.0],
[1.975000000000001, 0.9750000000000011, 0.0],
[1.9250000000000003, 0.9250000000000004, 0.0],
[1.8750000000000013, 0.9250000000000004, 0.0],
[1.8250000000000006, 0.9250000000000004, 0.0],
[1.775, 0.9250000000000004, 0.0],
[1.725000000000001, 0.9250000000000004, 0.0],
[1.6750000000000003, 0.8750000000000014, 0.0],
[1.6250000000000013, 0.8750000000000014, 0.0],
[1.5750000000000006, 0.8750000000000014, 0.0],
[1.525, 0.8750000000000014, 0.0],
[1.475000000000001, 0.8750000000000014, 0.0],
[1.4250000000000003, 0.8750000000000014, 0.0],
[1.3750000000000013, 0.8750000000000014, 0.0],
[1.3250000000000006, 0.8750000000000014, 0.0],
[1.275, 0.8750000000000014, 0.0],
[1.225000000000001, 0.8750000000000014, 0.0],
[1.1750000000000003, 0.8750000000000014, 0.0],
[1.1250000000000013, 0.9250000000000004, 0.0],
[1.0750000000000006, 0.9250000000000004, 0.0],
[1.025, 0.9750000000000011, 0.0],
[0.9750000000000011, 1.025, 0.0],
[-0.475, 0.17500000000000035, 0.0],
[-0.475, 0.22500000000000106, 0.0],
[-0.475, 0.275, 0.0],
[-0.475, 0.32500000000000073, 0.0],
[-0.475, 0.37500000000000144, 0.0],
[-0.475, 0.4250000000000004, 0.0],
[-0.475, 0.4750000000000011, 0.0],
[-0.475, 0.525, 0.0],
[-0.475, 0.5750000000000007, 0.0],
[-0.475, 0.6250000000000014, 0.0],
[-0.475, 0.6750000000000004, 0.0],
[-0.475, 0.7250000000000011, 0.0],
[-0.475, 0.775, 0.0],
[-0.475, 0.8250000000000007, 0.0],
[-0.475, 0.8750000000000014, 0.0],
[-0.42499999999999927, 0.9250000000000004, 0.0],
[-0.42499999999999927, 0.9750000000000011, 0.0],
[-0.42499999999999927, 1.025, 0.0],
[-0.42499999999999927, 1.0750000000000006, 0.0],
[-0.42499999999999927, 1.1250000000000013, 0.0],
[-0.42499999999999927, 1.1750000000000003, 0.0],
[-0.42499999999999927, 1.225000000000001, 0.0],
[-0.42499999999999927, 1.275, 0.0],
[-0.37499999999999856, 1.3250000000000006, 0.0],
[-0.3249999999999996, 1.3750000000000013, 0.0],
[-0.2749999999999989, 1.4250000000000003, 0.0],
[-0.225, 1.475000000000001, 0.0],
[-0.1749999999999993, 1.525, 0.0],
[-0.12499999999999858, 1.5750000000000006, 0.0],
[-0.07499999999999965, 1.6250000000000013, 0.0],
[-0.024999999999998933, 1.6750000000000003, 0.0],
[0.025, 1.725000000000001, 0.0],
[0.0750000000000007, 1.775, 0.0],
[0.12500000000000142, 1.8250000000000006, 0.0],
[0.17500000000000035, 1.8750000000000013, 0.0],
[0.22500000000000106, 1.9250000000000003, 0.0],
[0.275, 1.975000000000001, 0.0],
[0.32500000000000073, 2.025, 0.0],
[0.37500000000000144, 2.0750000000000006, 0.0],
[0.4250000000000004, 2.1250000000000013, 0.0],
[0.4750000000000011, 2.1750000000000003, 0.0],
[0.525, 2.225000000000001, 0.0],
[0.5750000000000007, 2.275, 0.0],
[0.6250000000000014, 2.3250000000000006, 0.0],
[0.6750000000000004, 2.3750000000000013, 0.0],
[0.7250000000000011, 2.4250000000000003, 0.0],
[0.775, 2.475000000000001, 0.0],
[0.8250000000000007, 2.475000000000001, 0.0],
[0.8750000000000014, 2.475000000000001, 0.0],
[0.9250000000000004, 2.475000000000001, 0.0],
[0.9750000000000011, 2.475000000000001, 0.0],
[1.025, 2.475000000000001, 0.0],
[1.0750000000000006, 2.475000000000001, 0.0],
[1.1250000000000013, 2.475000000000001, 0.0],
[1.1750000000000003, 2.475000000000001, 0.0],
[1.225000000000001, 2.475000000000001, 0.0],
[1.275, 2.475000000000001, 0.0],
[1.3250000000000006, 2.475000000000001, 0.0],
[1.3750000000000013, 2.475000000000001, 0.0],
[1.4250000000000003, 2.475000000000001, 0.0],
[1.475000000000001, 2.475000000000001, 0.0],
[1.525, 2.475000000000001, 0.0],
[1.5750000000000006, 2.475000000000001, 0.0],
[1.6250000000000013, 2.475000000000001, 0.0],
[1.6750000000000003, 2.475000000000001, 0.0],
[1.725000000000001, 2.475000000000001, 0.0],
[1.775, 2.475000000000001, 0.0],
[1.8250000000000006, 2.475000000000001, 0.0],
[1.8750000000000013, 2.475000000000001, 0.0],
[1.9250000000000003, 2.475000000000001, 0.0],
[1.975000000000001, 2.4250000000000003, 0.0],
[2.025, 2.4250000000000003, 0.0],
[2.0750000000000006, 2.475000000000001, 0.0],
[2.1250000000000013, 2.525, 0.0],
[2.1750000000000003, 2.5750000000000006, 0.0],
[2.225000000000001, 2.6250000000000013, 0.0],
[2.275, 2.6750000000000003, 0.0],
[2.725000000000001, 2.475000000000001, 0.0],
[2.725000000000001, 2.4250000000000003, 0.0],
[2.725000000000001, 2.3750000000000013, 0.0],
[2.725000000000001, 2.3250000000000006, 0.0],
[2.725000000000001, 2.275, 0.0],
[2.725000000000001, 2.225000000000001, 0.0],
[2.725000000000001, 2.1750000000000003, 0.0],
[2.725000000000001, 2.1250000000000013, 0.0],
[2.725000000000001, 2.0750000000000006, 0.0],
[2.725000000000001, 2.025, 0.0],
[2.725000000000001, 1.975000000000001, 0.0],
[2.725000000000001, 1.9250000000000003, 0.0],
[2.6750000000000003, 1.8750000000000013, 0.0],
[2.6750000000000003, 1.8250000000000006, 0.0],
[2.6750000000000003, 1.775, 0.0],
[2.6750000000000003, 1.725000000000001, 0.0],
[2.6750000000000003, 1.6750000000000003, 0.0],
[2.6250000000000013, 1.6250000000000013, 0.0],
[2.5750000000000006, 1.5750000000000006, 0.0],
[2.525, 1.525, 0.0],
[2.475000000000001, 1.475000000000001, 0.0],
[2.4250000000000003, 1.4250000000000003, 0.0],
[2.3750000000000013, 1.3750000000000013, 0.0],
[2.3250000000000006, 1.3250000000000006, 0.0],
[2.275, 1.275, 0.0],
[2.225000000000001, 1.225000000000001, 0.0],
[2.1750000000000003, 1.1750000000000003, 0.0],
[2.1250000000000013, 1.1250000000000013, 0.0],
[2.0750000000000006, 1.0750000000000006, 0.0],
[2.025, 1.025, 0.0],
[1.975000000000001, 0.9750000000000011, 0.0],
[1.9250000000000003, 0.9250000000000004, 0.0],
[1.8750000000000013, 0.9250000000000004, 0.0],
[1.8250000000000006, 0.9250000000000004, 0.0],
[1.775, 0.9250000000000004, 0.0],
[1.725000000000001, 0.9250000000000004, 0.0],
[1.6750000000000003, 0.9250000000000004, 0.0],
[1.6250000000000013, 0.9250000000000004, 0.0],
[1.5750000000000006, 0.9250000000000004, 0.0],
[1.525, 0.9750000000000011, 0.0],
[1.475000000000001, 0.9750000000000011, 0.0],
[1.4250000000000003, 0.9750000000000011, 0.0],
[1.3750000000000013, 0.9750000000000011, 0.0],
[1.3250000000000006, 0.9750000000000011, 0.0],
[1.275, 0.9750000000000011, 0.0],
[1.225000000000001, 0.9750000000000011, 0.0],
[1.1750000000000003, 0.9750000000000011, 0.0],
[1.1250000000000013, 0.9750000000000011, 0.0],
[1.0750000000000006, 0.9750000000000011, 0.0],
[1.025, 0.9750000000000011, 0.0],
[0.9750000000000011, 0.9750000000000011, 0.0],
[0.9250000000000004, 0.9750000000000011, 0.0],
[0.8750000000000014, 0.9750000000000011, 0.0],
[0.8250000000000007, 0.9750000000000011, 0.0],
[0.775, 0.9750000000000011, 0.0],
[0.7250000000000011, 0.9750000000000011, 0.0],
[0.6750000000000004, 1.025, 0.0],
[0.6250000000000014, 1.0750000000000006, 0.0],
[0.5750000000000007, 1.1250000000000013, 0.0],
[0.525, 1.1250000000000013, 0.0],
[0.4750000000000011, 1.0750000000000006, 0.0],
[0.4250000000000004, 1.025, 0.0],
[0.37500000000000144, 0.9750000000000011, 0.0],
[0.32500000000000073, 0.9250000000000004, 0.0],
[0.275, 0.8750000000000014, 0.0],
[0.22500000000000106, 0.8250000000000007, 0.0],
[0.17500000000000035, 0.775, 0.0],
[0.12500000000000142, 0.7250000000000011, 0.0],
[0.0750000000000007, 0.6750000000000004, 0.0],
[0.025, 0.6250000000000014, 0.0],
[-0.024999999999998933, 0.5750000000000007, 0.0],
[-0.07499999999999965, 0.525, 0.0],
[-0.12499999999999858, 0.4750000000000011, 0.0],
[-0.1749999999999993, 0.4250000000000004, 0.0],
[-0.225, 0.37500000000000144, 0.0],
[-0.2749999999999989, 0.32500000000000073, 0.0],
[-0.3249999999999996, 0.275, 0.0],
[-0.37499999999999856, 0.22500000000000106, 0.0],
[-0.42499999999999927, 0.17500000000000035, 0.0],
[-0.475, 0.12500000000000142, 0.0],
[-0.8249999999999996, 0.6250000000000014, 0.0],
[-0.7749999999999989, 0.6250000000000014, 0.0],
[-0.725, 0.6250000000000014, 0.0],
[-0.6749999999999993, 0.6250000000000014, 0.0],
[-0.6250000000000003, 0.6250000000000014, 0.0],
[-0.5749999999999996, 0.6250000000000014, 0.0],
[-0.5249999999999989, 0.6250000000000014, 0.0],
[-0.475, 0.6250000000000014, 0.0],
[-0.42499999999999927, 0.6250000000000014, 0.0],
[-0.37499999999999856, 0.6250000000000014, 0.0],
[-0.3249999999999996, 0.6250000000000014, 0.0],
[-0.2749999999999989, 0.6250000000000014, 0.0],
[-0.225, 0.6250000000000014, 0.0],
[-0.1749999999999993, 0.6250000000000014, 0.0],
[-0.12499999999999858, 0.6250000000000014, 0.0],
[-0.07499999999999965, 0.6250000000000014, 0.0],
[-0.024999999999998933, 0.6250000000000014, 0.0],
[0.025, 0.5750000000000007, 0.0],
[0.0750000000000007, 0.6250000000000014, 0.0],
[0.12500000000000142, 0.6250000000000014, 0.0],
[0.17500000000000035, 0.6250000000000014, 0.0],
[0.22500000000000106, 0.6250000000000014, 0.0],
[0.275, 0.6250000000000014, 0.0],
[0.32500000000000073, 0.6250000000000014, 0.0],
[0.37500000000000144, 0.6250000000000014, 0.0],
[0.4250000000000004, 0.6250000000000014, 0.0],
[0.4750000000000011, 0.6250000000000014, 0.0],
[0.525, 0.6250000000000014, 0.0],
[0.5750000000000007, 0.6250000000000014, 0.0],
[0.6250000000000014, 0.6250000000000014, 0.0],
[0.6750000000000004, 0.6250000000000014, 0.0],
[0.7250000000000011, 0.6250000000000014, 0.0],
[0.775, 0.6250000000000014, 0.0],
[0.8250000000000007, 0.6250000000000014, 0.0],
[0.8750000000000014, 0.6250000000000014, 0.0],
[0.9250000000000004, 0.6250000000000014, 0.0],
[0.9750000000000011, 0.6250000000000014, 0.0],
[1.025, 0.6250000000000014, 0.0],
[1.0750000000000006, 0.6250000000000014, 0.0],
[1.1250000000000013, 0.6250000000000014, 0.0],
[1.1750000000000003, 0.6250000000000014, 0.0],
[1.225000000000001, 0.6250000000000014, 0.0],
[1.275, 0.6250000000000014, 0.0],
[1.3250000000000006, 0.6250000000000014, 0.0],
[1.3750000000000013, 0.6250000000000014, 0.0],
[1.4250000000000003, 0.6250000000000014, 0.0],
[1.475000000000001, 0.6250000000000014, 0.0],
[1.525, 0.6250000000000014, 0.0],
[1.5750000000000006, 0.6250000000000014, 0.0],
[1.6250000000000013, 0.6250000000000014, 0.0],
[1.6750000000000003, 0.6750000000000004, 0.0],
[1.725000000000001, 0.7250000000000011, 0.0],
[1.775, 0.775, 0.0],
[1.8250000000000006, 0.8250000000000007, 0.0],
[1.8750000000000013, 0.8750000000000014, 0.0],
[1.9250000000000003, 0.9250000000000004, 0.0],
[1.975000000000001, 0.9750000000000011, 0.0],
[2.025, 1.025, 0.0],
[2.0750000000000006, 1.0750000000000006, 0.0],
[2.1250000000000013, 1.1250000000000013, 0.0],
[2.1250000000000013, 1.1750000000000003, 0.0],
[2.1250000000000013, 1.225000000000001, 0.0],
[2.1750000000000003, 1.275, 0.0],
[2.1750000000000003, 1.3250000000000006, 0.0],
[2.1750000000000003, 1.3750000000000013, 0.0],
[2.225000000000001, 1.4250000000000003, 0.0],
[2.225000000000001, 1.475000000000001, 0.0],
[2.225000000000001, 1.525, 0.0],
[2.225000000000001, 1.5750000000000006, 0.0],
[2.225000000000001, 1.6250000000000013, 0.0],
[2.225000000000001, 1.6750000000000003, 0.0],
[2.225000000000001, 1.725000000000001, 0.0],
[2.225000000000001, 1.775, 0.0],
[2.225000000000001, 1.8250000000000006, 0.0],
[2.225000000000001, 1.8750000000000013, 0.0],
[2.225000000000001, 1.9250000000000003, 0.0],
[2.225000000000001, 1.975000000000001, 0.0],
[2.275, 2.025, 0.0],
[2.3250000000000006, 2.0750000000000006, 0.0],
[2.3750000000000013, 2.1250000000000013, 0.0],
[2.4250000000000003, 2.1750000000000003, 0.0],
[2.475000000000001, 2.225000000000001, 0.0],
[2.525, 2.275, 0.0],
[2.5750000000000006, 2.3250000000000006, 0.0],
[2.6250000000000013, 2.3750000000000013, 0.0],
[2.6750000000000003, 2.4250000000000003, 0.0],
[2.725000000000001, 2.475000000000001, 0.0],
[2.775, 2.525, 0.0]

]

plot_waypoints(waypoints)