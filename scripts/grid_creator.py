def create_waypoints(init_coordinate, width, height, spacing):
    waypoints = []

    counter = 0
    current_coordinate = [init_coordinate[0] + (2*counter+1)*spacing, init_coordinate[1]]

    lower_limit = init_coordinate[0]
    upper_limit = init_coordinate[0] + height
    left_limit = init_coordinate[1]
    right_limit = init_coordinate[1] + width

    while (current_coordinate[0] <= upper_limit):
        if (counter % 2 == 0):
            waypoints.append((current_coordinate[0], current_coordinate[1]))
            waypoints.append((current_coordinate[0], current_coordinate[1] + width))
        else:
            waypoints.append((current_coordinate[0], current_coordinate[1] + width))
            waypoints.append((current_coordinate[0], current_coordinate[1]))

        counter += 1

        current_coordinate = [init_coordinate[0] + (2*counter+1)*spacing, init_coordinate[1]]

    return waypoints

def main():
    init_coordinate = (0.5,-2)
    width = 5
    height = 3
    spacing = 0.5

    waypoints = create_waypoints(init_coordinate, width, height, spacing)
    print(waypoints)

if __name__ == "__main__":
    main()


