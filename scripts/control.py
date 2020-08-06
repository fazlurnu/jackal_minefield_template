def set_limit(input, upper_limit, lower_limit):
    if (input > upper_limit):
        return upper_limit
    elif (input < lower_limit):
        return lower_limit
    else:
        return input