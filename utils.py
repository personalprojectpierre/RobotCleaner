def distance_reached(dist_target, dist_right, dist_left):
    if (dist_left >=dist_target) or (dist_right >= dist_target):
        return True
    else:
        return False


def get_seconds(time):
    minutes, seconds = time // 60, time % 60
    return seconds
