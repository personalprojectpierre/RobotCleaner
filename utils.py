def get_seconds(time):
    minutes, seconds = time // 60, time % 60
    return seconds

def write_file(file_name, x, y, z):
    my_file = open(file_name, "a")
    my_file.write(str(x)+" "+str(y)+" "+str(z)+"\n")
    my_file.close()