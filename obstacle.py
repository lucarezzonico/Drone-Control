from helpers import *
    
class Obstacle(Potential_Field):
#   Integrate Multiranger
# 	get_force()
# 	check_destination()
# 	box_detected()

    def __init__(self):
        super().__init__()

    def on_top(self,stream):
        # If there's an obstacle on the top
        if is_close_up(stream.up):
            print("Obstacle on top")
            return True
        return False

    def box_detected(self, stream, sequence, epsilon):
        # RETURN True if box detected
        # RETURN False otherwise
        if (stream.down < (sequence.fly_height-epsilon)):
            print('I am over the box at the position x={} y={} z={}'.format(stream.get_pos()[0], stream.get_pos()[1], stream.down))
            return True
        return False

    def box_side_detected(self,stream, flying_height, epsilon, time_between_two_detections):
        #RETURN True if box side is detected
        #RETURN False otherwise
        z_range_now = stream.get_z()-flying_height
        if (z_range_now < -epsilon) or (z_range_now > epsilon):
            for z in stream.get_z_list(time_between_two_detections)[:-1]:
                if (z-flying_height) < -epsilon or (z-flying_height) > epsilon:
                    return False
            print("box_side_detected() says z is:", stream.get_z())
            return True
        else:
            return False 

    def get_force(self, stream):
        # RETURN the force of repulsion of an obstacle if there is one
        # RETURN None otherwise
        
        f = stream.front
        b = stream.back
        r = stream.right
        l = stream.left

        force = Force()

        if is_close(f) and pow(f,2) != 0:
            force.x -= self.ko / pow(f,2)
            force.y -= self.krot / pow(f,2) # push drone to the right
        if is_close(b) and pow(b,2) != 0:
            force.x += self.ko / pow(b,2)
            force.y -= self.krot / pow(b,2)
        if is_close(l) and pow(l,2) != 0:
            force.x += self.krot / pow(l,2)
            force.y -= self.ko / pow(l,2)
        if is_close(r) and pow(r,2) != 0:
            force.x += self.krot / pow(r,2)
            force.y += self.ko / pow(r,2)
        if not force.is_zero():
            print("OBSTACLE")
        return force.get()

	
    def on_destination(self, goal, stream):
        # RETURN True if the destination is on an obstacle
        # RETURN False otherwise
        f = stream.front
        b = stream.back
        r = stream.right
        l = stream.left

        g = goal.get()
        p = stream.get_pos()
        v = (g[0]-p[0],g[1]-p[1]) #vector between position and goal

        s = 0.5 # security distance

        if is_close(f) and pow(f,2) != 0 and v[0]<(f+s) and v[0]>f and abs(v[1])<s:
            return True
        if is_close(b) and pow(b,2) != 0 and -v[0]<(b+s) and -v[0]>b and abs(v[1])<s:
            return True
        if is_close(l) and pow(l,2) != 0 and v[1]<(l+s) and v[1]>l and abs(v[0])<s:
            return True
        if is_close(r) and pow(r,2) != 0 and -v[1]<(r+s) and -v[1]>r and abs(v[0])<s:
            return True
        return  False