
from matplotlib.pyplot import axes
from helpers import *
from goal import Goal
from obstacle import Obstacle
from stream import Stream

from scipy.stats import linregress

# connect to crazyflie
URI = 'radio://0/80/2M/E7E7E7E70B'

# Global variable useful for everyone
cflib.crtp.init_drivers(enable_debug_driver=False)
cf = Crazyflie(rw_cache='./cache')

with SyncCrazyflie(URI, cf=cf) as scf:

    class Sequence:
        # Starter pack of attributes for a sub-class of Sequences
        # This way, every sequence have the same parameters such as mc, fly_height etc
        fly_height= 0.3 # m
        fly_speed_hori = 0.3 # m/s
        fly_speed_up = 0.5 # m/s
        fly_speed_down = 0.1 # m/s

        fly_speed_hori_slow = 0.15 # m/s
        dt = 0.01 # sec
        mc = MotionCommander(scf, default_height=fly_height)
        epsilon = 0.02
        time_between_two_detections = 0.5
        margin_from_home = 1

        def __init__(self, route):
            # Different depending on the class        
            self.route = route
            self.goal = Goal(route[0])
            self.it = 0
        
        def terminate(self):
            print("Terminate")
            self.mc.land(self.fly_speed_down)
            plt.figure()
            plt.plot(stream.x_list, stream.y_list)
            plt.plot(zip(self.goal.list_of_goals))
            plt.axis('equal')
            plt.show()
        
        def land(self):
            print("Land")
            self.mc.land(self.fly_speed_down)
        
        def moving_routine(self, obstacle_avoidance=True, slow=False):
            if obs.on_top(stream): # To stop the drone
                self.terminate()

            force = Force()
            if obstacle_avoidance:
                force.add(obs.get_force(stream))
            if force.is_zero():
                force.add(self.goal.get_force(stream))
            if slow:
                v = bound(v=force.to_velocity(dt=self.dt),b=self.fly_speed_hori_slow)
            else:
                v = bound(v=force.to_velocity(dt=self.dt),b=self.fly_speed_hori)

            self.mc.start_linear_motion(velocity_x_m=v[0], velocity_y_m=v[1], velocity_z_m=0, rate_yaw=0)

        def display(self):
            print("x:",round(stream.get_pos()[0],2),"\t y:",round(stream.get_pos()[1],2),"\t x_goal:", self.goal.get()[0], "\t y_goal:", self.goal.get()[1])


    class Go_to_zone(Sequence):
        # Sub-class of Sequences
        #
        # USAGE 1
        # START at beginning, on the home box
        # END when reaching the 1st point of the scouting route, 
        # or when detecting that this point is an obstacle
        #
        # USAGE 2 
        # START at waking up, on the target box
        # END when reaching a point nearby the home box, that cannot be the home box, or detecting 
        # that this point was an obstacle
        def __init__(self, route, offset, homing=False):
            # Inherit from Sequence
            stream.set_offset(offset)
             # Go straightforward
            if homing:
                route = [(route[0][0]-self.margin_from_home,offset[1])]
            else:
                route = [(route[0][0]-self.margin_from_home,offset[1])]
            super().__init__(route=route)
            print("Go to zone sequence")

        def f(self):
            # Routine of instructions to reach the objective
            self.mc.take_off(self.fly_height,self.fly_speed_up)
            time.sleep(1.5)

            while not obs.on_destination(self.goal,stream) and not destination_reached(self.goal,stream):
                if obs.on_top(stream):
                    self.terminate()
                self.moving_routine(obstacle_avoidance=True)
                time.sleep(self.dt)
            

            
    class Box_scout(Sequence):
        # Sub-class of Sequences
        #
        # START when Go_to_zone ends, i.e. when reaching the 1st point of the scouting route, 
        # or when detecting that this point is an obstacle
        # END when the box is detected 
        # 
        # USAGE 1
        # The route is along the whole zone
        # USAGE 2
        # The route is around the theoretical zone of home box to find it efficiently

        def __init__(self, route, homing=False):
            pos = stream.get_pos()
            if homing:
                route = self.set_home_route(route)
                route.insert(0,(pos[0]-self.margin_from_home,pos[1]))
            else:
                route.insert(0,(pos[0]+self.margin_from_home,pos[1]))
            print("route for go_to_zone")
            super().__init__(route=route) 

            self.a=-1                # Axis on which the drone flies when hitting the obstacle, 0=x, 1=y
            self.d=0                 # Direction along which the drone is flying the axis, -1 for neg, 1 for pos
            self.box_trigger_pos=-1  # Tuple of the detected position of the box
            print("Box scouting sequence")

        def f(self):
            # Routine of instructions to reach the objective
            #TODO
            while(1):
                # When reaching a set point

                if obs.on_destination(self.goal,stream) or destination_reached(self.goal,stream):
                    print("Destination reached")
                    self.it, new_goal = iterate(self.it, self.route)
                    if self.it is not LIST_END:
                        self.goal.set(new_goal)
                    else:
                        self.terminate()  # Should trigger home_scout instead 
                if obs.box_detected(stream, sequence, self.epsilon): #TODO
                    print("Box detected")
                    # Save axis and box position impact
                    self.set_final_axis_and_direction()
                    self.set_box_trigger_pos()
                    # Get back to the position 50cm before and hand out to box_landing
                    #vector_back = [0,0]
                    #vector_back[self.a]= -0.7 * self.d
                    #self.mc.move_distance(vector_back[0],vector_back[1],0,self.fly_speed_hori)
                    break

                # Move the drone one step further to the goal
                self.moving_routine(obstacle_avoidance=True)

                # Show positions
                #self.display()

                # Wait until next step
                time.sleep(self.dt)
        
        def get_box_trigger_pos(self):
            return self.box_trigger_pos

        def set_box_trigger_pos(self):
            self.box_trigger_pos = stream.get_pos()
            
        def get_final_axis(self):
            return self.a
        
        def set_home_route(self, route):
            outproute = [start_pos]
            for point in route:
                outproute.append(pos_sum(point,start_pos))
            return outproute
        
        def set_final_axis_and_direction(self):
            # Make a regression along the last 1 sec of fly
            traj_x = np.array(stream.get_x_list(1)) 
            traj_y = np.array(stream.get_y_list(1))
            res = linregress(traj_x, traj_y)

            # Fonction for direction deduction
            def set_direction(l):
                if l[0]>l[-1]:
                    return -1
                else:
                    return 1

            # Deduce the axis and direction
            if (abs(res.slope) < 1): #TODO
                # along axis x
                self.a = 0 
                self.d = set_direction(traj_x)
                print("Axis detected is x, res.slope=", res.slope)
            else:
                # along axis y
                self.a = 1 
                self.d = set_direction(traj_y)
                print("Axis detected is y, res.slope=", res.slope)


    class Box_landing(Sequence):
        # Sub-class of Sequences
        # START when Box_scout ends, i.e when the box is detected
        # END when the drone is landed on the box
        # 
        # WORKING PRINCIPLE: when the drone detects the box, it moves along an axis that we will call a. 
        # The drone keeps track of this axis and of the position of the box during box_scout sequence. 
        # This sequence will fly along the axis b, orthogonal to a, passing through the box position along a, 
        # until it "falls" from the box. Then, the position is deduced along the b axis from the falling.
        # Then, the drone goes landing to box position 
        # 
        # USAGE 1
        # For the target box
        # USAGE 2 
        # For the home box

        def __init__(self, track, box_impact_from_prev_seq, a, d):
            self.box_impact_from_prev_seq = box_impact_from_prev_seq
            self.flag_box_pos = False
            self.a = a      
            self.d = d  
            self.detection_time=time.time()
            self.exact_position_known = False
            self.box_sides=[box_impact_from_prev_seq]
            self.box_sides_per_axe = [[],[]]
            super().__init__(route=self.set_route(track)) 

            print("Box landing sequence")


        def set_route(self, track):
            # Computes the route from a and box_pos_from_prev_seq
            # The aim of this route is to make the drone fall on a road orthogonal to a, passing through
            # box_pos
            #route=[stream.get_past_pos(2)]
            route=[]
            #Define the direction of the route in relation to the direction of arrival
            print("len of track is: ", len(track))
            for i in range(len(track)):
                point = list(track[i])
                point[self.a] = self.d*point[self.a]
                route.append(tuple(point))

            #Route offset from the current position
            for i in range(len(route)):
                route[i] = tuple(sum(x) for x in zip(route[i], stream.get_pos()))

            return route
        
        def set_box_pos(self):
            #TODO
            # Computes the position of the real box position
            if len(self.box_sides_per_axe[0])==2 and len(self.box_sides_per_axe[1])==2:
                self.box_pos = find_rect_mid(self.box_sides_per_axe,self.a)
            if len(self.box_sides_per_axe[0])<2 and len(self.box_sides_per_axe[1])==2:
                self.box_pos = find_segment_mid(self.box_sides_per_axe[1])
            if len(self.box_sides_per_axe[0])==2 and len(self.box_sides_per_axe[1])<2:
                self.box_pos = find_segment_mid(self.box_sides_per_axe[0])
            if len(self.box_sides_per_axe[0])==1 and len(self.box_sides_per_axe[1])==1:
                self.box_pos = find_triangle_rectangle(self.box_sides_per_axe[0], self.box_sides_per_axe[1], self.a)
            else:
                self.box_pos = self.route[-1] # Better try this
            self.flag_box_pos=True
            return None

        def reco_finished(self):
            self.box_sides_per_axe[not self.a] = self.box_sides
            self.set_box_pos()
            self.goal.set(self.get_box_pos())
            self.exact_position_known = True
        
        def get_box_pos(self):
            if self.flag_box_pos:
                return self.box_pos
            else:
                print("Error: tried to read box_pos before assignment")
        
        def f(self):
            # Routine of instructions to reach the objective
            #ADD GO SLOW 
            side = 1
            while(1):
                if destination_reached(self.goal,stream):
                    if not self.it:
                        self.box_sides_per_axe[self.a] =  self.box_sides
                        self.box_sides = []
                    if self.exact_position_known and destination_reached(self.goal,stream, thresh=0.01):
                        self.landing_pos = stream.get_pos()
                        self.land()
                        break
                    self.it, new_goal = iterate(self.it, self.route)
                    if self.it is not LIST_END:
                        self.goal.set(new_goal)
                    else:
                        self.reco_finished()
                        #self.landing_pos = stream.get_pos()
                        #self.terminate()
                if (self.detection_time +self.time_between_two_detections< time.time()) and obs.box_side_detected(stream, flying_height=self.fly_height, epsilon=self.epsilon, time_between_two_detections = self.time_between_two_detections) and self.it != 0:
                    self.detection_time = time.time()
                    self.box_sides.append(stream.get_pos())
                    side += 1
                    print("Box side: ", self.box_sides)

                if side == 4:
                    self.reco_finished()
                self.moving_routine(obstacle_avoidance=False, slow=True)
                
                time.sleep(self.dt)
            return None


    class Sequences():
        def __init__(self):
            self.target_route = [(3.65,2.85),(3.65,0.15),(3.90,0.15),(3.90,2.85),\
                    (4.15,2.85),(4.15,0.15),(4.40,0.15),(4.40,2.85),\
                    (4.65,2.85),(4.65,0.15),(4.90,0.15),(4.90,2.85)]
            self.home_route = [(0.5,0.15),(0.65,0.15),(0.65,-0.15),(0.35,-0.15),\
                (0.35,0.30),(0.80,0.30),(0.80,-0.30),(0.20,-0.30),(0.20,0.45),\
                (0.45,0.45),(0.45,-0.45),(-0.45,-0.45),(-0.45,0.60),(0.60,0.60),\
                (0.60,-0.60),(-0.60,-0.60),(-0.60,0.75),(0.75,0.75),(0.75,-0.75),\
                (-0.75,-0.75),(-0.75,0.90),(0.90,0.90)]
            self.routes_landing = [[(0.5,0),(0.5,-0.45),(0.15,-0.45),(0.15,0.45),(0.15,0)],\
                                   [(0,0.5),(0.45,0.5),(0.45,0.15),(-0.45,0.15),(0.0,0.15)]]
            
        def run_go_to_target_zone(self):
            self.go_to_target_zone = Go_to_zone(route=[self.target_route[0]], offset=start_pos)
            self.go_to_target_zone.f()

        def run_target_scout(self):

            self.target_scout = Box_scout(route=self.target_route)
            self.target_scout.f()

        def run_target_landing(self):
            self.target_landing = Box_landing(track = self.routes_landing[self.target_scout.get_final_axis()],\
                                            box_impact_from_prev_seq = self.target_scout.get_box_trigger_pos(),\
                                            a = self.target_scout.get_final_axis(),\
                                            d = self.target_scout.d)
            self.target_landing.f()

        def run_go_to_home_zone(self):
            self.go_to_home_zone = Go_to_zone([(start_pos[0],start_pos[1])],\
                                                offset=self.target_landing.landing_pos) #TODO give a better route
            self.go_to_home_zone.f()

        def run_home_scout(self):
            self.home_scout = Box_scout(self.home_route[1:-1],\
                                        homing=True) #TODO give a better route
            self.home_scout.f()

        def run_home_landing(self): 
            self.home_landing = Box_landing(track = self.routes_landing[self.home_scout.get_final_axis()],\
                                            box_impact_from_prev_seq = self.home_scout.get_box_trigger_pos(),\
                                            a = self.home_scout.get_final_axis(),\
                                            d = self.home_scout.d)
            self.home_landing.f()



    # Main starts here
    if len(sys.argv) > 2:
        #URI = sys.argv[1]
        start_pos = (float(sys.argv[1]),float(sys.argv[2]))
    else:
        start_pos = (0.75,1.5) #TODO




    # Create the required objects
    stream = Stream(scf)
    sequence = Sequence([(-1,-1)])  #Only for using values

    obs = Obstacle()
    seq = Sequences()
    
    # Go for the sequences
    seq.run_go_to_target_zone()

    seq.run_target_scout() 

    seq.run_target_landing()

    seq.run_go_to_home_zone()

    seq.run_home_scout()

    seq.run_home_landing()

    print("Congrats! Fly is over")
    seq.target_landing.terminate()

  