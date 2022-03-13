from helpers import *

class Stream:
    FRONT = 'range.front'
    BACK = 'range.back'
    LEFT = 'range.left'
    RIGHT = 'range.right'
    UP = 'range.up'
    DOWN = 'range.zrange'

    def __init__(self, crazyflie, rate_ms=20, zranger=True):
        if isinstance(crazyflie, SyncCrazyflie):
            self._cf = crazyflie.cf
        else:
            self._cf = crazyflie
        
        self.rate_ms = rate_ms
        self.offset =(0.0,0.0)

        self._log_config = self._create_log_config(rate_ms)

        self._cf.connected.add_callback(self._create_log_config)
        self.start()

        # Range
        self._up_distance = None
        self._front_distance = None
        self._back_distance = None
        self._left_distance = None
        self._right_distance = None
        self._down_distance = None

        #Position
        self.x = 0
        self.y = 0
        self.z = 0
        self.x_list = []
        self.y_list = []
        self.z_list = []

        self.reset_kalman()

    def reset_kalman(self):
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')


    def _create_log_config(self, rate_ms):
        log_config = LogConfig('Project_EPFL', rate_ms)

        # Range
        log_config.add_variable(self.FRONT)
        log_config.add_variable(self.BACK)
        log_config.add_variable(self.LEFT)
        log_config.add_variable(self.RIGHT)
        log_config.add_variable(self.UP)
        log_config.add_variable(self.DOWN)

        #Position
        log_config.add_variable('kalman.stateX', 'float')
        log_config.add_variable('kalman.stateY', 'float')
        log_config.add_variable('kalman.stateZ', 'float')
    
        log_config.data_received_cb.add_callback(self._data_received)

        return log_config

    def start(self):
        self._cf.log.add_config(self._log_config)
        self._log_config.start()

    def _convert_log_to_distance(self, data):
        if data >= 8000:
            return None
        else:
            return data / 1000.0

    def _data_received(self, timestamp, data, logconf):
        # Range
        self._up_distance = self._convert_log_to_distance(data[self.UP])
        self._front_distance = self._convert_log_to_distance(data[self.FRONT])
        self._back_distance = self._convert_log_to_distance(data[self.BACK])
        self._left_distance = self._convert_log_to_distance(data[self.LEFT])
        self._right_distance = self._convert_log_to_distance(data[self.RIGHT])
        self._down_distance = self._convert_log_to_distance(data[self.DOWN])

        #Position
        self.x = data['kalman.stateX']
        self.y = data['kalman.stateY']
        self.z = data['kalman.stateZ']
        self.z_list.append(data['kalman.stateZ'])
        self.x_list.append(data['kalman.stateX'])
        self.y_list.append(data['kalman.stateY'])
        
        #print('pos: ({}, {}, {})'.format(self.x, self.y, self.z))	  
        

    # Range

    def stop(self):
        self._log_config.delete()

    @property
    def up(self):
        return self._up_distance

    @property
    def left(self):
        return self._left_distance

    @property
    def right(self):
        return self._right_distance

    @property
    def front(self):
        return self._front_distance

    @property
    def back(self):
        return self._back_distance

    @property
    def down(self):
        return self._down_distance

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


    # Position
    def get_pos(self):
        return (self.x+self.offset[0],self.y+self.offset[1])

    def get_z(self):
        return(self.z)        

    def get_x_list(self, time):
        # Return the position projected on this axis for the last *time* spent
        return(self.x_list[-round(time*1000/self.rate_ms):])                                                       
    
    def get_y_list(self, time):
        # Return the position projected on this axis for the last *time* spent
        return(self.y_list[-round(time*1000/self.rate_ms):])                                                       
    
    def get_z_list(self, time):
        # Return the position projected on this axis for the last *time* spent
        return(self.z_list[-round(time*1000/self.rate_ms):])                                                       
    
    def set_offset(self, offset):
        self.offset = offset

    def get_past_pos(self, time):
        return (self.x_list[-round(time*1000/self.rate_ms)], self.y_list[-round(time*1000/self.rate_ms)])