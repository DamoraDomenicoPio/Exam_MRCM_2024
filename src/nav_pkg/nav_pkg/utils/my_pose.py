import math

class MyPose():
        def __init__(self, x=0.0, y=0.0, yaw=0.0):
            self._x = x
            self._y = y
            self._yaw = yaw

        def set_pose(self, x=None, y=None, yaw=None):
            if x != None:
                self._x = x
            if y != None:
                self._y = y
            if yaw != None:
                self._yaw = yaw

        def get_x(self):
            return self._x
        
        def get_y(self):
            return self._y
        
        def get_yaw(self):
            return self._yaw
        
        def __str__(self): 
            string = f'x = {self.get_x()}, y = {self.get_y()}, yaw = {self.get_yaw()}'
            return string
        
        def euler_from_quaternion(self, quaternion):
            x = quaternion.x
            y = quaternion.y
            z = quaternion.z
            w = quaternion.w
            # roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
            # pitch = math.asin(2 * (w * y - z * x))
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
            return yaw
        
        def new_set_pose_from_msg(self, msg):
            x, y = msg.data.split(',')
            x = float(x)
            y = float(y)
            self.set_pose(x, y, 0)

        def set_pose_from_msg(self, msg):
            current_pose = msg.pose.pose
            yaw = self.euler_from_quaternion(current_pose.orientation)
            self.set_pose(current_pose.position.x, current_pose.position.y, math.degrees(yaw))
             
