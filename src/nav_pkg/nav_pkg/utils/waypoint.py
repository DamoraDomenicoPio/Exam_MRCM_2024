from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions

class Waypoint(): 
    
    def __init__(self, x, y, direction):
        self._x = x 
        self._y = y
        if not isinstance(direction, TurtleBot4Directions):
            raise TypeError('La direzione deve essere un\'istanza della classe TurtleBot4Directions')
        else: 
            self._direction = direction

    def set_x(self, x): 
        self._x = x

    def set_y(self, y): 
        self._y = y

    def set_direction(self, direction): 
        if not isinstance(direction, TurtleBot4Directions):
            raise TypeError('La direzione deve essere un\'istanza della classe TurtleBot4Directions')
        else: 
            self._direction = direction 

    def get_x(self): 
        return self._x

    def get_y(self): 
        return self._y
    
    def get_direction(self):
        return self._direction