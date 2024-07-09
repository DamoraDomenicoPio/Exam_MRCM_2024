from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions
from nav_pkg.utils.constants import Offsets

class JunctionPoint(): 

    def __init__(self, coordinates, name=None): 
        '''coordinates: è una lista di coordinate in cui il primo elemento è la x e il secondo è la y'''
        # Centro dell'incroio
        self._x = coordinates.value[0]
        self._y = coordinates.value[1]
        self._name = name
        
        # Punti a nord, sud, est e ovest del punto 
        self._nord = None 
        self._est = None
        self._sud = None
        self._ovest = None

    def is_in_junction(self, x, y): 
        result = True
        if not x < self.get_bbox_x(TurtleBot4Directions.SOUTH): 
            result = False
        if not x > self.get_bbox_x(TurtleBot4Directions.NORTH): 
            result = False
        if not y > self.get_bbox_y(TurtleBot4Directions.EAST): 
            result = False
        if not y < self.get_bbox_y(TurtleBot4Directions.WEST): 
            result = False
        return result


    def _get_bbox_point(self, direction):
        assert isinstance(direction, TurtleBot4Directions), "La direzione deve essere un'istanza della classe TurtleBot4Directions"
        if direction == TurtleBot4Directions.NORTH:
            y = self.get_y()
            x = self.get_x() - Offsets.x_offset.value
        elif direction == TurtleBot4Directions.SOUTH:
            y = self.get_y()
            x = self.get_x() + Offsets.x_offset.value
        elif direction == TurtleBot4Directions.EAST: 
            y = self.get_y() - Offsets.y_offset.value
            x = self.get_x() 
        elif direction == TurtleBot4Directions.WEST: 
            y = self.get_y() + Offsets.y_offset.value
            x = self.get_x() 
        return x, y
    
    def get_bbox_x(self, direction): 
        x, _ = self._get_bbox_point(direction)
        return x
    
    def get_bbox_y(self, direction): 
        _, y = self._get_bbox_point(direction)
        return y
    
        
    # Getter e setter
    def get_adjacent_junction(self, direction): 
        assert isinstance(direction, TurtleBot4Directions), 'La direzione deve essere un\'istanza della classe TurtleBot4Directions'
        junction = None
        if direction == TurtleBot4Directions.NORTH: 
            junction =  self.get_nord()
        elif direction == TurtleBot4Directions.SOUTH: 
            junction = self.get_sud()
        elif direction == TurtleBot4Directions.WEST: 
            junction = self.get_ovest()
        elif direction == TurtleBot4Directions.EAST: 
            junction = self.get_est()
        else: 
            raise Exception(f'La direzione {direction} direzione non esiste') 
        return junction
    
    def get_x(self): 
        return self._x
    
    def get_y(self):
        return self._y

    def get_name(self): 
        return self._name

    
    def get_nord(self): 
        return self._nord
    
    def get_sud(self): 
        return self._sud

    def get_est(self): 
        return self._est
    
    def get_ovest(self):
        return self._ovest
    
    def set_nord (self, nord): 
        self._nord = nord

    def set_sud(self, sud): 
        self._sud = sud
    
    def set_ovest(self, ovest): 
        self._ovest = ovest

    def set_est(self, est): 
        self._est = est


        
    