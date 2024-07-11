from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions
from nav_pkg.utils.constants import OffsetDetection, OffsetsRecovery, OffsetsEndpoint
import math

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

    def is_in_bbox(self, x, y, offset_type=None): 
        if offset_type is None: 
            offset_type = OffsetDetection
        assert offset_type in [OffsetDetection, OffsetsRecovery, OffsetsEndpoint], 'L\'offset deve essere un\'istanza di OffsetDetection, OffsetRecovery o OffsetEndpoint'
        result = True
        if not x < self.get_bbox_x(TurtleBot4Directions.SOUTH, offset_type): 
            result = False
        if not x > self.get_bbox_x(TurtleBot4Directions.NORTH, offset_type): 
            result = False
        if not y > self.get_bbox_y(TurtleBot4Directions.EAST, offset_type): 
            result = False
        if not y < self.get_bbox_y(TurtleBot4Directions.WEST, offset_type): 
            result = False
        return result


    def _get_standard_bbox_point(self, direction, offset_type=None): 
        if offset_type is None: 
            offset_type = OffsetDetection
        assert offset_type in [OffsetDetection, OffsetsRecovery, OffsetsEndpoint], 'L\'offset deve essere un\'istanza di OffsetDetection, OffsetRecovery o OffsetEndpoint'
        assert isinstance(direction, TurtleBot4Directions), "La direzione deve essere un'istanza della classe TurtleBot4Directions"
        if direction == TurtleBot4Directions.NORTH:
            y = self.get_y()
            x = self.get_x() - offset_type.x_offset.value
        elif direction == TurtleBot4Directions.SOUTH:
            y = self.get_y()
            x = self.get_x() + offset_type.x_offset.value
        elif direction == TurtleBot4Directions.EAST: 
            y = self.get_y() - offset_type.y_offset.value
            x = self.get_x() 
        elif direction == TurtleBot4Directions.WEST: 
            y = self.get_y() + offset_type.y_offset.value
            x = self.get_x() 
        
        return x, y
    
    def _get_complete_bbox_point(self, direction:TurtleBot4Directions): 
        x = round(abs(self.get_x() - self.get_adjacent_junction(direction).get_x())/2, 2)
        y = round(abs(self.get_y() - self.get_adjacent_junction(direction).get_y())/2, 2)
        return x, y


    def _get_bbox_point(self, direction, offset_type=None):
        x = None 
        y = None
        if offset_type:
            x, y = self._get_standard_bbox_point(direction, offset_type)
        else: 
            x, y = self._get_complete_bbox_point(direction)
        return x, y
        
    
    def get_bbox_x(self, direction, offset_type=None): 
        x, _ = self._get_bbox_point(direction, offset_type)
        return x
    
    def get_bbox_y(self, direction, offset_type=None):
        _, y = self._get_bbox_point(direction, offset_type)
        return y
    
        
    # Getter e setter
    def get_adjacent_junction(self, direction:TurtleBot4Directions): 
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


        
    