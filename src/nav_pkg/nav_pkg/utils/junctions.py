from nav_pkg.utils.constants import Signs, JunctionCoordinates
from nav_pkg.utils.waypoint import Waypoint
from nav_pkg.utils.junction_point import JunctionPoint
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions
from nav_pkg.utils.constants import OffsetDetection, OffsetsRecovery, OffsetsEndpoint
import sys

class Junctions(): 


    def __init__(self): 
        self.a = JunctionPoint(JunctionCoordinates.A, 'A')
        self.b = JunctionPoint(JunctionCoordinates.B, 'B')
        self.c = JunctionPoint(JunctionCoordinates.C, 'C')
        self.d = JunctionPoint(JunctionCoordinates.D, 'D')
        self.e = JunctionPoint(JunctionCoordinates.E, 'E')
        self.f = JunctionPoint(JunctionCoordinates.F, 'F')
        self.g = JunctionPoint(JunctionCoordinates.G, 'G')
        self.h = JunctionPoint(JunctionCoordinates.H, 'H')
        self.i = JunctionPoint(JunctionCoordinates.I, 'I')
        self.l = JunctionPoint(JunctionCoordinates.L, 'L')
        self.list = [self.a, self.b, self.c, self.d, self.e, self.f, self.g, self.h, self.i, self.l]
        self.initialize_graph()  # aggiunge ai singoli incroci informazioni sugli incroci limitrofi 
        
    def initialize_graph(self): 
        '''Per ogni incocio, specifica quale incrocio è a nord, quale è a sud, quale è a est e quale è a nord'''

        # Punto A
        self.a.set_nord(self.c)  # cioè a nord del punto a c'è il punto c
        self.a.set_est(self.b)   # ad est del punto a c'è il punto b
        self.a.set_sud(None)     # a sud del punto a non c'è niente (un muro ad esmepio)
        self.a.set_ovest(None)

        # Punto B 
        self.b.set_nord(self.d)
        self.b.set_est(None)
        self.b.set_sud(None)
        self.b.set_ovest(self.a)

        # Punto C
        self.c.set_nord(self.e)
        self.c.set_est(self.d)
        self.c.set_sud(self.a)
        self.c.set_ovest(None)

        # Punto D
        self.d.set_nord(self.f)
        self.d.set_est(None)
        self.d.set_sud(self.b)
        self.d.set_ovest(self.c)

        # Punto E
        self.e.set_nord(self.g)
        self.e.set_est(self.f)
        self.e.set_sud(self.c)
        self.e.set_ovest(None)

        # Punto F
        self.f.set_nord(self.h)
        self.f.set_est(None)
        self.f.set_sud(self.d)
        self.f.set_ovest(self.e)

        # Punto G
        self.g.set_nord(self.i)
        self.g.set_est(self.h)
        self.g.set_sud(self.e)
        self.g.set_ovest(None)

        # Punto H
        self.h.set_nord(self.l)
        self.h.set_est(None)
        self.h.set_sud(self.f)
        self.h.set_ovest(self.g)

        # Punto F
        self.f.set_nord(self.h)
        self.f.set_est(None)
        self.f.set_sud(self.d)
        self.f.set_ovest(self.e)

        # Punto I
        self.i.set_nord(None)
        self.i.set_est(self.l)
        self.i.set_sud(self.g)
        self.i.set_ovest(None)

        # Punto L
        self.l.set_nord(None)
        self.l.set_est(None)
        self.l.set_sud(self.h)
        self.l.set_ovest(self.i)

    # ------------------------------------------------------------------------------------

    def get_junction_by_point(self, x, y, offset_type=None): 
        result = None
        for junction in self.list: 
            if junction.is_in_bbox(x, y, offset_type):
                result = junction.get_name()
        return result
    
    def get_junction_by_name(self, name): 
        res = None
        for j in self.list:
            if name == j.get_name(): 
                res = j
        if res is None:
            raise Exception('Questo incrocio non esiste, scegli fra A, B, C, D, E, F, G, H, I e L (in maiuscolo)')
        return res

    def _get_next_direction(self, current_direction, sign): 
        assert current_direction in [0, 90, 180, 270], f'Questa direzione {current_direction} non esiste, scegli fra 0, 90, 180 e 270'
        next_direction = None
        if sign == Signs.LEFT.value: 
            next_direction = (current_direction + 90)%360
        elif sign == Signs.RIGHT.value:
            next_direction = (current_direction - 90)%360
        elif sign == Signs.GOBACK.value: 
            next_direction = (current_direction - 180)%360
        elif sign == Signs.STRAIGHTON.value or sign == Signs.STOP.value:
            next_direction = current_direction%360
        else: 
            raise Exception(f'Questo segnale ({sign} non esiste)')
        result = self.convert_directions(next_direction)
        return result
            

    def convert_directions(self, direction):
        assert direction in [0, 90, 180, 270], f'Questa direzione {direction} non esiste, scegli fra 0, 90, 180 e 270'
        new_dir = None
        if direction == 0:
            new_dir = TurtleBot4Directions.NORTH
        if direction == 90: 
            new_dir = TurtleBot4Directions.WEST
        if direction == 180: 
            new_dir = TurtleBot4Directions.SOUTH
        if direction == 270: 
            new_dir = TurtleBot4Directions.EAST
        return new_dir



    def get_next_junciton_waypoint(self, point_name, current_direction, sign):
        '''Restituisce il prossimo waypoint in cui andare in base al segnale ricevuto 
        e alla posa (coordinate + direzione) corrente'''
        current_point = self.get_junction_by_name(point_name)
        next_direction = self._get_next_direction(current_direction, sign)
        assert isinstance(next_direction, TurtleBot4Directions), 'next_directions deve essere istanza della classe TurtleBot4Directions'

        next_point = None
        next_point = current_point.get_adjacent_junction(next_direction)

        if next_point is None: 
            print('\nNESSUNA DIREZIONE\n')

        if not next_point:  # Se next_point è none
            raise Exception('Stai andando contro un muro')

        waypoint = Waypoint(next_point.get_x(), next_point.get_y(),  self.convert_directions(next_direction.value))
        # waypoint = Waypoint(next_point.get_bbox_x(next_direction, OffsetsEndpoint), next_point.get_bbox_y(next_direction, OffsetsEndpoint),  self.convert_directions(next_direction.value))

        return waypoint
     


if __name__ == '__main__':
    x = float(sys.argv[1])
    y = float(sys.argv[2])

    j = Junctions()
    res = j.get_junction_by_point(x, y)
    print(res)




    