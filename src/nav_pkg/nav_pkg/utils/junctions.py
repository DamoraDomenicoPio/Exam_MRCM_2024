from nav_pkg.utils.constants import Directions, Signs, JunctionCoordinates
from nav_pkg.utils.waypoint import Waypoint
from nav_pkg.utils.junction_point import JunctionPoint
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions
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

    def get_junction_by_point(self, x, y): 
        result = None
        for junction in self.list: 
            if junction.is_in_junction(x, y):
                result = junction.get_name()
        return result

    def get_junction_by_name(self, name): 
        j = None 
        if name == 'A':
            j = self.a
        elif name == 'B':
            j = self.b
        elif name == 'C':
            j = self.c
        elif name == 'D':
            j = self.d
        elif name == 'E':
            j = self.e
        elif name == 'F':
            j = self.f
        elif name == 'G':
            j = self.g
        elif name == 'H':
            j = self.h
        elif name == 'I':
            j = self.i
        elif name == 'L':
            j = self.l
        else: 
            raise Exception('Questo incrocio non esiste, scegli fra A, B, C, D, E, F, G, H, I e L (in maiuscolo)')
        return j

        
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


    def _get_next_direction(self, current_direction, sign):
        print('\nCurrent direction: ')
        next_direction = ''
        # Se è orientato verso nord
        if current_direction == Directions.NORD.value: 
            print('nord')
            if sign == Signs.LEFT.value: 
                next_direction = Directions.OVEST.value
            elif sign == Signs.RIGHT.value: 
                next_direction = Directions.EST.value
            elif sign == Signs.STRAIGHTON.value: 
                next_direction = Directions.NORD.value
            elif sign == Signs.GOBACK.value: 
                next_direction = Directions.SUD.value


        # Se è orientato verso sud 
        elif current_direction == Directions.SUD.value: 
            print('sud')
            if sign == Signs.LEFT.value: 
                next_direction = Directions.EST.value
            elif sign == Signs.RIGHT.value: 
                next_direction = Directions.OVEST.value
            elif sign == Signs.STRAIGHTON.value: 
                next_direction = Directions.SUD.value
            elif sign == Signs.GOBACK.value: 
                next_direction = Directions.NORD.value


        # Se è orientato verso est 
        elif current_direction == Directions.EST.value: 
            print('est')
            if sign == Signs.LEFT.value: 
                next_direction = Directions.NORD.value
            elif sign == Signs.RIGHT.value: 
                next_direction = Directions.SUD.value
            elif sign == Signs.STRAIGHTON.value: 
                next_direction = Directions.EST.value
            elif sign == Signs.GOBACK.value: 
                next_direction = Directions.OVEST.value


        # Se è orientato verso ovest
        elif current_direction == Directions.OVEST.value: 
            print('ovest')
            if sign == Signs.LEFT.value: 
                next_direction = Directions.SUD.value
            elif sign == Signs.RIGHT.value: 
                next_direction = Directions.NORD.value
            elif sign == Signs.STRAIGHTON.value: 
                next_direction = Directions.OVEST.value
            elif sign == Signs.GOBACK.value: 
                next_direction = Directions.EST.value

        return next_direction
    

    def _get_point_from_name(self, name):
        if not (name=='A' or name=='B'or name=='C'or name=='D' or name=='E' or name=='F' or name=='G' or name=='H' or name=='I' or name=='L'):
            raise Exception('Il nome di un punto può essere solo A, B, C, D, E, F, G, H, I oppure L')
        
        points = [self.a, self.b, self.c, self.d, self.e, self.f, self.g, self.h, self.i, self.l]
        for point in points: 
            if name == point.get_name(): 
                return point
            

    def convert_directions(self, direction):
        # TurtleBot4Directions
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
        current_point = self._get_point_from_name(point_name)
        next_direction = self._get_next_direction(current_direction, sign)


        next_point = '' 
        if next_direction == Directions.NORD.value: 
            print('nord')
            next_direction = Directions.NORD
            next_point = current_point.get_nord()
        elif next_direction == Directions.SUD.value: 
            print('sud')
            next_direction = Directions.SUD
            next_point = current_point.get_sud()
        elif next_direction == Directions.EST.value: 
            print('est')
            next_direction = Directions.EST
            next_point = current_point.get_est()
        elif next_direction == Directions.OVEST.value: 
            print('fatto ovest')
            next_direction = Directions.OVEST
            next_point = current_point.get_ovest()

        if next_point == '': 
            print('\nNESSUNA DIREZIONE\n')

        if not next_point:  # Se next_point è none
            raise Exception('Stai andando contro un muro')

        waypoint = Waypoint(next_point.get_x(), next_point.get_y(),  self.convert_directions(next_direction.value))

        return waypoint
        
     


if __name__ == '__main__':
    x = float(sys.argv[1])
    y = float(sys.argv[2])

    j = Junctions()
    res = j.get_junction_by_point(x, y)
    print(res)




    