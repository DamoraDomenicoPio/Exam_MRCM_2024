

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


        
    