import math
from enum import Enum

class CoordinateType(Enum):
    Cartesian = 1
    Spherical = 2

class Coordinates:
    azimuth = 0
    altitude = 0
    depth = 0
    x = 0
    y = 0
    z = 0  
    
    def __init__(self,CoordType,azimuth = 0, altitude = 0, depth = 0, x = 0, y = 0, z = 0):
        if CoordType == CoordinateType.Cartesian:
            self.x = x
            self.y = y
            self.z = z
            
            self.CalculateSpheric()
            
        if CoordType == CoordinateType.Spherical:
            self.azimuth = azimuth
            self.altitude = altitude
            self.depth = depth
            
            self.CalculateCartesian()
            
    
                    
    def CalculateSpheric(self):
        #Mein Hirn is grad irgendwie kaputt mach ich später xD
        return
    
    def CalculateCartesian(self):
        self.x = self.depth * math.sin(self.azimuth) * math.cos(self.altitude)
        self.y = self.depth * math.sin(self.azimuth) * math.sin(self.altitude)
        self.z = self.depth * math.cos(self.azimuth)
        
        return

