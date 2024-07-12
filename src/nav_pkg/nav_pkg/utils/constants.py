from enum import Enum
from abc import ABC, abstractmethod



class Signs(Enum): 
    STOP = 'STOP'
    RIGHT = 'RIGHT'
    LEFT = 'LEFT'
    STRAIGHTON = 'STRAIGHTON'
    GOBACK = 'GOBACK'

class JunctionCoordinates(Enum): 
    A = [-22.9, 0.11]   # old y = 0.044
    B = [-23.74, -8.72]
    C = [-3.11, 0.11]
    D = [-3.3, -9.51]
    E = [16.64, 0.03]
    F = [16.8, -9.66]
    G = [36.72, -0.88]
    H = [36.70, -10]
    I = [58, -2]
    L = [58, -11]

# class Offsets(Enum):
#     x_offset = 10
#     y_offset = 5

class OffsetDetection(Enum):
    x_offset = 10
    y_offset = 5

class OffsetsRecovery(Enum):
    x_offset = 3
    y_offset = 2

class OffsetsEndpoint(Enum):
    x_offset = 0
    y_offset = 0