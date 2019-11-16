import math as m
from copy import deepcopy

import objects
import data_structures as ds

class GazeboObject():
    def __init__(self, base, map_params):
        self.map_params = map_params
        self.base = base


OBJECT_HEIGHT = float(0.5)
OBJECT_SPAWN_Z = OBJECT_HEIGHT / 2

WALL_WIDTH = float(0.01)

class GazeboBox(GazeboObject):
    def __init__(self, base, map_params):
        super().__init__(base, map_params)

        if type(base) is not objects.Box:
            raise Exception('Invalid class passed')

    def get_position_str(self):
        # Maybe better to realize and use Box object method like 'get_pos()' with deep copy
        box_center = deepcopy(self.base.pos)
        
        box_center.x += 0.5
        box_center.y += 0.5
        
        box_center.y = self.map_params.n_cells.y - box_center.y
    
        box_center.x *= self.map_params.cell_sz.x
        box_center.y *= self.map_params.cell_sz.y
        
        return '{} {} {} 0 0 0'.format(box_center.x, box_center.y, OBJECT_SPAWN_Z)
        
    def get_size_str(self):
        return '{} {} {}'.format(self.map_params.cell_sz.x, self.map_params.cell_sz.y, OBJECT_HEIGHT)


class GazeboWall(GazeboObject):
    def __init__(self, base, map_params):
        super().__init__(base, map_params)

        if type(base) is not objects.Wall:
            raise Exception('Invalid class passed')
        
    def get_position_str(self):
        wall_center = ds.Point2D((self.base.p1.x + self.base.p2.x)/2,
                                 (self.base.p1.y + self.base.p2.y)/2)

        sub = self.base.p2 - self.base.p1

        wall_angle = m.atan2(sub.y, sub.x)
            
        wall_center.y = self.map_params.n_cells.y - wall_center.y
                
        wall_center.x *= self.map_params.cell_sz.x
        wall_center.y *= self.map_params.cell_sz.y
        
        return '{} {} {} 0 0 {}'.format(wall_center.x, wall_center.y, OBJECT_SPAWN_Z, -wall_angle)

    def get_size_str(self):
        sub = self.base.p2 - self.base.p1

        wall_length = m.sqrt((sub.x*self.map_params.cell_sz.y)**2 + 
                             (sub.y*self.map_params.cell_sz.y)**2)

        return '{} {} {}'.format(wall_length, WALL_WIDTH, OBJECT_HEIGHT)


class GazeboSign(GazeboObject):
    ORIENTATIONS_2_YAW_ANGLE = {
        objects.CellQuarter.LEFT_BOT: 0,
        objects.CellQuarter.RIGHT_BOT: m.pi/2,
        objects.CellQuarter.LEFT_TOP: m.pi*3/2,
        objects.CellQuarter.RIGHT_TOP: m.pi,
    }

    def __init__(self, base, map_params):
        super().__init__(base, map_params)

        if type(base) is not objects.Sign:
            raise Exception('Invalid class passed')

    def get_position_str(self):
        pos = deepcopy(self.base.pos)

        # Apply small shift
        if self.base.orient == objects.CellQuarter.RIGHT_TOP:
            pos.x += 0.75
            pos.y += 0.25
        elif self.base.orient == objects.CellQuarter.RIGHT_BOT:
            pos.x += 0.75
            pos.y += 0.75
        elif self.base.orient == objects.CellQuarter.LEFT_BOT:
            pos.x += 0.25
            pos.y += 0.75
        elif self.base.orient == objects.CellQuarter.LEFT_TOP:
            pos.x += 0.25
            pos.y += 0.25
        
        # Swap axes
        pos.y = self.map_params.n_cells.y - pos.y
        
        # Turn to physical
        pos.x *= self.map_params.cell_sz.x
        pos.y *= self.map_params.cell_sz.y

        yaw_angle = self.ORIENTATIONS_2_YAW_ANGLE[self.base.orient]

        return "{0} {1} 0 0 0 {2}".format(pos.x, pos.y, yaw_angle)

    def get_type(self):
        return self.base.type
