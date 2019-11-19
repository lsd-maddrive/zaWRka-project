import math as m
from copy import deepcopy

import objects
import data_structures as ds

OBJECT_HEIGHT = float(0.5)
OBJECT_SPAWN_Z = OBJECT_HEIGHT / 2
WALL_WIDTH = float(0.01)

ORIENTATIONS_2_YAW_ANGLE = {
    objects.CellQuarter.LEFT_BOT: 0,
    objects.CellQuarter.RIGHT_BOT: m.pi/2,
    objects.CellQuarter.LEFT_TOP: m.pi*3/2,
    objects.CellQuarter.RIGHT_TOP: m.pi,
}

class GazeboObject():
    def __init__(self, base, map_params):
        self.map_params = map_params
        self.base = base
    def _swap_axes(self, pos):
        pos.y = self.map_params.n_cells.y - pos.y
    def _turn_to_physical(self, pos):
        pos.x *= self.map_params.cell_sz.x
        pos.y *= self.map_params.cell_sz.y


class GazeboBox(GazeboObject):
    def __init__(self, base, map_params):
        super().__init__(base, map_params)

        if type(base) is not objects.Box:
            raise Exception('Invalid class passed')

    def get_position_str(self):
        # Maybe better to realize and use Box object method like 'get_pos()' with deep copy
        center = deepcopy(self.base.pos)
        
        center.x += 0.5
        center.y += 0.5
        
        self._swap_axes(center)
        self._turn_to_physical(center)
        
        return '{} {} {} 0 0 0'.format(center.x, center.y, OBJECT_SPAWN_Z)
        
    def get_size_str(self):
        return '{} {} {}'.format(self.map_params.cell_sz.x, self.map_params.cell_sz.y, OBJECT_HEIGHT)


class GazeboWall(GazeboObject):
    def __init__(self, base, map_params):
        super().__init__(base, map_params)

        if type(base) is not objects.Wall:
            raise Exception('Invalid class passed')
        
    def get_position_str(self):
        center = ds.Point2D((self.base.p1.x + self.base.p2.x)/2,
                                 (self.base.p1.y + self.base.p2.y)/2)

        sub = self.base.p2 - self.base.p1

        wall_angle = m.atan2(sub.y, sub.x)
            
        self._swap_axes(center)
        self._turn_to_physical(center)
        
        return '{} {} {} 0 0 {}'.format(center.x, center.y, OBJECT_SPAWN_Z, -wall_angle)

    def get_size_str(self):
        sub = self.base.p2 - self.base.p1

        wall_length = m.sqrt((sub.x*self.map_params.cell_sz.y)**2 + 
                             (sub.y*self.map_params.cell_sz.y)**2)

        return '{} {} {}'.format(wall_length, WALL_WIDTH, OBJECT_HEIGHT)


class GazeboSquare(GazeboObject):
    def __init__(self, base, map_params):
        super().__init__(base, map_params)

        self.pillar_width = ds.Size2D(map_params.cell_sz.x / 10, 
                                      map_params.cell_sz.y / 10)

        if type(base) is not objects.Square:
            raise Exception('Invalid class passed')

    def get_position_strs(self):
        results = []
        center = deepcopy(self.base.pos)
        
        positions = [
            ds.Point2D(.0, .0),
            ds.Point2D(.5, .0),
            ds.Point2D(1., .0),
            ds.Point2D(.0, .5),
            ds.Point2D(1., .5),
            ds.Point2D(.0, 1.),
            ds.Point2D(.5, 1.),
            ds.Point2D(1., 1.)
        ]
        
        for pos in positions:
            pillar_cntr = center + pos
            self._swap_axes(pillar_cntr)
            self._turn_to_physical(pillar_cntr)
            results += ['{} {} {} 0 0 0'.format(pillar_cntr.x, pillar_cntr.y, OBJECT_SPAWN_Z)]

        return results

    def get_size_str(self):
        return '{} {} {}'.format(self.pillar_width.x, self.pillar_width.y, OBJECT_HEIGHT)
        

class GazeboSign(GazeboObject):
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
        
        self._swap_axes(pos)
        self._turn_to_physical(pos)

        yaw_angle = ORIENTATIONS_2_YAW_ANGLE[self.base.orient]

        return "{0} {1} 0 0 0 {2}".format(pos.x, pos.y, yaw_angle)

    def get_type(self):
        return self.base.type


class GazeboTrafficLight(GazeboObject):
    def __init__(self, base, map_params):
        super().__init__(base, map_params)
        if type(base) is not objects.TrafficLight:
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
        
        self._swap_axes(pos)
        self._turn_to_physical(pos)

        yaw_angle = ORIENTATIONS_2_YAW_ANGLE[self.base.orient]

        return "{0} {1} 0 0 0 {2}".format(pos.x, pos.y, yaw_angle)
