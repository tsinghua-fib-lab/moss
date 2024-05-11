from enum import Enum
from typing import List

from tqdm.auto import tqdm

from .convert import PbMap
from .geom import Polyline


class LaneType(Enum):
    DRIVING = PbMap.LANE_TYPE_DRIVING
    WALKING = PbMap.LANE_TYPE_WALKING


class LaneTurn(Enum):
    # UNSPECIFIED = PbMap.LANE_TURN_UNSPECIFIED
    STRAIGHT = PbMap.LANE_TURN_STRAIGHT
    LEFT = PbMap.LANE_TURN_LEFT
    RIGHT = PbMap.LANE_TURN_RIGHT
    AROUND = PbMap.LANE_TURN_AROUND


class LaneConnection(Enum):
    # UNSPECIFIED = PbMap.LANE_CONNECTION_TYPE_UNSPECIFIED
    HEAD = PbMap.LANE_CONNECTION_TYPE_HEAD
    TAIL = PbMap.LANE_CONNECTION_TYPE_TAIL


class Lane:
    def __init__(self, data):
        self.data = data
        self.id: int = data['id']
        self.index: int = None
        self.type = LaneType(data['type'])
        self.turn = LaneTurn(data['turn'])
        self.max_speed: float = data['max_speed']
        self.length: float = data['length']
        # width
        self.geom = Polyline.from_nodes(data['center_line']['nodes'])
        self.predecessors: List[Lane] = [i['id'] for i in data['predecessors']]
        self.successors: List[Lane] = [i['id'] for i in data['successors']]
        self.parent: int = data['parent_id']
        self.parent_road: Road = None
        self.parent_junction: Junction = None


class Road:
    def __init__(self, data):
        self.data = data
        self.id: int = data['id']
        self.index: int = None
        self.lanes: List[Lane] = data['lane_ids']


class RoadPair:
    def __init__(self, _in: Road, _out: Road):
        self.in_road = _in
        self.out_road = _out


class Junction:
    def __init__(self, data):
        self.data = data
        self.id: int = data['id']
        self.index: int = None
        self.lanes: List[Lane] = data['lane_ids']
        self.road_pairs: List[RoadPair] = None


class LanePosition:
    def __init__(self, data):
        self.data = data
        self.id: int = data['lane_id']
        self.s: float = data['s']


class Aoi:
    def __init__(self, data):
        self.data = data
        self.id: int = data['id']
        self.driving_positions = [LanePosition(i) for i in data['driving_positions']]


class Map:
    def __init__(self, db, use_tqdm=False):
        self.lanes: List[Lane] = []
        self.roads: List[Road] = []
        self.junctions: List[Junction] = []
        self.aois: List[Aoi] = []
        for i in tqdm(db.find(), total=db.estimated_document_count(), disable=not use_tqdm):
            if i['class'] == 'header':
                self.header = i['data']
            elif i['class'] == 'lane':
                self.lanes.append(Lane(i['data']))
            elif i['class'] == 'road':
                self.roads.append(Road(i['data']))
            elif i['class'] == 'junction':
                self.junctions.append(Junction(i['data']))
            elif i['class'] == 'aoi':
                self.aois.append(Aoi(i['data']))
        self.lane_map = {i.id: i for i in self.lanes}
        self.road_map = {i.id: i for i in self.roads}
        self.junction_map = {i.id: i for i in self.junctions}
        self.aoi_map = {i.id: i for i in self.aois}
        for i, l in enumerate(self.lanes):
            l.index = i
            l.predecessors = [self.lane_map[i] for i in l.predecessors]
            l.successors = [self.lane_map[i] for i in l.successors]
            l.parent_road = self.road_map.get(l.parent)
            l.parent_junction = self.junction_map.get(l.parent)
        for i, r in enumerate(self.roads):
            r.index = i
            r.lanes = [self.lane_map[i] for i in r.lanes]
        for i, j in enumerate(self.junctions):
            j.index = i
            j.lanes = [self.lane_map[i] for i in j.lanes]
