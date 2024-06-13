from enum import Enum
from typing import List

import pyproj

from .convert import PbMap, PbTl, save_pb
from .geom import PI, Polyline


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
    def __init__(self, pb):
        self.pb = pb
        self.id: int = pb.id
        self.index: int = None
        self.type = LaneType(pb.type)
        self.turn = LaneTurn(pb.turn)
        self.max_speed: float = pb.max_speed
        self.length: float = pb.length
        # width
        self.geom = Polyline.from_pb(pb.center_line.nodes)
        self.predecessors: List[Lane] = [i.id for i in pb.predecessors]
        self.successors: List[Lane] = [i.id for i in pb.successors]
        self.parent: int = pb.parent_id
        self.parent_road: Road = None
        self.parent_junction: Junction = None

    def __repr__(self):
        return f"<Lane {self.id} at L[{self.index}] in P[{self.parent}]>"


class Road:
    def __init__(self, pb):
        self.pb = pb
        self.id: int = pb.id
        self.index: int = None
        self.lanes: List[Lane] = pb.lane_ids

    def __repr__(self):
        return f"<Road {self.id} at R[{self.index}]>"


class RoadPair:
    def __init__(self, _in: Road, _out: Road):
        self.in_road = _in
        self.out_road = _out


class LightState(Enum):
    UNSPECIFIED = PbTl.LIGHT_STATE_UNSPECIFIED
    RED = PbTl.LIGHT_STATE_RED
    GREEN = PbTl.LIGHT_STATE_GREEN
    YELLOW = PbTl.LIGHT_STATE_YELLOW


class LightPhase:
    def __init__(self, pb):
        self.pb = pb
        self.duration: float = pb.duration
        self.states: List[LightState] = [LightState(i) for i in pb.states]

    def __repr__(self):
        return f'<LightPhase T={self.duration:.1f} S='+''.join(i.name[0] for i in self.states)+'>'


class TrafficLight:
    def __init__(self, pb):
        self.pb = pb
        self.junction_id: int = pb.junction_id
        self.phases: List[LightPhase] = [LightPhase(i) for i in pb.phases]

    def __repr__(self):
        return f'<TrafficLight for J[{self.junction_id}] with {len(self.phases)} phases>'


class Junction:
    def __init__(self, pb):
        self.pb = pb
        self.id: int = pb.id
        self.index: int = None
        self.lanes: List[Lane] = pb.lane_ids
        self.road_pairs: List[RoadPair] = None
        self.tl: TrafficLight = (
            TrafficLight(pb.fixed_program) if pb.fixed_program.phases else None
        )

    def __repr__(self):
        return f"<Junction {self.id} at J[{self.index}]>"


class LanePosition:
    def __init__(self, pb):
        self.pb = pb
        self.id: int = pb.lane_id
        self.s: float = pb.s

    def __repr__(self):
        return f'<LanePosition at L[{self.id}] {self.s:.3f}]>'


class Aoi:
    def __init__(self, pb):
        self.pb = pb
        self.id: int = pb.id
        self.driving_positions = [LanePosition(i) for i in pb.driving_positions]

    def __repr__(self):
        return f"<Aoi {self.id}>"


class Map:
    def __init__(self, map_file):
        self.pb = m = PbMap.Map()
        m.ParseFromString(open(map_file, "rb").read())
        self.proj = pyproj.Proj(self.pb.header.projection)
        self.lanes = [Lane(i) for i in m.lanes]
        self.roads = [Road(i) for i in m.roads]
        self.junctions = [Junction(i) for i in m.junctions]
        self.aois = [Aoi(i) for i in m.aois]
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

    def parse_standard_junction(self):
        """
        Parse all the junctions in the map as standard junctions. 4 in roads and 4 out roads.
        """
        for jc in self.junctions:
            in_roads = set()
            out_roads = set()
            for l in jc.lanes:
                for ll in l.predecessors:
                    if ll.parent_road:
                        in_roads.add(ll.parent_road.id)
                for ll in l.successors:
                    if ll.parent_road:
                        out_roads.add(ll.parent_road.id)
            assert len(in_roads) == len(out_roads) == 4
            pairs = {
                r.lanes[0].geom.angle_out: [r]
                for r in (self.road_map[i] for i in in_roads)
            }
            for i in out_roads:
                r = self.road_map[i]
                ang = r.lanes[0].geom.angle_in
                a = min(pairs, key=lambda x: abs((ang - x) % (2 * PI) - PI))
                pairs[a].append(r)
            assert all(len(i) == 2 for i in pairs.values())
            ang = min(pairs, key=lambda x: abs((x + PI) % (2 * PI) - PI))
            pairs = sorted(pairs.items(), key=lambda x: (x[0] - ang) % (2 * PI))
            jc.road_pairs = [RoadPair(*i[1]) for i in pairs]
            for i in range(4):
                assert (
                    abs(
                        (
                            jc.road_pairs[(i + 1) % 4].in_road.lanes[0].geom.angle_out
                            - jc.road_pairs[i].in_road.lanes[0].geom.angle_out
                        )
                        % (2 * PI)
                        - PI / 2
                    )
                    < PI / 8
                )

    def save(self, map_file):
        for x, y in [
            [self.pb.lanes, self.lanes],
            [self.pb.roads, self.roads],
            [self.pb.junctions, self.junctions],
            [self.pb.aois, self.aois],
        ]:
            del x[:]
            for i in y:
                x.append(i.pb)
        save_pb(self.pb, map_file)

    def __repr__(self):
        return f'<Map with {len(self.lanes)} lanes {len(self.roads)} roads {len(self.junctions)} junctions {len(self.aois)} aois>'
