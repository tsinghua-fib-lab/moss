import math
from copy import deepcopy

import shapely
from google.protobuf import json_format
from google.protobuf.message import Message
from pycityproto.city.map.v2 import light_pb2 as PbTl
from pycityproto.city.map.v2 import map_pb2 as PbMap
from pycityproto.city.person.v1 import person_pb2 as PbAgent
from pycityproto.city.routing.v2 import routing_pb2 as PbRouting
from pycityproto.city.trip.v2 import trip_pb2 as PbTrip
from pymongo.collection import Collection
from shapely.ops import substring
from tqdm.auto import tqdm


def get_dir(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)


TRIM = 15
SIMPLIFICATION = 0.1
SPEED_JUNCTION = 20 / 3.6
AOI_POS = 5


def pb2dict(pb: Message):
    return json_format.MessageToDict(
        pb,
        including_default_value_fields=True,
        preserving_proto_field_name=True,
        use_integers_for_enums=True,
    )


def dict2pb(d: dict, pb: Message, ignore_unknown_fields=False):
    return json_format.ParseDict(d, pb, ignore_unknown_fields=ignore_unknown_fields)


def save_pb(pb, path):
    with open(path, 'wb') as f:
        f.write(pb.SerializeToString())


def convert_from_mongo(map_col: Collection, agent_col: Collection, out_map: str, out_agent: str, use_tqdm=False, ignore_unknown_fields=False):
    """
    Convert MongoDB collections into binary files
    """
    header = next(iter(map_col.find()))['data']
    lanes = []
    roads = []
    juncs = []
    aois = []
    for i in tqdm(map_col.find(), total=map_col.estimated_document_count(), ncols=90, disable=not use_tqdm):
        i['data'].pop('external', None)
        if i['class'] == 'lane':
            lanes.append(i['data'])
        if i['class'] == 'road':
            roads.append(i['data'])
        if i['class'] == 'junction':
            juncs.append(i['data'])
        if i['class'] == 'aoi':
            aois.append(i['data'])
    _map = PbMap.Map()
    dict2pb({
        'header': header,
        'lanes': lanes,
        'roads': roads,
        'junctions': juncs,
        'aois': aois,
    }, _map, ignore_unknown_fields=ignore_unknown_fields)
    save_pb(_map, out_map)
    agents = []
    err = 0
    for i in tqdm(agent_col.find(), total=agent_col.estimated_document_count(), ncols=90, disable=not use_tqdm):
        try:
            a = i['data']
            assert len(a['schedules']) == 1
            assert len(a['schedules'][0]['trips']) == 1
            assert a['schedules'][0]['trips'][0]['mode'] == PbTrip.TRIP_MODE_DRIVE_ONLY
            agents.append(a)
        except:
            err += 1
    if err:
        print(f'Imported {len(agents)} agents with {err} errors')
    _agents = PbAgent.Persons()
    dict2pb(
        {'persons': agents},
        _agents,
        ignore_unknown_fields=ignore_unknown_fields
    )
    save_pb(_agents, out_agent)


def convert_from_cityflow(data_map, data_agents, max_agent_start_time):
    raise NotImplementedError  # FIXME
    is_virtual = {i['id']: i['virtual'] for i in data_map['intersections']}

    lane_id = 0 - 1
    lanes = {}
    road_id = 2_0000_0000 - 1
    roads = {}
    road_by_name = {}
    junc_id = 3_0000_0000 - 1
    juncs = []
    aoi_id = 5_0000_0000 - 1
    aois = []

    ids = {
        'road': [],
        'lane': [],
        'junction': [],
        'agent': []
    }

    in_dir = {}
    out_dir = {}
    turn_map = {
        'go_straight': PbMap.LANE_TURN_STRAIGHT,
        'turn_left': PbMap.LANE_TURN_LEFT,
        'turn_right': PbMap.LANE_TURN_RIGHT,
    }
    start_aoi = {}
    end_aoi = {}
    connected = set()
    ignored = set()

    for road in data_map['roads']:
        if road['startIntersection'] == road['endIntersection']:
            ignored.add(road['id'])
            print(f'Warning: ignore road {road["id"]} as it starts and ends at the same intersection {road["startIntersection"]}')
            continue
        l = shapely.LineString([[i['x'], i['y']] for i in road['points']])
        if l.length < 1:
            print(f'Warning: the length of road {road["id"]} is too short: {l.length:.3f}m')
        t = min(TRIM, l.length/3)
        l = substring(
            l,
            0 if is_virtual[road['startIntersection']] else t,
            l.length if is_virtual[road['endIntersection']] else -t
        )
        ls = [l.simplify(SIMPLIFICATION)]
        ws = 0
        for lane in road['lanes']:
            w = lane['width']
            ls.append(l.offset_curve(-(ws + w / 2)).simplify(SIMPLIFICATION))
            ls.append(l.offset_curve(-(ws + w)).simplify(SIMPLIFICATION))
            ws += w
        _ids = [lane_id + 1 + i for i in range(len(road['lanes']))]
        road_id += 1
        ids['road'].append(road['id'])
        road_by_name[road['id']] = roads[road_id] = {
            "id": road_id,
            "lane_ids": _ids,
        }
        l = l.coords
        in_dir[road_id] = get_dir(*l[0], *l[1])
        out_dir[road_id] = get_dir(*l[-2], *l[-1])
        for i, (l, agent, b, c) in enumerate(zip(road['lanes'], ls[::2], ls[1::2], ls[2::2])):
            lane_id += 1
            lanes[lane_id] = {
                "id": lane_id,
                "type": PbMap.LANE_TYPE_DRIVING,
                "turn": PbMap.LANE_TURN_STRAIGHT,
                "max_speed": l['maxSpeed'],
                "length": b.length,
                "width": l['width'],
                "center_line": {"nodes": [{'x': x, 'y': y} for x, y in b.coords]},
                "left_border_line": {"nodes": [{'x': x, 'y': y} for x, y in agent.coords]},
                "right_border_line": {"nodes": [{'x': x, 'y': y} for x, y in c.coords]},
                "predecessors": [],
                "successors": [],
                "left_lane_ids": _ids[:i],
                "right_lane_ids": _ids[i + 1:],
                "parent_id": road_id,
                "overlaps": [],
                "aoi_ids": []
            }
            ids['lane'].append(road['id'] + f'_{i}')
        assert lane_id == _ids[-1]
        aoi_id += 1
        lanes[lane_id]['aoi_ids'].append(aoi_id)
        pos = substring(ls[-1], 0, AOI_POS).coords[-1]
        aois.append({
            "id": aoi_id,
            "type": PbMap.AOI_TYPE_OTHER,
            "positions": [],
            "driving_positions": [{
                "lane_id": lane_id,
                "s": AOI_POS,
            }],
            "driving_gates": [{
                'x': pos[0],
                'y': pos[1],
            }],
            "walking_positions": [],
            "walking_gates": [],
            'area': 1
        })
        start_aoi[road['id']] = aois[-1]
        aoi_id += 1
        lanes[lane_id]['aoi_ids'].append(aoi_id)
        pos = substring(ls[-1], 0, -AOI_POS).coords[-1]
        aois.append({
            "id": aoi_id,
            "type": PbMap.AOI_TYPE_OTHER,
            "positions": [],
            "driving_positions": [{
                "lane_id": lane_id,
                "s": lanes[lane_id]['length'] - AOI_POS,
            }],
            "driving_gates": [{
                'x': pos[0],
                'y': pos[1],
            }],
            "walking_positions": [],
            "walking_gates": [],
            'area': 1
        })
        end_aoi[road['id']] = aois[-1]

    for _j in data_map['intersections']:
        if _j['virtual']:
            continue
        junc_id += 1
        junc = {
            'id': junc_id,
            'lane_ids': [],
            'driving_lane_groups': []
        }
        juncs.append(junc)
        ids['junction'].append(_j['id'])
        for rl in _j['roadLinks']:
            if rl['startRoad'] in ignored or rl['endRoad'] in ignored:
                junc['driving_lane_groups'].append({'lane_ids': []})
                continue
            connected.add((rl['startRoad'], rl['endRoad']))
            turn = turn_map[rl['type']]
            group = {
                'in_road_id': road_by_name[rl['startRoad']]['id'],
                'in_angle': out_dir[road_by_name[rl['startRoad']]['id']],
                'out_road_id': road_by_name[rl['endRoad']]['id'],
                'out_angle': in_dir[road_by_name[rl['endRoad']]['id']],
                'lane_ids': [],
                'turn': turn,
            }
            junc['driving_lane_groups'].append(group)
            for ll in rl['laneLinks']:
                _pre = lanes[road_by_name[rl['startRoad']]['lane_ids'][ll['startLaneIndex']]]
                _suc = lanes[road_by_name[rl['endRoad']]['lane_ids'][ll['endLaneIndex']]]
                w = _pre['width']
                if len(ll['points']) == 0:
                    l = shapely.LineString([[i['x'], i['y']] for i in [_pre['center_line']['nodes'][-1], _suc['center_line']['nodes'][0]]])
                else:
                    l = shapely.LineString([[i['x'], i['y']] for i in ll['points']])
                a = l.offset_curve(w / 2).simplify(SIMPLIFICATION)
                b = l.simplify(SIMPLIFICATION)
                c = l.offset_curve(-w / 2).simplify(SIMPLIFICATION)
                lane_id += 1
                lanes[lane_id] = {
                    "id": lane_id,
                    "type": PbMap.LANE_TYPE_DRIVING,
                    "turn": turn,
                    "max_speed": SPEED_JUNCTION,
                    "length": b.length,
                    "width": w,
                    "center_line": {"nodes": [{'x': x, 'y': y} for x, y in b.coords]},
                    "left_border_line": {"nodes": [{'x': x, 'y': y} for x, y in a.coords]},
                    "right_border_line": {"nodes": [{'x': x, 'y': y} for x, y in c.coords]},
                    "predecessors": [{
                        'id': _pre['id'],
                        'type': PbMap.LANE_CONNECTION_TYPE_TAIL,
                    }],
                    "successors": [{
                        'id': _suc['id'],
                        'type': PbMap.LANE_CONNECTION_TYPE_HEAD,
                    }],
                    "left_lane_ids": [],
                    "right_lane_ids": [],
                    "parent_id": junc_id,
                    "overlaps": [],  # TODO
                    "aoi_ids": []
                }
                _pre['successors'].append({
                    'id': lane_id,
                    'type': PbMap.LANE_CONNECTION_TYPE_HEAD,
                })
                _suc['predecessors'].append({
                    'id': lane_id,
                    'type': PbMap.LANE_CONNECTION_TYPE_TAIL,
                })
                group['lane_ids'].append(lane_id)
                junc['lane_ids'].append(lane_id)

        junc['fixed_program'] = tl = {
            'junction_id': junc_id,
            'phases': []
        }
        if junc['lane_ids']:
            for _p in _j['trafficLight']['lightphases']:
                g = set(sum((
                    junc['driving_lane_groups'][i]['lane_ids'] for i in _p['availableRoadLinks']
                ), []))
                tl['phases'].append({
                    'duration': _p['time'],
                    'states': [
                        PbTl.LIGHT_STATE_GREEN if i in g else PbTl.LIGHT_STATE_RED for i in junc['lane_ids']
                    ]
                })
        junc['driving_lane_groups'] = [i for i in junc['driving_lane_groups'] if i['lane_ids']]

    agents = []
    agent_id = -1
    for flow_id, _a in enumerate(data_agents):
        if _a['startTime'] > max_agent_start_time:
            print(f'Discard flow_{flow_id} for its start time is later than max')
        valid = True
        for i, j in zip(_a['route'], _a['route'][1:]):
            if (i, j) not in connected:
                print(f'Discard flow_{agent_id} for there is no route from {i} to {j}')
                valid = False
                break
        if not valid:
            continue
        agent = {
            "id": agent_id,
            "attribute": {
                "length": _a['vehicle']['length'],
                "width": _a['vehicle']['width'],
                "max_speed": _a['vehicle']['maxSpeed'],
                "max_acceleration": _a['vehicle']['maxPosAcc'],
                "max_braking_acceleration": -_a['vehicle']['maxNegAcc'],
                "usual_acceleration": _a['vehicle']['usualPosAcc'],
                "usual_braking_acceleration": -_a['vehicle']['usualNegAcc']
            },
            "home": {
                "aoi_position": {
                    "aoi_id": start_aoi[_a['route'][0]]['id']
                }
            },
            "schedules": [{
                "trips": [{
                    "mode": PbTrip.TRIP_MODE_DRIVE_ONLY,
                    "end": {
                        "aoi_position": {
                            "aoi_id": end_aoi[_a['route'][-1]]['id']
                        }
                    },
                    "routes": [{
                        'type': PbRouting.JOURNEY_TYPE_DRIVING,
                        'driving': {
                            'road_ids': [road_by_name[i]['id'] for i in _a['route']]
                        }
                    }]
                }],
                "departure_time": _a['startTime'],
                "loop_count": 1
            }],
            "vehicle_attribute": {
                "lane_change_length": 10,
                "min_gap": _a['vehicle']['minGap']
            },
        }
        _i = 0
        st = _a['startTime']
        et = _a['endTime']
        dt = _a['interval']
        while st + _i * dt <= et:
            agent_id += 1
            agent['id'] = agent_id
            agent['schedules'][0]['departure_time'] = st + _i * dt
            agents.append(deepcopy(agent))
            ids['agent'].append(f'flow_{flow_id}_{_i}')
            _i += 1
    _map = PbMap.Map()
    dict2pb({
        'header': {},  # TODO
        'lanes': list(lanes.values()),
        'roads': list(roads.values()),
        'junctions': list(juncs),
        'aois': aois,
    }, _map)
    _agents = PbAgent.Agents()
    dict2pb(
        {'agents': agents},
        _agents
    )
    return _map, _agents, ids
