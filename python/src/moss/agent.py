from typing import List

from .convert import PbAgent, save_pb


class Agent:
    def __init__(self, pb):
        self.pb = pb
        self.start = pb.home
        self.end = pb.schedules[0].trips[0].end
        self.route: List[int] = pb.schedules[0].trips[0].routes[0].driving.road_ids if len(pb.schedules[0].trips[0].routes) else None
        self.departure_time: float = pb.schedules[0].departure_time


class Agents:
    def __init__(self, agent_file):
        self.pb = a = PbAgent.Agents()
        a.ParseFromString(open(agent_file, 'rb').read())
        self.agents = [Agent(i) for i in a.agents]

    def save(self, agent_file):
        save_pb(self.pb, agent_file)
