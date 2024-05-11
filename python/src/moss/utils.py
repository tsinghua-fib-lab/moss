from enum import Enum

import numpy as np

from .engine import Engine


class VehicleInfo:
    class Status(Enum):
        INIT = 0
        TO_INSERT = 1
        SLEEP = 2
        CROWD = 3
        WAITING_FOR_LEAVING = 4
        WALKING = 5
        DRIVING = 6
        FINISHED = 7

    class Reason(Enum):
        NONE = 0
        LANE_CHANGE_N = 1
        LANE_CHANGE_V = 2
        TO_LIMIT = 3
        TO_END = 4
        CAR_FOLLOW = 5
        CAR_FOLLOW_SHADOW = 6
        LANE_AHEAD = 7
        LANE_AHEAD_SHADOW = 8

    class State:
        def __init__(self, info):
            (
                status,
                self.lane,
                self.s,
                self.shadow_s,
                self.speed,
                self.is_lane_changing,
                self.shadow_lane,
                self.lc_length,
                self.lc_length,
                self.lc_total_length,
                self.lc_complete_length,
            ) = info
            self.status = VehicleInfo.Status(status)

    def __init__(self, info):
        misc, drive, snapshot, runtime, route = info
        self.start_time, = misc
        self.v, self.acc, reason, self.ahead_id, self.front_id = drive
        self.acc_reason = self.Reason(reason)
        self.snapshot = self.State(snapshot)
        self.runtime = self.State(runtime)
        self.route, self.route_index, self.route_lc_offset = route


class Recorder:
    def __init__(self, eng: Engine, enable=True):
        self.enable = enable
        if not enable:
            return
        self.eng = eng
        self.data = {
            'lanes': eng.get_lane_geoms(),
            'steps': []
        }

    def record(self):
        if not self.enable:
            return
        self.data['steps'].append({
            'veh_id_positions': self.eng.get_vehicle_id_positions().astype(np.float32),
            'lane_states': self.eng.get_lane_statuses(),
        })

    def save(self, filepath):
        if not self.enable:
            return
        np.savez_compressed(filepath, data=self.data)
