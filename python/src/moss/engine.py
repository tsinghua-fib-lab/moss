import importlib.machinery
import importlib.util
import json
import os
import pickle
import threading
from enum import Enum
from glob import glob
from typing import Dict, List, Tuple, Union

import numpy as np
from numpy.typing import NDArray

from .agent import Agents
from .convert import convert_from_cityflow, save_pb
from .map import Map


def _import():
    path = glob(os.path.dirname(os.path.abspath(__file__)) + "/_simulet*.so")[0]
    loader = importlib.machinery.ExtensionFileLoader("_simulet", path)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    _simulet = importlib.util.module_from_spec(spec)
    loader.exec_module(_simulet)
    return _simulet


_simulet = _import()
_thread_local = threading.local()


class TlPolicy(Enum):
    MANUAL = 0
    FIXED_TIME = 1
    MAX_PRESSURE = 2
    NONE = 3


class LaneChange(Enum):
    """
    `NONE`: Vehicles will not do lane change according to road condition.

    `SUMO`: Vehicles will attempt to switch to a faster lane, as in SUMO.

    `MOBIL`: Vehicles will take the action that maximize the change in acceleration of self and affected vehicles, as in MOBIL.
    """

    NONE = 0
    SUMO = 1
    MOBIL = 2


class Verbosity(Enum):
    NO_OUTPUT = 0
    INIT_ONLY = 1
    ALL = 2


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
        LANE_CHANGE_HARD_STOP = 9

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
        (self.start_time,) = misc
        (
            self.v,
            self.acc,
            reason,
            self.reason_detail,
            self.front_id,
            self.nl_id,
            self.nl_speed,
            self.nl_state,
        ) = drive
        self.acc_reason = self.Reason(reason)
        self.snapshot = self.State(snapshot)
        self.runtime = self.State(runtime)
        self.route, self.route_index, self.route_lc_offset = route


class Engine:
    """
    Simulet Engine

    NOTE: Cannot create multiple Engines on different device. For that purpose, use `simulet.parallel.ParallelEngine` instead.
    """

    __version__ = _simulet.__version__

    def __init__(
        self,
        map_file: str,
        agent_file: str,
        id_file: str = "",
        start_step: int = 0,
        total_step: int = 0,  # 只有在需要写输出文件时需要填写
        step_interval: float = 1,
        seed: int = 43,
        verbose_level=Verbosity.NO_OUTPUT,
        agent_limit: int = -1,
        disable_aoi_out_control: bool = False,
        disable_junction: bool = False,
        junction_blocking_count: int = -1,
        junction_yellow_time: float = 0,
        phase_pressure_coeff: float = 1.5,
        lane_change: LaneChange = LaneChange.SUMO,
        mobil_lc_forbidden_distance: float = 15,
        lane_veh_add_buffer_size: int = 300,
        lane_veh_remove_buffer_size: int = 300,
        device: int = 0,
    ):
        assert junction_yellow_time >= 0
        if not hasattr(_thread_local, "device"):
            _thread_local.device = device
        elif _thread_local.device != device:
            raise RuntimeError(
                "Cannot create multiple Engines on different device! Use simulet.parallel.ParallelEngine instead."
            )
        self._e = _simulet.Engine(
            map_file,
            agent_file,
            start_step,
            total_step,
            step_interval,
            seed,
            verbose_level.value,
            agent_limit,
            disable_aoi_out_control,
            disable_junction,
            junction_blocking_count,
            junction_yellow_time,
            phase_pressure_coeff,
            lane_change.value,
            mobil_lc_forbidden_distance,
            lane_veh_add_buffer_size,
            lane_veh_remove_buffer_size,
            device,
        )
        self.map_file = map_file
        self.agent_file = agent_file
        self.lane_cnt = self._e.get_lane_count()
        self.device = device
        if id_file:
            self.has_id = True
            ids = pickle.load(open(id_file, "rb"))
            self.id2road = ids["road"]
            self.id2lane = ids["lane"]
            self.id2junc = ids["junction"]
            self.id2agent = ids["agent"]
            self.junc2id = {j: i for i, j in enumerate(self.id2junc)}
        else:
            self.has_id = False

    @staticmethod
    def convert_map_agent(
        in_map: str,
        in_agent: str,
        out_map: str,
        out_agent: str,
        out_id: str,
        max_agent_start_time=1e999,
    ):
        _map, _agents, ids = convert_from_cityflow(
            data_map=json.load(open(in_map, "rb")),
            data_agents=json.load(open(in_agent, "rb")),
            max_agent_start_time=max_agent_start_time,
        )
        save_pb(_map, out_map)
        save_pb(_agents, out_agent)
        pickle.dump(ids, open(out_id, "wb"))

    @staticmethod
    def from_cityflow(
        cityflow_config: dict, output_path=None, max_agent_start_time=1e999
    ):
        map_file = cityflow_config["roadnetFile"]
        agent_file = cityflow_config["flowFile"]
        if output_path is None:
            output_path = os.path.dirname(os.path.abspath(map_file))
        os.makedirs(output_path, exist_ok=True)
        Engine.convert_map_agent(
            in_map=map_file,
            in_agent=agent_file,
            out_map=output_path + "/_simulet_map.bin",
            out_agent=output_path + "/_simulet_agent.bin",
            out_id=output_path + "/_simulet_id.bin",
            max_agent_start_time=max_agent_start_time,
        )
        return Engine(
            map_file=output_path + "/_simulet_map.bin",
            agent_file=output_path + "/_simulet_agent.bin",
            id_file=output_path + "/_simulet_id.bin",
            start_step=0,
            total_step=1 << 30,
            step_interval=cityflow_config["interval"],
            seed=cityflow_config["seed"],
            verbose=False,
        )

    @property
    def vehicle_count(self) -> int:
        """
        The number of vehicles in the agent file
        """
        return self._e.get_vehicle_count()

    @property
    def lane_count(self) -> int:
        """
        The number of lanes
        """
        return self._e.get_lane_count()

    @property
    def road_count(self) -> int:
        """
        The number of roads
        """
        return self._e.get_road_count()

    @property
    def junction_count(self) -> int:
        """
        The number of junctions
        """
        return self._e.get_junction_count()

    def get_map(self):
        """
        Get the Map object
        """
        return Map(self.map_file)

    def get_agents(self):
        """
        Get the Agents object
        """
        return Agents(self.agent_file)

    def get_current_time(self) -> float:
        """
        Get the current time
        """
        return self._e.get_current_time()

    def get_running_vehicle_count(self) -> int:
        """
        Get the total number of running vehicles
        """
        return self._e.get_running_vehicle_count()

    def get_lane_statuses(self) -> NDArray[np.int8]:
        """
        Get the traffic light status of each lane, `0`-green / `1`-yellow / `2`-red / `3`-restriction
        """
        return self._e.get_lane_statuses()

    def get_lane_geoms(self) -> List[NDArray[np.float32]]:
        """
        Get the geometry of each lane, `[[x1,y1], ..., [xn,yn]]`
        """
        return [
            np.array(i, np.float32).reshape(-1, 2) for i in self._e.get_lane_geoms()
        ]

    def get_lane_lengths(self) -> NDArray[np.int8]:
        """
        Get the length of each lane
        """
        return self._e.get_lane_lengths()

    def get_lane_vehicles(self) -> Union[Dict[str, List[str]], List[List[int]]]:
        """
        Get the list of vehicles of each lane
        """
        if self.has_id:
            s = len(self.id2lane)
            ls = [[] for _ in range(s)]
            ret = {i: j for i, j in zip(self.id2lane, ls)}
            veh_lane = self._e.get_vehicle_lanes()
            mask = np.flatnonzero((veh_lane >= 0) & (veh_lane < s))
            for i, lane in zip(mask, veh_lane[mask]):
                ls[lane].append(self.id2agent[i])
            return ret
        ls = [[] for _ in range(self.lane_cnt)]
        veh_lane = self._e.get_vehicle_lanes()
        mask = np.flatnonzero((veh_lane >= 0) & (veh_lane < self.lane_cnt))
        for i, lane in zip(mask, veh_lane[mask]):
            ls[lane].append(i)
        return ls

    def get_lane_vehicle_counts(self) -> NDArray[np.int32]:
        """
        Get the number of vehicles of each lane
        """
        return self._e.get_lane_vehicle_counts()

    def get_lane_waiting_vehicle_counts(
        self, speed_threshold: float = 0.1
    ) -> Union[Dict[str, int], List[int]]:
        """
        Get the number of vehicles of each lane with speed lower than `speed_threshold`
        """
        if self.has_id:
            return {
                i: j
                for i, j in zip(
                    self.id2lane,
                    self._e.get_lane_waiting_vehicle_counts(speed_threshold),
                )
            }
        return self._e.get_lane_waiting_vehicle_counts(speed_threshold)

    def get_lane_waiting_at_end_vehicle_counts(
        self, speed_threshold: float = 0.1, distance_to_end: float = 100
    ) -> Union[Dict[str, int], List[int]]:
        """
        Get the number of vehicles of each lane with speed lower than `speed_threshold` and distance to end lower than `distance_to_end`
        """
        if self.has_id:
            return {
                i: j
                for i, j in zip(
                    self.id2lane,
                    self._e.get_lane_waiting_at_end_vehicle_counts(
                        speed_threshold, distance_to_end
                    ),
                )
            }
        return self._e.get_lane_waiting_at_end_vehicle_counts(
            speed_threshold, distance_to_end
        )

    def get_lane_ids(self) -> NDArray[np.int32]:
        """
        Get the ids of the lanes
        """
        return self._e.get_lane_ids()

    def get_junction_ids(self) -> NDArray[np.int32]:
        """
        Get the ids of the junctions
        """
        return self._e.get_junction_ids()

    def get_junction_lanes(self) -> List[List[int]]:
        """
        Get the `index` of the lanes inside each junction
        """
        return self._e.get_junction_lanes()

    def get_junction_inout_lanes(self) -> Tuple[List[List[int]], List[List[int]]]:
        """
        Get the `index` of the `in` and `out` lanes of each junction
        """
        return self._e.get_junction_inout_lanes()

    def get_junction_phase_lanes(self) -> List[List[Tuple[List[int], List[int]]]]:
        """
        Get the `index` of the `in` and `out` lanes of each phase of each junction
        """
        return self._e.get_junction_phase_lanes()

    def get_junction_phase_ids(self) -> NDArray[np.int32]:
        """
        Get the phase id of each junction, `-1` if it has no traffic lights
        """
        return self._e.get_junction_phase_ids()

    def get_junction_phase_counts(self) -> NDArray[np.int32]:
        """
        Get the number of available phases of each junction
        """
        return self._e.get_junction_phase_counts()

    def get_junction_dynamic_roads(self) -> List[List[int]]:
        """
        Get the ids of the dynamic roads connected to each junction
        """
        return self._e.get_junction_dynamic_roads()

    def get_road_lane_plans(self, road_id: int) -> List[List[slice]]:
        """
        Get the dynamic lane plan of the road `road_id`,
        represented as list of lane groups:
        ```
        [
            [slice(lane_start, lane_end), ...]
        ]
        ```
        """
        return [
            [slice(a, b) for a, b in i] for i in self._e.get_road_lane_plans(road_id)
        ]

    def get_vehicle_lanes(self) -> Union[Dict[str, int], NDArray[np.int32]]:
        """
        Get the lane index of each vehicle, `-1` for not running on the road
        """
        if self.has_id:
            x = self._e.get_vehicle_lanes()
            mask = np.flatnonzero(x >= 0)
            return {self.id2agent[i]: j for i, j in zip(mask, x[mask])}
        return self._e.get_vehicle_lanes()

    def get_vehicle_speeds(self) -> Union[Dict[str, float], NDArray[np.float32]]:
        """
        Get the speed of each vehicle, `-1` for not running on the road
        """
        if self.has_id:
            x = self._e.get_vehicle_speeds()
            mask = np.flatnonzero(x >= 0)
            return {self.id2agent[i]: j for i, j in zip(mask, x[mask])}
        return self._e.get_vehicle_speeds()

    def get_vehicle_statuses(self) -> NDArray[np.int8]:
        """
        Get the status of each vehicle, `1`-driving / `2`-finished / `0`-otherwise
        """
        return self._e.get_vehicle_statuses()

    def get_vehicle_raw_statuses(self) -> NDArray[np.int8]:
        """
        Get the raw status of each vehicle, only for debugging.
        """
        return self._e.get_vehicle_raw_statuses()

    def get_vehicle_traveling_or_departure_times(self) -> NDArray[np.float32]:
        """
        - For `finished` vehicles, it is the traveling time.
        - For `driving` vehicles, it is the negative departure time.
        """
        return self._e.get_vehicle_traveling_or_departure_times()

    def get_finished_vehicle_count(self) -> int:
        """
        Get the number of the finished vehicles
        """
        return self._e.get_finished_vehicle_count()

    def get_finished_vehicle_average_traveling_time(self) -> float:
        """
        Get the average traveling time of the finished vehicles
        """
        return self._e.get_finished_vehicle_average_traveling_time()

    def get_running_vehicle_average_traveling_time(self) -> float:
        """
        Get the average traveling time of the running vehicles
        """
        return self._e.get_running_vehicle_average_traveling_time()

    def get_departed_vehicle_average_traveling_time(self) -> float:
        """
        Get the average traveling time of the departed vehicles (running+finished)
        """
        return self._e.get_departed_vehicle_average_traveling_time()

    def get_vehicle_distances(self) -> Union[Dict[str, float], NDArray[np.float32]]:
        """
        Get the traveled distance of each vehicle on the current lane
        """
        if self.has_id:
            x = self._e.get_vehicle_distances()
            mask = np.flatnonzero(x >= 0)
            return {self.id2agent[i]: j for i, j in zip(mask, x[mask])}
        return self._e.get_vehicle_distances()

    def get_vehicle_total_distances(
        self,
    ) -> Union[Dict[str, float], NDArray[np.float32]]:
        """
        Get the total traveled distance of each vehicle
        """
        if self.has_id:
            return {
                i: j
                for i, j in zip(self.id2agent, self._e.get_vehicle_total_distances())
            }
        return self._e.get_vehicle_total_distances()

    def get_vehicle_positions(self) -> NDArray[np.float64]:
        """
        Get the (x,y,dir) of each running vehicle
        """
        return self._e.get_vehicle_positions().reshape(-1, 3)

    def get_vehicle_positions_all(self) -> NDArray[np.float64]:
        """
        Get the (x,y,dir) of all vehicles
        """
        return self._e.get_vehicle_positions_all().reshape(-1, 3)

    def get_vehicle_id_positions(self) -> NDArray[np.float64]:
        """
        Get the (id,x,y,dir) of each vehicle
        """
        return self._e.get_vehicle_id_positions().reshape(-1, 4)

    def get_road_lane_plan_id(self, road_id: int) -> int:
        """
        Get the lane plan of road `road_id`
        """
        return self._e.get_road_lane_plan_id(road_id)

    def get_road_vehicle_counts(self) -> NDArray[np.int32]:
        """
        Get the number of vehicles of each road
        """
        return self._e.get_road_vehicle_counts()

    def get_road_waiting_vehicle_counts(
        self, speed_threshold: float = 0.1
    ) -> Union[Dict[str, int], List[int]]:
        """
        Get the number of vehicles with speed lower than `speed_threshold` of each road
        """
        if self.has_id:
            return {
                i: j
                for i, j in zip(
                    self.id2road,
                    self._e.get_road_waiting_vehicle_counts(speed_threshold),
                )
            }
        return self._e.get_road_waiting_vehicle_counts(speed_threshold)

    def set_vehicle_enable(self, vehicle_id: int, enable: bool):
        """
        Enable or disable vehicle `vehicle_id`
        """
        self._e.set_vehicle_enable(vehicle_id, enable)

    def set_vehicle_enable_batch(
        self, vehicle_ids: List[int], enable: Union[bool, List[bool]]
    ):
        """
        Enable or disable vehicle `vehicle_id`
        """
        self._e.set_vehicle_enable_batch(
            vehicle_ids,
            [enable] * len(vehicle_ids) if isinstance(enable, bool) else enable,
        )

    def set_tl_policy(self, junction_id: int, policy: TlPolicy):
        """
        Set the traffic light policy of junction `junction_id` to `policy`
        """
        self._e.set_tl_policy(junction_id, policy.value)

    def set_tl_policy_batch(self, junction_ids: List[int], policy: TlPolicy):
        """
        Set the traffic light policy of all junctions in `junction_ids` to `policy`
        """
        self._e.set_tl_policy_batch(junction_ids, policy.value)

    def set_tl_duration(self, junction_id: int, duration: int):
        """
        Set the traffic light switch duration of junction `junction_id` to `duration`

        NOTE: This is only effective for `TlPolicy.FIXED_TIME` and `TlPolicy.MAX_PRESSURE`.

        NOTE: Set duration to `0` to use the predefined duration in the `map_file`
        """
        self._e.set_tl_duration(junction_id, duration)

    def set_tl_duration_batch(self, junction_ids: List[int], duration: int):
        """
        Set the traffic light switch duration of all junctions in `junction_ids` to `duration`

        NOTE: This is only effective for `TlPolicy.FIXED_TIME` and `TlPolicy.MAX_PRESSURE`

        NOTE: Set duration to `0` to use the predefined duration in the `map_file`
        """
        self._e.set_tl_duration_batch(junction_ids, duration)

    def set_tl_phase(self, junction_id: Union[str, int], phase_id: int):
        """
        Set the phase of `junction_id` to `phase_id`
        """
        if self.has_id:
            junction_id = self.junc2id[junction_id]
        self._e.set_tl_phase(junction_id, phase_id)

    def set_tl_phase_batch(self, junction_ids: List[int], phase_ids: List[int]):
        """
        Set the phase of `junction_id` to `phase_id` in batch
        """
        assert len(junction_ids) == len(phase_ids)
        if self.has_id:
            raise NotImplementedError
        self._e.set_tl_phase_batch(junction_ids, phase_ids)

    def set_road_lane_plan(self, road_id: int, plan_id: int):
        """
        Set the lane plan of road `road_id`
        """
        self._e.set_road_lane_plan(road_id, plan_id)

    def set_road_lane_plan_batch(self, road_ids: List[int], plan_ids: List[int]):
        """
        Set the lane plan of road `road_id`
        """
        assert len(road_ids) == len(plan_ids)
        self._e.set_road_lane_plan_batch(road_ids, plan_ids)

    def set_lane_restriction(self, lane_id: int, flag: bool):
        """
        Set the restriction state of lane `lane_id`
        """
        self._e.set_lane_restriction(lane_id, flag)

    def set_lane_restriction_batch(self, lane_ids: List[int], flags: List[bool]):
        """
        Set the restriction state of lane `lane_id`
        """
        assert len(lane_ids) == len(flags)
        self._e.set_lane_restriction_batch(lane_ids, flags)

    def set_lane_max_speed(self, lane_id: int, max_speed: float):
        """
        Set the max_speed of lane `lane_id`
        """
        self._e.set_lane_max_speed(lane_id, max_speed)

    def set_lane_max_speed_batch(
        self, lane_ids: List[int], max_speeds: Union[float, List[float]]
    ):
        """
        Set the max_speed of lane `lane_id`
        """
        if hasattr(max_speeds, "__len__"):
            assert len(lane_ids) == len(max_speeds)
        else:
            max_speeds = [max_speeds] * len(lane_ids)
        self._e.set_lane_max_speed_batch(lane_ids, max_speeds)

    def debug_vehicle_info(self) -> NDArray[np.float64]:
        """
        Get debug info
        """
        return self._e.debug_vehicle_info().reshape(-1, 5)

    def debug_vehicle_full_info(self, vehicle_id: int) -> Tuple:
        """
        Get full debug vehicle info
        """
        return self._e.debug_vehicle_full(vehicle_id)

    def debug_lane_info(self):
        """
        Get debug info
        """
        return self._e.debug_lane_info()

    def next_step(self, n=1):
        """
        Move forward `n` steps
        """
        self._e.next_step(n)

    def make_checkpoint(self) -> int:
        """
        Make a checkpoint of the current state of the simulator and return the checkpoint id
        """
        return self._e.make_checkpoint()

    def restore_checkpoint(self, checkpoint_id):
        """
        Restore the state of the simulator to a previous checkpoint
        """
        self._e.restore_checkpoint(checkpoint_id)
