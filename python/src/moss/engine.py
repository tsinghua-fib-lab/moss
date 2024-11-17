import importlib
import importlib.machinery
import importlib.util
import os
import threading
from enum import Enum
from glob import glob
from typing import Dict, List, Tuple, Union

import numpy as np
from numpy.typing import NDArray
from pycityproto.city.map.v2.map_pb2 import Map
from pycityproto.city.person.v2.person_pb2 import Persons

from .convert import pb2dict

__all__ = ["Engine", "TlPolicy", "Verbosity", "PersonStatus"]


def _import():
    path = glob(os.path.dirname(os.path.abspath(__file__)) + "/_moss*.so")[0]
    loader = importlib.machinery.ExtensionFileLoader("_moss", path)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    _moss = importlib.util.module_from_spec(spec)
    loader.exec_module(_moss)
    return _moss


_moss = _import()
_thread_local = threading.local()


class TlPolicy(Enum):
    MANUAL = 0
    FIXED_TIME = 1
    MAX_PRESSURE = 2
    NONE = 3


class Verbosity(Enum):
    NO_OUTPUT = 0
    INIT_ONLY = 1
    ALL = 2


class PersonStatus(Enum):
    SLEEP = 0
    WALKING = 1
    DRIVING = 2
    FINISHED = 3


class Engine:
    """
    Moss Engine

    NOTE: Cannot create multiple Engines on different device. For that purpose, use `moss.parallel.ParallelEngine` instead.
    """

    __version__ = _moss.__version__

    def __init__(
        self,
        name: str,
        map_file: str,
        person_file: str,
        start_step: int = 0,
        step_interval: float = 1,
        seed: int = 43,
        verbose_level=Verbosity.NO_OUTPUT,
        person_limit: int = -1,
        junction_yellow_time: float = 0,
        phase_pressure_coeff: float = 1.5,
        speed_stat_interval: int = 0,
        output_dir: str = "",
        out_xmin: float = -1e999,
        out_ymin: float = -1e999,
        out_xmax: float = 1e999,
        out_ymax: float = 1e999,
        device: int = 0,
        device_mem: float = 0,
    ):
        """
        Args:
        - name: The name of the task (for directory naming in output)
        - map_file: The path to the map file (Protobuf format)
        - person_file: The path to the person file (Protobuf format)
        - start_step: The starting step of the simulation
        - step_interval: The interval of each step (unit: seconds)
        - seed: The random seed
        - verbose_level: The verbosity level
        - person_limit: The maximum number of persons to simulate (-1 means no limit)
        - junction_yellow_time: The yellow time of the junction traffic light
        - phase_pressure_coeff: The coefficient of the phase pressure
        - speed_stat_interval: The interval of speed statistics. Set to `0` to disable speed statistics.
        - output_dir: The AVRO output directory
        - out_xmin: The minimum x coordinate of the output bounding box
        - out_ymin: The minimum y coordinate of the output bounding box
        - out_xmax: The maximum x coordinate of the output bounding box
        - out_ymax: The maximum y coordinate of the output bounding box
        - device: The CUDA device index
        - device_mem: The memory limit of the CUDA device (unit: GiB). Set to `0` to use the self-adaptive mode (80% of the free GPU memory).
        """

        self._fetched_persons = None
        self._fetched_lanes = None

        assert junction_yellow_time >= 0
        if not hasattr(_thread_local, "device"):
            _thread_local.device = device
        elif _thread_local.device != device:
            raise RuntimeError(
                "Cannot create multiple Engines on different device! Use moss.parallel.ParallelEngine instead."
            )
        self.speed_stat_interval = speed_stat_interval
        """
        The interval of speed statistics. Set to `0` to disable speed statistics.
        """
        if speed_stat_interval < 0:
            raise ValueError("Cannot set speed_stat_interval to be less than 0")
        self._e = _moss.Engine(
            name,
            map_file,
            person_file,
            start_step,
            step_interval,
            seed,
            verbose_level.value,
            person_limit,
            junction_yellow_time,
            phase_pressure_coeff,
            speed_stat_interval,
            output_dir,
            out_xmin,
            out_ymin,
            out_xmax,
            out_ymax,
            device,
            device_mem,
        )
        self._map = Map()
        with open(map_file, "rb") as f:
            self._map.ParseFromString(f.read())
        self.id2lanes = {lane.id: lane for lane in self._map.lanes}
        """
        Dictionary of lanes (Protobuf format) indexed by lane id
        """
        self.id2roads = {road.id: road for road in self._map.roads}
        """
        Dictionary of roads (Protobuf format) indexed by road id
        """
        self.id2junctions = {junction.id: junction for junction in self._map.junctions}
        """
        Dictionary of junctions (Protobuf format) indexed by junction id
        """
        self.id2aois = {aoi.id: aoi for aoi in self._map.aois}
        """
        Dictionary of AOIs (Protobuf format) indexed by AOI id
        """

        self.lane_index2id = self.fetch_lanes()["id"]
        """
        Numpy array of lane ids indexed by lane index
        """
        self.junc_index2id = self._e.get_junction_ids()
        """
        Numpy array of junction ids indexed by junction index
        """
        self.road_index2id = self._e.get_road_ids()
        """
        Numpy array of road ids indexed by road index
        """
        self.lane_id2index = {v.item(): k for k, v in enumerate(self.lane_index2id)}
        """
        Dictionary of lane index indexed by lane id
        """
        self.junc_id2index = {v.item(): k for k, v in enumerate(self.junc_index2id)}
        """
        Dictionary of junction id indexed by junction index
        """
        self.road_id2index = {v.item(): k for k, v in enumerate(self.road_index2id)}
        """
        Dictionary of road index indexed by road id
        """

        self._persons = Persons()
        with open(person_file, "rb") as f:
            self._persons.ParseFromString(f.read())
        self._map_bbox = (out_xmin, out_ymin, out_xmax, out_ymax)
        self.start_step = start_step
        """
        The starting step of the simulation
        """

        self.device = device
        """
        The CUDA device index
        """

    @property
    def person_count(self) -> int:
        """
        The number of vehicles in the agent file
        """
        return len(self._persons.persons)

    @property
    def lane_count(self) -> int:
        """
        The number of lanes
        """
        return len(self.id2lanes)

    @property
    def road_count(self) -> int:
        """
        The number of roads
        """
        return len(self.id2roads)

    @property
    def junction_count(self) -> int:
        """
        The number of junctions
        """
        return len(self.id2junctions)

    def get_map(self, dict_return: bool = True) -> Union[Map, Dict]:
        """
        Get the Map object.
        Map is a protobuf message defined in `pycityproto.city.map.v2.map_pb2` in the `pycityproto` package.
        The documentation url is https://docs.fiblab.net/cityproto#city.map.v2.Map

        Args:
        - dict_return: Whether to return the object as a dictionary

        Returns:
        - The Map object or the dictionary
        """
        if dict_return:
            return pb2dict(self._map)
        else:
            return self._map

    def get_persons(self, dict_return: bool = True) -> Union[Persons, Dict]:
        """
        Get the Persons object.
        Persons is a protobuf message defined in `pycityproto.city.person.v2.person_pb2` in the `pycityproto` package.
        The documentation url is https://docs.fiblab.net/cityproto#city.person.v2.Persons

        Args:
        - dict_return: Whether to return the object as a dictionary

        Returns:
        - The Persons object or the dictionary
        """
        if dict_return:
            return pb2dict(self._persons)
        else:
            return self._persons

    def get_current_time(self) -> float:
        """
        Get the current time
        """
        return self._e.get_current_time()

    def fetch_persons(self) -> Dict[str, NDArray]:
        """
        Fetch the persons' information.

        The result values is a dictionary with the following keys:
        - id: The id of the person
        - status: The status of the person
        - lane_id: The id of the lane the person is on
        - lane_parent_id: The id of the road the lane belongs to
        - s: The s value of the person
        - aoi_id: The id of the AOI the person is in
        - v: The velocity of the person
        - shadow_lane_id: The id of the shadow lane the person is on
        - shadow_s: The s value of the shadow lane
        - lc_yaw: The yaw of the lane change
        - lc_completed_ratio: The completed ratio of the lane change
        - is_forward: Whether the person is moving forward
        - x: The x coordinate of the person
        - y: The y coordinate of the person
        - dir: The direction of the person
        - schedule_index: The index of the schedule
        - trip_index: The index of the trip
        - departure_time: The departure time of the person
        - traveling_time: The traveling time of the person
        - total_distance: The total distance of the person

        We strongly recommend using `pd.DataFrame(e.fetch_persons())` to convert the result to a DataFrame for better visualization and analysis.
        """
        if self._fetched_persons is None:
            (
                ids,
                statuses,
                lane_ids,
                lane_parent_ids,
                ss,
                aoi_ids,
                vs,
                shadow_lane_ids,
                shadow_ss,
                lc_yaws,
                lc_completed_ratios,
                is_forwards,
                xs,
                ys,
                dirs,
                schedule_indexs,
                trip_indexs,
                departure_times,
                traveling_times,
                total_distances,
            ) = self._e.fetch_persons()
            self._fetched_persons = {
                "id": ids,
                "status": statuses,
                "lane_id": lane_ids,
                "lane_parent_id": lane_parent_ids,
                "s": ss,
                "aoi_id": aoi_ids,
                "v": vs,
                "shadow_lane_id": shadow_lane_ids,
                "shadow_s": shadow_ss,
                "lc_yaw": lc_yaws,
                "lc_completed_ratio": lc_completed_ratios,
                "is_forward": is_forwards,
                "x": xs,
                "y": ys,
                "dir": dirs,
                "schedule_index": schedule_indexs,
                "trip_index": trip_indexs,
                "departure_time": departure_times,
                "traveling_time": traveling_times,
                "total_distance": total_distances,
            }
        return self._fetched_persons

    def fetch_lanes(self) -> Dict[str, NDArray]:
        """
        Fetch the lanes' information.

        The result values is a dictionary with the following keys:
        - id: The id of the lane
        - status: The status of the lane
        - v_avg: The average speed of the lane

        We strongly recommend using `pd.DataFrame(e.fetch_lanes())` to convert the result to a DataFrame for better visualization and analysis.
        """
        if self._fetched_lanes is None:
            ids, statuses, v_avgs = self._e.fetch_lanes()
            self._fetched_lanes = {
                "id": ids,
                "status": statuses,
                "v_avg": v_avgs,
            }
        return self._fetched_lanes

    def get_running_person_count(self) -> int:
        """
        Get the total number of running persons (including driving and walking)
        """
        persons = self.fetch_persons()
        status: NDArray[np.uint8] = persons["status"]
        return (
            status == PersonStatus.DRIVING.value | status == PersonStatus.WALKING.value
        ).sum()

    def get_lane_statuses(self) -> NDArray[np.int8]:
        """
        Get the traffic light status of each lane, `0`-green / `1`-yellow / `2`-red / `3`-restriction.
        The lane id of the entry `i` can be obtained by `e.lane_index2id[i]`.
        """
        lanes = self.fetch_lanes()
        return lanes["status"]

    def get_lane_waiting_vehicle_counts(
        self, speed_threshold: float = 0.1
    ) -> Dict[int, int]:
        """
        Get the number of vehicles of each lane with speed lower than `speed_threshold`

        Returns:
        - Dict: lane id -> number of vehicles
        """

        persons = self.fetch_persons()
        lane_id = persons["lane_id"]
        status = persons["status"]
        v = persons["v"]
        filter = (status == PersonStatus.DRIVING.value) & (v < speed_threshold)
        filtered_lane_id = lane_id[filter]
        # count for the lane id
        unique, counts = np.unique(filtered_lane_id, return_counts=True)

        return dict(zip(unique, counts))

    def get_lane_waiting_at_end_vehicle_counts(
        self, speed_threshold: float = 0.1, distance_to_end: float = 100
    ) -> Dict[int, int]:
        """
        Get the number of vehicles of each lane with speed lower than `speed_threshold` and distance to end lower than `distance_to_end`

        Returns:
        - Dict: lane id -> number of vehicles
        """

        persons = self.fetch_persons()
        lane_id = persons["lane_id"]
        status = persons["status"]
        v = persons["v"]
        s = persons["s"]
        filter = (status == PersonStatus.DRIVING.value) & (v < speed_threshold)
        filtered_lane_id = lane_id[filter]
        filtered_s = s[filter]
        # find the distance to the end of the lane
        lane_ids_for_count = []
        for i, s in zip(filtered_lane_id, filtered_s):
            if self.id2lanes[i].length - s < distance_to_end:
                lane_ids_for_count.append(i)
        # count for the lane id
        unique, counts = np.unique(lane_ids_for_count, return_counts=True)
        return dict(zip(unique, counts))

    def get_lane_ids(self) -> NDArray[np.int32]:
        """
        Get the ids of the lanes as a numpy array
        """
        return self.lane_index2id

    def get_lane_average_vehicle_speed(self, lane_index: int) -> float:
        """
        Get the average speed of the vehicles on the lane `lane_index`
        """
        if self.speed_stat_interval == 0:
            raise RuntimeError(
                "Please set speed_stat_interval to enable speed statistics"
            )
        lanes = self.fetch_lanes()
        v_args: NDArray[np.float32] = lanes["v_avg"]
        return v_args[lane_index].item()

    def get_junction_ids(self) -> NDArray[np.int32]:
        """
        Get the ids of the junctions
        """
        return self.junc_index2id

    def get_junction_phase_lanes(self) -> List[List[Tuple[List[int], List[int]]]]:
        """
        Get the `index` of the `in` and `out` lanes of each phase of each junction

        Examples: TODO
        """
        return self._e.get_junction_phase_lanes()

    def get_junction_phase_ids(self) -> NDArray[np.int32]:
        """
        Get the phase id of each junction, `-1` if it has no traffic lights.
        The junction id of the entry `i` can be obtained by `e.junc_index2id[i]`.
        """
        return self._e.get_junction_phase_ids()

    def get_junction_phase_counts(self) -> NDArray[np.int32]:
        """
        Get the number of available phases of each junction.
        The junction id of the entry `i` can be obtained by `e.junc_index2id[i]`.
        """
        return self._e.get_junction_phase_counts()

    def get_junction_dynamic_roads(self) -> List[List[int]]:
        """
        Get the ids of the dynamic roads connected to each junction.
        The junction id of the entry `i` can be obtained by `e.junc_index2id
        """
        return self._e.get_junction_dynamic_roads()

    def get_road_lane_plans(self, road_index: int) -> List[List[slice]]:
        """
        Get the dynamic lane plan of the road `road_index`,
        represented as list of lane groups:
        ```
        [
            [slice(lane_start, lane_end), ...]
        ]
        ```
        """
        return [
            [slice(a, b) for a, b in i] for i in self._e.get_road_lane_plans(road_index)
        ]

    def get_road_average_vehicle_speed(self, road_index: int) -> float:
        """
        Get the average speed of the vehicles on the road `road_index`
        """
        if self.speed_stat_interval == 0:
            raise RuntimeError(
                "Please set speed_stat_interval to enable speed statistics"
            )
        lanes = self.fetch_lanes()
        road_id = self.road_index2id[road_index]
        lane_ids = self.id2roads[road_id].lane_ids
        lane_indexes = [self.lane_id2index[i] for i in lane_ids]
        v_args: NDArray[np.float32] = lanes["v_avg"]
        return v_args[lane_indexes].mean().item()

    def get_finished_person_count(self) -> int:
        """
        Get the number of the finished persons
        """
        persons = self.fetch_persons()
        status: NDArray[np.uint8] = persons["status"]
        return (status == PersonStatus.FINISHED.value).sum()

    def get_finished_person_average_traveling_time(self) -> float:
        """
        Get the average traveling time of the finished persons
        """
        persons = self.fetch_persons()
        status: NDArray[np.uint8] = persons["status"]
        traveling_time = persons["traveling_time"]
        return traveling_time[status == PersonStatus.FINISHED.value].mean()

    def get_running_person_average_traveling_time(self) -> float:
        """
        Get the average traveling time of the running persons
        """
        persons = self.fetch_persons()
        status: NDArray[np.uint8] = persons["status"]
        traveling_time = persons["traveling_time"]
        return traveling_time[status == PersonStatus.DRIVING.value].mean()

    def get_departed_person_average_traveling_time(self) -> float:
        """
        Get the average traveling time of the departed persons (running+finished)
        """
        persons = self.fetch_persons()
        status: NDArray[np.uint8] = persons["status"]
        traveling_time = persons["traveling_time"]
        return traveling_time[status != PersonStatus.SLEEP.value].mean()

    def get_road_lane_plan_index(self, road_index: int) -> int:
        """
        Get the lane plan of road `road_index`
        """
        return self._e.get_road_lane_plan_index(road_index)

    def get_road_vehicle_counts(self) -> Dict[int, int]:
        """
        Get the number of vehicles of each road

        Returns:
        - Dict: road id -> number of vehicles
        """
        persons = self.fetch_persons()
        road_id = persons["lane_parent_id"]
        status = persons["status"]
        filter = status == PersonStatus.DRIVING.value
        filtered_road_id = road_id[filter]
        # count for the road id
        unique, counts = np.unique(filtered_road_id, return_counts=True)
        return dict(zip(unique, counts))

    def get_road_waiting_vehicle_counts(
        self, speed_threshold: float = 0.1
    ) -> Dict[int, int]:
        """
        Get the number of vehicles with speed lower than `speed_threshold` of each road

        Returns:
        - Dict: road id -> number of vehicles
        """

        persons = self.fetch_persons()
        road_id = persons["lane_parent_id"]
        status = persons["status"]
        v = persons["v"]
        filter = (
            (status == PersonStatus.DRIVING.value)
            & (v < speed_threshold)
            & (road_id < 3_0000_0000)  # the road id ranges [2_0000_0000, 3_0000_0000)
        )
        filtered_road_id = road_id[filter]
        # count for the road id
        unique, counts = np.unique(filtered_road_id, return_counts=True)
        return dict(zip(unique, counts))

    def set_tl_policy(self, junction_index: int, policy: TlPolicy):
        """
        Set the traffic light policy of junction `junction_index` to `policy`
        """
        self._e.set_tl_policy(junction_index, policy.value)

    def set_tl_policy_batch(self, junction_indices: List[int], policy: TlPolicy):
        """
        Set the traffic light policy of all junctions in `junction_indices` to `policy`
        """
        self._e.set_tl_policy_batch(junction_indices, policy.value)

    def set_tl_duration(self, junction_index: int, duration: int):
        """
        Set the traffic light switch duration of junction `junction_index` to `duration`

        NOTE: This is only effective for `TlPolicy.FIXED_TIME` and `TlPolicy.MAX_PRESSURE`.

        NOTE: Set duration to `0` to use the predefined duration in the `map_file`
        """
        self._e.set_tl_duration(junction_index, duration)

    def set_tl_duration_batch(self, junction_indices: List[int], duration: int):
        """
        Set the traffic light switch duration of all junctions in `junction_indices` to `duration`

        NOTE: This is only effective for `TlPolicy.FIXED_TIME` and `TlPolicy.MAX_PRESSURE`

        NOTE: Set duration to `0` to use the predefined duration in the `map_file`
        """
        self._e.set_tl_duration_batch(junction_indices, duration)

    def set_tl_phase(self, junction_index: Union[str, int], phase_index: int):
        """
        Set the phase of `junction_index` to `phase_index`
        """
        self._e.set_tl_phase(junction_index, phase_index)

    def set_tl_phase_batch(self, junction_indices: List[int], phase_indices: List[int]):
        """
        Set the phase of `junction_index` to `phase_index` in batch
        """
        assert len(junction_indices) == len(phase_indices)
        self._e.set_tl_phase_batch(junction_indices, phase_indices)

    def set_road_lane_plan(self, road_index: int, plan_index: int):
        """
        Set the lane plan of road `road_index`
        """
        self._e.set_road_lane_plan(road_index, plan_index)

    def set_road_lane_plan_batch(
        self, road_indices: List[int], plan_indices: List[int]
    ):
        """
        Set the lane plan of road `road_index`
        """
        assert len(road_indices) == len(plan_indices)
        self._e.set_road_lane_plan_batch(road_indices, plan_indices)

    def set_lane_restriction(self, lane_index: int, flag: bool):
        """
        Set the restriction state of lane `lane_index`
        """
        self._e.set_lane_restriction(lane_index, flag)

    def set_lane_restriction_batch(self, lane_indices: List[int], flags: List[bool]):
        """
        Set the restriction state of lane `lane_index`
        """
        assert len(lane_indices) == len(flags)
        self._e.set_lane_restriction_batch(lane_indices, flags)

    def set_lane_max_speed(self, lane_index: int, max_speed: float):
        """
        Set the max_speed of lane `lane_index`
        """
        self._e.set_lane_max_speed(lane_index, max_speed)

    def set_lane_max_speed_batch(
        self, lane_indices: List[int], max_speeds: Union[float, List[float]]
    ):
        """
        Set the max_speed of lane `lane_index`
        """
        if hasattr(max_speeds, "__len__"):
            assert len(lane_indices) == len(max_speeds)
        else:
            max_speeds = [max_speeds] * len(lane_indices)
        self._e.set_lane_max_speed_batch(lane_indices, max_speeds)

    def next_step(self, n=1):
        """
        Move forward `n` steps
        """
        self._fetched_persons = None
        self._fetched_lanes = None
        self._e.next_step(n)
