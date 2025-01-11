# {py:mod}`moss.engine`

```{py:module} moss.engine
```

```{autodoc2-docstring} moss.engine
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`TlPolicy <moss.engine.TlPolicy>`
  -
* - {py:obj}`Verbosity <moss.engine.Verbosity>`
  -
* - {py:obj}`Engine <moss.engine.Engine>`
  - ```{autodoc2-docstring} moss.engine.Engine
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`_import <moss.engine._import>`
  - ```{autodoc2-docstring} moss.engine._import
    :summary:
    ```
* - {py:obj}`_populate_waiting_at_lane_end <moss.engine._populate_waiting_at_lane_end>`
  - ```{autodoc2-docstring} moss.engine._populate_waiting_at_lane_end
    :summary:
    ```
* - {py:obj}`_populate_waiting_at_lane <moss.engine._populate_waiting_at_lane>`
  - ```{autodoc2-docstring} moss.engine._populate_waiting_at_lane
    :summary:
    ```
````

### Data

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`__all__ <moss.engine.__all__>`
  - ```{autodoc2-docstring} moss.engine.__all__
    :summary:
    ```
* - {py:obj}`_moss <moss.engine._moss>`
  - ```{autodoc2-docstring} moss.engine._moss
    :summary:
    ```
* - {py:obj}`_thread_local <moss.engine._thread_local>`
  - ```{autodoc2-docstring} moss.engine._thread_local
    :summary:
    ```
* - {py:obj}`SLEEP <moss.engine.SLEEP>`
  - ```{autodoc2-docstring} moss.engine.SLEEP
    :summary:
    ```
* - {py:obj}`WALKING <moss.engine.WALKING>`
  - ```{autodoc2-docstring} moss.engine.WALKING
    :summary:
    ```
* - {py:obj}`DRIVING <moss.engine.DRIVING>`
  - ```{autodoc2-docstring} moss.engine.DRIVING
    :summary:
    ```
* - {py:obj}`FINISHED <moss.engine.FINISHED>`
  - ```{autodoc2-docstring} moss.engine.FINISHED
    :summary:
    ```
````

### API

````{py:data} __all__
:canonical: moss.engine.__all__
:value: >
   ['Engine', 'TlPolicy', 'Verbosity', 'SLEEP', 'WALKING', 'DRIVING', 'FINISHED']

```{autodoc2-docstring} moss.engine.__all__
```

````

````{py:function} _import()
:canonical: moss.engine._import

```{autodoc2-docstring} moss.engine._import
```
````

````{py:data} _moss
:canonical: moss.engine._moss
:value: >
   '_import(...)'

```{autodoc2-docstring} moss.engine._moss
```

````

````{py:data} _thread_local
:canonical: moss.engine._thread_local
:value: >
   'local(...)'

```{autodoc2-docstring} moss.engine._thread_local
```

````

````{py:function} _populate_waiting_at_lane_end(enable: numpy.ndarray, status: numpy.ndarray, lane_id: numpy.ndarray, lane_length_array: numpy.ndarray, v: numpy.ndarray, s: numpy.ndarray, speed_threshold: float, distance_to_end: float)
:canonical: moss.engine._populate_waiting_at_lane_end

```{autodoc2-docstring} moss.engine._populate_waiting_at_lane_end
```
````

````{py:function} _populate_waiting_at_lane(enable: numpy.ndarray, status: numpy.ndarray, lane_id: numpy.ndarray, v: numpy.ndarray, speed_threshold: float)
:canonical: moss.engine._populate_waiting_at_lane

```{autodoc2-docstring} moss.engine._populate_waiting_at_lane
```
````

`````{py:class} TlPolicy
:canonical: moss.engine.TlPolicy

Bases: {py:obj}`enum.Enum`

````{py:attribute} MANUAL
:canonical: moss.engine.TlPolicy.MANUAL
:value: >
   0

```{autodoc2-docstring} moss.engine.TlPolicy.MANUAL
```

````

````{py:attribute} FIXED_TIME
:canonical: moss.engine.TlPolicy.FIXED_TIME
:value: >
   1

```{autodoc2-docstring} moss.engine.TlPolicy.FIXED_TIME
```

````

````{py:attribute} MAX_PRESSURE
:canonical: moss.engine.TlPolicy.MAX_PRESSURE
:value: >
   2

```{autodoc2-docstring} moss.engine.TlPolicy.MAX_PRESSURE
```

````

````{py:attribute} NONE
:canonical: moss.engine.TlPolicy.NONE
:value: >
   3

```{autodoc2-docstring} moss.engine.TlPolicy.NONE
```

````

`````

`````{py:class} Verbosity
:canonical: moss.engine.Verbosity

Bases: {py:obj}`enum.Enum`

````{py:attribute} NO_OUTPUT
:canonical: moss.engine.Verbosity.NO_OUTPUT
:value: >
   0

```{autodoc2-docstring} moss.engine.Verbosity.NO_OUTPUT
```

````

````{py:attribute} INIT_ONLY
:canonical: moss.engine.Verbosity.INIT_ONLY
:value: >
   1

```{autodoc2-docstring} moss.engine.Verbosity.INIT_ONLY
```

````

````{py:attribute} ALL
:canonical: moss.engine.Verbosity.ALL
:value: >
   2

```{autodoc2-docstring} moss.engine.Verbosity.ALL
```

````

`````

````{py:data} SLEEP
:canonical: moss.engine.SLEEP
:value: >
   0

```{autodoc2-docstring} moss.engine.SLEEP
```

````

````{py:data} WALKING
:canonical: moss.engine.WALKING
:value: >
   1

```{autodoc2-docstring} moss.engine.WALKING
```

````

````{py:data} DRIVING
:canonical: moss.engine.DRIVING
:value: >
   2

```{autodoc2-docstring} moss.engine.DRIVING
```

````

````{py:data} FINISHED
:canonical: moss.engine.FINISHED
:value: >
   3

```{autodoc2-docstring} moss.engine.FINISHED
```

````

`````{py:class} Engine(name: str, map_file: str, person_file: str, start_step: int = 0, step_interval: float = 1, seed: int = 43, verbose_level=Verbosity.NO_OUTPUT, person_limit: int = -1, junction_yellow_time: float = 0, phase_pressure_coeff: float = 1.5, speed_stat_interval: int = 0, output_dir: str = '', out_xmin: float = -inf, out_ymin: float = -inf, out_xmax: float = inf, out_ymax: float = inf, device: int = 0)
:canonical: moss.engine.Engine

```{autodoc2-docstring} moss.engine.Engine
```

```{rubric} Initialization
```

```{autodoc2-docstring} moss.engine.Engine.__init__
```

````{py:attribute} __version__
:canonical: moss.engine.Engine.__version__
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.__version__
```

````

````{py:attribute} speed_stat_interval
:canonical: moss.engine.Engine.speed_stat_interval
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.speed_stat_interval
```

````

````{py:attribute} id2lanes
:canonical: moss.engine.Engine.id2lanes
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.id2lanes
```

````

````{py:attribute} id2roads
:canonical: moss.engine.Engine.id2roads
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.id2roads
```

````

````{py:attribute} id2junctions
:canonical: moss.engine.Engine.id2junctions
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.id2junctions
```

````

````{py:attribute} id2aois
:canonical: moss.engine.Engine.id2aois
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.id2aois
```

````

````{py:attribute} lane_index2id
:canonical: moss.engine.Engine.lane_index2id
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.lane_index2id
```

````

````{py:attribute} junc_index2id
:canonical: moss.engine.Engine.junc_index2id
:value: >
   'get_junction_ids(...)'

```{autodoc2-docstring} moss.engine.Engine.junc_index2id
```

````

````{py:attribute} road_index2id
:canonical: moss.engine.Engine.road_index2id
:value: >
   'get_road_ids(...)'

```{autodoc2-docstring} moss.engine.Engine.road_index2id
```

````

````{py:attribute} lane_id2index
:canonical: moss.engine.Engine.lane_id2index
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.lane_id2index
```

````

````{py:attribute} junc_id2index
:canonical: moss.engine.Engine.junc_id2index
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.junc_id2index
```

````

````{py:attribute} road_id2index
:canonical: moss.engine.Engine.road_id2index
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.road_id2index
```

````

````{py:attribute} lane_length_array
:canonical: moss.engine.Engine.lane_length_array
:value: >
   'array(...)'

```{autodoc2-docstring} moss.engine.Engine.lane_length_array
```

````

````{py:attribute} start_step
:canonical: moss.engine.Engine.start_step
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.start_step
```

````

````{py:attribute} device
:canonical: moss.engine.Engine.device
:value: >
   None

```{autodoc2-docstring} moss.engine.Engine.device
```

````

````{py:method} _map_warning()
:canonical: moss.engine.Engine._map_warning

```{autodoc2-docstring} moss.engine.Engine._map_warning
```

````

````{py:property} person_count
:canonical: moss.engine.Engine.person_count
:type: int

```{autodoc2-docstring} moss.engine.Engine.person_count
```

````

````{py:property} lane_count
:canonical: moss.engine.Engine.lane_count
:type: int

```{autodoc2-docstring} moss.engine.Engine.lane_count
```

````

````{py:property} road_count
:canonical: moss.engine.Engine.road_count
:type: int

```{autodoc2-docstring} moss.engine.Engine.road_count
```

````

````{py:property} junction_count
:canonical: moss.engine.Engine.junction_count
:type: int

```{autodoc2-docstring} moss.engine.Engine.junction_count
```

````

````{py:method} get_map(dict_return: bool = True) -> typing.Union[pycityproto.city.map.v2.map_pb2.Map, typing.Dict]
:canonical: moss.engine.Engine.get_map

```{autodoc2-docstring} moss.engine.Engine.get_map
```

````

````{py:method} get_persons(dict_return: bool = True) -> typing.Union[pycityproto.city.person.v2.person_pb2.Persons, typing.Dict]
:canonical: moss.engine.Engine.get_persons

```{autodoc2-docstring} moss.engine.Engine.get_persons
```

````

````{py:method} get_current_time() -> float
:canonical: moss.engine.Engine.get_current_time

```{autodoc2-docstring} moss.engine.Engine.get_current_time
```

````

````{py:method} fetch_persons(fields: typing.List[str] = []) -> typing.Dict[str, numpy.typing.NDArray]
:canonical: moss.engine.Engine.fetch_persons

```{autodoc2-docstring} moss.engine.Engine.fetch_persons
```

````

````{py:method} fetch_lanes() -> typing.Dict[str, numpy.typing.NDArray]
:canonical: moss.engine.Engine.fetch_lanes

```{autodoc2-docstring} moss.engine.Engine.fetch_lanes
```

````

````{py:method} get_running_person_count() -> int
:canonical: moss.engine.Engine.get_running_person_count

```{autodoc2-docstring} moss.engine.Engine.get_running_person_count
```

````

````{py:method} get_lane_statuses() -> numpy.typing.NDArray[numpy.int8]
:canonical: moss.engine.Engine.get_lane_statuses

```{autodoc2-docstring} moss.engine.Engine.get_lane_statuses
```

````

````{py:method} get_lane_waiting_vehicle_counts(speed_threshold: float = 0.1) -> typing.Tuple[numpy.typing.NDArray[numpy.int32], numpy.typing.NDArray[numpy.int32]]
:canonical: moss.engine.Engine.get_lane_waiting_vehicle_counts

```{autodoc2-docstring} moss.engine.Engine.get_lane_waiting_vehicle_counts
```

````

````{py:method} get_lane_waiting_at_end_vehicle_counts(speed_threshold: float = 0.1, distance_to_end: float = 100) -> typing.Tuple[numpy.typing.NDArray[numpy.int32], numpy.typing.NDArray[numpy.int32]]
:canonical: moss.engine.Engine.get_lane_waiting_at_end_vehicle_counts

```{autodoc2-docstring} moss.engine.Engine.get_lane_waiting_at_end_vehicle_counts
```

````

````{py:method} get_lane_ids() -> numpy.typing.NDArray[numpy.int32]
:canonical: moss.engine.Engine.get_lane_ids

```{autodoc2-docstring} moss.engine.Engine.get_lane_ids
```

````

````{py:method} get_lane_average_vehicle_speed(lane_index: int) -> float
:canonical: moss.engine.Engine.get_lane_average_vehicle_speed

```{autodoc2-docstring} moss.engine.Engine.get_lane_average_vehicle_speed
```

````

````{py:method} get_junction_ids() -> numpy.typing.NDArray[numpy.int32]
:canonical: moss.engine.Engine.get_junction_ids

```{autodoc2-docstring} moss.engine.Engine.get_junction_ids
```

````

````{py:method} get_junction_phase_lanes() -> typing.List[typing.List[typing.Tuple[typing.List[int], typing.List[int]]]]
:canonical: moss.engine.Engine.get_junction_phase_lanes

```{autodoc2-docstring} moss.engine.Engine.get_junction_phase_lanes
```

````

````{py:method} get_junction_phase_ids() -> numpy.typing.NDArray[numpy.int32]
:canonical: moss.engine.Engine.get_junction_phase_ids

```{autodoc2-docstring} moss.engine.Engine.get_junction_phase_ids
```

````

````{py:method} get_junction_phase_counts() -> numpy.typing.NDArray[numpy.int32]
:canonical: moss.engine.Engine.get_junction_phase_counts

```{autodoc2-docstring} moss.engine.Engine.get_junction_phase_counts
```

````

````{py:method} get_junction_dynamic_roads() -> typing.List[typing.List[int]]
:canonical: moss.engine.Engine.get_junction_dynamic_roads

```{autodoc2-docstring} moss.engine.Engine.get_junction_dynamic_roads
```

````

````{py:method} get_road_lane_plans(road_index: int) -> typing.List[typing.List[slice]]
:canonical: moss.engine.Engine.get_road_lane_plans

```{autodoc2-docstring} moss.engine.Engine.get_road_lane_plans
```

````

````{py:method} get_road_average_vehicle_speed(road_index: int) -> float
:canonical: moss.engine.Engine.get_road_average_vehicle_speed

```{autodoc2-docstring} moss.engine.Engine.get_road_average_vehicle_speed
```

````

````{py:method} get_finished_person_count() -> int
:canonical: moss.engine.Engine.get_finished_person_count

```{autodoc2-docstring} moss.engine.Engine.get_finished_person_count
```

````

````{py:method} get_finished_person_average_traveling_time() -> float
:canonical: moss.engine.Engine.get_finished_person_average_traveling_time

```{autodoc2-docstring} moss.engine.Engine.get_finished_person_average_traveling_time
```

````

````{py:method} get_running_person_average_traveling_time() -> float
:canonical: moss.engine.Engine.get_running_person_average_traveling_time

```{autodoc2-docstring} moss.engine.Engine.get_running_person_average_traveling_time
```

````

````{py:method} get_departed_person_average_traveling_time() -> float
:canonical: moss.engine.Engine.get_departed_person_average_traveling_time

```{autodoc2-docstring} moss.engine.Engine.get_departed_person_average_traveling_time
```

````

````{py:method} get_road_lane_plan_index(road_index: int) -> int
:canonical: moss.engine.Engine.get_road_lane_plan_index

```{autodoc2-docstring} moss.engine.Engine.get_road_lane_plan_index
```

````

````{py:method} get_road_vehicle_counts() -> typing.Tuple[numpy.typing.NDArray[numpy.int32], numpy.typing.NDArray[numpy.int32]]
:canonical: moss.engine.Engine.get_road_vehicle_counts

```{autodoc2-docstring} moss.engine.Engine.get_road_vehicle_counts
```

````

````{py:method} get_road_waiting_vehicle_counts(speed_threshold: float = 0.1) -> typing.Tuple[numpy.typing.NDArray[numpy.int32], numpy.typing.NDArray[numpy.int32]]
:canonical: moss.engine.Engine.get_road_waiting_vehicle_counts

```{autodoc2-docstring} moss.engine.Engine.get_road_waiting_vehicle_counts
```

````

````{py:method} set_person_enable(person_index: int, enable: bool)
:canonical: moss.engine.Engine.set_person_enable

```{autodoc2-docstring} moss.engine.Engine.set_person_enable
```

````

````{py:method} set_person_enable_batch(person_indices: typing.List[int], enable: typing.Union[bool, typing.List[bool]])
:canonical: moss.engine.Engine.set_person_enable_batch

```{autodoc2-docstring} moss.engine.Engine.set_person_enable_batch
```

````

````{py:method} set_tl_policy(junction_index: int, policy: moss.engine.TlPolicy)
:canonical: moss.engine.Engine.set_tl_policy

```{autodoc2-docstring} moss.engine.Engine.set_tl_policy
```

````

````{py:method} set_tl_policy_batch(junction_indices: typing.List[int], policy: moss.engine.TlPolicy)
:canonical: moss.engine.Engine.set_tl_policy_batch

```{autodoc2-docstring} moss.engine.Engine.set_tl_policy_batch
```

````

````{py:method} set_tl_duration(junction_index: int, duration: int)
:canonical: moss.engine.Engine.set_tl_duration

```{autodoc2-docstring} moss.engine.Engine.set_tl_duration
```

````

````{py:method} set_tl_duration_batch(junction_indices: typing.List[int], duration: int)
:canonical: moss.engine.Engine.set_tl_duration_batch

```{autodoc2-docstring} moss.engine.Engine.set_tl_duration_batch
```

````

````{py:method} set_tl_phase(junction_index: typing.Union[str, int], phase_index: int)
:canonical: moss.engine.Engine.set_tl_phase

```{autodoc2-docstring} moss.engine.Engine.set_tl_phase
```

````

````{py:method} set_tl_phase_batch(junction_indices: typing.List[int], phase_indices: typing.List[int])
:canonical: moss.engine.Engine.set_tl_phase_batch

```{autodoc2-docstring} moss.engine.Engine.set_tl_phase_batch
```

````

````{py:method} set_road_lane_plan(road_index: int, plan_index: int)
:canonical: moss.engine.Engine.set_road_lane_plan

```{autodoc2-docstring} moss.engine.Engine.set_road_lane_plan
```

````

````{py:method} set_road_lane_plan_batch(road_indices: typing.List[int], plan_indices: typing.List[int])
:canonical: moss.engine.Engine.set_road_lane_plan_batch

```{autodoc2-docstring} moss.engine.Engine.set_road_lane_plan_batch
```

````

````{py:method} set_lane_restriction(lane_index: int, flag: bool)
:canonical: moss.engine.Engine.set_lane_restriction

```{autodoc2-docstring} moss.engine.Engine.set_lane_restriction
```

````

````{py:method} set_lane_restriction_batch(lane_indices: typing.List[int], flags: typing.List[bool])
:canonical: moss.engine.Engine.set_lane_restriction_batch

```{autodoc2-docstring} moss.engine.Engine.set_lane_restriction_batch
```

````

````{py:method} set_lane_max_speed(lane_index: int, max_speed: float)
:canonical: moss.engine.Engine.set_lane_max_speed

```{autodoc2-docstring} moss.engine.Engine.set_lane_max_speed
```

````

````{py:method} set_lane_max_speed_batch(lane_indices: typing.List[int], max_speeds: typing.Union[float, typing.List[float]])
:canonical: moss.engine.Engine.set_lane_max_speed_batch

```{autodoc2-docstring} moss.engine.Engine.set_lane_max_speed_batch
```

````

````{py:method} set_vehicle_route(person_id: int, route: typing.List[int])
:canonical: moss.engine.Engine.set_vehicle_route

```{autodoc2-docstring} moss.engine.Engine.set_vehicle_route
```

````

````{py:method} next_step(n=1)
:canonical: moss.engine.Engine.next_step

```{autodoc2-docstring} moss.engine.Engine.next_step
```

````

````{py:method} make_checkpoint() -> int
:canonical: moss.engine.Engine.make_checkpoint

```{autodoc2-docstring} moss.engine.Engine.make_checkpoint
```

````

````{py:method} restore_checkpoint(checkpoint_id: int)
:canonical: moss.engine.Engine.restore_checkpoint

```{autodoc2-docstring} moss.engine.Engine.restore_checkpoint
```

````

`````
