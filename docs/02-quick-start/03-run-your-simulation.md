# Run Simulation

## Create MOSS engine

The following code is a simple demo to run the simulation.

```python
from moss import Engine, Verbosity
from moss.export import DBRecorder
MAP_PATH = "./data/temp/map.pb"
TRIP_PATH = "./data/temp/persons.pb"
e = Engine(
    "name",
    map_file=MAP_PATH,
    person_file=TRIP_PATH,
    start_step=0,
    step_interval=1,
    speed_stat_interval=300,  # open road status statistics
    verbose_level=Verbosity.ALL,
)
recorder = DBRecorder(
    e,
    "postgres://user:password@url:port/simulation",
    "map_db.map_coll",  # map collection used for webui-backend
    "name",
)  # used for PostgreSQL output (optional)
for _ in range(3600):
    e.next_step(1)
    recorder.record() # (optional)
    # YOU CAN DO SOMETHING HERE
    # persons = e.fetch_persons()
    # ...
# save the simulation results to the database (optional)
recorder.flush()
```
- `map_file` and `person_file` are what we generated in Step `Build Map` and `Build Trip`.
- `start_step` determines the time (in seconds) of the simulation begins. `8 * 3600` means the simulation begins at 8 a.m.
- `next_step` is used to simulate step by step for 600 steps. You can also specify `next_step(n)` to simulate a certain number of steps at once.
