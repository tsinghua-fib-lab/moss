# Build Trip

In this step, we generate random persons with specific movements between `lanes` within the map.

## Generate persons with only origin and destination positions

### Initialize the `AigcGenerator` and Generate the persons

`AigcGenerator` is a tool to generate persons on specific map.

```python
import geopandas as gpd
import pyproj
from shapely.geometry import Polygon

from mosstool.trip.generator import AigcGenerator, TripGenerator
from mosstool.trip.route import RoutingClient, pre_route
from mosstool.type import Map, Person, Persons, TripMode
from mosstool.util.geo_match_pop import geo2pop

aigc_generator = AigcGenerator()
POP_TIF_PATH = "chn_ppp_2020_UNadj_constrained.tif"  # world pop data for china: "https://hub.worldpop.org/geodata/summary?id=49730"
YOUR_ACCESS_TOKEN = "1"

with open("data/temp/map.pb", "rb") as f:
    m = Map()
    m.ParseFromString(f.read())
projector = pyproj.Proj(m.header.projection)
geos = []
for aoi in m.aois:
    geo_xy = aoi.positions
    geos.append([aoi.id, Polygon([projector(p.x, p.y, inverse=True) for p in geo_xy])])
gdf = gpd.GeoDataFrame(geos, columns=["id", "geometry"], crs="EPSG:4326")  # type: ignore
area = geo2pop(gdf, POP_TIF_PATH, enable_tqdm=True)


aigc_generator.set_satetoken(YOUR_ACCESS_TOKEN)
area = gpd.read_file("data/gravitygenerator/Beijing-shp/beijing.shp")
aigc_generator.load_area(area)
od_matrix = aigc_generator.generate()

tg = TripGenerator(
    m=m,
)
persons = tg.generate_persons(
    od_matrix=od_matrix,
    areas=area,
    agent_num=100,
)
print(persons)
```

- `agent_num` specifies the number of persons to be generated.

## Fill in the route of the persons' all schedules (Optional)

This step requires extra packages released on [Releases · routing](https://github.com/tsinghua-fib-lab/routing/releases/).
Activate the `routing` service before running codes below with `./routing -map data/temp/map.pb`.
```python
client = RoutingClient("http://localhost:52101")
ok_persons = []
for p in persons:
    p = await pre_route(client, p)
    if len(p.schedules) > 0 and len(p.schedules[0].trips) > 0:
        ok_persons.append(p)
print(ok_persons)
print("final length: ", len(ok_persons))
pb = Persons(persons=ok_persons)
with open("data/temp/persons.pb", "wb") as f:
    f.write(pb.SerializeToString())
```
- For local running routing service, the default listening host is `52101`, so in this example the input argument of `RoutingClient` is set to "http://localhost:52101". To change this gRPC listening address, add extra input arg `-listen "localhost:<HOST_NUM>"` when activating `routing` service. 
- Due to the use of the `await` keyword in the request navigation, the entire function needs to be an **asynchronous function** (declared with async)
- `if len(p.schedules) > 0 and len(p.schedules[0].trips) > 0` keeps `person` woh has at least one trip with valid route.
