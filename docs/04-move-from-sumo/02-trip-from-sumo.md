# Trip Convert

To facilitate user migration from SUMO, we designed to convert trips from SUMO to MOSS [Protobuf Trip format](../04-trip-generation/01-format.md). To perform this conversion, you need perform a map convert with [Map Converter](01-map-from-sumo.md) first.

## Load SUMO map file and Convert the map

```python
from mosstool.map.sumo.map import MapConverter
from mosstool.trip.sumo.route import RouteConverter
from mosstool.type import Map, Persons
from mosstool.util.format_converter import dict2pb
s2m = MapConverter(
    net_path="data/sumo/shenzhen.net.xml",
)
m = s2m.convert_map()
id2uid = s2m.get_sumo_id_mappings()
map_pb = dict2pb(m, Map())
s2r = RouteConverter(
    converted_map=map_pb,
    sumo_id_mappings=id2uid,
    route_path="./data/sumo/trips.xml",
)
r = s2r.convert_route()
pb = dict2pb(r, Persons())
with open("data/temp/sumo_persons.pb", "wb") as f:
    f.write(pb.SerializeToString())
```

- `route_path` is the path of [SUMO trip file](https://sumo.dlr.de/docs/Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.html) `xml` file. Either incomplete trips ([trips and flows](https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#incomplete_routes_trips_and_flows)) or complete trips ([routes](https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#routes)) conversion are supported.
