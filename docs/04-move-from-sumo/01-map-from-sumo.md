# Map Convert

To facilitate user migration from SUMO, we designed to convert maps from SUMO to MOSS [Protobuf Map format](../03-map-building/01-format.md#protobuf-map-format).

## Load SUMO map file and Convert the map

```python
from mosstool.map.sumo.map import MapConverter
from mosstool.type import Map
from mosstool.util.format_converter import dict2pb
s2m = MapConverter(
    net_path="data/sumo/shenzhen.net.xml"
)
m = s2m.convert_map()
pb = dict2pb(m, Map())
with open("data/temp/sumo_map.pb", "wb") as f:
    f.write(pb.SerializeToString())
```

`net_path` is the path of [SUMO road net](https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html) `xml` file.

Control parameters
- `default_lane_width (float)`: the default value of lane width for the lanes without a `width` attribute.
  
Here we get the converted MOSS map and dump it to `data/temp/sumo_map.pb`.

## Advanced Usage

### Traffic Light Convert

[SUMO traffic-light](https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html) `xml` files can be converted and incorporated into the output map by providing the `traffic_light_path` parameter during the initialization of `MapConverter`.

Control parameters
- `green_time (float)`: the green light duration in traffic light.
- `yellow_time (float)`: the yellow light duration in traffic light.
- `traffic_light_min_direction_group (int)`: for those junctions without input traffic-light and its number of lane directions are greater than or equal to `traffic_light_min_direction_group`, `MapConverter` will generate default fixed time traffic-light for it.

### AOI and POI Convert

[SUMO AOIs and POIs](https://sumo.dlr.de/docs/Simulation/Shapes.html) `xml` files can be converted and incorporated into the output map by providing the `poly_path` parameter during the initialization of `MapConverter`.

Control parameters
- `merge_aoi (bool)`: if true, nearby AOIs will be merged into one larger AOI, thus reducing overall AOI numbers.
