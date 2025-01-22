# Map Builder

In this section, we will introduce you to the advanced features of the `Map Builder`.

## Detailed Map Content Input 

### `Road Network`

- `net (Union[FeatureCollection, Map])` is the input of map road network. `Builder` accepts two types of net input: `FeatureCollection` and `Map`. If a `Map` is provided as input, `Builder` constructs the map while preserving the original `lanes`, `roads`, and `junctions`. If a `FeatureCollection` in [the specified format](../03-map-building/01-format.md#roadnet) is given, the `lanes`, `roads`, and `junctions` are constructed based on the provided `LineStrings` and `MultiPoints`.
- `proj_str (Optional[str] = None)`, as discussed, is a [PROJ.4](https://proj.org/en/9.5/) projection string, used to transform longitude and latitude coordinates into planar xy coordinates. If `proj_str` is not provided to `Builder`, it is assumed that the coordinates in the `FeatureCollection` are already in planar xy format, and no projection transformation will be performed.

### `AOI`

- `aois (Optional[FeatureCollection] = None)`: input aois data in [specific format](../03-map-building/01-format.md#aois).
- `aoi_mode (Union[Literal["append"], Literal["overwrite"]] = "overwrite")`: `aoi_mode` provides two options: `overwrite` and `append`. When the `net` input to the Builder is a `FeatureCollection`, there is no difference between these modes. However, when the net input is a `Map`, `append` means that the original `AOIs` in the map will be retained, with new AOIs added based on the input `aois`. On the other hand, `overwrite` means that the original `AOIs` in the input map will be deleted, and only the new AOIs from the input `aois` will be added.
- `aoi_matching_distance_threshold (float = 30.0)`: in output map, all `AOIs` must be near at least one `lane`. The `aoi_matching_distance_threshold` defines the distance threshold for determining proximity, with the unit being meters.
- `merge_aoi (bool)`: if true, nearby AOIs will be merged into one larger AOI, thus reducing overall AOI numbers.

### `POI`

- `pois (Optional[FeatureCollection] = None)`: input pois data in [specific format](../03-map-building/01-format.md#pois).

## Control the Shape of `Road Network`

### Driving lane generation

- `road_expand_mode (Union[Literal["L"], Literal["M"], Literal["R"]] = "R")`: `road_expand_mode` has three options: `L`, `M`, and `R`. `L` represents generating the geometry for all `lanes` by translating the `LineString` of the road's right boundary to the left; `M` represents generating the geometry for all lanes by treating the `LineString` of the road as the centerline and translating it to both sides; `R` represents generating the geometry for all lanes by translating the `LineString` of the road's left boundary to the right.
- `expand_roads (bool = False)`: if true, the number of lanes for roads that connect to straight and U-turn roads will be increased by 2.
- `default_lane_width (float = 3.2)`: the default value of lane width for the lanes without a `width` attribute.
  
### Walking lane generation

- `gen_sidewalk_speed_limit (float) = 0`: after generating all the driving lanes based on the road's `LineString`, a decision is made whether to generate sidewalks for each road based on its [highway level](https://wiki.openstreetmap.org/wiki/Key:highway) or speed limit. The gen_sidewalk_speed_limit is the threshold speed value, in units of meters per second, above which sidewalks will be generated alongside the roads.

## Generation of Traffic Light

When creating a map with `Builder`, traffic light control situations in [specified format](../03-map-building/01-format.md#traffic-light-format) within junctions are automatically generated based on its lane connections. The following are some parameters that control the generation of these signal controls.

- `traffic_light_min_direction_group (int = 3)`: for junctions whose number of lane directions are greater than or equal to `traffic_light_min_direction_group` will be chosen to generate default fixed time traffic-light and feasible phase for max pressure algorithm.
- `traffic_light_mode (Union[Literal["green_red"], Literal["green_yellow_red"], Literal["green_yellow_clear_red"]] = "green_yellow_clear_red")`: `traffic_light_mode` offers three options: `green_red`, `green_yellow_red`, and `green_yellow_clear_red`. These represent different fixed-phase signal control modes: only red and green lights, including red, green, and yellow lights, and including red, green, and yellow lights along with a pedestrian clearance red light.
- `green_time (float = 30.0)`: the green light duration in traffic light.
- `yellow_time (float = 5.0)`: the yellow light duration in traffic light.

## Output Format Check

`Builder` provides default checks for the correctness of input and output structures. 
- Additionally, you can set `output_lane_length_check` to True to verify whether the lanes connected in junctions are excessively long due to a large enclosed area. 
- If `strict_mode` is set to True, a `ValueError` will be raised if there are errors in the [properties.turn](../03-map-building/01-format.md#linestring-type-field-description) annotations.
