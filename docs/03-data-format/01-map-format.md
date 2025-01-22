# Map Format

MOSS designed a two-level road network data structure.
## Intermediate GeoJSON format
The first level uses the ([GeoJSON](https://geojson.org/)) standard, which can be readily converted from publicly available data sources.
### RoadNet

RoadNet data consists of two types of geometric elements below.

1. `LineString`, corresponding to `road` in map
2. `MultiPoint`, corresponding to `junction` in map

#### `LineString` type field description

Example

```python
{
    "type": "Feature",
    "id": 25409666,
    "geometry": {
        "type": "LineString",
        "coordinates": [
            [
                113.902208,
                22.590202
            ],
            [
                113.894869,
                22.59406
            ]
        ]
    },
    "properties": {
        "id": 25409666,
        "lanes": 3,
        "highway": "motorway_link",
        "max_speed": 8.333333333333334,
        "name": "",
        "turn": ["ALS","S","SR"], # Around, Left, Staright | Staright | Staright, Right
        "width": 3.2
    }
}
```

| field                                  | type                | description                                                  |
| -------------------------------------- | ------------------- | ------------------------------------------------------------ |
| geometry.coordinates                   | list[list[float64]] | The latitude and longitude coordinates of the `road` in WGS84 format |
| properties.id                          | int                 | The original id of the `road`, which is usually different from the id in the generated map |
| properties.lanes                       | int                 | The number of `lanes` on this `road`                         |
| properties.highway                     | string              | Road level from OpenStreetMap (https://wiki.openstreetmap.org/wiki/Key:highway) |
| properties.max_speed                   | float64             | The speed limit on this `road` in meters per second          |
| properties.name                        | string              | Name of this `road`                                          |
| properties.turn                        | list[string]        | Optional field: A string representing the allowed turn types for each `lane`, in order from left to right along the direction of the `road`. A: Left turn, S: Straight, L: Right turn. A `lane` can have multiple turn types allowed, e.g. "SL" means the `lane` can be used for either a straight or a left turn. Allowable turn types are: A (left turn), S (straight), L (right turn) |
| properties.width, properties.lanewidth | float64             | Width (in meters) of each `lane` on this `road`              |
| properties.walk_lane_width             | float64             | Optional field: Walk lane width (in meters) of each `lane` on this `road` |
| properties.walk_lane_offset            | float64             | Optional field: The default offset between sidewalk and the `lane` centerlines is the average between sidewalk and lane width, this offset is adjusted from the original centerlines, so the real offset will be: (width+walk_lane_width)*0.5+walk_lane_offset (m) |

#### `MultiPoint` type field description

Example

```python
{
  "type": "Feature",
    "id": 50000000013,
    "geometry": {
      "type": "MultiPoint",
      "coordinates": [
          [
            113.906995,
            22.552684
          ]
        ]
  },
      "properties": {
        "id": 50000000013,
        "in_ways": [
          974379908
        ],
        "out_ways": [
          25436150,
          953806719
        ]
  }
}
```

| field                | type                | description                                                  |
| -------------------- | ------------------- | ------------------------------------------------------------ |
| geometry.coordinates | list[list[float64]] | The latitude and longitude coordinates of the `junction` in WGS84 format |
| properties.id        | int                 | The original id of the `junction`, which is usually different from the id in the generated map |
| properties.in_ways   | list[int]           | The original ids of all `roads` entering this `junction`     |
| properties.out_ways  | list[int]           | The original ids of all `roads` exiting this `junction`      |

### AOIs

| field                | type                      | description                                                  |
| -------------------- | ------------------------- | ------------------------------------------------------------ |
| geometry.coordinates | list[list[list[float64]]] | The latitude and longitude coordinates of the `AOI` exterior in WGS84 format |
| properties.id        | int                       | The original id of the `AOI`, which is usually different from the id in the generated map |
| properties.osm_tags  | list[dict]                | The tags corresponding to the OSM (OpenStreetMap) data for the Area of Interest (AOI) may have a length greater than 1, as the AOI could be a result of merging multiple smaller AOIs. Currently, only five fields within the tags are processed:<br/>1. `landuse`, `leisure`, `amenity`, `building`: These are used to determine the land use type of the AOI. For a simple manual annotation of the AOI type, choose from the following options:<br/>   - `"landuse":"residential"` - Residential land,<br/>   - `"landuse":"education"` - Educational and research land,<br/>   - `"landuse":"commercial"` - Commercial and service facilities land,<br/>   - `"landuse":"industrial"` - Industrial land,<br/>   - `"building":"government"` - Administrative and office land,<br/>   - `"landuse":"other"` - Other land uses<br/>2. `name`: This is used to obtain the name of the AOI.<br/><br/>For specific values of these fields and their corresponding meanings, please refer to the OpenStreetMap Wiki at https://wiki.openstreetmap.org/wiki/Key:{key_name}, replacing `{key_name}` with the respective tag key (e.g., 'landuse', 'leisure', etc.). |

Example

```python
{
      "type": "Feature",
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [
              31.129645,
              29.976933
            ],
            [
              31.130864,
              29.976937
            ],
            [
              31.131819,
              29.97694
            ],
            [
              31.131828,
              29.975046
            ],
            [
              31.129654,
              29.975039
            ]
          ]
        ]
      },
      "properties": {
        "id": 0,
        "osm_tags": [
          {
            "landuse":"retail",
            "name:en": "Pyramid of Khafre",
            "name:zh": "卡夫拉金字塔",
          }
        ]
      }
}
```



### POIs

| field                | type          | description                                                  |
| -------------------- | ------------- | ------------------------------------------------------------ |
| geometry.coordinates | list[float64] | The latitude and longitude coordinates of the `POI` in WGS84 format |
| properties.id        | int           | The original id of the `POI`, which is usually different from the id in the generated map |
| properties.name      | string        | Name of this `POI`                                           |
| properties.catg      | string        | Category code, defined in (https://lbs.qq.com/service/webService/webServiceGuide/webServiceAppendix) |

## Protobuf Map format
The second level is represented in ([Protobuf](https://protobuf.dev/)) format, a compact binary format suitable for computer usage.

### Header

Header contains meta data of this map

```python
{
  "class": "header",
  "data": {
    "name": "hangzhou",
    "date": "Fri Aug 19 15:12:01 2023",
    "north": 20047.15060441926, 
    "south": -32338.87769681991,
    "west": -18106.391120057444,
    "east": 23241.800579354865,
    "projection": "+proj=tmerc +lat_0=30.2435 +lon_0=120.1648"
  }
}
```

| field                                       | type   | description                        |
| ------------------------------------------- | ------ | ---------------------------------- |
| data.name                                   | string | Name of this map                   |
| data.date                                   | string | The time when this map was created |
| data.north, data.south, data.west,data.east | double | Bounding box of this map           |
| data.taz_x_step, data.taz_y_step            | double | Step size of the TAZ in the x and y direction|
| data.projection                             | string | Proj.4 projection string           |

### Lane

```python
{
  "class": "lane",
  "data": {
    "id": 0,
    "type": 1,                 
    "turn": 1,                     
    "max_speed": 11.11111111111111,  
    "length": 405.77050412705626,   
    "width": 3.2,                  
    "center_line": {                
      "nodes": [
        {
          "x": -333.409877363847,
          "y": 3892.689241038861
        },
        {
          "x": -270.3333568115683,
          "y": 3896.3202265304885
        },
        {
          "x": -150.42210531337474,
          "y": 3900.0537339189864
        },
      ]
    },
    "predecessors": [   
      {
        "id": 89785,    
        "type": 2      
      },
      {
        "id": 89788,
        "type": 2
      }
    ],
    "successors": [
      {
        "id": 89790,
        "type": 1
      },
      {
        "id": 89791,
        "type": 1
      }
    ],
    "left_lane_ids": [],    
    "right_lane_ids": [],   
    "parent_id": 200000000, 
    "overlaps": [],         
    "aoi_ids": [          
      500001796,
      500010169,
      500010170,
    ]
  }
}
```

| field                                   | type                    | description                                                  |
| --------------------------------------- | ----------------------- | ------------------------------------------------------------ |
| data.id                                 | int                     | Distinct id of this `lane`                                   |
| data.type                               | int                     | Type of this `lane`. 0: Unspecified, 1: Driving, 2: Walking. |
| data.turn                               | int                     | Turn type of this `lane`. 0: Unspecified, 1: Straight, 2: Left, 3: Right, 4: Around. |
| data.max_speed                          | float                   | The speed limit on this `lane` in meters per second          |
| data.length                             | float                   | The length of this `lane`                                    |
| data.width                              | float                   | The width of this `lane`                                     |
| data.center_line                        | dict[string,list[dict]] | Centerline geometry points of this `lane`                    |
| data.predecessors, data.successors      | list[dict]              | Predecessor/successor `lanes` of this `lane` , including id of connected `lane` and connection type, 1: connected to the head of other lane, 2: connected to the tail of other lane. |
| data.left_lane_ids, data.right_lane_ids | list[int]               | Left/Right adjacent `lane` id in the same `road` (arranged from near to far) |
| data.parent_id                          | int                     | The `road` id or `junction` id to which it belongs           |
| data.overlaps                           | list[dict]              | Overlap points between lanes (valid only within a `junction`) |
| data.aoi_ids                            | list[int]               | All `AOI` ids  connected to this `lane`                      |

### Road

```python
{
  "class": "road",
  "data": {
    "id": 200000000,
    "lane_ids": [
      0,
      63893
    ],
  }
}
```

| field         | type      | description                                                  |
| ------------- | --------- | ------------------------------------------------------------ |
| data.id       | int       | Distinct id of this `road`                                   |
| data.lane_ids | list[int] | The ids of all `lanes` that make up the road, arranged from left to right along the direction of the road. Driving lane first, then walking lanes. |

### Junction

```python
{
  "class": "junction",
  "data": {
    "id": 300000000,
    "lane_ids": [  
      89785,
      89786,
      89787,
      89788,
      89789,
      180968,
    ]
  }
}
```

| field              | type           | description                                            |
| ------------------ | -------------- | ------------------------------------------------------ |
| data.id            | int            | Distinct id of this `junction`                         |
| data.lane_ids      | list[int]      | The ids of all `lanes` that belonging to the junction. |
| data.phases        | AvailablePhase | Distinct id of this `junction`                         |
| data.fixed_program | TrafficLight   | The ids of all `lanes` that belonging to the junction. |

#### Traffic Light Format

| message types       | field            | type                      | description                                                                  |
| ------------------- | ---------------- | ------------------------- | ---------------------------------------------------------------------------- |
| LightState          |                  | enum                      | 0: unspecified; 1: red light; 2: green light 3: yellow light                 |
| AvailablePhase      | states           | list[LightState]          | the feasible phase for max pressure algorithm, consisting of lighting control situation for each lane in the junction |
| TrafficLight        | junction_id      | int                       | the unique junction ID                                                       |
|                     | phases           | list[Phase]               | the lighting control situation of each lane in this phase, and the lane corresponds one-to-one with junction.lane_ids |
| Phase               | duration         | float                     | the phase duration time (seconds)                                            |
|                     | states           | list[LightState]          | the lighting control situation of each lane in this phase, and the lane corresponds one-to-one with junction.lane_ids |
### AOI

```python
{
  "class": "aoi",
  "data": {
    "id": 500000000,
    "positions": [
      {
        "x": 871.8707642603008,
        "y": 3971.670224670372
      },
      {
        "x": 1145.657731407196,
        "y": 4268.304979330834
      },
      {
        "x": 1357.7498147156743,
        "y": 4100.025863656549
      },
      {
        "x": 871.8707642603008,
        "y": 3971.670224670372
      }
    ],
    "area": 11961.66874272599,   
    "driving_positions": [     
      {
        "lane_id": 9990,
        "s": 222.80558616295468
      }
    ],
    "driving_gates": [         
      {
        "x": 875.0335524878568,
        "y": 3972.2371491341605
      }
    ],
    "walking_positions": [  
      {
        "lane_id": 67871,
        "s": 223.98105794200876
      }
    ],
    "walking_gates": [     
      {
        "x": 878.1963407154128,
        "y": 3972.8040735979494
      }
    ],
    "poi_ids":[],
    "name": "国家大剧院",
    "urban_land_use": "A2"
  }
}
```

| field                                          | type       | description                                                  |
| ---------------------------------------------- | ---------- | ------------------------------------------------------------ |
| data.id                                        | int        | Distinct id of this `AOI`                                    |
| data.area                                      | float      | Area of this `AOI`                                           |
| data.positions                                 | list[dict] | Exterior geometry points of this `AOI`                       |
| data.driving_positions, data.walking_positions | list[dict] | The position of the connection point on the `lane` (represented by `lane` id and `s`, where `s` is the distance from the starting point of `lane` to the target point along `lane`) |
| data.driving_gates, data.walking_gates         | list[dict] | The entrances and exits connected to various lanes on the `AOI` boundary, corresponding one-to-one with the order of `driving_positions`/`walking_positions` |
| data.poi_ids                                   | list[int]  | All `POI` ids contained in this `AOI`                        |
| data.name                                      | string     | Name of this `AOI`                                           |
| data.urban_land_use                            | string     | Land use type, according to https://www.planning.org.cn/law/uploads/2013/1383993139.pdf |

### POI

```python
{
  "class": "poi",
  "data": {
    "id": 700000000,
    "name": "悦庭迷你",
    "category": "111000",
    "position": { 
      "x": 9198.956125039857,
      "y": -15926.569325368913
    },
    "aoi_id": 500004914,
  }
}
```

| field          | type               | description                                                  |
| -------------- | ------------------ | ------------------------------------------------------------ |
| data.id        | int                | Distinct id of this `POI`                                    |
| data.name      | string             | Name of this `POI`                                           |
| data.category  | string             | Category code, defined in (https://lbs.qq.com/service/webService/webServiceGuide/webServiceAppendix) |
| data.positions | dict[string,float] | Point coordinate of this `POI`                               |
| aoi_id         | int                | The id of the `AOI`  which contained this `POI`              |
