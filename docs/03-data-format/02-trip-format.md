# Trip Format

In MOSS, trip is composed of `Person`s. Below is an introduction to the attributes of `Person`.

## Transport Attribute

### Vehicle

| field: data.vehicle_attribute         | type                 | description                                                  |
| ------------------------------------- | -------------------- | ------------------------------------------------------------ |
| lane_change_length                    | float64              | distance required to complete lane change: m                 |
| min_gap                               | float64              | the minimum distance between the vehicle and the vehicle in front: m |
| length                                | float64              | length: m                                                    |
| width                                 | float64              | width: m                                                     |
| max_speed                             | float64              | max speed: m/s                                               |
| max_acceleration                      | float64              | max acceleration: m/s^2 (positive value)                     |
| max_braking_acceleration              | float64              | max deceleration: m/s^2 (negative value)                     |
| usual_acceleration                    | float64              | usual acceleration: m/s^2 (positive value), required to be less than the max acceleration    |
| usual_braking_acceleration            | float64              | usual deceleration: m/s^2 (negative value), required to be greater than the max deceleration |
| headway                               | float64              | safe time headway: m/s^2 (positive value)                    |
| lane_max_speed_recognition_deviation  | float64              | the deviation of the vehicle's recognition of lane max speed, must in range (0,1) |
| emission_attribute                    | EmissionAttribute    | the emission parameters of this vehicle                      |

#### EmissionAttribute

| field: vehicle_attribute.emission_attribute      | type                     | description                                                                             |
| ------------------------------------------------ | ------------------------ | --------------------------------------------------------------------------------------- |
| weight                                           | float64                  | weight of this vehicle: kg                                                              |
| type                                             | VehicleEngineType        | engine type of this vehicle, 0: unspecified; 1: gasoline vehicle; 2: electric vehicle   |
| coefficient_drag                                 | float64                  | drag coefficient of the vehicle                                                         |
| lambda_s                                         | float64                  | pavement friction coefficient                                                           |
| frontal_area                                     | float64                  | frontal area: m^2                                                                       |
| fuel_efficiency                                  | VehicleEngineEfficiency  | Optional, fuel vehicle efficiency                                                       |
| electric_efficiency                              | VehicleEngineEfficiency  | Optional, electric vehicle efficiency                                                   |

##### VehicleEngineEfficiency

| message types            | field                                 | type                      | description                                                                         |
| ------------------------ | ------------------------------------- | ------------------------- | ----------------------------------------------------------------------------------- |
| VehicleEngineEfficiency  | energy_conversion_efficiency          | float64                   | the energy conversion efficiency: `E_{vehicle consumed} / E_{fuel or electricity}`  |

### Bike

| field: data.bike_attribute | type    | description                                    |
| -------------------------- | ------- | ---------------------------------------------- |
| speed                      | float64 | moving speed of this person if he rides a bike |

### Walking

| field: data.pedestrian_attribute | type    | description                             |
| -------------------------------- | ------- | --------------------------------------- |
| speed                            | float64 | moving speed of this person if he walks |

## Schedules Attribute

### Position

| Position types        | field   | type    | description                                                  |
| --------------------- | ------- | ------- | ------------------------------------------------------------ |
| Position.LanePosition | lane_id | int     | LaneID                                                       |
|                       | s       | float64 | s is the distance from the point on the lane to the starting point of the lane |
| Position.AoiPosition  | aoi_id  | int     | AOI ID                                                       |
|                       | poi_id  | int     | POI ID, needs to be a sub-poi of aoi_id, otherwise the value is invalid |

### Journey

| message types       | field            | type                      | description                                                  |
| ------------------- | ---------------- | ------------------------- | ------------------------------------------------------------ |
| DrivingJourneyBody  | road_ids         | list[int]                 | road sequence from origin to destination                     |
|                     | eta              | float64                   | estimation time of arrival                                   |
| WalkingRouteSegment | lane_id          | int                       | Lane ID                                                      |
|                     | moving_direction | MovingDirection           | moving direction; 0: unspecified; 1: in the same direction as the positive lane direction; 2: in the opposite direction as the positive lane direction. |
| WalkingJourneyBody  | route            | list[WalkingRouteSegment] | the (Lane+direction) sequence from the origin to destination |
|                     | eta              | float64                   | estimation time of arrival                                   |
| Journey             | type             | JourneyType               | 0: unspecified; 1: driving; 2: walking                       |
|                     | driving          | DrivingJourneyBody        | Optional. routing results for driving journey                |
|                     | walking          | WalkingJourneyBody        | Optional. routing results for walking journey                |

### Trip

| field          | type     | description                                                  |
| -------------- | -------- | ------------------------------------------------------------ |
| mode           | TripMode | trip mode, 0: unspecified; 1: walking only; 2: driving only; 5: Riding bikes if avaible, otherwise walking. |
| end            | Position | destination                                                  |
| departure_time | float64  | expected departure time (in seconds), optional               |
| wait_time      | float64  | the expected waiting time (in seconds), if departure_time is empty, wait_time defaults to 0, optional |
| arrival_time   | float64  | expected arrival time (in seconds), optional                 |
| activity       | string   | the activity name of the destination for this trip           |
| routes         | Journey  | pre calculated routing results                               |

### Schedule

| field          | type       | description                                                  |
| -------------- | ---------- | ------------------------------------------------------------ |
| trips          | list[Trip] | list of trips                                                |
| loop_count     | int        | the number of times trips are executed, where 0 represents infinite loops and greater than 0 represents how many times they are executed |
| departure_time | float64    | expected departure time (in seconds), optional. FAQ: Q1-What would happen if both the Schedule and the departuretime of the first Trip were specified simultaneously?  A1-only depend on the departuretime of Trip. Q2-What would happen if both the Schedule and the first Trip were specified with wait_time=10 at the same time? A2-the waiting time is 10+10=20 |
| wait_time      | float64    | expected waiting time (in seconds), if departure_time is empty, wait_time defaults to 0. |

## Main Attribute

| field                                     | type                 | description                                                  |
| ----------------------------------------- | -------------------- | ------------------------------------------------------------ |
| data.home                                 | Position             | initial position                                             |
| data.schedules                            | list[Schedule]       | initial schedules                                            |
| data.labels                               | dict[string, string] | [can be empty] additional tags (e.g. repair vehicle type -> power grid) |

## Example

```json
{
  "class": "person",
  "data": {
    "attribute": {},
    "home": {
      "lane_position": {
        "lane_id": 130104,
        "s": 115.71712716462363
      }
    },
    "schedules": [
      {
        "trips": [
          {
            "mode": 2,
            "end": {
              "lane_position": {
                "lane_id": 22867,
                "s": 57.59639027707855
              }
            },
            "activity": "education",
            "routes": [
              {
                "type": 1,
                "driving": {
                  "road_ids": [
                    200018684,
                    200007666,
                    200011019,
                    200000708,
                    200000709,
                    200000710,
                    200011018
                  ],
                  "eta": 994.2598904793631
                }
              }
            ]
          }
        ],
        "loop_count": 1,
        "departure_time": 31793.10010598981
      }
    ],
    "vehicle_attribute": {
      "lane_change_length": 10,
      "min_gap": 1,
      "headway":1.5,
      "length": 5,
      "width": 2,
      "max_speed": 41.666666666666664,
      "max_acceleration": 3,
      "max_braking_acceleration": -10,
      "usual_acceleration": 2,
      "usual_braking_acceleration": -4.5,
      "model": "normal",
      "lane_max_speed_recognition_deviation": 1,
      "emission_attribute" {
        "weight": 18000,
        "type": 1,
        "coefficient_drag": 0.251,
        "lambda_s": 0.29,
        "frontal_area": 2.52,
        "fuel_efficiency": {
          "energy_conversion_efficiency": 0.013230000000000002
        }
}
    },
    "bike_attribute": {
      "speed": 5
    },
    "pedestrian_attribute": {
      "speed": 1.34
    },
    "id": 0,
    "labels": {}
  }
}
```
