# Trip Generator

In this section, we will introduce you to the advanced features of the `Trip Generator`.

## Person Template Assignment

To facilitate user-friendly customization of `Person` attributes for trip generation, the `Trip Generator` provides `template_func` input parameter, which is a function, who generates a copy of a `Person` object with specified attributes each time it is called.
Of course, users can customize their own `template_func`. We also provide several commonly used template_func options.

### `default_vehicle_template_generator`
This function returns a `Person` with fixed parameters each time it is called.

```python
Person(
        attribute=PersonAttribute(),
        vehicle_attribute=VehicleAttribute(
            length=5,
            width=2,
            max_speed=150 / 3.6,
            max_acceleration=3,
            max_braking_acceleration=-10,
            usual_acceleration=2,
            usual_braking_acceleration=-4.5,
            headway=1.5,
            lane_max_speed_recognition_deviation=1.0,
            lane_change_length=10,
            min_gap=1,
            emission_attribute=EmissionAttribute(
                weight=2100,
                type=VehicleEngineType.VEHICLE_ENGINE_TYPE_FUEL,
                coefficient_drag=0.251,
                lambda_s=0.29,
                frontal_area=2.52,
                fuel_efficiency=VehicleEngineEfficiency(
                    energy_conversion_efficiency=0.27 * 0.049
                ),
            ),
            model="normal",
        ),
        pedestrian_attribute=PedestrianAttribute(speed=1.34, model="normal"),
        bike_attribute=BikeAttribute(speed=5, model="normal"),
)

```
### `ProbabilisticTemplateGenerator`

This class initialize with specified discrete probabilities and ranges.

```python
from mosstool.trip.generator import ProbabilisticTemplateGenerator
pg = ProbabilisticTemplateGenerator(
    headway_values=[1.5, 2, 2.5], headway_probabilities=[1, 1, 1]
)
```

`ProbabilisticTemplateGenerator.template_generator` returns a `Person` with [vehicle attributes](../04-trip-generation/01-format.md#vehicle) distributed according to the initialization.

### `GaussianTemplateGenerator`

This class initialize with Gaussian distributions with specific mean and variance.

```python
from mosstool.trip.generator import GaussianTemplateGenerator
gg = GaussianTemplateGenerator(
    max_speed_mean=50,
    max_speed_std=10,
)
```

`GaussianTemplateGenerator.template_generator` returns a `Person` with [vehicle attributes](../04-trip-generation/01-format.md#vehicle) distributed according to the initialization.

### `UniformTemplateGenerator`

This class initialize with Uniform distributions with specific lower bound and higher bound.

```python
from mosstool.trip.generator import UniformTemplateGenerator
ug = UniformTemplateGenerator(
    max_speed_min=90,
    max_speed_max=110,
)
```

`UniformTemplateGenerator.template_generator` returns a `Person` with [vehicle attributes](../04-trip-generation/01-format.md#vehicle) distributed according to the initialization.

### `CalibratedTemplateGenerator`

This class initialize with specified discrete probabilities and ranges.

```python
from mosstool.trip.generator import CalibratedTemplateGenerator
cg = CalibratedTemplateGenerator()
```

`CalibratedTemplateGenerator.template_generator` returns a `Person` with [vehicle attributes](../04-trip-generation/01-format.md#vehicle) distributed according to the driving behavior parameters calibrated for Chinese drivers.

## O-D Matrix Based Trip Generation

### O-D matrix generation

We offer two methods for generating the O-D matrix: one is the classic gravity model, and the other is based on the Diffusion Model.

#### Gravity model

##### Initialize the model and Generate the O-D matrix

```python
import numpy as np
import geopandas as gpd
from mosstool.trip.generator import GravityGenerator
gravity_generator = GravityGenerator(
    Lambda=0.2,
    Alpha=0.5,
    Beta=0.5,
    Gamma=0.5
)
area = gpd.read_file("data/gravitygenerator/Beijing-shp/beijing.shp")
pops = np.load("data/gravitygenerator/worldpop.npy")[:, 0]
gravity_generator.load_area(area)
od_matrix = gravity_generator.generate(pops)
```

- `area` is `GeoDataFrame` data, containing the geometries that serve as the O-D regions.
- `pops` is a one-dimensional array containing the population numbers corresponding to each region in `area`.

#### AIGC

##### Initialize the model and Generate the O-D matrix

```python
import geopandas as gpd
from mosstool.trip.generator import AigcGenerator
from mosstool.type import Map, Persons
aigc_generator = AigcGenerator()
YOUR_ACCESS_TOKEN = "1"
aigc_generator.set_satetoken(YOUR_ACCESS_TOKEN)
area = gpd.read_file("data/gravitygenerator/Beijing-shp/beijing.shp")
aigc_generator.load_area(area)
od_matrix = aigc_generator.generate()
```

- Here we initialize the `aigc_generator` with specific `ACCESS_TOKEN`, which can be applied from [ArcGIS](https://www.arcgis.com/home/item.html?id=10df2279f9684e4a9f6a7f08febac2a9).
- `area` is `GeoDataFrame` data, containing the geometries that serve as the O-D regions.

### Generate trips with O-D matrix

In this step we utilize `generator.TripGenerator` to generated `Persons` with assigned movements between elements in the map.

#### Initialize the generator and Generate trips

```python
from mosstool.trip.generator import TripGenerator
tg = TripGenerator(
    m=m,
)
od_persons = tg.generate_persons(
        od_matrix=od_matrix,
        areas=area,
        agent_num=10000,
    )
pb = Persons(persons=od_persons)
with open("data/temp/persons.pb", "wb") as f:
    f.write(pb.SerializeToString())
```

`od_persons` is a list of `Person`. To get the specific movement trajectories of each `Person`, an additional [routing service](../02-get-started/03-build-your-simulation.md#fill-in-the-route-of-the-persons-all-schedules) still needs to be invoked.

#### Advanced usage on `generator.TripGenerator`

`generator.TripGenerator` provides various control parameters to enable the generation of trips that meet user requirements.

##### Person activity distribution

The default activity distribution is the `dict` below.

```python
HUMAN_MODE_STATS = {
    "HWH": 18.79,
    "HWH+": 20.03,
    "HW+WH": 13.76,
    "HWHWH": 1.09,
    "HSH": 3.1,
    "HSH+": 3.36,
    "HOH": 10.91,
    "HOH+": 11.45,
    "HWHWH+": 1.43,
    "HSOSH": 0.00,
}
```

- The `dict` represents the distribution of individuals' daily activities in percentages. Here, 'H' stands for 'Home', 'S' for 'Study', 'O' and '+' for 'Other', and 'W' for 'Work'. 
The keys are combinations of these letters indicating different patterns of activities throughout the day, while the values represent the percentage of people following each respective pattern. 
For example, "HWH" indicates going from Home to Work and then back to Home, which accounts for 18.79% of individuals. Similarly, "HSH+" denotes the sequence of moving from Home to Study and then to Other activities, representing 3.36% of people's daily routines. 
- To control the proportion of actions taken by generated Persons, you can provide a dictionary similar to `HUMAN_MODE_STATS` as an input parameter to `activity_distributions`. This dictionary allows you to specify the distribution of activities, where the keys represent sequences of daily activities and the values indicate the percentage of individuals engaging in each activity pattern.

##### Person departure time control

The `departure_time_curve` controls the distribution of travel times for generated `Persons`. 
It consists of a list of floats, where each float represents the probability of departure at a given time point. 
These time points are calculated as the index position of the float divided by the total length of the list, multiplied by 24 hours.

If this parameter is provided, the list **must be at least 24** elements long, ensuring a minimum resolution of one hour.

##### Traffic mode control

We control the proportion of transportation modes used by all generated `Persons` through a combination of travel costs and additional penalties associated with each mode of transportation.

- `cost with driving`: `driving_speed` for time cost calculation, `parking_fee` for money cost and extra time penalty `driving_penalty`.
- `cost with walking or bike`: `bike_speed` for time cost calculation and extra time penalty `bike_penalty`.

##### Person attribute assignment

`generator.TripGenerator` supports `Person` attributes assignment with input argument [template-func](#person-template-assignment).
