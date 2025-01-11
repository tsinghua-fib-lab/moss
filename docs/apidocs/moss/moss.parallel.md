# {py:mod}`moss.parallel`

```{py:module} moss.parallel
```

```{autodoc2-docstring} moss.parallel
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`ParallelEngine <moss.parallel.ParallelEngine>`
  - ```{autodoc2-docstring} moss.parallel.ParallelEngine
    :summary:
    ```
* - {py:obj}`ParallelManager <moss.parallel.ParallelManager>`
  - ```{autodoc2-docstring} moss.parallel.ParallelManager
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`_force_exit <moss.parallel._force_exit>`
  - ```{autodoc2-docstring} moss.parallel._force_exit
    :summary:
    ```
* - {py:obj}`_instance <moss.parallel._instance>`
  - ```{autodoc2-docstring} moss.parallel._instance
    :summary:
    ```
* - {py:obj}`_proxy_init <moss.parallel._proxy_init>`
  - ```{autodoc2-docstring} moss.parallel._proxy_init
    :summary:
    ```
* - {py:obj}`_proxy_method <moss.parallel._proxy_method>`
  - ```{autodoc2-docstring} moss.parallel._proxy_method
    :summary:
    ```
* - {py:obj}`_proxy_property <moss.parallel._proxy_property>`
  - ```{autodoc2-docstring} moss.parallel._proxy_property
    :summary:
    ```
* - {py:obj}`_proxy_method_batch <moss.parallel._proxy_method_batch>`
  - ```{autodoc2-docstring} moss.parallel._proxy_method_batch
    :summary:
    ```
* - {py:obj}`_proxy_property_batch <moss.parallel._proxy_property_batch>`
  - ```{autodoc2-docstring} moss.parallel._proxy_property_batch
    :summary:
    ```
* - {py:obj}`_init <moss.parallel._init>`
  - ```{autodoc2-docstring} moss.parallel._init
    :summary:
    ```
````

### Data

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`__all__ <moss.parallel.__all__>`
  - ```{autodoc2-docstring} moss.parallel.__all__
    :summary:
    ```
````

### API

````{py:data} __all__
:canonical: moss.parallel.__all__
:value: >
   ['ParallelEngine', 'ParallelManager']

```{autodoc2-docstring} moss.parallel.__all__
```

````

`````{py:class} ParallelEngine(name: str, map_file: str, person_file: str, start_step: int = 0, step_interval: float = 1, seed: int = 43, verbose_level=Verbosity.NO_OUTPUT, person_limit: int = -1, junction_yellow_time: float = 0, phase_pressure_coeff: float = 1.5, speed_stat_interval: int = 0, output_dir: str = '', out_xmin: float = -inf, out_ymin: float = -inf, out_xmax: float = inf, out_ymax: float = inf, device: int = 0)
:canonical: moss.parallel.ParallelEngine

Bases: {py:obj}`moss.engine.Engine`

```{autodoc2-docstring} moss.parallel.ParallelEngine
```

```{rubric} Initialization
```

```{autodoc2-docstring} moss.parallel.ParallelEngine.__init__
```

````{py:method} init()
:canonical: moss.parallel.ParallelEngine.init

```{autodoc2-docstring} moss.parallel.ParallelEngine.init
```

````

`````

`````{py:class} ParallelManager(engines: typing.List[moss.parallel.ParallelEngine])
:canonical: moss.parallel.ParallelManager

Bases: {py:obj}`moss.engine.Engine`

```{autodoc2-docstring} moss.parallel.ParallelManager
```

```{rubric} Initialization
```

```{autodoc2-docstring} moss.parallel.ParallelManager.__init__
```

````{py:method} __getitem__(i) -> moss.parallel.ParallelEngine
:canonical: moss.parallel.ParallelManager.__getitem__

```{autodoc2-docstring} moss.parallel.ParallelManager.__getitem__
```

````

````{py:method} __len__()
:canonical: moss.parallel.ParallelManager.__len__

```{autodoc2-docstring} moss.parallel.ParallelManager.__len__
```

````

````{py:method} execute(func: typing.Callable[[int, moss.engine.Engine], typing.Any]) -> typing.List[typing.Any]
:canonical: moss.parallel.ParallelManager.execute

```{autodoc2-docstring} moss.parallel.ParallelManager.execute
```

````

`````

````{py:function} _force_exit()
:canonical: moss.parallel._force_exit

```{autodoc2-docstring} moss.parallel._force_exit
```
````

````{py:function} _instance(args, kwds, q_in: queue.SimpleQueue, q_out: queue.SimpleQueue)
:canonical: moss.parallel._instance

```{autodoc2-docstring} moss.parallel._instance
```
````

````{py:function} _proxy_init(self, *args, **kwds)
:canonical: moss.parallel._proxy_init

```{autodoc2-docstring} moss.parallel._proxy_init
```
````

````{py:function} _proxy_method(self, name, *args, **kwds)
:canonical: moss.parallel._proxy_method

```{autodoc2-docstring} moss.parallel._proxy_method
```
````

````{py:function} _proxy_property(name, self, *args, **kwds)
:canonical: moss.parallel._proxy_property

```{autodoc2-docstring} moss.parallel._proxy_property
```
````

````{py:function} _proxy_method_batch(self, name, *args, **kwds)
:canonical: moss.parallel._proxy_method_batch

```{autodoc2-docstring} moss.parallel._proxy_method_batch
```
````

````{py:function} _proxy_property_batch(name, self, *args, **kwds)
:canonical: moss.parallel._proxy_property_batch

```{autodoc2-docstring} moss.parallel._proxy_property_batch
```
````

````{py:function} _init()
:canonical: moss.parallel._init

```{autodoc2-docstring} moss.parallel._init
```
````
