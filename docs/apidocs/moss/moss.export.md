# {py:mod}`moss.export`

```{py:module} moss.export
```

```{autodoc2-docstring} moss.export
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`StringIteratorIO <moss.export.StringIteratorIO>`
  -
* - {py:obj}`_CopyWriter <moss.export._CopyWriter>`
  - ```{autodoc2-docstring} moss.export._CopyWriter
    :summary:
    ```
* - {py:obj}`DBRecorder <moss.export.DBRecorder>`
  - ```{autodoc2-docstring} moss.export.DBRecorder
    :summary:
    ```
````

### Data

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`__all__ <moss.export.__all__>`
  - ```{autodoc2-docstring} moss.export.__all__
    :summary:
    ```
````

### API

````{py:data} __all__
:canonical: moss.export.__all__
:value: >
   ['DBRecorder']

```{autodoc2-docstring} moss.export.__all__
```

````

`````{py:class} StringIteratorIO(iter: typing.Iterator[str])
:canonical: moss.export.StringIteratorIO

Bases: {py:obj}`io.TextIOBase`

````{py:method} readable() -> bool
:canonical: moss.export.StringIteratorIO.readable

````

````{py:method} _read1(n: typing.Optional[int] = None) -> str
:canonical: moss.export.StringIteratorIO._read1

```{autodoc2-docstring} moss.export.StringIteratorIO._read1
```

````

````{py:method} read(n: typing.Optional[int] = None) -> str
:canonical: moss.export.StringIteratorIO.read

````

`````

`````{py:class} _CopyWriter(dsn: str, proj_str: str, carid2model: typing.Dict[int, str], pedid2model: typing.Dict[int, str])
:canonical: moss.export._CopyWriter

```{autodoc2-docstring} moss.export._CopyWriter
```

```{rubric} Initialization
```

```{autodoc2-docstring} moss.export._CopyWriter.__init__
```

````{py:method} awrite(table_name: str, step, persons, tls, roads)
:canonical: moss.export._CopyWriter.awrite
:async:

```{autodoc2-docstring} moss.export._CopyWriter.awrite
```

````

`````

`````{py:class} DBRecorder(eng: moss.engine.Engine, db_url: str, mongo_map: str, output_name: str, total_steps: int = 86400, use_tqdm: bool = False, max_unwaited: int = 16)
:canonical: moss.export.DBRecorder

```{autodoc2-docstring} moss.export.DBRecorder
```

```{rubric} Initialization
```

```{autodoc2-docstring} moss.export.DBRecorder.__init__
```

````{py:method} __del__()
:canonical: moss.export.DBRecorder.__del__

```{autodoc2-docstring} moss.export.DBRecorder.__del__
```

````

````{py:method} _init_table()
:canonical: moss.export.DBRecorder._init_table

```{autodoc2-docstring} moss.export.DBRecorder._init_table
```

````

````{py:method} record()
:canonical: moss.export.DBRecorder.record

```{autodoc2-docstring} moss.export.DBRecorder.record
```

````

````{py:method} flush()
:canonical: moss.export.DBRecorder.flush

```{autodoc2-docstring} moss.export.DBRecorder.flush
```

````

`````
