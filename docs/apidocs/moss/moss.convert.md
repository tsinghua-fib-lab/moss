# {py:mod}`moss.convert`

```{py:module} moss.convert
```

```{autodoc2-docstring} moss.convert
:allowtitles:
```

## Module Contents

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`pb2json <moss.convert.pb2json>`
  - ```{autodoc2-docstring} moss.convert.pb2json
    :summary:
    ```
* - {py:obj}`pb2dict <moss.convert.pb2dict>`
  - ```{autodoc2-docstring} moss.convert.pb2dict
    :summary:
    ```
* - {py:obj}`pb2coll <moss.convert.pb2coll>`
  - ```{autodoc2-docstring} moss.convert.pb2coll
    :summary:
    ```
* - {py:obj}`json2pb <moss.convert.json2pb>`
  - ```{autodoc2-docstring} moss.convert.json2pb
    :summary:
    ```
* - {py:obj}`dict2pb <moss.convert.dict2pb>`
  - ```{autodoc2-docstring} moss.convert.dict2pb
    :summary:
    ```
* - {py:obj}`coll2pb <moss.convert.coll2pb>`
  - ```{autodoc2-docstring} moss.convert.coll2pb
    :summary:
    ```
* - {py:obj}`save_pb <moss.convert.save_pb>`
  - ```{autodoc2-docstring} moss.convert.save_pb
    :summary:
    ```
````

### Data

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`__all__ <moss.convert.__all__>`
  - ```{autodoc2-docstring} moss.convert.__all__
    :summary:
    ```
* - {py:obj}`T <moss.convert.T>`
  - ```{autodoc2-docstring} moss.convert.T
    :summary:
    ```
````

### API

````{py:data} __all__
:canonical: moss.convert.__all__
:value: >
   ['pb2json', 'pb2dict', 'pb2coll', 'json2pb', 'dict2pb', 'coll2pb', 'save_pb']

```{autodoc2-docstring} moss.convert.__all__
```

````

````{py:function} pb2json(pb: google.protobuf.message.Message)
:canonical: moss.convert.pb2json

```{autodoc2-docstring} moss.convert.pb2json
```
````

````{py:function} pb2dict(pb: google.protobuf.message.Message)
:canonical: moss.convert.pb2dict

```{autodoc2-docstring} moss.convert.pb2dict
```
````

````{py:function} pb2coll(pb: google.protobuf.message.Message, coll: pymongo.collection.Collection, insert_chunk_size: int = 0, drop: bool = False)
:canonical: moss.convert.pb2coll

```{autodoc2-docstring} moss.convert.pb2coll
```
````

````{py:data} T
:canonical: moss.convert.T
:value: >
   'TypeVar(...)'

```{autodoc2-docstring} moss.convert.T
```

````

````{py:function} json2pb(json: str, pb: moss.convert.T) -> moss.convert.T
:canonical: moss.convert.json2pb

```{autodoc2-docstring} moss.convert.json2pb
```
````

````{py:function} dict2pb(d: dict, pb: moss.convert.T) -> moss.convert.T
:canonical: moss.convert.dict2pb

```{autodoc2-docstring} moss.convert.dict2pb
```
````

````{py:function} coll2pb(coll: pymongo.collection.Collection, pb: moss.convert.T) -> moss.convert.T
:canonical: moss.convert.coll2pb

```{autodoc2-docstring} moss.convert.coll2pb
```
````

````{py:function} save_pb(pb, path: str)
:canonical: moss.convert.save_pb

```{autodoc2-docstring} moss.convert.save_pb
```
````
