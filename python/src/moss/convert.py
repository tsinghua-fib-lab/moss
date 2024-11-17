from collections import defaultdict
from re import T
from typing import TypeVar

import stringcase
from google.protobuf import json_format
from google.protobuf.message import Message
from pymongo.collection import Collection

__all__ = ["pb2json", "pb2dict", "pb2coll", "json2pb", "dict2pb", "coll2pb", "save_pb"]


def pb2json(pb: Message):
    """
    Convert a protobuf message to a JSON string.

    Args:
    - pb: The protobuf message to be converted.

    Returns:
    - The JSON string.
    """
    return json_format.MessageToJson(
        pb,
        including_default_value_fields=True,
        preserving_proto_field_name=True,
        use_integers_for_enums=True,
    )


def pb2dict(pb: Message):
    """
    Convert a protobuf message to a Python dictionary.

    Args:
    - pb: The protobuf message to be converted.

    Returns:
    - The Python dict.
    """
    return json_format.MessageToDict(
        pb,
        including_default_value_fields=True,
        preserving_proto_field_name=True,
        use_integers_for_enums=True,
    )


def pb2coll(
    pb: Message, coll: Collection, insert_chunk_size: int = 0, drop: bool = False
):
    """
    Convert a protobuf message to a MongoDB collection.

    Args:
    - pb: The protobuf message to be converted.
    - coll: The MongoDB collection to be inserted.
    - insert_chunk_size: The chunk size for inserting the collection. If it is 0, insert all the data at once.
    - drop: Drop the MongoDB collection or not. True for drop, False for not.
    """
    # Check all fields contained in pb, requiring fields to be message type or repeated type
    # The class field of each data in coll corresponds to the type name of the message type
    # The data field of each data in coll corresponds to the message type content
    if drop:
        coll.drop()
    jsons = []
    for field in pb.DESCRIPTOR.fields:
        class_name = stringcase.spinalcase(field.message_type.name)
        if field.label == field.LABEL_REPEATED:
            for pb_field in getattr(pb, field.name):
                data = pb2dict(pb_field)
                jsons.append({"class": class_name, "data": data})
        else:
            data = pb2dict(getattr(pb, field.name))
            jsons.append({"class": class_name, "data": data})
    if insert_chunk_size > 0:
        for i in range(0, len(jsons), insert_chunk_size):
            coll.insert_many(jsons[i : i + insert_chunk_size])
    else:
        coll.insert_many(jsons)


# Generic
T = TypeVar("T", bound=Message)


def json2pb(json: str, pb: T) -> T:
    """
    Convert a JSON string to a protobuf message.

    Args:
    - json: The JSON string to be converted.
    - pb: The protobuf message to be filled.

    Returns:
    - The protobuf message.
    """
    return json_format.Parse(json, pb, ignore_unknown_fields=True)


def dict2pb(d: dict, pb: T) -> T:
    """
    Convert a Python dictionary to a protobuf message.

    Args:
    - d: The Python dict to be converted.
    - pb: The protobuf message to be filled.

    Returns:
    - The protobuf message.
    """
    return json_format.ParseDict(d, pb, ignore_unknown_fields=True)


def coll2pb(coll: Collection, pb: T) -> T:
    """
    Convert a MongoDB collection to a protobuf message.

    Args:
    - coll: The MongoDB collection to be converted.
    - pb: The protobuf message to be filled.

    Returns:
    - The protobuf message.
    """

    # Check all fields contained in pb, requiring fields to be message type or repeated type
    # The class field of each data in coll corresponds to the type name of the message type
    # The data field of each data in coll corresponds to the message type content
    bsons = defaultdict(list)
    for one in coll.find():
        if "class" not in one or "data" not in one:
            raise ValueError(
                "The collection documents must contain 'class' and 'data' fields."
            )
        class_name = one["class"]
        data = one["data"]
        bsons[class_name].append(data)
    # Group bson according to class field
    for field in pb.DESCRIPTOR.fields:
        class_name = stringcase.spinalcase(field.message_type.name)
        if class_name not in bsons:
            continue
        if field.label == field.LABEL_REPEATED:
            for data in bsons[class_name]:
                pb_field = getattr(pb, field.name).add()
                dict2pb(data, pb_field)
        else:
            data = bsons[class_name][0]
            dict2pb(data, getattr(pb, field.name))
    return pb


def save_pb(pb, path: str):
    with open(path, "wb") as f:
        f.write(pb.SerializeToString())
