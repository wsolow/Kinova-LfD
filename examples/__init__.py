# Sets backwards compatibility with protobuf package
# os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import collections
import collections.abc
import os

# Python 3.10+ removed these aliases from collections; old protobuf versions
# (pulled in by kortex_api) still expect them there.
for name in ("MutableMapping", "Mapping", "MutableSequence", "Sequence", "Iterable"):
    if not hasattr(collections, name):
        setattr(collections, name, getattr(collections.abc, name))
