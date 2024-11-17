import os
import threading
import traceback
from collections import defaultdict
from functools import partial, partialmethod
from inspect import signature
from queue import SimpleQueue as Queue
from typing import Any, Callable, List

from .engine import Engine

__all__ = ["ParallelEngine", "ParallelManager"]


class ParallelEngine(Engine):
    """
    Moss Engine for parallel simulation
    """

    def init(self):
        self._q_in.put(None)
        self._q_out.get()


class ParallelManager(Engine):
    """
    Helper class for managing multiple `ParallelEngine`s.

    Method calls on this object will be broadcast to all managed engines
    """

    def __init__(self, engines: List[ParallelEngine]):
        if not all(isinstance(e, ParallelEngine) for e in engines):
            raise NotImplementedError(
                "ParallelManager can only be used with ParallelEngine!"
            )
        self.engines = engines
        self.es = es = defaultdict(list)
        self._input = None
        self._exec = None
        self._output = None
        for i, e in enumerate(engines):
            es[e.device].append([i, e])
        self._b1 = threading.Barrier(len(es) + 1)
        self._b2 = threading.Barrier(len(es) + 1)
        _b_init = threading.Barrier(len(es) + 1)

        def worker(es):
            # 初始化
            for i, e in es:
                e.init()
            _b_init.wait()
            # 批量调用
            while True:
                self._b1.wait()
                for i, e in es:
                    if self._exec is None:
                        e._q_in.put(self._input)
                        self._output[i] = e._q_out.get()
                    else:
                        self._output[i] = self._exec(i, e)
                self._b2.wait()

        for i in es.values():
            threading.Thread(target=worker, args=(i,), daemon=True).start()
        _b_init.wait()

    def __getitem__(self, i) -> ParallelEngine:
        return self.engines[i]

    def __len__(self):
        return len(self.engines)

    def execute(self, func: Callable[[int, Engine], Any]) -> List[Any]:
        """
        Run func(idx, eng) in parallel and return the results as a list
        """
        self._exec = func
        self._output = [None] * len(self.engines)
        self._b1.wait()
        self._b2.wait()
        self._exec = None
        ret = self._output
        self._output = None
        return ret


def _force_exit():
    os.killpg(0, 9)


def _instance(args, kwds, q_in: Queue, q_out: Queue):
    try:
        q_in.get()
        eng = Engine(*args, **kwds)
        q_out.put(None)
        while True:
            name, args, kwds = q_in.get()
            if name.startswith("@"):
                q_out.put(getattr(eng, name[1:]))
            else:
                q_out.put(getattr(eng, name)(*args, **kwds))
    except:
        print(traceback.format_exc(), flush=True)
        _force_exit()
        return


def _proxy_init(self, *args, **kwds):
    s = signature(Engine.__init__).bind(None, *args, **kwds)
    s.apply_defaults()
    self.device = s.arguments["device"]
    self._q_in = Queue()
    self._q_out = Queue()
    self.__proc = threading.Thread(
        target=_instance, args=(args, kwds, self._q_in, self._q_out), daemon=True
    )
    self.__proc.start()


def _proxy_method(self, name, *args, **kwds):
    signature(getattr(Engine, name)).bind(None, *args, **kwds)
    self._q_in.put((name, args, kwds))
    return self._q_out.get()


def _proxy_property(name, self, *args, **kwds):
    self._q_in.put(("@" + name, args, kwds))
    return self._q_out.get()


def _proxy_method_batch(self, name, *args, **kwds):
    signature(getattr(Engine, name)).bind(None, *args, **kwds)
    self._input = (name, args, kwds)
    self._output = [None] * len(self.engines)
    self._b1.wait()
    self._b2.wait()
    ret = self._output
    self._output = None
    return ret


def _proxy_property_batch(name, self, *args, **kwds):
    self._input = ("@" + name, args, kwds)
    self._output = [None] * len(self.engines)
    self._b1.wait()
    self._b2.wait()
    ret = self._output
    self._output = None
    return ret


def _init():
    os.setpgrp()
    _proxy_init.__signature__ = signature(Engine.__init__)
    ParallelEngine.__init__ = _proxy_init


def _init():
    os.setpgrp()
    _proxy_init.__signature__ = signature(Engine.__init__)
    ParallelEngine.__init__ = _proxy_init
    for name in dir(Engine):
        if name.startswith("_"):
            continue
        f = getattr(Engine, name)
        if isinstance(f, property):
            setattr(ParallelEngine, name, property(partial(_proxy_property, name)))
            setattr(
                ParallelManager, name, property(partial(_proxy_property_batch, name))
            )
        elif callable(f):
            setattr(ParallelEngine, name, partialmethod(_proxy_method, name))
            setattr(ParallelManager, name, partialmethod(_proxy_method_batch, name))


_init()
