import asyncio
import atexit
import os
import subprocess
import sys
import threading
import time
from typing import List, Tuple

import grpc
from pycityproto.city.routing.v2.routing_pb2 import RouteType
from pycityproto.city.routing.v2.routing_service_pb2 import (
    GetDrivingCostsRequest, GetRouteRequest, SetDrivingCostsRequest)
from pycityproto.city.routing.v2.routing_service_pb2_grpc import \
    RoutingServiceStub


def _routing(proc, flag):
    for l in proc.stderr:
        # 初始化完成
        if 'server listening at' in l:
            flag[0] = True
            break
    proc.communicate()


class Router:
    def __init__(self, map_file: str, routing_bin: str, addr: str = 'localhost:52101', use_existing_router=False):
        """
        map_file:       path to the map file
        routing_bin:    path to the routing binary
        addr:           the address to be used for TCP socket
        """
        if not use_existing_router:
            proc = subprocess.Popen([
                os.path.abspath(routing_bin), '-map', os.path.abspath(map_file), '-listen', addr
            ], stderr=subprocess.PIPE, bufsize=1, encoding='utf8')
            atexit.register(os.kill, proc.pid, 15)
            # 初始化完成的标志位
            ok = [False]
            threading.Thread(target=_routing, args=(proc, ok), daemon=True).start()
            while not ok[0]:
                time.sleep(0.1)
        self.stub = RoutingServiceStub(grpc.insecure_channel(addr))
        self.addr = addr

    def get_route(self, start_lane: int, end_lane: int) -> List[int]:
        """
        Get the route from `start_lane` to `end_lane` in the form of a list of road ids 
        """
        req = GetRouteRequest()
        req.type = RouteType.ROUTE_TYPE_DRIVING
        req.start.lane_position.lane_id = start_lane
        req.end.lane_position.lane_id = end_lane
        res = self.stub.GetRoute(req)
        if len(res.journeys) != 1:
            return []
        return res.journeys[0].driving.road_ids

    async def get_route_async(self, start_end_lanes: List[Tuple[int, int]], callback=None, n_workers=8):
        """
        Get the route from `start_lane` to `end_lane` in the form of a list of road ids 
        """
        async with grpc.aio.insecure_channel(self.addr) as chan:
            stub = RoutingServiceStub(chan)
            l = []

            async def process(done):
                if callback:
                    for res in done:
                        res = await res
                        callback([] if len(res.journeys) != 1 else res.journeys[0].driving.road_ids)
            for start_lane, end_lane in start_end_lanes:
                req = GetRouteRequest()
                req.type = RouteType.ROUTE_TYPE_DRIVING
                req.start.lane_position.lane_id = start_lane
                req.end.lane_position.lane_id = end_lane
                l.append(stub.GetRoute(req))
                if len(l) >= n_workers:
                    done, l = await asyncio.wait(l, return_when='FIRST_COMPLETED')
                    await process(done)
                    l = list(l)
            await process(await asyncio.wait(l))

    def get_aoi_route(self, start_aoi: int, end_aoi: int) -> List[int]:
        """
        Get the route from `start_aoi` to `end_aoi` in the form of a list of road ids 
        """
        req = GetRouteRequest()
        req.type = RouteType.ROUTE_TYPE_DRIVING
        req.start.aoi_position.aoi_id = start_aoi
        req.end.aoi_position.aoi_id = end_aoi
        res = self.stub.GetRoute(req)
        if len(res.journeys) != 1:
            return []
        return res.journeys[0].driving.road_ids

    async def get_aoi_route_async(self, start_end_aois: List[Tuple[int, int]], callback=None, n_workers=os.cpu_count()):
        """
        Get the route from `start_aoi` to `end_aoi` in the form of a list of road ids 
        """
        async with grpc.aio.insecure_channel(self.addr) as chan:
            stub = RoutingServiceStub(chan)
            l = []

            async def process(done):
                if callback:
                    for res in done:
                        res = await res
                        callback([] if len(res.journeys) != 1 else res.journeys[0].driving.road_ids)
            for start_aoi, end_aoi in start_end_aois:
                req = GetRouteRequest()
                req.type = RouteType.ROUTE_TYPE_DRIVING
                req.start.aoi_position.aoi_id = start_aoi
                req.end.aoi_position.aoi_id = end_aoi
                l.append(stub.GetRoute(req))
                if len(l) >= n_workers:
                    done, l = await asyncio.wait(l, return_when='FIRST_COMPLETED')
                    await process(done)
                    l = list(l)
            await process(await asyncio.wait(l))

    def set_road_costs(self, road_costs: List[Tuple[int, float]]):
        req = SetDrivingCostsRequest()
        for road, cost in road_costs:
            c = req.costs.add()
            c.id = road
            c.cost = cost
        self.stub.SetDrivingCosts(req)

    def get_road_costs(self, roads: List[int]) -> List[float]:
        req = GetDrivingCostsRequest()
        for road in roads:
            req.costs.add().id = road
        res = self.stub.GetDrivingCosts(req)
        return [i.cost for i in res.costs]
