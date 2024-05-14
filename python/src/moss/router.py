from typing import List, Tuple

from igraph import Graph

from .map import LaneType, Map


class Router:
    def __init__(self, m: Map):
        # 用2i和2i+1分别表示道路i的起点和终点
        edges = [(2*i, 2*i+1) for i in range(len(m.roads))]
        for r in m.roads:
            s = set()
            for l in r.lanes:
                if l.type == LaneType.DRIVING:
                    for ll in l.successors:
                        for lll in ll.successors:
                            s.add(lll.parent_road.index)
            edges += ((2*r.index+1, 2*i) for i in s)
        self.aoi2index = {}
        self.aoi2road = {}
        length = [r.lanes[0].length for r in m.roads]+[0]*(len(edges)-len(m.roads))
        for i, a in enumerate(m.aois, len(m.roads)):
            self.aoi2index[a.id] = i
            rs = set()
            for p in a.driving_positions:
                l = m.lane_map[p.id]
                rs.add(l.parent_road.id)
                # “正确”的做法是让aoi到道路起点的边长度为负数，但这样在找最短路时似乎有问题
                edges.append((2*i, 2*l.parent_road.index))
                length.append(2)  # -> length.append(-p.s)
                edges.append((2*i, 2*l.parent_road.index+1))
                length.append(1)  # -> length.append(l.length-p.s)
                edges.append((2*l.parent_road.index, 2*i+1))
                length.append(p.s)
            self.aoi2road[a.id] = rs
        self.G = Graph(edges, directed=True)
        self.G.es['length'] = length
        self.id2index = {
            r.id: r.index for r in m.roads
        }
        self.index2id = [r.id for r in m.roads]

    def get_route(self, start_road: int, end_road: int) -> List[int]:
        """
        Get the route from `start_road` to `end_road` in the form of a list of road ids 
        """
        path = self.G.get_shortest_paths(self.id2index[start_road]*2+1, self.id2index[end_road]*2, weights='length', output='vpath')[0]
        if not path:
            return []
        path = [path[0]//2]+[i//2 for i in path[1::2]]
        if not sum(self.G.es[i]['length'] for i in path) < 1e999:
            return []
        return [self.index2id[i] for i in path]

    def get_aoi_route(self, start_aoi: int, end_aoi: int, check=False) -> List[int]:
        """
        Get the route from `start_aoi` to `end_aoi` in the form of a list of road ids 
        """
        assert start_aoi != end_aoi
        path = self.G.get_shortest_paths(self.aoi2index[start_aoi]*2, self.aoi2index[end_aoi]*2+1, weights='length', output='vpath')[0]
        if not path:
            return []
        assert len(path) > 2
        path = [path[1]//2]+[i//2 for i in path[2:-1:2]]
        if not sum(self.G.es[i]['length'] for i in path) < 1e999:
            return []
        path = [self.index2id[i] for i in path]
        if check:
            assert len(set(path)) == len(path)
            assert path[0] in self.aoi2road[start_aoi] and path[-1] in self.aoi2road[end_aoi]
        return path

    def set_road_cost(self, road_id: int, cost: float):
        self.G.es[self.id2index[road_id]]['length'] = cost

    def set_road_costs(self, road_costs: List[Tuple[int, float]]):
        for i, c in road_costs:
            self.G.es[self.id2index[i]]['length'] = c

    def get_road_costs(self, roads: List[int]) -> List[float]:
        return [self.G.es[self.id2index[i]]['length'] for i in roads]
