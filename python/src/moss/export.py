import io
from typing import Iterator, Optional

import psycopg2
import pyproj
from tqdm import tqdm

from .engine import Engine

__all__ = ["DBRecorder"]

class StringIteratorIO(io.TextIOBase):
    def __init__(self, iter: Iterator[str]):
        self._iter = iter
        self._buff = ''

    def readable(self) -> bool:
        return True

    def _read1(self, n: Optional[int] = None) -> str:
        while not self._buff:
            try:
                self._buff = next(self._iter)
            except StopIteration:
                break
        ret = self._buff[:n]
        self._buff = self._buff[len(ret):]
        return ret

    def read(self, n: Optional[int] = None) -> str:
        line = []
        if n is None or n < 0:
            while True:
                m = self._read1()
                if not m:
                    break
                line.append(m)
        else:
            while n > 0:
                m = self._read1(n)
                if not m:
                    break
                n -= len(m)
                line.append(m)
        return ''.join(line)


class DBRecorder:
    """
    DBRecorder is for web visualization and writes to Postgres Database

    The table schema is as follows:
    - meta_simple: The metadata of the simulation.
        - name: The name of the simulation.
        - start: The start step of the simulation.
        - steps: The total steps of the simulation.
        - time: The time of the simulation.
        - total_agents: The total agents of the simulation.
        - map: The map of the simulation.
        - min_lng: The minimum longitude of the simulation.
        - min_lat: The minimum latitude of the simulation.
        - max_lng: The maximum longitude of the simulation.
        - max_lat: The maximum latitude of the simulation.
        - road_status_v_min: The minimum speed of the road status.
        - road_status_interval: The interval of the road status.
    - {output_name}_s_cars: The vehicles of the simulation.
        - step: The step of the simulation.
        - id: The id of the vehicle.
        - parent_id: The parent id of the vehicle.
        - direction: The direction of the vehicle.
        - lng: The longitude of the vehicle.
        - lat: The latitude of the vehicle.
        - model: The model of the vehicle.
        - z: The z of the vehicle.
        - pitch: The pitch of the vehicle.
        - v: The speed of the vehicle.
    - {output_name}_s_people: The people of the simulation.
        - step: The step of the simulation.
        - id: The id of the people.
        - parent_id: The parent id of the people.
        - direction: The direction of the people.
        - lng: The longitude of the people.
        - lat: The latitude of the people.
        - z: The z of the people.
        - v: The speed of the people.
        - model: The model of the people.
    - {output_name}_s_traffic_light: The traffic lights of the simulation.
        - step: The step of the simulation.
        - id: The id of the traffic light.
        - state: The state of the traffic light.
        - lng: The longitude of the traffic light.
        - lat: The latitude of the traffic light.
    - {output_name}_s_road: The road status of the simulation.
        - step: The step of the simulation.
        - id: The id of the road.
        - level: The level of the road.
        - v: The speed of the road.
        - in_vehicle_cnt: The in vehicle count of the road.
        - out_vehicle_cnt: The out vehicle count of the road.
        - cnt: The count of the road.

    The index of the table is as follows:
    - {output_name}_s_cars: (step, lng, lat)
    - {output_name}_s_people: (step, lng, lat)
    - {output_name}_s_traffic_light: (step, lng, lat)
    - {output_name}_s_road: (step)
    """

    def __init__(self, eng: Engine):
        """
        Args:
        - eng: The engine to be recorded.
        """
        self.eng = eng
        self.data = []

    def record(self):
        """
        Record the data of the engine.
        """
        self.data.append([
            self.eng._e.get_current_step(),
            self.eng._e.get_output_vehicles(),
            self.eng._e.get_output_tls(),
        ])

    def save(self, db_url: str, mongo_map: str, output_name: str, use_tqdm=False):
        """
        Save the data to the Postgres Database.

        Args
        - db_url: The URL of the Postgres Database.
        - mongo_map: The map path of the simulation in mongodb (if you use mongodb). The format is like {db}.{coll}.
        - output_name: The name of the simulation that will be saved to the database.
        - use_tqdm: Whether to use tqdm or not.
        """
        vehs = []
        tls = []
        xs = []
        ys = []
        proj = pyproj.Proj(self.eng._map.header.projection)
        for step, (vs, vx, vy), (ts, tx, ty) in self.data:
            if vs:
                x, y = proj(vx, vy, True)
                xs.extend(x)
                ys.extend(y)
                vehs.append([step, vs, x, y])
            if ts:
                x, y = proj(tx, ty, True)
                xs.extend(x)
                ys.extend(y)
                tls.append([step, ts, x, y])
        if xs:
            min_lon, max_lon, min_lat, max_lat = min(xs), max(xs), min(ys), max(ys)
        else:
            x1, y1, x2, y2 = self.eng._map_bbox
            min_lon,  min_lat = proj(x1, y1, True)
            max_lon,  max_lat = proj(x2, y2, True)
        with psycopg2.connect(db_url) as conn:
            with conn.cursor() as cur:
                # create table meta_simple
                cur.execute("""
                CREATE TABLE IF NOT EXISTS public.meta_simple (
                    "name" text NOT NULL,
                    "start" int4 NOT NULL,
                    steps int4 NOT NULL,
                    "time" float8 NOT NULL,
                    total_agents int4 NOT NULL,
                    "map" text NOT NULL,
                    min_lng float8 NOT NULL,
                    min_lat float8 NOT NULL,
                    max_lng float8 NOT NULL,
                    max_lat float8 NOT NULL,
                    road_status_v_min float8 NULL,
                    road_status_interval int4 NULL,
                    CONSTRAINT meta_simple_pkey PRIMARY KEY (name)
                );
                """)
                conn.commit()

                # 删除指定记录
                # delete from public.meta_simple where name='output_name';
                cur.execute(f"DELETE FROM public.meta_simple WHERE name='{output_name}';")
                conn.commit()

                # 插入新记录
                # insert into public.meta_simple values ('output_name', 0, 1000, 1, 1, 'map', 0, 0, 1, 1, 0, 300);
                cur.execute(
                    f"INSERT INTO public.meta_simple VALUES ('{output_name}', {self.eng.start_step}, {len(self.data)}, 1, 1, '{mongo_map}', {min_lon}, {min_lat}, {max_lon}, {max_lat}, 0, 300);")
                conn.commit()

                # 删除表格
                # drop table if exists public.output_name_s_cars;
                cur.execute(f"DROP TABLE IF EXISTS public.{output_name}_s_cars;")
                cur.execute(f"DROP TABLE IF EXISTS public.{output_name}_s_people;")
                cur.execute(
                    f"DROP TABLE IF EXISTS public.{output_name}_s_traffic_light;"
                )
                cur.execute(f"DROP TABLE IF EXISTS public.{output_name}_s_road;")
                conn.commit()

                # 创建表格
                # create table public.output_name_s_cars
                cur.execute(
                    f"""
                CREATE TABLE public.{output_name}_s_cars (
                    step int4 NOT NULL,
                    id int4 NOT NULL,
                    parent_id int4 NOT NULL,
                    direction float8 NOT NULL,
                    lng float8 NOT NULL,
                    lat float8 NOT NULL,
                    model text NOT NULL,
                    z float8 NOT NULL,
                    pitch float8 NOT NULL,
                    v float8 NOT NULL
                );
                """
                )
                cur.execute(
                    f"CREATE INDEX {output_name}_s_cars_step_lng_lat_idx ON public.{output_name}_s_cars USING btree (step, lng, lat);"
                )
                conn.commit()

                # 创建表格
                # create table public.output_name_s_people
                cur.execute(
                    f"""
                CREATE TABLE public.{output_name}_s_people (
                    step int4 NOT NULL,
                    id int4 NOT NULL,
                    parent_id int4 NOT NULL,
                    direction float8 NOT NULL,
                    lng float8 NOT NULL,
                    lat float8 NOT NULL,
                    z float8 NOT NULL,
                    v float8 NOT NULL,
                    model text NOT NULL
                );
                """
                )
                cur.execute(
                    f"CREATE INDEX {output_name}_s_people_step_lng_lat_idx ON public.{output_name}_s_people USING btree (step, lng, lat);"
                )
                conn.commit()

                # 创建表格
                # create table public.output_name_s_traffic_light
                cur.execute(
                    f"""
                CREATE TABLE public.{output_name}_s_traffic_light (
                    step int4 NOT NULL,
                    id int4 NOT NULL,
                    state int4 NOT NULL,
                    lng float8 NOT NULL,
                    lat float8 NOT NULL
                );
                """
                )
                cur.execute(
                    f"CREATE INDEX {output_name}_s_traffic_light_step_lng_lat_idx ON public.{output_name}_s_traffic_light USING btree (step, lng, lat);"
                )
                conn.commit()

                # 创建表格
                # create table public.output_name_s_road
                cur.execute(
                    f"""
                CREATE TABLE public.{output_name}_s_road (
                    step int4 NOT NULL,
                    id int4 NOT NULL,
                    "level" int4 NOT NULL,
                    v float8 NOT NULL,
                    in_vehicle_cnt int4 NOT NULL,
                    out_vehicle_cnt int4 NOT NULL,
                    cnt int4 NOT NULL
                );
                """
                )
                cur.execute(
                    f"CREATE INDEX {output_name}_s_road_step_idx ON public.{output_name}_s_road USING btree (step);"
                )
                conn.commit()

                cur.copy_from(
                    StringIteratorIO(
                        f"{step},{p},{l},{round(d,3)},{x},{y},,0,0,{round(v,3)}\n"
                        for step, vs, x, y in tqdm(vehs, ncols=90, disable=not use_tqdm)
                        for (p, l, d, v), x, y in zip(vs, x, y)
                    ),
                    f"{output_name}_s_cars",
                    sep=",",
                )
                cur.copy_from(
                    StringIteratorIO(
                        f"{step},{p},{s},{x},{y}\n"
                        for step, ts, x, y in tqdm(tls, ncols=90, disable=not use_tqdm)
                        for (p, s), x, y in zip(ts, x, y)
                    ),
                    f"{output_name}_s_traffic_light",
                    sep=",",
                )
                conn.commit()
