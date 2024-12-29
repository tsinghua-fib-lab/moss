import io
from typing import Iterator, Optional

import psycopg
import pyproj
import ray
from psycopg import sql
from tqdm import tqdm

from .engine import Engine

__all__ = ["DBRecorder"]


class StringIteratorIO(io.TextIOBase):
    def __init__(self, iter: Iterator[str]):
        self._iter = iter
        self._buff = ""

    def readable(self) -> bool:
        return True

    def _read1(self, n: Optional[int] = None) -> str:
        while not self._buff:
            try:
                self._buff = next(self._iter)
            except StopIteration:
                break
        ret = self._buff[:n]
        self._buff = self._buff[len(ret) :]
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
        return "".join(line)


@ray.remote
class _CopyWriter:
    def __init__(self, dsn: str, proj_str: str):
        self._dsn = dsn
        self._proj = pyproj.Proj(proj_str)

    async def awrite(self, table_name: str, step, persons, tls, roads):
        # 1. reorganize the data
        person_info, xs, ys = persons
        lngs, lats = self._proj(xs, ys, inverse=True)
        cars = []
        peds = []
        for p, lng, lat in zip(person_info, lngs, lats):
            id, status, parent_id, dir, v = p
            if status == 2:
                cars.append([id, parent_id, dir, lng, lat, v])
            elif status == 1:
                peds.append([id, parent_id, dir, lng, lat, v])
        tls_out, xs, ys = tls
        lngs, lats = self._proj(xs, ys, inverse=True)
        # 2. write to database
        async with await psycopg.AsyncConnection.connect(self._dsn) as aconn:
            # 1. copy to cars
            copy_sql = sql.SQL(
                "COPY {} (step, id, parent_id, direction, lng, lat, model, z, pitch, v, num_passengers) FROM STDIN"
            ).format(sql.Identifier(f"{table_name}_s_cars"))
            async with aconn.cursor() as cur:
                async with cur.copy(copy_sql) as copy:
                    for id, parent_id, dir, lng, lat, v in cars:
                        await copy.write_row(
                            (
                                step,
                                id,
                                parent_id,
                                round(dir, 3),
                                lng,
                                lat,
                                "",
                                0,
                                0,
                                round(v, 3),
                                0,
                            )
                        )
            # 2. copy to people
            copy_sql = sql.SQL(
                "COPY {} (step, id, parent_id, direction, lng, lat, z, v, model) FROM STDIN"
            ).format(sql.Identifier(f"{table_name}_s_people"))
            async with aconn.cursor() as cur:
                async with cur.copy(copy_sql) as copy:
                    for id, parent_id, dir, lng, lat, v in peds:
                        await copy.write_row(
                            (
                                step,
                                id,
                                parent_id,
                                round(dir, 3),
                                lng,
                                lat,
                                0,
                                round(v, 3),
                                "",
                            )
                        )
            # 3. copy to traffic light
            copy_sql = sql.SQL("COPY {} (step, id, state, lng, lat) FROM STDIN").format(
                sql.Identifier(f"{table_name}_s_traffic_light")
            )
            async with aconn.cursor() as cur:
                async with cur.copy(copy_sql) as copy:
                    for (id, s), lng, lat in zip(tls_out, lngs, lats):
                        await copy.write_row((step, id, s, lng, lat))
            # 4. copy to road
            copy_sql = sql.SQL(
                "COPY {} (step, id, level, v, in_vehicle_cnt, out_vehicle_cnt, cnt) FROM STDIN"
            ).format(sql.Identifier(f"{table_name}_s_road"))
            async with aconn.cursor() as cur:
                async with cur.copy(copy_sql) as copy:
                    for id, level, v_avg in roads:
                        await copy.write_row(
                            (step, id, level, round(v_avg, 3), 0, 0, 0)
                        )


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
        - version: The version of the simulation result schema (now is 2).
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
    - {output_name}_s_aoi: The number of people in AOI (no content, only for compatibility)
        - step: The step of the simulation.
        - id: The id of the AOI.
        - cnt: The count of the people in the AOI.

    The index of the table is as follows:
    - {output_name}_s_cars: (step, lng, lat)
    - {output_name}_s_people: (step, lng, lat)
    - {output_name}_s_traffic_light: (step, lng, lat)
    - {output_name}_s_road: (step)
    - {output_name}_s_aoi: (step)
    """

    def __init__(
        self,
        eng: Engine,
        db_url: str,
        mongo_map: str,
        output_name: str,
        total_steps: int = 86400,
        use_tqdm: bool = False,
    ):
        """
        Args:
        - eng: The engine to be recorded.
        - db_url: The URL of the database.
        - mongo_map: The map of the simulation.
        - output_name: The name of the output.
        - total_steps: The total steps of the simulation.
        """
        self.eng = eng
        self._db_url = db_url
        self._mongo_map = mongo_map
        self._output_name = output_name
        self._proj = pyproj.Proj(self.eng._map.header.projection)
        self._total_steps = total_steps

        self._init_table()
        ray.init(ignore_reinit_error=True)
        self._writer = _CopyWriter.remote(db_url, self.eng._map.header.projection)
        self._unwaited = []
        self._use_tqdm = use_tqdm

    def __del__(self):
        self.flush()

    def _init_table(self):
        output_name = self._output_name
        with psycopg.connect(self._db_url) as conn:
            with conn.cursor() as cur:
                # create table meta_simple
                cur.execute(
                    f"""
                CREATE TABLE IF NOT EXISTS meta_simple (
                    "name" text NOT NULL PRIMARY KEY,
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
                    version INT NOT NULL
                );
                """
                )
                conn.commit()

                # 删除指定记录
                # delete from meta_simple where name='output_name';
                cur.execute("DELETE FROM meta_simple WHERE name=%s;", (output_name,))
                conn.commit()

                # 插入新记录
                # insert into meta_simple values ('output_name', 0, 1000, 1, 1, 'map', 0, 0, 1, 1, 0, 300, 2);
                xmin, ymin, xmax, ymax = self.eng._map_bbox
                min_lon, min_lat = self._proj(xmin, ymin, inverse=True)
                max_lon, max_lat = self._proj(xmax, ymax, inverse=True)
                cur.execute(
                    """
                    INSERT INTO meta_simple (name, start, steps, time, total_agents, map, min_lng, min_lat, max_lng, max_lat, road_status_v_min, road_status_interval, version)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s);
                    """,
                    (
                        output_name,
                        self.eng.start_step,
                        self._total_steps,
                        1,
                        1,
                        self._mongo_map,
                        min_lon,
                        min_lat,
                        max_lon,
                        max_lat,
                        0,
                        (
                            self.eng.speed_stat_interval
                            if self.eng.speed_stat_interval > 0
                            else None
                        ),
                        2,
                    ),
                )
                conn.commit()

                # 删除表格
                # drop table if exists output_name_s_cars;
                cur.execute(
                    sql.SQL("DROP TABLE IF EXISTS {}").format(
                        sql.Identifier(f"{output_name}_s_cars")
                    )
                )
                cur.execute(
                    sql.SQL("DROP TABLE IF EXISTS {}").format(
                        sql.Identifier(f"{output_name}_s_people")
                    )
                )
                cur.execute(
                    sql.SQL("DROP TABLE IF EXISTS {}").format(
                        sql.Identifier(f"{output_name}_s_traffic_light")
                    )
                )
                cur.execute(
                    sql.SQL("DROP TABLE IF EXISTS {}").format(
                        sql.Identifier(f"{output_name}_s_road")
                    )
                )
                cur.execute(
                    sql.SQL("DROP TABLE IF EXISTS {}").format(
                        sql.Identifier(f"{output_name}_s_aoi")
                    )
                )
                conn.commit()

                # 创建表格
                # create table output_name_s_cars
                cur.execute(
                    sql.SQL(
                        """
                CREATE TABLE {} (
                    step int4 NOT NULL,
                    id int4 NOT NULL,
                    parent_id int4 NOT NULL,
                    direction float8 NOT NULL,
                    lng float8 NOT NULL,
                    lat float8 NOT NULL,
                    model text NOT NULL,
                    z float8 NOT NULL,
                    pitch float8 NOT NULL,
                    v float8 NOT NULL,
                    num_passengers int4 NOT NULL
                );
                """
                    ).format(sql.Identifier(f"{output_name}_s_cars"))
                )
                cur.execute(
                    sql.SQL(
                        "CREATE INDEX {} ON {} USING btree (step, lng, lat);"
                    ).format(
                        sql.Identifier(f"{output_name}_s_cars_step_lng_lat_idx"),
                        sql.Identifier(f"{output_name}_s_cars"),
                    )
                )
                conn.commit()

                # 创建表格
                # create table output_name_s_people
                cur.execute(
                    sql.SQL(
                        """
                CREATE TABLE {} (
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
                    ).format(sql.Identifier(f"{output_name}_s_people"))
                )
                cur.execute(
                    sql.SQL(
                        "CREATE INDEX {} ON {} USING btree (step, lng, lat);"
                    ).format(
                        sql.Identifier(f"{output_name}_s_people_step_lng_lat_idx"),
                        sql.Identifier(f"{output_name}_s_people"),
                    )
                )
                conn.commit()

                # 创建表格
                # create table output_name_s_traffic_light
                cur.execute(
                    sql.SQL(
                        """
                CREATE TABLE {} (
                    step int4 NOT NULL,
                    id int4 NOT NULL,
                    state int4 NOT NULL,
                    lng float8 NOT NULL,
                    lat float8 NOT NULL
                );
                """
                    ).format(sql.Identifier(f"{output_name}_s_traffic_light"))
                )
                cur.execute(
                    sql.SQL(
                        "CREATE INDEX {} ON {} USING btree (step, lng, lat);"
                    ).format(
                        sql.Identifier(
                            f"{output_name}_s_traffic_light_step_lng_lat_idx"
                        ),
                        sql.Identifier(f"{output_name}_s_traffic_light"),
                    )
                )
                conn.commit()

                # 创建表格
                # create table output_name_s_road
                cur.execute(
                    sql.SQL(
                        """
                CREATE TABLE {} (
                    step int4 NOT NULL,
                    id int4 NOT NULL,
                    "level" int4 NOT NULL,
                    v float8 NOT NULL,
                    in_vehicle_cnt int4 NOT NULL,
                    out_vehicle_cnt int4 NOT NULL,
                    cnt int4 NOT NULL
                );
                """
                    ).format(sql.Identifier(f"{output_name}_s_road"))
                )
                cur.execute(
                    sql.SQL("CREATE INDEX {} ON {} USING btree (step);").format(
                        sql.Identifier(f"{output_name}_s_road_step_idx"),
                        sql.Identifier(f"{output_name}_s_road"),
                    )
                )
                conn.commit()

                # 创建表格
                # create table output_name_s_aoi
                cur.execute(
                    sql.SQL(
                        """
                CREATE TABLE {} (
                    step int4 NOT NULL,
                    id int4 NOT NULL,
                    cnt int4 NOT NULL
                );
                """
                    ).format(sql.Identifier(f"{output_name}_s_aoi"))
                )
                cur.execute(
                    sql.SQL("CREATE INDEX {} ON {} USING btree (step);").format(
                        sql.Identifier(f"{output_name}_s_aoi_step_idx"),
                        sql.Identifier(f"{output_name}_s_aoi"),
                    )
                )
                conn.commit()

    def record(self):
        """
        Record the data of the engine.
        """
        step = self.eng._e.get_current_step()
        persons = self.eng._e.get_output_persons()
        tls = self.eng._e.get_output_tls()
        roads = self.eng._e.get_output_roads()
        task = self._writer.awrite.remote(self._output_name, step, persons, tls, roads)
        self._unwaited.append(task)

    def flush(self):
        """
        Flush the data to the database.
        """
        if self._use_tqdm:
            for task in tqdm(self._unwaited, desc="Waiting for writing"):
                ray.get(task)
        else:
            ray.get(self._unwaited)
        self._unwaited = []
