import numpy as np
import psycopg
import pyproj
from tqdm import tqdm

from .engine import Engine


class Recorder:
    """
    Recorder is for local visualization and writes to a local file
    """

    def __init__(self, eng: Engine, enable=True):
        self.enable = enable
        if not enable:
            return
        self.eng = eng
        self.data = {
            'lanes': eng.get_lane_geoms(),
            'steps': []
        }

    def record(self):
        if not self.enable:
            return
        self.data['steps'].append({
            'veh_id_positions': self.eng.get_vehicle_id_positions().astype(np.float32),
            'lane_states': self.eng.get_lane_statuses(),
        })

    def save(self, filepath):
        if not self.enable:
            return
        np.savez_compressed(filepath, data=self.data)


class DBRecorder:
    """
    DBRecorder is for web visualization and writes to Postgres Database
    """

    def __init__(self, eng: Engine):
        assert eng.enable_output
        self.eng = eng
        self.vehs = []
        self.peds = []
        self.tls = []
        self.xys = []
        self.data = []

    def record(self):
        self.data.append([
            self.eng._e.get_current_step(),
            self.eng._e.get_output_vehicles(),
            self.eng._e.get_output_tls(),
        ])

    def save(self, db_url: str, mongo_map: str, output_name: str, batch_size=1000, use_tqdm=False):
        vehs = []
        tls = []
        xs = []
        ys = []
        proj = pyproj.Proj(self.eng._e.get_map_projection())
        for step, vs, ts in self.data:
            if vs:
                p, l, x, y, d, v = zip(*vs)
                x, y = proj(x, y, True)
                xs += x
                ys += y
                for p, l, x, y, d, v in zip(p, l, x, y, d, v):
                    vehs.append(f"({step},{p},{l},{round(d,3)},{x},{y},'',0,0,{round(v,3)})")
            if ts:
                p, s, x, y = zip(*ts)
                x, y = proj(x, y, True)
                xs += x
                ys += y
                for p, s, x, y in zip(p, s, x, y):
                    tls.append(f"({step},{p},{s},{x},{y})")
        if xs:
            min_lon, max_lon, min_lat, max_lat = min(xs), max(xs), min(ys), max(ys)
        else:
            x1, y1, x2, y2 = self.eng._e.get_map_bbox()
            min_lon,  min_lat = proj(x1, y1, True)
            max_lon,  max_lat = proj(x2, y2, True)
        with psycopg.connect(db_url) as conn:
            with conn.cursor() as cur:
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
)""")
                cur.execute(f"DELETE FROM public.meta_simple WHERE name='{output_name}'")
                cur.execute(
                    f"INSERT INTO public.meta_simple VALUES ('{output_name}', {self.eng.start_step}, {len(self.data)}, 1, 1, '{mongo_map}', {min_lon}, {min_lat}, {max_lon}, {max_lat}, 0, 300)")
                cur.execute(f"DROP TABLE IF EXISTS {output_name}_s_cars")
                cur.execute(f"DROP TABLE IF EXISTS {output_name}_s_people")
                cur.execute(f"DROP TABLE IF EXISTS {output_name}_s_traffic_light")
                cur.execute(f"DROP TABLE IF EXISTS {output_name}_s_road")
                cur.execute(f"""
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
)""")
                cur.execute(f"CREATE INDEX {output_name}_s_cars_step_lng_lat_idx ON public.{output_name}_s_cars USING btree (step, lng, lat)")
                cur.execute(f"""
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
)""")
                cur.execute(f"CREATE INDEX {output_name}_s_people_step_lng_lat_idx ON public.{output_name}_s_people USING btree (step, lng, lat)")
                cur.execute(f"""
CREATE TABLE public.{output_name}_s_traffic_light (
	step int4 NOT NULL,
	id int4 NOT NULL,
	state int4 NOT NULL,
	lng float8 NOT NULL,
	lat float8 NOT NULL
)""")
                cur.execute(f"CREATE INDEX {output_name}_s_traffic_light_step_lng_lat_idx ON public.{output_name}_s_traffic_light USING btree (step, lng, lat)")
                cur.execute(f"""
CREATE TABLE public.{output_name}_s_road (
	step int4 NOT NULL,
	id int4 NOT NULL,
	"level" int4 NOT NULL,
	v float8 NOT NULL,
	in_vehicle_cnt int4 NOT NULL,
	out_vehicle_cnt int4 NOT NULL,
	cnt int4 NOT NULL
)""")
                cur.execute(f"CREATE INDEX {output_name}_s_road_step_idx ON public.{output_name}_s_road USING btree (step)")
                for i in tqdm(range(0, len(vehs), batch_size), ncols=90, disable=not use_tqdm):
                    cur.execute(
                        f"INSERT INTO public.{output_name}_s_cars VALUES "
                        + ",".join(vehs[i: i + batch_size])
                    )
                for i in tqdm(range(0, len(tls), batch_size), ncols=90, disable=not use_tqdm):
                    cur.execute(
                        f"INSERT INTO public.{output_name}_s_traffic_light VALUES "
                        + ",".join(tls[i: i + batch_size])
                    )
