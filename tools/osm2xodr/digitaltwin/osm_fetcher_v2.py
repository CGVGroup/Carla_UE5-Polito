"""tools/osm2xodr/digitaltwin/osm_fetcher_v2.py

Module 1 — OSM Data Fetcher

Fetches OSM data from Overpass API, parses raw XML using lxml, and returns structured
Python dictionaries for roads, buildings, sidewalks, and junction nodes.

Key features:
- Overpass API queries using bbox
- Optional bbox resolution from place name via Nominatim
- Exponential backoff (max 3 attempts)
- Way chaining to reconstruct continuous roads split at intersections
- Projection: WGS84 lat/lon -> UTM local ENU (x=E, y=N, z=Up)

"""

from __future__ import annotations

import dataclasses
import logging
import math
import time
from typing import Any, Dict, Iterable, List, Literal, Optional, Sequence, Tuple

import requests
from lxml import etree
from pyproj import CRS, Transformer
from shapely.geometry import LineString, Polygon

logger = logging.getLogger(__name__)

OVERPASS_ENDPOINT = "https://overpass-api.de/api/interpreter"
NOMINATIM_SEARCH = "https://nominatim.openstreetmap.org/search"


BBox = Tuple[float, float, float, float]  # (min_lat, min_lon, max_lat, max_lon)


dataclasses.dataclass(frozen=True)
class ProjectionInfo:
    crs_utm: CRS
    proj4: str
    origin_latlon: Tuple[float, float]
    origin_xy: Tuple[float, float]
    transformer_ll_to_xy: Transformer


def _utm_crs_from_latlon(lat: float, lon: float) -> CRS:
    zone = int((lon + 180.0) // 6.0) + 1
    is_north = lat >= 0
    epsg = 32600 + zone if is_north else 32700 + zone
    return CRS.from_epsg(epsg)


def build_projection_from_bbox(bbox: BBox) -> ProjectionInfo:
    min_lat, min_lon, max_lat, max_lon = bbox
    lat0 = 0.5 * (min_lat + max_lat)
    lon0 = 0.5 * (min_lon + max_lon)

    crs_utm = _utm_crs_from_latlon(lat0, lon0)
    transformer = Transformer.from_crs(CRS.from_epsg(4326), crs_utm, always_xy=True)
    x0, y0 = transformer.transform(lon0, lat0)

    # Store PROJ string for OpenDRIVE geoReference
    proj4 = crs_utm.to_proj4()

    return ProjectionInfo(
        crs_utm=crs_utm,
        proj4=proj4,
        origin_latlon=(lat0, lon0),
        origin_xy=(x0, y0),
        transformer_ll_to_xy=transformer,
    )


def project_points_ll_to_local_xy(
    proj: ProjectionInfo,
    latlon: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    x0, y0 = proj.origin_xy
    for lat, lon in latlon:
        x, y = proj.transformer_ll_to_xy.transform(lon, lat)
        out.append((x - x0, y - y0))
    return out


def place_to_bbox(place: str, user_agent: str = "CGVGroup-osm2xodr") -> BBox:
    # Nominatim requires a User-Agent identifying the application.
    params = {
        "q": place,
        "format": "json",
        "limit": 1,
        "polygon_geojson": 0,
    }
    headers = {"User-Agent": user_agent}
    resp = requests.get(NOMINATIM_SEARCH, params=params, headers=headers, timeout=30)
    resp.raise_for_status()
    results = resp.json()
    if not results:
        raise ValueError(f"Nominatim could not resolve place name: {place}")
    bb = results[0]["boundingbox"]  # [south, north, west, east] as strings
    south = float(bb[0])
    north = float(bb[1])
    west = float(bb[2])
    east = float(bb[3])
    return (south, west, north, east)


def build_overpass_query(bbox: BBox) -> str:
    min_lat, min_lon, max_lat, max_lon = bbox
    # Fetch highways (ways), buildings (ways + relations), and nodes for junction degrees.
    # Also pull ways tagged sidewalk=* explicitly.
    return f"""
[out:xml][timeout:180];
(
  way[highway]({min_lat},{min_lon},{max_lat},{max_lon});
  way[building]({min_lat},{min_lon},{max_lat},{max_lon});
  relation[building]({min_lat},{min_lon},{max_lat},{max_lon});
  way[sidewalk]({min_lat},{min_lon},{max_lat},{max_lon});
);
(._;>;);
out body;
""".strip()


def _request_overpass_xml(query: str) -> etree._Element:
    backoff_s = 1.0
    for attempt in range(3):
        try:
            resp = requests.post(OVERPASS_ENDPOINT, data=query.encode("utf-8"), timeout=180)
            resp.raise_for_status()
            return etree.fromstring(resp.content)
        except Exception as e:
            if attempt == 2:
                raise
            logger.warning("Overpass request failed (attempt %d/3): %s", attempt + 1, e)
            time.sleep(backoff_s)
            backoff_s *= 2.0
    raise RuntimeError("Unreachable")


def _parse_tags(el: etree._Element) -> Dict[str, str]:
    tags: Dict[str, str] = {}
    for t in el.findall("tag"):
        k = t.get("k")
        v = t.get("v")
        if k is not None and v is not None:
            tags[k] = v
    return tags


def _safe_int(value: Optional[str]) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _parse_height_m(tags: Dict[str, str]) -> float:
    # height tag can be like "12", "12 m".
    h = tags.get("height")
    if h:
        try:
            return float(h.replace("m", "").strip())
        except Exception:
            pass

    levels = _safe_int(tags.get("building:levels"))
    if levels is not None:
        return float(levels) * 3.0
    return 9.0


def _parse_oneway(tags: Dict[str, str]) -> Literal["yes", "no", "-1"]:
    v = (tags.get("oneway") or "").strip().lower()
    if v in {"yes", "true", "1"}:
        return "yes"
    if v == "-1":
        return "-1"
    return "no"


def _parse_lanes(tags: Dict[str, str]) -> int:
    lanes_raw = tags.get("lanes")
    if lanes_raw is None:
        logger.warning("Missing lanes tag; defaulting to 1")
        return 1
    try:
        lanes = int(lanes_raw)
        if lanes <= 0:
            raise ValueError
        return lanes
    except Exception:
        logger.warning("Unparseable lanes=%r; defaulting to 1", lanes_raw)
        return 1


def _way_node_refs(way_el: etree._Element) -> List[int]:
    refs: List[int] = []
    for nd in way_el.findall("nd"):
        r = nd.get("ref")
        if r is not None:
            refs.append(int(r))
    return refs


def _collect_nodes(osm_root: etree._Element) -> Dict[int, Tuple[float, float]]:
    nodes: Dict[int, Tuple[float, float]] = {}
    for n in osm_root.findall("node"):
        nid = int(n.get("id"))
        lat = float(n.get("lat"))
        lon = float(n.get("lon"))
        nodes[nid] = (lat, lon)
    return nodes


def _degree_map(roads_way_node_refs: Iterable[List[int]]) -> Dict[int, int]:
    deg: Dict[int, int] = {}
    for refs in roads_way_node_refs:
        # Degree computed on endpoints primarily for chaining.
        if not refs:
            continue
        for nid in (refs[0], refs[-1]):
            deg[nid] = deg.get(nid, 0) + 1
    return deg


def _chain_ways(
    ways: List[Dict[str, Any]],
    snap_tol_m: float,
    stop_degree: int,
    stop_on_tag_change: bool,
    proj: ProjectionInfo,
    node_ll: Dict[int, Tuple[float, float]],
    endpoint_degree: Dict[int, int],
) -> List[Dict[str, Any]]:
    # Chain ways that share endpoints and compatible tags.
    # Chaining uses projected local XY endpoints for snap tolerance.

    def endpoint_xy(nid: int) -> Tuple[float, float]:
        lat, lon = node_ll[nid]
        x, y = proj.transformer_ll_to_xy.transform(lon, lat)
        x0, y0 = proj.origin_xy
        return (x - x0, y - y0)

    def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    remaining = ways[:]
    chained: List[Dict[str, Any]] = []

    # Precompute endpoints.
    for w in remaining:
        refs = w["node_refs"]
        w["_start"] = refs[0]
        w["_end"] = refs[-1]

    while remaining:
        seed = remaining.pop()
        group = [seed]

        changed = True
        while changed:
            changed = False
            end_id = group[-1]["_end"]

            # Stop at intersections.
            if endpoint_degree.get(end_id, 0) >= stop_degree:
                break

            end_xy = endpoint_xy(end_id)

            for i, cand in enumerate(list(remaining)):
                # Tag compatibility.
                if stop_on_tag_change:
                    if (cand.get("highway"), cand.get("name"), cand.get("oneway"), cand.get("lanes")) != (
                        seed.get("highway"),
                        seed.get("name"),
                        seed.get("oneway"),
                        seed.get("lanes"),
                    ):
                        continue

                # Candidate can connect if its start matches our end (or near within tol).
                c_start = cand["_start"]
                c_end = cand["_end"]

                c_start_xy = endpoint_xy(c_start)
                c_end_xy = endpoint_xy(c_end)

                if dist(end_xy, c_start_xy) <= snap_tol_m:
                    group.append(cand)
                    remaining.remove(cand)
                    changed = True
                    break
                if dist(end_xy, c_end_xy) <= snap_tol_m:
                    # Reverse candidate.
                    cand["node_refs"] = list(reversed(cand["node_refs"]))
                    cand["_start"], cand["_end"] = cand["_end"], cand["_start"]
                    group.append(cand)
                    remaining.remove(cand)
                    changed = True
                    break

        # Merge node refs.
        merged_refs: List[int] = []
        for idx, part in enumerate(group):
            refs = part["node_refs"]
            if idx == 0:
                merged_refs.extend(refs)
            else:
                merged_refs.extend(refs[1:])

        out = dict(seed)
        out["id"] = int(seed["id"])
        out["node_refs"] = merged_refs
        chained.append(out)

    return chained


class OSMDataFetcher:
    def __init__(
        self,
        bbox: Optional[BBox] = None,
        place: Optional[str] = None,
        snap_tolerance_m: float = 0.2,
        chain_stop_degree: int = 3,
        chain_stop_on_tag_change: bool = True,
        user_agent: str = "CGVGroup-osm2xodr",
    ) -> None:
        if bbox is None and place is None:
            raise ValueError("Either bbox or place must be provided")
        self.bbox = bbox
        self.place = place
        self.snap_tolerance_m = snap_tolerance_m
        self.chain_stop_degree = chain_stop_degree
        self.chain_stop_on_tag_change = chain_stop_on_tag_change
        self.user_agent = user_agent

    def fetch(self) -> Dict[str, Any]:
        bbox = self.bbox
        if bbox is None:
            bbox = place_to_bbox(self.place or "", user_agent=self.user_agent)

        proj = build_projection_from_bbox(bbox)
        query = build_overpass_query(bbox)
        root = _request_overpass_xml(query)

        node_ll = _collect_nodes(root)

        # Ways.
        road_ways: List[Dict[str, Any]] = []
        building_ways: List[Dict[str, Any]] = []
        sidewalk_ways: List[Dict[str, Any]] = []

        for way in root.findall("way"):
            wid = int(way.get("id"))
            tags = _parse_tags(way)
            refs = _way_node_refs(way)

            if "highway" in tags:
                road_ways.append(
                    {
                        "id": wid,
                        "node_refs": refs,
                        "highway": tags.get("highway", ""),
                        "lanes": _parse_lanes(tags),
                        "oneway": _parse_oneway(tags),
                        "sidewalk": tags.get("sidewalk", ""),
                        "name": tags.get("name", ""),
                        "tags": tags,
                    }
                )

            if "building" in tags:
                building_ways.append(
                    {
                        "id": wid,
                        "node_refs": refs,
                        "height": _parse_height_m(tags),
                        "tags": tags,
                    }
                )

            if "sidewalk" in tags and "highway" not in tags:
                sidewalk_ways.append(
                    {
                        "id": wid,
                        "node_refs": refs,
                        "sidewalk": tags.get("sidewalk", ""),
                        "tags": tags,
                    }
                )

        endpoint_degree = _degree_map([w["node_refs"] for w in road_ways])
        junction_nodes = [nid for nid, d in endpoint_degree.items() if d >= 3]

        # Chain road ways.
        chained_roads = _chain_ways(
            road_ways,
            snap_tol_m=self.snap_tolerance_m,
            stop_degree=self.chain_stop_degree,
            stop_on_tag_change=self.chain_stop_on_tag_change,
            proj=proj,
            node_ll=node_ll,
            endpoint_degree=endpoint_degree,
        )

        # Expand node refs into lat/lon list.
        roads: List[Dict[str, Any]] = []
        for r in chained_roads:
            latlon = [node_ll[nid] for nid in r["node_refs"] if nid in node_ll]
            roads.append(
                {
                    "id": r["id"],
                    "nodes": [(lat, lon) for (lat, lon) in latlon],
                    "highway": r["highway"],
                    "lanes": r["lanes"],
                    "oneway": r["oneway"],
                    "sidewalk": r.get("sidewalk", ""),
                    "name": r.get("name", ""),
                    "tags": r.get("tags", {}),
                }
            )

        buildings: List[Dict[str, Any]] = []
        for b in building_ways:
            latlon = [node_ll.get(nid) for nid in b["node_refs"]]
            latlon2 = [p for p in latlon if p is not None]
            # Must be polygon-like.
            if len(latlon2) < 3:
                logger.warning("Skipping building %s: not enough nodes", b["id"])
                continue
            buildings.append(
                {
                    "id": b["id"],
                    "nodes": [(lat, lon) for (lat, lon) in latlon2],
                    "height": float(b["height"]),
                    "tags": b.get("tags", {}),
                }
            )

        sidewalks: List[Dict[str, Any]] = []
        for s in sidewalk_ways:
            latlon = [node_ll.get(nid) for nid in s["node_refs"]]
            latlon2 = [p for p in latlon if p is not None]
            if len(latlon2) < 2:
                continue
            sidewalks.append(
                {
                    "id": s["id"],
                    "nodes": [(lat, lon) for (lat, lon) in latlon2],
                    "sidewalk": s.get("sidewalk", ""),
                    "tags": s.get("tags", {}),
                }
            )

        return {
            "bbox": bbox,
            "projection": {
                "proj4": proj.proj4,
                "origin_latlon": proj.origin_latlon,
                "origin_xy": proj.origin_xy,
            },
            "roads": roads,
            "buildings": buildings,
            "sidewalks": sidewalks,
            "junction_nodes": junction_nodes,
        }

"""
End of module.
