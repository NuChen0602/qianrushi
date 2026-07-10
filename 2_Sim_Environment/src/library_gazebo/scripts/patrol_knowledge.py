#!/usr/bin/env python3
import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class SemanticTarget:
    area_id: str
    name: str
    label: str
    x: float
    y: float
    speech: str


def _default_map_path() -> str:
    return os.path.join(
        get_package_share_directory("library_gazebo"),
        "config",
        "library_semantic_map.json",
    )


def load_library_map(path: Optional[str] = None) -> Dict:
    with open(path or _default_map_path(), "r", encoding="utf-8") as f:
        return json.load(f)


def _score_text(text: str, keywords: List[str]) -> int:
    normalized = text.lower()
    return sum(1 for keyword in keywords if keyword.lower() in normalized)


def lookup_semantic_target(user_text: str, library_map: Optional[Dict] = None) -> SemanticTarget:
    data = library_map or load_library_map()
    areas = {area["id"]: area for area in data.get("areas", [])}

    best_area = None
    best_score = 0

    for holding in data.get("holdings", []):
        score = _score_text(user_text, holding.get("keywords", []))
        if score > best_score and holding.get("area_id") in areas:
            best_score = score
            best_area = areas[holding["area_id"]]

    for area in data.get("areas", []):
        score = _score_text(user_text, area.get("keywords", []))
        if score > best_score:
            best_score = score
            best_area = area

    if best_area is None:
        return SemanticTarget(
            area_id="unknown",
            name="未知区域",
            label="未知区域",
            x=0.0,
            y=0.0,
            speech="抱歉，我暂时没有匹配到明确的书架或巡检区域，请换一种说法。",
        )

    return SemanticTarget(
        area_id=best_area["id"],
        name=best_area["name"],
        label=best_area["label"],
        x=float(best_area["x"]),
        y=float(best_area["y"]),
        speech=f"好的，我将带您前往{best_area['name']}，也就是{best_area['label']}。",
    )


def patrol_route_targets(library_map: Optional[Dict] = None) -> List[SemanticTarget]:
    data = library_map or load_library_map()
    areas = {area["id"]: area for area in data.get("areas", [])}
    targets = []
    for area_id in data.get("patrol_route", []):
        area = areas.get(area_id)
        if not area:
            continue
        targets.append(
            SemanticTarget(
                area_id=area["id"],
                name=area["name"],
                label=area["label"],
                x=float(area["x"]),
                y=float(area["y"]),
                speech=f"巡检前往{area['name']}：{area['label']}。",
            )
        )
    return targets
