#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from app.book_catalog import BookCatalog
from app.ci302_commands import BOOK_FRAME_BY_ID, build_ci302_commands
from app.utils import load_config

catalog=BookCatalog(ROOT/"config")
points=load_config(ROOT/"config","points").get("points",{})
missions=load_config(ROOT/"config","missions").get("missions",{})
commands=build_ci302_commands(catalog)
frames=[item["frame"] for item in commands]
if len(frames)!=len(set(frames)): raise RuntimeError("CI302码表存在重复帧")
if len(BOOK_FRAME_BY_ID)!=15 or set(BOOK_FRAME_BY_ID)!={int(book["id"]) for book in catalog.books}:
    raise RuntimeError("CI302书名帧必须与15本书ID一一对应")
if len(set(BOOK_FRAME_BY_ID.values()))!=15: raise RuntimeError("15个CI302书名帧不唯一")
for book in catalog.books:
    if not str(book.get("summary","")).strip(): raise RuntimeError(f"图书{book['id']}缺少固定介绍")
    if book["shelf_point"] not in points: raise RuntimeError(f"图书{book['id']}导航点不存在：{book['shelf_point']}")
if "FULL_PATROL" not in missions: raise RuntimeError("缺少真正的FULL_PATROL任务")
full=next(item for item in commands if item["id"]=="FULL_PATROL")
if full.get("mission")!="FULL_PATROL": raise RuntimeError("CI302全图巡检映射错误")
secret_pattern=re.compile(r"\b[0-9a-fA-F]{32}\.[A-Za-z0-9_-]{8,}\b")
for suffix in ("*.py","*.sh","*.json","*.yaml","*.yml"):
    for path in ROOT.rglob(suffix):
        if secret_pattern.search(path.read_text(encoding="utf-8",errors="ignore")):
            raise RuntimeError(f"源码疑似包含硬编码API密钥：{path}")
print(f"voice config OK: books={len(catalog.books)} ci302_frames={len(frames)} source={catalog.source_path}")
