import json
import re
import unicodedata
from pathlib import Path

from app.utils import load_config


def normalize_book_text(value):
    text = unicodedata.normalize("NFKC", str(value or "")).lower()
    text = text.replace("华尔登湖", "瓦尔登湖")
    return re.sub(r"[\s《》()（）\[\]【】,，。.!！?？:：;；·\-]", "", text)


class BookCatalog:
    """Strict merge: metadata from books.json, introductions only from database."""

    def __init__(self, config_dir):
        config_dir = Path(config_dir)
        data = load_config(config_dir, "books")
        source = (config_dir / str(data.get("source", ""))).resolve()
        if not source.is_file():
            raise RuntimeError(f"图书介绍数据库不存在：{source}")
        duplicate_keys=[]
        def unique_object(pairs):
            result={}
            for key,value in pairs:
                if key in result: duplicate_keys.append(key)
                result[key]=value
            return result
        raw = json.loads(source.read_text(encoding="utf-8"),object_pairs_hook=unique_object)
        if duplicate_keys: raise RuntimeError(f"图书介绍数据库存在重复字段或ID：{sorted(set(duplicate_keys))}")
        configured_items=list(data.get("books", []))
        configured_ids=[int(item["id"]) for item in configured_items]
        if len(configured_ids)!=len(set(configured_ids)):
            raise RuntimeError("books.json存在重复图书ID")
        configured = {int(item["id"]): dict(item) for item in configured_items}
        source_ids = {int(key) for key in raw}
        config_ids = set(configured)
        if source_ids != config_ids:
            raise RuntimeError(
                f"图书ID不一致：数据库独有={sorted(source_ids-config_ids)}，"
                f"配置独有={sorted(config_ids-source_ids)}"
            )
        shelves = data.get("shelves", {})
        if len(source_ids) != 15:
            raise RuntimeError(f"正式书库必须包含15本书，当前为{len(source_ids)}本")

        books = []
        normalized_names=[normalize_book_text(raw[str(book_id)].get("title","")) for book_id in sorted(source_ids)]
        if len(normalized_names)!=len(set(normalized_names)): raise RuntimeError("图书介绍数据库存在重复书名")
        names = set(normalized_names)
        aliases = {}
        for book_id in sorted(source_ids):
            item = raw[str(book_id)]
            extra = configured[book_id]
            name = str(item.get("title", "")).strip()
            summary = str(item.get("summary", "")).strip()
            shelf_id = str(item.get("shelf_id", "")).strip()
            shelf = shelves.get(shelf_id)
            if not name or not summary:
                raise RuntimeError(f"图书 {book_id} 缺少名称或固定介绍")
            if not shelf or not shelf.get("point") or not shelf.get("name"):
                raise RuntimeError(f"图书 {book_id} 的书架 {shelf_id!r} 配置不完整")
            normalized_name = normalize_book_text(name)
            book_aliases = [str(x).strip() for x in extra.get("aliases", []) if str(x).strip()]
            for alias in book_aliases:
                key = normalize_book_text(alias)
                if not key: raise RuntimeError(f"图书 {book_id} 存在空别名")
                if key in names and key != normalized_name:
                    raise RuntimeError(f"别名 {alias} 与其他图书书名重复")
                if key in aliases and aliases[key] != book_id:
                    raise RuntimeError(f"重复别名：{alias} 同时属于 {aliases[key]} 和 {book_id}")
                aliases[key] = book_id
            books.append({
                "id": book_id,
                "name": name,
                "aliases": book_aliases,
                "category": item.get("category_cn", item.get("category", "")),
                "shelf_id": shelf_id,
                "shelf_point": str(shelf["point"]),
                "shelf_name": str(shelf["name"]),
                "rank": int(extra.get("rank", 1)),
                "summary": summary,
            })
        self.source_path = source
        self.books = books
        self.by_id = {item["id"]: item for item in books}

    def get(self, book_id):
        try:
            return self.by_id.get(int(book_id))
        except (TypeError, ValueError):
            return None

    def match_text(self, text):
        normalized = normalize_book_text(text)
        book_context = any(word in normalized for word in ("书", "小说", "教材", "介绍", "寻找", "找", "讲"))
        matches = []
        for book in self.books:
            candidates = [book["name"], *book.get("aliases", [])]
            score = 0
            for candidate in candidates:
                key = normalize_book_text(candidate)
                # Short/common aliases are accepted only with explicit book context.
                if len(key) < 4 and not book_context:
                    continue
                if key and key in normalized:
                    score = max(score, len(key))
            if score:
                matches.append((score, book))
        if not matches:
            return None
        best_score = max(score for score, _ in matches)
        winners = [book for score, book in matches if score == best_score]
        return winners[0] if len(winners) == 1 else None

    def public_list(self):
        return [dict(book) for book in self.books]
