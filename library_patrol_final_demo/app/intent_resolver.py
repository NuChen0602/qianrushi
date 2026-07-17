from dataclasses import dataclass

from app.book_catalog import normalize_book_text


INTRO_WORDS = ("介绍", "简介", "讲讲", "讲一下", "说说", "了解", "讲一讲")
FIND_WORDS = ("寻找", "找书", "找一下", "帮我找", "带我去找", "带路", "前往书架", "去书架找", "找")
NEGATIONS = ("不要", "别", "不用", "不需要", "不是找", "取消", "停止")


@dataclass(frozen=True)
class LocalIntent:
    kind: str
    book_id: int | None = None
    reason: str = ""


def has_introduction_intent(text):
    normalized = normalize_book_text(text)
    return any(word in normalized for word in INTRO_WORDS)


def has_explicit_find_intent(text):
    normalized = normalize_book_text(text)
    return any(word in normalized for word in FIND_WORDS)


def has_negated_find_intent(text):
    normalized = normalize_book_text(text)
    return any(word in normalized for word in NEGATIONS)


def classify_local_intent(text, catalog):
    intro = has_introduction_intent(text)
    find = has_explicit_find_intent(text)
    negated = has_negated_find_intent(text)
    book = catalog.match_text(text)
    if intro and (negated or not find):
        return LocalIntent("introduce_book", book["id"] if book else None, "explicit introduction")
    if find and not intro and not negated:
        return LocalIntent("find_book", book["id"] if book else None, "explicit find")
    if find and intro:
        return LocalIntent("clarify", book["id"] if book else None, "conflicting intents")
    if find and negated:
        return LocalIntent("other", book["id"] if book else None, "negated find")
    if book:
        return LocalIntent("book_mention", book["id"], "book without action")
    return LocalIntent("other", None, "no deterministic intent")


def enforce_execution_safety(text, decision, catalog):
    """Return a decision that can never navigate without explicit positive find words."""
    local = classify_local_intent(text, catalog)
    book = catalog.get(decision.get("book_id")) if isinstance(decision, dict) else None
    if local.book_id is not None:
        book = catalog.get(local.book_id)
    if local.kind == "introduce_book":
        if not book:
            return {"action": "chat", "book_id": None, "reply": "请再说一下想介绍的书名。"}
        return {"action": "introduce_book", "book_id": book["id"], "reply": ""}
    if local.kind == "find_book":
        if not book:
            return {"action": "chat", "book_id": None, "reply": "请再说一下想寻找的书名。"}
        return {"action": "find_book", "book_id": book["id"], "reply": ""}
    if decision.get("action") == "find_book" or str(decision.get("command_id", "")).startswith("FIND_"):
        return {"action": "chat", "book_id": None, "reply": "请明确说寻找哪一本书。"}
    return decision
