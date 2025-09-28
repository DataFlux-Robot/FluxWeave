"""在 STL 文件中嵌入 URDF Kitchen 元数据的实用函数集合。"""
from __future__ import annotations

import atexit
import os
import tempfile
from pathlib import Path
from typing import Callable, Optional, Tuple

METADATA_START = "###URDF_KITCHEN_METADATA_START###"
METADATA_END = "###URDF_KITCHEN_METADATA_END###"
_DEFAULT_ENCODING = "utf-8"
_CLEANUP_PATHS: set[str] = set()


def _is_ascii_stl(data: bytes) -> bool:
    """尽力判断给定的 STL 内容是否为 ASCII 格式。"""
    if len(data) < 5:
        return False
    if data[:5].lower() != b"solid":
        return False
    return b"\x00" not in data[:512]


def _decode_text(data: bytes) -> str:
    try:
        return data.decode(_DEFAULT_ENCODING)
    except UnicodeDecodeError:
        return data.decode("latin-1")


def _encode_text(text: str) -> bytes:
    try:
        return text.encode(_DEFAULT_ENCODING)
    except UnicodeEncodeError:
        return text.encode("latin-1")


def extract_metadata_text(path: str) -> Optional[str]:
    """若存在，则返回 STL 文件中嵌入的元数据 XML。"""
    data = Path(path).read_bytes()
    if not _is_ascii_stl(data):
        return None
    text = _decode_text(data)
    start = text.find(METADATA_START)
    if start == -1:
        return None
    start += len(METADATA_START)
    end = text.find(METADATA_END, start)
    if end == -1:
        metadata = text[start:]
    else:
        metadata = text[start:end]
    return metadata.strip() or None


def strip_metadata_text(text: str) -> str:
    """从给定的 ASCII STL 文本中移除所有元数据块。"""
    while True:
        start = text.find(METADATA_START)
        if start == -1:
            break
        end = text.find(METADATA_END, start)
        if end == -1:
            text = text[:start]
            break
        end += len(METADATA_END)
        text = text[:start].rstrip("\r\n ") + "\n" + text[end:].lstrip("\r\n ")
    return text.rstrip() + "\n"


def write_embedded_metadata(path: str, metadata_xml: str) -> None:
    """将元数据 XML 写入 ASCII STL 文件，并覆盖原有的标记块。"""
    file_path = Path(path)
    data = file_path.read_bytes()
    if not _is_ascii_stl(data):
        raise ValueError("当前仅支持在 ASCII STL 文件中嵌入元数据")
    text = _decode_text(data)
    text = strip_metadata_text(text)
    if text and not text.endswith("\n"):
        text += "\n"
    metadata_clean = metadata_xml.strip()
    block = f"{METADATA_START}\n{metadata_clean}\n{METADATA_END}\n"
    if not text.endswith("\n\n"):
        text += "\n"
    text += block
    file_path.write_text(text, encoding=_DEFAULT_ENCODING)


def prepare_stl_for_read(path: str) -> Tuple[str, Callable[[], None]]:
    """返回可直接供 STL 读取器使用的文件路径。

    如果文件中嵌入了元数据，则会生成剥离后的临时副本；
    使用完毕后需调用返回的清理回调。"""
    original = Path(path)
    data = original.read_bytes()
    if not _is_ascii_stl(data):
        return str(original), lambda: None
    text = _decode_text(data)
    if METADATA_START not in text:
        return str(original), lambda: None
    clean_text = strip_metadata_text(text)
    tmp = tempfile.NamedTemporaryFile("w", suffix=".stl", delete=False, encoding=_DEFAULT_ENCODING)
    tmp.write(clean_text)
    tmp_path = tmp.name
    tmp.close()
    _CLEANUP_PATHS.add(tmp_path)
    return tmp_path, lambda p=tmp_path: _cleanup_temp_file(p)


def _cleanup_temp_file(path: str) -> None:
    if path in _CLEANUP_PATHS and os.path.exists(path):
        try:
            os.remove(path)
        finally:
            _CLEANUP_PATHS.discard(path)


def _cleanup_all_temp_files() -> None:
    for path in list(_CLEANUP_PATHS):
        try:
            os.remove(path)
        except OSError:
            pass
    _CLEANUP_PATHS.clear()


def copy_without_metadata(src: str, dest: str) -> None:
    src_path = Path(src)
    dest_path = Path(dest)
    data = src_path.read_bytes()
    dest_path.parent.mkdir(parents=True, exist_ok=True)
    if _is_ascii_stl(data) and METADATA_START.encode() in data:
        text = _decode_text(data)
        clean_text = strip_metadata_text(text)
        dest_path.write_text(clean_text, encoding=_DEFAULT_ENCODING)
    else:
        dest_path.write_bytes(data)


atexit.register(_cleanup_all_temp_files)
