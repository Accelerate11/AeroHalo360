import copy
import hashlib
import math
import os
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple

import yaml


def parse_bool(name: str, raw: str) -> bool:
    value = raw.strip().lower()
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise RuntimeError(f'{name} 必须是 true 或 false，当前值为: {raw}')


def parse_int(name: str, raw: str) -> int:
    try:
        return int(raw.strip())
    except ValueError as exc:
        raise RuntimeError(f'{name} 必须是整数，当前值为: {raw}') from exc


def parse_vector(name: str, raw: str, count: int) -> Tuple[float, ...]:
    parts = raw.strip().strip('[]()').replace(',', ' ').split()
    if len(parts) != count:
        raise RuntimeError(f'{name} 必须包含 {count} 个数值，当前内容为: {raw}')
    try:
        values = tuple(float(item) for item in parts)
    except ValueError as exc:
        raise RuntimeError(f'{name} 包含无法解析的数值: {raw}') from exc
    if not all(math.isfinite(item) for item in values):
        raise RuntimeError(f'{name} 只能包含有限数值: {raw}')
    return values


def normalize_config_path(path: str, name: str) -> str:
    normalized = os.path.abspath(os.path.expanduser(path.strip()))
    if not os.path.isfile(normalized):
        raise RuntimeError(f'{name} 文件不存在或不是普通文件: {normalized}')
    return normalized


def file_sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(65536), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _extract_ros_parameters(document: Any, path: str) -> Dict[str, Any]:
    if not isinstance(document, dict):
        raise RuntimeError(f'配置文件顶层必须是映射: {path}')
    wildcard = document.get('/**')
    if not isinstance(wildcard, dict):
        raise RuntimeError(f'配置文件缺少 /** 节点: {path}')
    parameters = wildcard.get('ros__parameters')
    if not isinstance(parameters, dict):
        raise RuntimeError(f'配置文件缺少 /**/ros__parameters: {path}')
    return copy.deepcopy(parameters)


def load_parameter_file(path: str) -> Dict[str, Any]:
    try:
        with open(path, encoding='utf-8') as handle:
            document = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f'无法读取 YAML 配置 {path}: {exc}') from exc
    return _extract_ros_parameters(document, path)


def deep_merge(base: Dict[str, Any], overlay: Mapping[str, Any]) -> Dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, Mapping) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def flatten_parameters(values: Mapping[str, Any], prefix: str = '') -> Dict[str, Any]:
    output: Dict[str, Any] = {}
    for key, value in values.items():
        full_key = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(value, Mapping):
            output.update(flatten_parameters(value, full_key))
        elif isinstance(value, list) and not value:
            # launch_ros 无法为无元素列表推断参数类型，由节点强类型默认值声明。
            continue
        else:
            output[full_key] = value
    return output


def build_effective_parameters(
    base_path: str,
    profile_path: Optional[str],
    cli_overrides: Mapping[str, Any],
    installation_path: Optional[str] = None,
) -> Dict[str, Any]:
    merged = load_parameter_file(base_path)
    if profile_path:
        merged = deep_merge(merged, load_parameter_file(profile_path))
    if installation_path:
        merged = deep_merge(merged, load_parameter_file(installation_path))
    flattened = flatten_parameters(merged)
    flattened.update(cli_overrides)
    return flattened


def optional_cli_overrides(
    raw_values: Mapping[str, str],
    bool_keys: Iterable[str],
    int_keys: Iterable[str],
) -> Dict[str, Any]:
    bool_set = set(bool_keys)
    int_set = set(int_keys)
    output: Dict[str, Any] = {}
    for key, raw in raw_values.items():
        value = raw.strip()
        if value == '':
            continue
        if key in bool_set:
            output[key] = parse_bool(key, value)
        elif key in int_set:
            output[key] = parse_int(key, value)
        else:
            output[key] = value
    return output
