#!/usr/bin/env python3
import json
import pathlib
import re
import stat
import sys
import xml.etree.ElementTree as element_tree

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
EXCLUDED_PARTS = {'.git', 'release', 'build', 'install', 'log', '__pycache__'}
TEXT_SUFFIXES = {
    '.md', '.txt', '.py', '.cpp', '.hpp', '.h', '.xml', '.yaml', '.yml',
    '.json', '.sh', '.service', '.in', '.timer', '.example', '.rviz',
}
FORBIDDEN = (
    '/home/' + 'accelerate',
    '190032000F51' + '333530373733',
    '192.168.1.' + '158',
)


def release_files():
    for path in ROOT.rglob('*'):
        if not path.is_file() or any(part in EXCLUDED_PARTS for part in path.parts):
            continue
        if path.suffix == '.pyc':
            continue
        yield path


def check_text_hygiene(path, errors):
    if path.suffix not in TEXT_SUFFIXES and path.name != 'LICENSE':
        return
    data = path.read_bytes()
    relative = path.relative_to(ROOT)
    if data.startswith(b'\xef\xbb\xbf'):
        errors.append(f'{relative}: 禁止 UTF-8 BOM')
    if b'\r\n' in data or b'\r' in data:
        errors.append(f'{relative}: 必须使用 LF 行尾')
    try:
        text = data.decode('utf-8')
    except UnicodeDecodeError as exc:
        errors.append(f'{relative}: UTF-8 解码失败: {exc}')
        return
    for value in FORBIDDEN:
        if value in text:
            errors.append(f'{relative}: 包含禁止的真实硬件或个人值 {value}')


def check_structured_files(path, errors):
    relative = path.relative_to(ROOT)
    try:
        if path.suffix in ('.yaml', '.yml'):
            yaml.safe_load(path.read_text(encoding='utf-8'))
        elif path.suffix == '.json':
            json.loads(path.read_text(encoding='utf-8'))
        elif path.name == 'package.xml':
            element_tree.parse(path)
    except Exception as exc:  # noqa: BLE001
        errors.append(f'{relative}: 结构化文件解析失败: {exc}')


def check_markdown_links(path, errors):
    if path.suffix != '.md':
        return
    text = path.read_text(encoding='utf-8')
    for target in re.findall(r'\[[^\]]+\]\(([^)]+)\)', text):
        target = target.strip().split('#', 1)[0]
        if not target or '://' in target or target.startswith(('mailto:', '#')):
            continue
        linked = (path.parent / target).resolve()
        if not linked.exists():
            errors.append(f'{path.relative_to(ROOT)}: 本地链接不存在: {target}')


def check_executable(path, errors):
    if path.suffix not in ('.py', '.sh'):
        return
    first_line = path.read_bytes().splitlines()[:1]
    if first_line and first_line[0].startswith(b'#!'):
        if not path.stat().st_mode & stat.S_IXUSR:
            errors.append(f'{path.relative_to(ROOT)}: shebang 脚本缺少可执行位')


def main():
    errors = []
    for path in release_files():
        check_text_hygiene(path, errors)
        check_structured_files(path, errors)
        check_markdown_links(path, errors)
        check_executable(path, errors)

    package = element_tree.parse(ROOT / 'aero_halo_360' / 'package.xml')
    version = package.findtext('version')
    if version != '0.2.1':
        errors.append(f'package.xml 版本应为 0.2.1，当前为 {version}')

    if errors:
        print('静态检查失败：', file=sys.stderr)
        for error in errors:
            print(f'- {error}', file=sys.stderr)
        return 1
    print('静态检查通过：UTF-8/LF、YAML/JSON/XML、本地链接、可执行位和敏感默认值。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
