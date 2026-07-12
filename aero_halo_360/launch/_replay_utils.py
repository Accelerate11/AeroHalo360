import os


def validate_bag_path(raw_path: str) -> str:
    path = os.path.abspath(os.path.expanduser(raw_path.strip()))
    if not raw_path.strip():
        raise RuntimeError('bag_path 必填')
    if not os.path.exists(path):
        raise RuntimeError(f'rosbag 路径不存在: {path}')
    if os.path.isdir(path) and not os.path.isfile(os.path.join(path, 'metadata.yaml')):
        raise RuntimeError(f'rosbag 目录缺少 metadata.yaml: {path}')
    if not os.path.isdir(path) and not path.endswith(('.db3', '.mcap')):
        raise RuntimeError(f'rosbag 文件必须是 .db3 或 .mcap: {path}')
    return path
