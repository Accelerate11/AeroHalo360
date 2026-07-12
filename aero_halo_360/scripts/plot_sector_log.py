#!/usr/bin/env python3
import argparse
import csv


def main():
    parser = argparse.ArgumentParser(description='读取包含 72 个扇区距离的 CSV 文件并绘图')
    parser.add_argument('csv_path')
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit('未安装 matplotlib，请先安装该依赖') from exc

    rows = []
    with open(args.csv_path, newline='', encoding='utf-8') as handle:
        reader = csv.reader(handle)
        for row in reader:
            values = [float(item) for item in row[:72]]
            if len(values) == 72:
                rows.append(values)

    if not rows:
        raise SystemExit('文件中没有有效的 72 扇区数据')

    latest = rows[-1]
    angles = [i * 5 for i in range(72)]
    plt.figure(figsize=(10, 4))
    plt.plot(angles, latest, marker='o')
    plt.xlabel('机体顺时针方向角（度）')
    plt.ylabel('障碍物距离（厘米）')
    plt.grid(True)
    plt.title('AeroHalo360 扇区距离')
    plt.show()


if __name__ == '__main__':
    main()
