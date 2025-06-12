#!/usr/bin/env python3
import rosbag
import argparse

def main():
    p = argparse.ArgumentParser(
        description="计算 rosbag 中各话题的平均发布频率"
    )
    p.add_argument("bagfile", help="rosbag 文件路径")
    p.add_argument(
        "-t","--topic",
        action="append",
        help="只统计这个话题（可多次使用，省略则统计所有话题）"
    )
    args = p.parse_args()

    bag = rosbag.Bag(args.bagfile, 'r')
    start = bag.get_start_time()
    end  = bag.get_end_time()
    duration = end - start
    if duration <= 0:
        print("无法获取 bag 的有效时长，请检查文件")
        return

    # get_type_and_topic_info()[1] 返回 dict: topic -> TopicTuple
    info = bag.get_type_and_topic_info()[1]

    # 如果指定了 -t，就只统计那些话题，否则全统计
    topics = args.topic if args.topic else list(info.keys())

    print(f"bag 时长: {duration:.3f} s, 话题数: {len(topics)}\n")
    print(f"{'话题名称':<60} {'消息数':>8}  {'频率 (Hz)':>8}")
    print("-"*80)
    for top in topics:
        if top not in info:
            print(f"{top:<60}  没有在 bag 中找到")
            continue
        ti = info[top]
        count = ti.message_count
        freq  = count / duration
        print(f"{top:<60} {count:8d}    {freq:8.2f}")

    bag.close()

if __name__ == "__main__":
    main()