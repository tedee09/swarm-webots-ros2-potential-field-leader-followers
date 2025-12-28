#!/usr/bin/env python3
import argparse
import csv
import os
import re
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def sanitize_topic(topic: str) -> str:
    s = topic.strip("/")
    s = s.replace("/", "__")
    s = re.sub(r"[^a-zA-Z0-9_\-\.]+", "_", s)
    return s or "root"

def msg_to_row(msg):
    # Common cases you have:
    # geometry_msgs/Point -> x,y,z
    if hasattr(msg, "x") and hasattr(msg, "y") and hasattr(msg, "z"):
        return {"x": float(msg.x), "y": float(msg.y), "z": float(msg.z)}

    # std_msgs/Bool / Float32 / String -> data
    if hasattr(msg, "data") and not hasattr(msg, "linear"):
        d = msg.data
        return {"data": d if isinstance(d, (int, float, str, bool)) else str(d)}

    # geometry_msgs/Twist -> linear & angular
    if hasattr(msg, "linear") and hasattr(msg, "angular"):
        return {
            "lin_x": float(msg.linear.x), "lin_y": float(msg.linear.y), "lin_z": float(msg.linear.z),
            "ang_x": float(msg.angular.x), "ang_y": float(msg.angular.y), "ang_z": float(msg.angular.z),
        }

    # PoseStamped -> pose position + orientation
    if hasattr(msg, "pose") and hasattr(msg.pose, "position") and hasattr(msg.pose, "orientation"):
        p = msg.pose.position
        q = msg.pose.orientation
        return {
            "x": float(p.x), "y": float(p.y), "z": float(p.z),
            "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
        }

    # PoseArray -> simpan jumlah pose (biar tidak meledak)
    if hasattr(msg, "poses") and isinstance(msg.poses, (list, tuple)):
        return {"count": int(len(msg.poses))}

    # Int32MultiArray -> simpan list sebagai string
    if hasattr(msg, "data") and isinstance(msg.data, (list, tuple)):
        return {"data_list": ",".join(str(int(x)) for x in msg.data)}

    # fallback
    return {"repr": str(msg)}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_path", help="Folder rosbag (yang berisi metadata.yaml dan *.db3)")
    ap.add_argument("-o", "--out_dir", default=None, help="Folder output CSV")
    ap.add_argument("--storage", default="sqlite3", help="Storage id (default sqlite3)")
    ap.add_argument("--topics", nargs="*", default=[], help="Daftar topic yang mau diekspor. Kosong = semua topic.")
    args = ap.parse_args()

    bag_path = args.bag_path.rstrip("/")
    out_dir = args.out_dir or (bag_path + "_csv")
    os.makedirs(out_dir, exist_ok=True)

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=args.storage)
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_and_types}

    wanted = set(args.topics) if args.topics else None

    writers = {}
    files = {}

    def get_writer(topic, fieldnames):
        if topic in writers:
            return writers[topic]
        fname = os.path.join(out_dir, sanitize_topic(topic) + ".csv")
        f = open(fname, "w", newline="")
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        writers[topic] = w
        files[topic] = f
        return w

    count = 0
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if wanted is not None and topic not in wanted:
            continue

        msg_type = type_map.get(topic)
        if msg_type is None:
            continue

        msg_cls = get_message(msg_type)
        msg = deserialize_message(data, msg_cls)

        row = msg_to_row(msg)
        row = {
            "t_sec": t_ns / 1e9,
            "t_ns": int(t_ns),
            **row
        }

        w = get_writer(topic, list(row.keys()))
        w.writerow(row)

        count += 1
        if count % 5000 == 0:
            print(f"Exported {count} messages...")

    for f in files.values():
        f.close()

    print(f"Selesai. Total messages diekspor: {count}")
    print(f"Output CSV di: {out_dir}")

if __name__ == "__main__":
    main()
