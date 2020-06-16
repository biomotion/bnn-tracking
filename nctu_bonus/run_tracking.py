#!/usr/bin/env python3

import argparse
import os
import json
import numpy as np
from pathlib import Path
from tqdm import tqdm
from ab3dmot import AB3DMOT
from argoverse.data_loading.object_label_record import json_label_dict_to_obj_record
from argoverse.utils.se2 import SE2
from transform_utils import (
    yaw_to_quaternion3d, 
    se2_to_yaw, 
    get_B_SE2_A,
    rotmat2d
)
from json_utils import read_json_file, save_json_dict

CLASS_LIST = ['CAR', 'MOTOR', 'PEDESTRIAN']

def yaw_from_bbox_corners(det_corners: np.ndarray) -> float:
    """
    Use basic trigonometry on cuboid to get orientation angle.

        Args:
        -   det_corners: corners of bounding box

        Returns:
        -   yaw
    """
    p1 = det_corners[1]
    p5 = det_corners[5]
    dy = p1[1] - p5[1]
    dx = p1[0] - p5[0]
    # the orientation angle of the car
    yaw = np.arctan2(dy, dx)
    return yaw

def run_tracking(classname, labels_folder, output):
    lis = os.listdir(labels_folder)
    lidar_timestamps = [int(f.split(".")[0]) for f in lis]
    lidar_timestamps.sort()
    ab3dmot = AB3DMOT()
    # print(labels_folder)

    for i, stamp in tqdm(enumerate(lidar_timestamps)):
        with open(labels_folder +"/"+ str(stamp) + ".json") as dets_file:
            j_file = json.load(dets_file)
            # print(j_file)
            # Process each detection instances
            labels = []
            for det in j_file:
                det["center"]["x"] = float(det["center"]["x"])
                det["center"]["y"] = float(det["center"]["y"])
                det["center"]["z"] = float(det["center"]["z"])
                det["length"] = float(det["length"])
                det["width"] = float(det["width"])
                det["height"] = float(det["height"])
                det_obj = json_label_dict_to_obj_record(det)
                # print(det)
                det_bbox = det_obj.as_3d_bbox()
                yaw = yaw_from_bbox_corners(det_bbox)
                labels.append([det_obj.translation[0],
                               det_obj.translation[0],
                               det_obj.translation[0],
                               yaw,
                               det["length"],
                               det["width"],
                               det["height"]])
            labels = np.array(labels)
            dets_all = {
                "dets":labels,
                "info":np.zeros(labels.shape)
            }
            # print(dets_all)
            dets_with_object_id = ab3dmot.update(dets_all, classname)
            # print(dets_with_object_id)
            tracked_labels = []
            for det in dets_with_object_id:
                qx,qy,qz,qw = yaw_to_quaternion3d(det[3])
                tracked_labels.append({
                    "center": {"x": det[0], "y": det[1], "z": det[2]},
                    "rotation": {"x": qx , "y": qy, "z": qz , "w": qw},
                    "length": det[4],
                    "width": det[5],
                    "height": det[6],
                    "track_label_uuid": det[7],
                    "timestamp": stamp,
                    "label_class": classname
                })

            # print(tracked_labels)
            if not os.path.exists(output):
                os.mkdir(output)
            output_name = os.path.join(output, str(stamp) + ".json")
            if os.path.exists(output_name):
                prev_labels = read_json_file(output_name)
                tracked_labels.extend(prev_labels)
            save_json_dict(output_name, tracked_labels)



if __name__ == "__main__":
    labels_folder = "/data/nctu/Second_ground_truth"
    output = "result2"

    # print(os.listdir(labels_folder))
    for classname in CLASS_LIST:
        print("tracking class: ", classname)
        run_tracking(classname, labels_folder, output)

    