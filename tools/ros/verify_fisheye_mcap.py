#!/usr/bin/env python3
"""Check the replayable fisheye product against every offline source array."""
import argparse,json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def verify(dataset):
    entries=json.loads((dataset/'manifest.json').read_text())['frames'];frames={e['stamp_ns']:e['file'] for e in entries}
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(dataset/'fisheye_mcap'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()};groups={};cached_stamp=None;arrays=None;previous=-1
    while reader.has_next():
        topic,payload,stamp=reader.read_next()
        if stamp<previous:raise RuntimeError('MCAP replay timestamps are not ordered')
        previous=stamp
        if types[topic].__name__!='Image':continue
        message=deserialize_message(payload,types[topic]);parts=topic.split('/');name,suffix=parts[-2:]
        if stamp!=message.header.stamp.sec*10**9+message.header.stamp.nanosec:raise RuntimeError('fisheye image stamp mismatch')
        if stamp!=cached_stamp:arrays=np.load(dataset/'bev'/frames[stamp]);cached_stamp=stamp
        expected=arrays[f'{name}_{suffix}'];expected=expected.astype(np.uint8)*255 if suffix=='valid' else expected
        dtype={'rgb8':np.uint8,'mono8':np.uint8,'16UC1':np.dtype('<u2'),'32FC1':np.dtype('<f4')}[message.encoding]
        actual=np.frombuffer(message.data,dtype=dtype).reshape(expected.shape)
        np.testing.assert_array_equal(actual,expected);groups.setdefault(stamp,set()).add(topic)
    if len(groups)!=len(frames) or any(len(g)!=20 for g in groups.values()):raise RuntimeError('missing fisheye products')
    (dataset/'bev'/'mcap_verification.json').write_text(json.dumps({'status':'passed','groups':len(groups),'arrays_checked':sum(map(len,groups.values()))},indent=2))
    print(f'PASS fisheye MCAP replay: {len(groups)} groups, {len(groups)*20} arrays identical')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('dataset',type=Path);verify(parser.parse_args().dataset)
