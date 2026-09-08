#!/usr/bin/env python3
"""Export the offline equidistant products as ROS MCAP for replay/Foxglove."""
import argparse,json,math
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time


def export(dataset):
    manifest=json.loads((dataset/'manifest.json').read_text());writer=rosbag2_py.SequentialWriter();topics=set();count=0
    writer.open(rosbag2_py.StorageOptions(uri=str(dataset/'fisheye_mcap'),storage_id='mcap',
      storage_config_uri=str(dataset/'mcap_storage.yaml')),rosbag2_py.ConverterOptions('',''))
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(dataset/'mcap'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    metadata={topic.name:topic for topic in reader.get_all_topics_and_types()};pending=None
    def write(topic,msg,stamp):
        nonlocal count
        if topic not in topics:
            kind=msg.__class__.__module__.split('.')[0]+'/msg/'+msg.__class__.__name__
            writer.create_topic(rosbag2_py.TopicMetadata(id=len(topics),name=topic,type=kind,serialization_format='cdr'));topics.add(topic)
        writer.write(topic,serialize_message(msg),stamp);count+=1
    try:
        for entry in manifest['frames']:
            stem=Path(entry['file']).stem;data=np.load(dataset/'bev'/f'{stem}.npz');stamp=entry['stamp_ns'];t=Time(sec=stamp//10**9,nanosec=stamp%10**9)
            while pending is not None or reader.has_next():
                topic,payload,source_stamp=pending if pending is not None else reader.read_next();pending=None
                if source_stamp>stamp:
                    pending=(topic,payload,source_stamp);break
                if topic in ('/tf','/loader/terrain_state'):
                    if topic not in topics:
                        writer.create_topic(rosbag2_py.TopicMetadata(id=len(topics),name=topic,type=metadata[topic].type,serialization_format='cdr'));topics.add(topic)
                    writer.write(topic,payload,source_stamp);count+=1
            for name in ('front','left','rear','right'):
                frame=f'camera_{name}_optical';base=f'/loader/fisheye/{name}'
                for suffix,encoding in [('rgb','rgb8'),('range','32FC1'),('semantic','mono8'),('instance','16UC1'),('valid','mono8')]:
                    array=data[f'{name}_{suffix}']
                    if suffix=='valid':array=array.astype(np.uint8)*255
                    array=np.ascontiguousarray(array);message=Image();message.header.stamp=t;message.header.frame_id=frame
                    message.height,message.width=array.shape[:2];message.encoding=encoding;message.is_bigendian=0
                    message.step=array.strides[0];message.data=array.tobytes();write(f'{base}/{suffix}',message,stamp)
                info=CameraInfo();info.header.stamp=t;info.header.frame_id=frame;info.width=640;info.height=480
                focal=639/math.radians(160);info.distortion_model='equidistant';info.d=[0.,0.,0.,0.]
                info.k=[focal,0.,319.5,0.,focal,239.5,0.,0.,1.];info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
                write(f'{base}/camera_info',info,stamp)
            write('/loader/fisheye/visible_boxes',String(data=(dataset/'bev'/f'{stem}_boxes.json').read_text()),stamp)
            write('/clock',Clock(clock=t),stamp)
    finally:writer.close()
    (dataset/'bev'/'mcap_export.json').write_text(json.dumps({'groups':len(manifest['frames']),'messages':count,'topics':sorted(topics),
      'range_convention':'radial metres, not optical Z','instance_ids':'frame-local Gazebo instances','projection':'equidistant K and zero polynomial distortion'},indent=2))
    print(f'Exported {len(manifest["frames"])} fisheye groups to MCAP')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('dataset',type=Path);export(parser.parse_args().dataset)
