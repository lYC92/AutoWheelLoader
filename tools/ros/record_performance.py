#!/usr/bin/env python3
"""Wall-time RTF and Gazebo process resource trace; no simulated control input."""
import argparse,csv,json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--pid',type=int,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();rclpy.init();node=Node('loader_performance_recorder');sim=[0.]
    node.create_subscription(Clock,'/clock',lambda m:sim.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),1)
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    start=time.monotonic();previous=start;previous_sim=0.;rows=[]
    try:
        with args.output.open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['wall_seconds','sim_seconds','rtf','gazebo_cpu_seconds','gazebo_rss_mib','threads'])
            while running:
                rclpy.spin_once(node,timeout_sec=.1);now=time.monotonic()
                if now-previous<1:continue
                try:
                    fields=Path(f'/proc/{args.pid}/stat').read_text().split(') ',1)[1].split()
                    cpu=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK')
                    rss=int(fields[21])*os.sysconf('SC_PAGE_SIZE')/2**20;threads=int(fields[17])
                except FileNotFoundError:break
                row=[now-start,sim[0],(sim[0]-previous_sim)/(now-previous),cpu,rss,threads]
                writer.writerow(row);stream.flush();rows.append(row);previous=now;previous_sim=sim[0]
    finally:
        if rows:
            args.output.with_suffix('.json').write_text(json.dumps({'samples':len(rows),'total_wall_s':rows[-1][0],'total_sim_s':rows[-1][1],
              'aggregate_rtf':rows[-1][1]/rows[-1][0],'peak_gazebo_rss_mib':max(r[4] for r in rows),
              'note':'includes startup, pauses and offline processing; cycle reports separately measure active task duration'},indent=2))
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
