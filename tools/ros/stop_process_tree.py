#!/usr/bin/env python3
"""Stop one owned launcher and its descendants, including ROS CLI children."""
import argparse,os,signal,time
from pathlib import Path

def processes():
    found={}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            fields=(path/'stat').read_text().split(') ',1)[1].split()
            found[int(path.name)]=(int(fields[1]),fields[19],fields[0])
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return found

def stop_tree(root,grace=3.):
    snapshot=processes();owned={root} if root in snapshot else set()
    while True:
        children={pid for pid,(parent,_,_) in snapshot.items() if parent in owned}
        if children<=owned:break
        owned|=children
    def alive():
        current=processes()
        return {pid for pid in owned if pid in current and current[pid][1]==snapshot[pid][1] and current[pid][2]!='Z'}
    def send(sig):
        # Snapshot start times prevent signaling a reused PID.
        for pid in alive():
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
    send(signal.SIGINT);deadline=time.monotonic()+grace
    while alive() and time.monotonic()<deadline:time.sleep(.05)
    send(signal.SIGTERM);deadline=time.monotonic()+1
    while alive() and time.monotonic()<deadline:time.sleep(.05)
    send(signal.SIGKILL)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('pid',type=int);args=parser.parse_args()
    if args.pid<=1 or args.pid==os.getpid():raise ValueError('invalid owned launcher PID')
    stop_tree(args.pid)
