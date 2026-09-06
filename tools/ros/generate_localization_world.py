#!/usr/bin/env python3
"""Add reproducible static landmarks to the existing perception world."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
tree = ET.parse(root/"simulation/worlds/loader_soil_perception.sdf")
world = tree.getroot().find("world")
for i, (x, y, h) in enumerate([(-10,-6,3),(-6,7,4),(0,-8,2),(4,8,3),
                               (10,-7,4),(14,8,2),(-14,3,3),(18,-2,4)]):
    model = ET.SubElement(world, "model", name=f"localization_landmark_{i}")
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x} {y} {h/2} 0 0 {i*.2}"
    link = ET.SubElement(model, "link", name="link")
    for kind in ("collision", "visual"):
        component = ET.SubElement(link, kind, name=kind)
        box = ET.SubElement(ET.SubElement(component, "geometry"), "box")
        ET.SubElement(box, "size").text = f"1.2 1.8 {h}"
tree.write(sys.argv[1], encoding="utf-8", xml_declaration=True)
