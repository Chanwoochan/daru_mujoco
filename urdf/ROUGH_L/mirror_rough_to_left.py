#!/usr/bin/env python3
from __future__ import annotations

import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


SOURCE_TOKEN = "ROUGH_R"
TARGET_TOKEN = "ROUGH_L"
MIRROR_MATRIX = np.diag([1.0, -1.0, 1.0])


def format_float(value: float) -> str:
    if abs(value) < 1e-15:
        value = 0.0
    return f"{value:.16g}"


def format_vector(values: np.ndarray | list[float]) -> str:
    return " ".join(format_float(float(v)) for v in values)


def parse_vector(text: str, expected: int) -> np.ndarray:
    values = [float(part) for part in text.split()]
    if len(values) != expected:
        raise ValueError(f"Expected {expected} values, got {len(values)}: {text!r}")
    return np.array(values, dtype=float)


def mirror_position(text: str) -> str:
    vec = parse_vector(text, 3)
    vec[1] *= -1.0
    return format_vector(vec)


def mirror_axis(text: str, joint_type: str) -> str:
    vec = parse_vector(text, 3)
    mirrored = MIRROR_MATRIX @ vec
    if joint_type == "hinge":
        mirrored *= -1.0
    return format_vector(mirrored)


def mirror_euler(text: str) -> str:
    vec = parse_vector(text, 3)
    vec[0] *= -1.0
    vec[2] *= -1.0
    return format_vector(vec)


def mirror_quaternion(text: str) -> str:
    quat_wxyz = parse_vector(text, 4)
    quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
    rotation = Rotation.from_quat(quat_xyzw).as_matrix()
    mirrored = MIRROR_MATRIX @ rotation @ MIRROR_MATRIX
    quat_xyzw_out = Rotation.from_matrix(mirrored).as_quat()
    quat_wxyz_out = np.array(
        [quat_xyzw_out[3], quat_xyzw_out[0], quat_xyzw_out[1], quat_xyzw_out[2]],
        dtype=float,
    )
    return format_vector(quat_wxyz_out)


def mirror_fullinertia(text: str) -> str:
    values = parse_vector(text, 6)
    values[3] *= -1.0  # Ixy
    values[5] *= -1.0  # Iyz
    return format_vector(values)


def rename_entries(root: Path) -> None:
    paths = sorted(
        (path for path in root.rglob("*") if SOURCE_TOKEN in path.name),
        key=lambda path: len(path.parts),
        reverse=True,
    )
    for path in paths:
        target = path.with_name(path.name.replace(SOURCE_TOKEN, TARGET_TOKEN))
        path.rename(target)


def transform_xml(xml_path: Path) -> None:
    tree = ET.parse(xml_path)
    root = tree.getroot()

    for elem in root.iter():
        for key, value in list(elem.attrib.items()):
            if SOURCE_TOKEN in value:
                elem.set(key, value.replace(SOURCE_TOKEN, TARGET_TOKEN))

        if "pos" in elem.attrib:
            elem.set("pos", mirror_position(elem.attrib["pos"]))

        if "axis" in elem.attrib:
            joint_type = elem.attrib.get("type", "hinge")
            elem.set("axis", mirror_axis(elem.attrib["axis"], joint_type))

        if "euler" in elem.attrib:
            elem.set("euler", mirror_euler(elem.attrib["euler"]))

        if "quat" in elem.attrib:
            elem.set("quat", mirror_quaternion(elem.attrib["quat"]))

        if "fullinertia" in elem.attrib:
            elem.set("fullinertia", mirror_fullinertia(elem.attrib["fullinertia"]))

    ET.indent(tree, space="  ")
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)


def transform_stl(stl_path: Path) -> None:
    mesh = trimesh.load_mesh(stl_path, file_type="stl", force="mesh", process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected Trimesh, got {type(mesh)!r} for {stl_path}")
    mesh.apply_scale([1.0, -1.0, 1.0])
    mesh.invert()
    mesh.export(stl_path, file_type="stl")


def validate_tree(target_dir: Path) -> None:
    missing = []
    for xml_path in sorted(target_dir.rglob("*.xml")):
        tree = ET.parse(xml_path)
        root = tree.getroot()
        for mesh in root.findall(".//mesh"):
            mesh_file = mesh.attrib.get("file")
            if not mesh_file:
                continue
            resolved = xml_path.parent / mesh_file
            if not resolved.exists():
                missing.append(f"{xml_path.name}: {mesh_file}")
        for model in root.findall(".//model"):
            model_file = model.attrib.get("file")
            if not model_file:
                continue
            resolved = xml_path.parent / model_file
            if not resolved.exists():
                missing.append(f"{xml_path.name}: {model_file}")
        for include in root.findall(".//include"):
            include_file = include.attrib.get("file")
            if not include_file:
                continue
            resolved = xml_path.parent / include_file
            if not resolved.exists():
                missing.append(f"{xml_path.name}: {include_file}")

    if missing:
        raise FileNotFoundError("Missing referenced files:\n" + "\n".join(missing))


def main() -> int:
    source_dir = Path.cwd()
    target_dir = source_dir.parent / TARGET_TOKEN

    if source_dir.name != SOURCE_TOKEN:
        raise RuntimeError(f"Run this script from {SOURCE_TOKEN}, got {source_dir}")
    if target_dir.exists():
        raise FileExistsError(f"Target already exists: {target_dir}")

    shutil.copytree(source_dir, target_dir)
    rename_entries(target_dir)

    for xml_path in sorted(target_dir.rglob("*.xml")):
        transform_xml(xml_path)

    for stl_path in sorted(target_dir.rglob("*.STL")):
        transform_stl(stl_path)

    validate_tree(target_dir)
    print(target_dir)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
