# Copyright 2026 Duatic AG
# Duatic Commercial License 1.0 (DCL-1)

"""
Shared URDF / SRDF checks and their command line entry point.

Usage (paths are relative to the share directory of PACKAGE):
    python3 -m duatic_helpers.test_descriptions --package PKG --files urdf/robot.urdf.xacro
    python3 -m duatic_helpers.test_descriptions --package PKG --files srdf/robot.srdf.xacro urdf/robot.urdf.xacro

One file is checked as a URDF, two files as an SRDF followed by the URDF it belongs to.

Unknown options (e.g. --junit-xml=FILE) are forwarded to pytest.
Normally run through the CMake functions duatic_add_urdf_tests / duatic_add_srdf_test.
"""

import argparse
import os
import sys
import unittest
from xml.dom.minidom import parseString

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

VALID_COLLISION_REASONS = {"Adjacent", "Always", "Never", "Default", "User"}


def _children(node, tag):
    """Direct child elements of `node` with the given tag"""
    return [c for c in node.childNodes if c.nodeType == c.ELEMENT_NODE and c.tagName == tag]


def _names(dom, tag):
    """Set of the `name` attributes of the `tag` elements directly under <robot>"""
    # Not getElementsByTagName: <ros2_control> and <transmission> contain <joint> tags of their own
    return {e.getAttribute("name") for e in _children(dom.documentElement, tag)}


class _DescriptionTestCase(unittest.TestCase):
    # Set by _ParamsPlugin: .package and .files (relative to the package share directory)
    params = None

    def setUp(self):
        self.pkg_share = get_package_share_directory(self.params.package)

    def load_xacro(self, rel_path):
        """Processes a xacro file (relative to the package share directory) into a DOM"""
        try:
            return parseString(xacro.process_file(os.path.join(self.pkg_share, rel_path)).toxml())
        except Exception as e:
            self.fail(f"Failed to process xacro file {rel_path}: {e}")


class TestURDF(_DescriptionTestCase):
    def setUp(self):
        super().setUp()
        self.urdf_file = self.params.files[0]
        self.dom = self.load_xacro(self.urdf_file)

    def test_meshes(self):
        """Checks that every referenced mesh file exists"""
        for mesh in self.dom.getElementsByTagName("mesh"):
            path = mesh.getAttribute("filename")
            if not path:
                continue
            # package:// only, file:// is not supported anymore
            if path.startswith("package://"):
                pkg_name, rel_path = path.replace("package://", "").split("/", 1)
                try:
                    path = os.path.join(get_package_share_directory(pkg_name), rel_path)
                except Exception:
                    self.fail(
                        f"Could not resolve package in mesh path: {mesh.getAttribute('filename')}"
                    )
            self.assertTrue(
                os.path.exists(path),
                f"Mesh file not found: {path} (referenced in {self.urdf_file})",
            )

    def test_inertials(self):
        """Checks that every link has at most one <inertial> and a non-negative mass"""
        for link in self.dom.getElementsByTagName("link"):
            name = link.getAttribute("name")
            inertials = link.getElementsByTagName("inertial")
            self.assertLessEqual(
                len(inertials), 1, f"Link '{name}' in {self.urdf_file} has multiple <inertial> tags"
            )
            if inertials and (masses := inertials[0].getElementsByTagName("mass")):
                self.assertGreaterEqual(
                    float(masses[0].getAttribute("value")),
                    0.0,
                    f"Link '{name}' in {self.urdf_file} has negative mass",
                )
                # You might verify inertia tensor basics here too if desired,
                # but parsing the matrix elements is more involved.

    def test_joint_limits(self):
        """Checks that revolute/prismatic joints have complete, sane limits"""
        for joint in self.dom.getElementsByTagName("joint"):
            # Fixed, continuous, floating and planar joints don't strictly need limits
            if joint.getAttribute("type") not in ["revolute", "prismatic"]:
                continue

            name = joint.getAttribute("name")
            limits = joint.getElementsByTagName("limit")
            self.assertTrue(limits, f"Joint '{name}' in {self.urdf_file} missing <limit> tag")

            value = {}
            for attr in ["effort", "velocity", "lower", "upper"]:
                raw = limits[0].getAttribute(attr)
                self.assertTrue(raw, f"Joint '{name}' missing {attr} limit in {self.urdf_file}")
                value[attr] = float(raw)

            self.assertLessEqual(
                value["lower"],
                value["upper"],
                f"Joint '{name}' lower limit > upper limit in {self.urdf_file}",
            )
            for attr in ["velocity", "effort"]:
                self.assertGreater(
                    value[attr],
                    0.0,
                    f"Joint '{name}' {attr} limit must be positive in {self.urdf_file}",
                )


class TestSRDF(_DescriptionTestCase):
    def setUp(self):
        super().setUp()
        srdf_file, urdf_file = self.params.files
        self.srdf = self.load_xacro(srdf_file)

        urdf = self.load_xacro(urdf_file)
        self.links = _names(urdf, "link")
        self.joints = _names(urdf, "joint")
        self.parent_of = {}  # child link -> parent link, used to check that chains are connected
        self.joint_of = {}  # child link -> name of the joint connecting it to its parent
        self.limits = {}  # joint name -> (lower, upper), only for joints defining both
        for joint in _children(urdf.documentElement, "joint"):
            name = joint.getAttribute("name")
            child = joint.getElementsByTagName("child")[0].getAttribute("link")
            self.parent_of[child] = joint.getElementsByTagName("parent")[0].getAttribute("link")
            self.joint_of[child] = name
            limit = joint.getElementsByTagName("limit")
            if limit and limit[0].hasAttribute("lower") and limit[0].hasAttribute("upper"):
                self.limits[name] = (
                    float(limit[0].getAttribute("lower")),
                    float(limit[0].getAttribute("upper")),
                )

    def _group_joints(self, group_name, _seen=()):
        """Returns the names of all joints belonging to a group (incl. chains and subgroups)"""
        group = {g.getAttribute("name"): g for g in self._groups()}[group_name]
        joints = {j.getAttribute("name") for j in _children(group, "joint")}

        # Joints connecting the group's links to their parents
        link_names = {link.getAttribute("name") for link in _children(group, "link")}
        joints |= {self.joint_of[link] for link in link_names if link in self.joint_of}

        # Joints between a chain's tip and its base (the base link's own joint is excluded)
        for chain in _children(group, "chain"):
            link, base = chain.getAttribute("tip_link"), chain.getAttribute("base_link")
            while link != base and link in self.parent_of:
                joints.add(self.joint_of[link])
                link = self.parent_of[link]

        for subgroup in _children(group, "group"):
            sname = subgroup.getAttribute("name")
            if sname not in _seen:
                joints |= self._group_joints(sname, _seen=(*_seen, group_name))
        return joints

    def _ancestors(self, link):
        """Returns the link itself and all its ancestors up to the root"""
        chain = [link]
        while chain[-1] in self.parent_of:
            chain.append(self.parent_of[chain[-1]])
        return chain

    def _groups(self):
        # Only direct children of <robot>, so nested <group> subgroup references are excluded
        return _children(self.srdf.documentElement, "group")

    def test_groups(self):
        """
        Checks for every group that:
        1. The name is unique
        2. It is not empty
        3. Referenced links, joints and chains exist in the URDF (chain: base is ancestor of tip)
        4. Referenced subgroups are defined
        """
        groups = self._groups()
        if not groups:
            self.skipTest("No <group> found in the SRDF")

        names = [g.getAttribute("name") for g in groups]
        self.assertEqual(len(names), len(set(names)), f"Duplicate group names in SRDF: {names}")

        for group in groups:
            name = group.getAttribute("name")
            links, joints = _children(group, "link"), _children(group, "joint")
            chains, subgroups = _children(group, "chain"), _children(group, "group")

            self.assertTrue(
                links or joints or chains or subgroups, f"Group '{name}' in SRDF is empty"
            )

            for link in links:
                lname = link.getAttribute("name")
                self.assertIn(lname, self.links, f"Group '{name}': link '{lname}' not in URDF")

            for joint in joints:
                jname = joint.getAttribute("name")
                self.assertIn(jname, self.joints, f"Group '{name}': joint '{jname}' not in URDF")

            for chain in chains:
                base, tip = chain.getAttribute("base_link"), chain.getAttribute("tip_link")
                self.assertIn(base, self.links, f"Group '{name}': chain base '{base}' not in URDF")
                self.assertIn(tip, self.links, f"Group '{name}': chain tip '{tip}' not in URDF")
                self.assertIn(
                    base,
                    self._ancestors(tip),
                    f"Group '{name}': chain base '{base}' is not an ancestor of tip '{tip}'",
                )

            for subgroup in subgroups:
                sname = subgroup.getAttribute("name")
                self.assertIn(sname, names, f"Group '{name}': subgroup '{sname}' is not defined")
                self.assertNotEqual(sname, name, f"Group '{name}' references itself")

    def test_end_effectors(self):
        """
        Checks for every <end_effector> that:
        1. Its group is defined
        2. Its parent link exists in the URDF
        3. Its parent group (optional) is defined
        """
        names = [g.getAttribute("name") for g in self._groups()]
        for ee in self.srdf.getElementsByTagName("end_effector"):
            name, group = ee.getAttribute("name"), ee.getAttribute("group")
            parent = ee.getAttribute("parent_link")
            self.assertIn(group, names, f"End effector '{name}': group '{group}' is not defined")
            self.assertIn(parent, self.links, f"End effector '{name}': link '{parent}' not in URDF")

            parent_group = ee.getAttribute("parent_group")
            if parent_group:
                self.assertIn(
                    parent_group,
                    names,
                    f"End effector '{name}': parent group '{parent_group}' is not defined",
                )

    def test_group_states(self):
        """
        Checks for every <group_state> that:
        1. Its group is defined and the name is unique within that group
        2. Every joint exists in the URDF and belongs to the group
        3. Every value is a number within the joint limits (if the URDF defines them)
        """
        names = [g.getAttribute("name") for g in self._groups()]
        seen = set()
        for state in self.srdf.getElementsByTagName("group_state"):
            name, group = state.getAttribute("name"), state.getAttribute("group")
            self.assertIn(group, names, f"Group state '{name}': group '{group}' is not defined")
            self.assertNotIn((name, group), seen, f"Group '{group}': duplicate state '{name}'")
            seen.add((name, group))

            group_joints = self._group_joints(group)
            for joint in _children(state, "joint"):
                jname, raw = joint.getAttribute("name"), joint.getAttribute("value")
                where = f"Group state '{name}' (group '{group}'): joint '{jname}'"
                self.assertIn(jname, self.joints, f"{where} not in URDF")
                self.assertIn(jname, group_joints, f"{where} is not part of the group")
                try:
                    value = float(raw)
                except ValueError:
                    self.fail(f"{where} has non-numeric value '{raw}'")
                if jname in self.limits:
                    lower, upper = self.limits[jname]
                    self.assertTrue(
                        lower <= value <= upper,
                        f"{where} value {value} outside limits [{lower}, {upper}]",
                    )

    def test_disabled_collisions(self):
        """
        Checks for every <disable_collisions> that:
        1. Both links exist in the URDF
        2. The pair is not a link with itself
        3. The reason is valid
        4. The pair is not listed twice (in either order)
        """
        seen = set()
        for dc in self.srdf.getElementsByTagName("disable_collisions"):
            link1, link2 = dc.getAttribute("link1"), dc.getAttribute("link2")
            reason = dc.getAttribute("reason")

            self.assertIn(link1, self.links, f"disable_collisions: link '{link1}' not in URDF")
            self.assertIn(link2, self.links, f"disable_collisions: link '{link2}' not in URDF")
            self.assertNotEqual(link1, link2, f"disable_collisions: '{link1}' paired with itself")
            self.assertIn(
                reason,
                VALID_COLLISION_REASONS,
                f"disable_collisions ({link1}, {link2}): invalid reason '{reason}'",
            )

            pair = frozenset((link1, link2))
            self.assertNotIn(pair, seen, f"disable_collisions: duplicate pair ({link1}, {link2})")
            seen.add(pair)


class _ParamsPlugin:
    """Hands the command line arguments to the test classes as `self.params`"""

    def __init__(self, params):
        self._params = params

    @pytest.fixture(autouse=True)
    def _inject_params(self, request):
        request.cls.params = self._params


def main(argv=None):
    parser = argparse.ArgumentParser(description="Shared URDF / SRDF checks")
    parser.add_argument("--package", required=True, help="ROS package name")
    parser.add_argument(
        "--files",
        nargs="+",
        required=True,
        help="One file: the URDF xacro. Two files: the SRDF xacro followed by the URDF xacro "
        "(relative to the package share directory)",
    )
    args, pytest_args = parser.parse_known_args(argv)
    if len(args.files) > 2:
        parser.error(f"--files takes one (URDF) or two (SRDF, URDF) files, got {len(args.files)}")

    # Run only the test class matching the number of files
    test_class = "TestURDF" if len(args.files) == 1 else "TestSRDF"
    return pytest.main([f"{__file__}::{test_class}", *pytest_args], plugins=[_ParamsPlugin(args)])


if __name__ == "__main__":
    sys.exit(main())
