#!/usr/bin/env python3
"""Evaluate the repository launch descriptions without launching anything.

The WMX3 SDK is absent in CI, so the nodes themselves cannot be built there.
Building the LaunchDescription objects still catches broken substitutions, a
and a LifecycleNode declared without a namespace.
"""

import importlib.util
import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import LifecycleNode

# Importing the launch files must not litter __pycache__ into the source tree:
# those directories end up installed by install(DIRECTORY launch ...) and break
# a later install step when the destination is not writable. Set before load()
# imports anything.
sys.dont_write_bytecode = True

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LAUNCH_PACKAGES = ('wmx_r2_package', 'wmx_r2_control')

# Every lifecycle node these launch files start, so a rename or a dropped node
# does not go unnoticed.
EXPECTED_LIFECYCLE_NODES = {
    'wmx_r2_general_nodes.launch.py': 3,
    'wmx_r2_cr3a_manipulator.launch.py': 4,
    'wmx_r2_cr5a_manipulator.launch.py': 3,
    'wmx_r2_diffbot_navigation.launch.py': 2,
    # wmx_r2_control drives ros2_control_node plus spawners, so it declares no
    # lifecycle nodes of its own; the count still guards against one appearing
    # unnoticed.
    'wmx_r2_control_cr3a_manipulator.launch.py': 0,
    'wmx_r2_control_cr5a_manipulator.launch.py': 0,
    'wmx_r2_control_diffbot_navigation.launch.py': 0,
}


def ensure_package_share(package):
    """Make get_package_share_directory(package) resolve, stubbing it if unbuilt.

    The robot launch files ask for their own share directory to build paths to
    configs and to the general-nodes launch file. Both uses are lazy, so a stub
    prefix is enough to evaluate the description in a workspace where the
    package has not been built (as in CI).
    """
    try:
        get_package_share_directory(package)
        return
    except PackageNotFoundError:
        pass

    prefix = tempfile.mkdtemp(prefix='launch_check_')
    os.makedirs(os.path.join(prefix, 'share', package, 'launch'))
    os.makedirs(os.path.join(prefix, 'share', package, 'config'))
    marker_dir = os.path.join(prefix, 'share', 'ament_index', 'resource_index', 'packages')
    os.makedirs(marker_dir)
    open(os.path.join(marker_dir, package), 'w').close()

    os.environ['AMENT_PREFIX_PATH'] = os.pathsep.join(
        [prefix, os.environ.get('AMENT_PREFIX_PATH', '')]).rstrip(os.pathsep)
    print(f'stubbed share directory for {package} at {prefix}')


def load(path):
    """Import a launch file and return its LaunchDescription."""
    spec = importlib.util.spec_from_file_location('launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def entities_of(description):
    """Top-level entities, with OpaqueFunctions expanded on argument defaults.

    Node actions are never visited: visiting one would launch the process.
    """
    context = LaunchContext()
    entities = []
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)
        elif isinstance(entity, OpaqueFunction):
            entities.extend(entity.execute(context) or [])
        else:
            entities.append(entity)
    return entities


def main():
    failures = []

    for package in LAUNCH_PACKAGES:
        ensure_package_share(package)

    launch_files = []
    for package in LAUNCH_PACKAGES:
        launch_dir = os.path.join(REPO_ROOT, package, 'launch')
        if not os.path.isdir(launch_dir):
            failures.append(f'{package}: no launch directory at {launch_dir}')
            continue
        for name in sorted(os.listdir(launch_dir)):
            if name.endswith('.launch.py'):
                launch_files.append((name, os.path.join(launch_dir, name)))

    for name, path in launch_files:
        try:
            description = load(path)
        except Exception as exc:  # noqa: BLE001 - report, do not abort the sweep
            failures.append(f'{name}: failed to build LaunchDescription: {exc}')
            continue

        lifecycle_nodes = [
            entity for entity in entities_of(description) if isinstance(entity, LifecycleNode)
        ]
        expected = EXPECTED_LIFECYCLE_NODES.get(name)
        if expected is not None and len(lifecycle_nodes) != expected:
            failures.append(
                f'{name}: expected {expected} lifecycle nodes, found {len(lifecycle_nodes)}')

        print(f'{name}: OK ({len(lifecycle_nodes)} lifecycle nodes)')

    for failure in failures:
        print(f'ERROR: {failure}', file=sys.stderr)

    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
