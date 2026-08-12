#!/usr/bin/env python3
"""Restore the pre-patch PX4 Iris SDF and corridor world backups, if present."""
import argparse
import shutil
from pathlib import Path


def discover_layout(root):
    for base in (root / 'Tools' / 'sitl_gazebo',
                 root / 'Tools' / 'simulation' / 'gazebo-classic' / 'sitl_gazebo-classic'):
        if (base / 'models' / 'iris').exists():
            return base, base / 'models' / 'iris'
    raise FileNotFoundError('Gazebo Classic Iris model not found.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-root', required=True)
    args = parser.parse_args()
    base, iris = discover_layout(Path(args.px4_root).expanduser().resolve())
    restored = False
    for model in (iris / 'iris.sdf.jinja', iris / 'iris.sdf'):
        backup = model.with_name(model.name + '.uav_semantic_search.bak')
        if backup.exists():
            shutil.copy2(backup, model)
            print('Restored', model)
            restored = True
    world = base / 'worlds' / 'corridor_rooms.world'
    backup = world.with_name(world.name + '.uav_semantic_search.bak')
    if backup.exists():
        shutil.copy2(backup, world)
        print('Restored', world)
        restored = True
    if not restored:
        print('No uav_semantic_search backups were found; nothing changed.')


if __name__ == '__main__':
    main()
