#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Public ROS entry point for the binary simulation-referee core."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import rospkg


PACKAGE_NAME = "tianracer_sim_referee"
CORE_MODULE = "_referee_core"


def load_referee():
    """Load the private core from the package's share directory."""
    package_path = Path(rospkg.RosPack().get_path(PACKAGE_NAME))
    core_path = package_path / "scripts" / (CORE_MODULE + ".so")
    if not core_path.is_file():
        raise RuntimeError("裁判核心不存在：%s" % core_path)

    spec = spec_from_file_location(CORE_MODULE, str(core_path))
    if spec is None or spec.loader is None:
        raise RuntimeError("无法创建裁判核心加载器：%s" % core_path)

    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        return module.referee
    except AttributeError as error:
        raise RuntimeError("裁判核心缺少 referee 入口：%s" % core_path) from error


def main():
    load_referee().start()


if __name__ == "__main__":
    main()
