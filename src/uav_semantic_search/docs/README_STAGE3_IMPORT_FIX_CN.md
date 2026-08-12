# Stage-3 v0.3.1：frontier_core 导入修复

## 根因

`frontier_core.py` 是被其他 Python 节点 `import` 的算法工具模块，不应出现在
`catkin_install_python(PROGRAMS ...)` 中。该 CMake 宏会在 `devel/lib/uav_semantic_search/`
生成可执行包装器。运行节点时，Python 优先导入这个包装器；包装器不会将
`FrontierCluster`、`extract_frontier_clusters` 等符号导出为普通模块属性，因此出现
`ImportError`。

## 修复

- `frontier_core.py` 从 `catkin_install_python(PROGRAMS ...)` 移除；
- 将其作为普通伴随文件安装到 `${CATKIN_PACKAGE_BIN_DESTINATION}`；
- `frontier_extractor.py` 和 `autonomous_search_manager.py` 强制将自身脚本目录插入
  `sys.path` 的最前方，从而导入真正的工具模块文件。

## 安装后必须清理旧包装器

```bash
cd ~/harp_sar_ws
rm -f devel/lib/uav_semantic_search/frontier_core.py
rm -rf devel/lib/uav_semantic_search/__pycache__
catkin_make --pkg uav_semantic_search --force-cmake
source devel/setup.bash
```

之后重新启动 `racer_stage3.launch`。
