"""
Author       : gxs
Date         : 2026-05-31 16:44:00
LastEditors  : gxs
LastEditTime : 2026-05-31 16:44:00
FilePath     : /visionAlgorithm/scripts/split2slice.py
Description  :
  加载 ONNX 模型, 遍历所有 Split 节点;
  如果 Split 的输出通道不是均匀分割的 (即 splits 参数不是标量, 而是多个不同的值),
  则将该 Split 节点替换为多个 Slice 节点, 分别沿指定维度进行切片;

  使用方式:
    python scripts/split2slice.py --onnx_path=path/to/model.onnx --save_path=path/to/output.onnx

使用 scripts/pt2onnx.py 导出的 onnx : yolov10 / yolo11 / yolo26 因为注意力 split 得到的三个输出通道不一样
OpenCV 不能推理, 所以使用改代码将 Split 节点 替换为 多个 Slice 节点

Copyright (c) 2026 by gxs, All Rights Reserved.
"""

import argparse
import os
import sys
from typing import Dict, List, Optional, Tuple

import onnxsim

import onnx
from onnx import TensorProto, helper


def _collect_existing_names(graph: onnx.GraphProto) -> Tuple[set, set]:
    """收集图中已存在的节点名称和 tensor 名称, 用于去重"""
    node_names = set()
    tensor_names = set()

    for node in graph.node:
        node_names.add(node.name)
        for name in node.input:
            tensor_names.add(name)
        for name in node.output:
            tensor_names.add(name)

    for init in graph.initializer:
        tensor_names.add(init.name)

    for vi in graph.value_info:
        tensor_names.add(vi.name)

    for inp in graph.input:
        tensor_names.add(inp.name)

    for out in graph.output:
        tensor_names.add(out.name)

    return node_names, tensor_names


def _get_unique_name(
    base_name: str,
    existing_names: set,
    max_attempts: int = 1000,
) -> str:
    """生成一个在 existing_names 中不存在的名称"""
    if base_name not in existing_names:
        return base_name
    for i in range(1, max_attempts + 1):
        candidate = f"{base_name}_{i}"
        if candidate not in existing_names:
            return candidate
    raise RuntimeError(
        f"无法为 {base_name} 生成唯一名称, 尝试次数已达上限 {max_attempts}"
    )


def _get_split_attrs(
    node: onnx.NodeProto,
    initializer_dict: Dict[str, onnx.TensorProto],
) -> Optional[Dict]:
    """
    解析 Split 节点的属性, 返回 splits 信息;
    在 ONNX opset 13+ 中, Split 的 split 参数通过 input[1] 传入 (作为 initializer),
    而不是 attribute;

    返回:
      - 如果是均匀分割 (splits 为标量或未指定), 返回 None
      - 如果是非均匀分割, 返回 dict: {"splits": list[int], "axis": int}
    """
    # 获取 axis, 默认为 0
    axis = 0
    for attr in node.attribute:
        if attr.name == "axis":
            axis = attr.i
            break

    # 检查是否有 split 输入 (input[1]), 如果有则从 initializer 中获取 splits 值
    if len(node.input) > 1:
        split_input_name = node.input[1]
        if split_input_name in initializer_dict:
            split_tensor = initializer_dict[split_input_name]
            # 从 tensor 中读取 splits 值
            import numpy as np

            splits = np.frombuffer(split_tensor.raw_data, dtype=np.int64).tolist()
            # 判断是否为非均匀分割: splits 长度 > 1 且值不全相等
            if len(splits) > 1 and len(set(splits)) > 1:
                return {"splits": splits, "axis": axis}
            # splits 长度为 1 或所有值相等, 说明是均匀分割, 不需要替换
            return None

    # 没有 split 输入, 说明是均匀分割 (所有输出通道数相同)
    return None


def _build_slice_node(
    input_name: str,
    output_name: str,
    axis: int,
    start_val: int,
    end_val: int,
    node_name: str,
) -> List[onnx.NodeProto]:
    """
    构建一个 Slice 节点, 从 input_name 沿 axis 维度切片 [start_val, end_val);
    Slice 节点的输入为: data, starts, ends, axes, steps
    """
    # 创建常量节点来提供 starts, ends, axes, steps 的值
    # 使用 onnx.helper.make_node 创建常量

    # starts 常量
    starts_tensor = helper.make_tensor(
        name=f"{node_name}_starts",
        data_type=TensorProto.INT64,
        dims=[1],
        vals=[start_val],
    )
    starts_node = helper.make_node(
        "Constant",
        inputs=[],
        outputs=[f"{node_name}_starts"],
        name=f"{node_name}_const_starts",
        value=starts_tensor,
    )

    # ends 常量
    ends_tensor = helper.make_tensor(
        name=f"{node_name}_ends",
        data_type=TensorProto.INT64,
        dims=[1],
        vals=[end_val],
    )
    ends_node = helper.make_node(
        "Constant",
        inputs=[],
        outputs=[f"{node_name}_ends"],
        name=f"{node_name}_const_ends",
        value=ends_tensor,
    )

    # axes 常量
    axes_tensor = helper.make_tensor(
        name=f"{node_name}_axes",
        data_type=TensorProto.INT64,
        dims=[1],
        vals=[axis],
    )
    axes_node = helper.make_node(
        "Constant",
        inputs=[],
        outputs=[f"{node_name}_axes"],
        name=f"{node_name}_const_axes",
        value=axes_tensor,
    )

    # steps 常量
    steps_tensor = helper.make_tensor(
        name=f"{node_name}_steps",
        data_type=TensorProto.INT64,
        dims=[1],
        vals=[1],
    )
    steps_node = helper.make_node(
        "Constant",
        inputs=[],
        outputs=[f"{node_name}_steps"],
        name=f"{node_name}_const_steps",
        value=steps_tensor,
    )

    # Slice 节点
    slice_node = helper.make_node(
        "Slice",
        inputs=[
            input_name,
            f"{node_name}_starts",
            f"{node_name}_ends",
            f"{node_name}_axes",
            f"{node_name}_steps",
        ],
        outputs=[output_name],
        name=node_name,
    )

    return [starts_node, ends_node, axes_node, steps_node, slice_node]


def _replace_split_with_slices(
    graph: onnx.GraphProto,
    split_node: onnx.NodeProto,
    split_info: Dict,
    existing_node_names: set,
    existing_tensor_names: set,
) -> List[onnx.NodeProto]:
    """
    将一个非均匀分割的 Split 节点替换为多个 Slice 节点;

    参数:
      graph: ONNX 图
      split_node: 要替换的 Split 节点
      split_info: {"splits": list[int], "axis": int}
      existing_node_names: 图中已有的节点名称集合 (会被更新)
      existing_tensor_names: 图中已有的 tensor 名称集合 (会被更新)

    返回:
      替换后的新节点列表 (包含 Constant 节点和 Slice 节点)
    """
    splits = split_info["splits"]
    axis = split_info["axis"]
    input_name = split_node.input[0]
    output_names = list(split_node.output)

    # 使用原 Split 节点的名称作为前缀
    split_name = split_node.name or "Split"

    # 计算累积偏移量
    offsets = [0]
    for s in splits:
        offsets.append(offsets[-1] + s)

    new_nodes = []

    for i, (out_name, start, end) in enumerate(
        zip(output_names, offsets[:-1], offsets[1:])
    ):
        # 节点名称: 原 Split 名称 + _toSlice_{idx}
        base_name = f"{split_name}_toSlice_{i}"
        node_name = _get_unique_name(base_name, existing_node_names)
        existing_node_names.add(node_name)

        # 生成唯一的中间 tensor 名称
        slice_out_name = _get_unique_name(out_name, existing_tensor_names)
        existing_tensor_names.add(slice_out_name)

        # 构建 Slice 节点及其常量输入节点
        slice_nodes = _build_slice_node(
            input_name=input_name,
            output_name=slice_out_name,
            axis=axis,
            start_val=start,
            end_val=end,
            node_name=node_name,
        )

        # 将常量节点的名称和 tensor 名称加入集合
        for n in slice_nodes:
            existing_node_names.add(n.name)
            for t_name in n.output:
                existing_tensor_names.add(t_name)

        new_nodes.extend(slice_nodes)

        # 如果输出名称与原始 Split 的输出名称不同, 需要添加 Identity 节点来保持输出名称一致
        if slice_out_name != out_name:
            identity_name = _get_unique_name(
                f"{node_name}_identity", existing_node_names
            )
            existing_node_names.add(identity_name)
            identity_node = helper.make_node(
                "Identity",
                inputs=[slice_out_name],
                outputs=[out_name],
                name=identity_name,
            )
            new_nodes.append(identity_node)

    return new_nodes


def _validate_model_after_replace(
    original_model: onnx.ModelProto,
    new_model: onnx.ModelProto,
) -> bool:
    """
    验证替换后的模型是否正确;
    检查:
      1. 输入输出是否一致
      2. 模型能否通过 onnx.checker 校验
    """
    # 检查输入输出
    orig_inputs = [i.name for i in original_model.graph.input]
    new_inputs = [i.name for i in new_model.graph.input]
    if orig_inputs != new_inputs:
        print(f"错误: 模型输入不匹配; 原始: {orig_inputs}, 新: {new_inputs}")
        return False

    orig_outputs = [out.name for out in original_model.graph.output]
    new_outputs = [out.name for out in new_model.graph.output]
    if orig_outputs != new_outputs:
        print(f"错误: 模型输出不匹配; 原始: {orig_outputs}, 新: {new_outputs}")
        return False

    # 校验模型
    try:
        onnx.checker.check_model(new_model)
        print("模型校验通过")
        return True
    except Exception as e:
        print(f"模型校验失败: {e}")
        return False


def split2slice(onnx_path: str, save_path: str) -> bool:
    """
    主函数: 加载 ONNX 模型, 遍历所有 Split 节点, 将非均匀分割的 Split 替换为 Slice 节点;

    参数:
      onnx_path: 输入 ONNX 模型路径
      save_path: 输出 ONNX 模型路径

    返回:
      True 表示成功, False 表示失败
    """
    # 检查输入文件是否存在
    if not os.path.exists(onnx_path):
        print(f"错误: 输入文件不存在: {onnx_path}")
        return False

    # 创建输出目录
    os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)

    # 加载 ONNX 模型
    print(f"加载模型: {onnx_path}")
    model = onnx.load(onnx_path)
    graph = model.graph

    # 收集现有名称
    existing_node_names, existing_tensor_names = _collect_existing_names(graph)

    # 构建 initializer 字典, 用于查找 Split 的 split 输入
    initializer_dict = {init.name: init for init in graph.initializer}

    # 遍历所有节点, 找到 Split 节点
    split_nodes_to_replace = []
    for node_idx, node in enumerate(graph.node):
        if node.op_type == "Split":
            split_info = _get_split_attrs(node, initializer_dict)
            if split_info is not None:
                split_nodes_to_replace.append((node_idx, node, split_info))
                print(
                    f"发现非均匀分割 Split 节点: {node.name or '(unnamed)'}, "
                    f"splits={split_info['splits']}, axis={split_info['axis']}"
                )

    if not split_nodes_to_replace:
        print("未发现非均匀分割的 Split 节点, 无需替换")
        # 如果没有需要替换的节点, 直接保存原模型
        onnx.save(model, save_path)
        print(f"模型已保存至: {save_path}")
        return True

    print(f"共发现 {len(split_nodes_to_replace)} 个需要替换的 Split 节点")

    # 构建新的节点列表
    new_nodes = []
    replaced_count = 0

    for node_idx, node in enumerate(graph.node):
        if node.op_type == "Split" and any(
            n is node for _, n, _ in split_nodes_to_replace
        ):
            # 找到对应的 split_info
            split_info = None
            for _, n, info in split_nodes_to_replace:
                if n is node:
                    split_info = info
                    break

            if split_info is not None:
                print(
                    f"替换 Split 节点: {node.name or '(unnamed)'}, "
                    f"splits={split_info['splits']}"
                )
                slice_nodes = _replace_split_with_slices(
                    graph=graph,
                    split_node=node,
                    split_info=split_info,
                    existing_node_names=existing_node_names,
                    existing_tensor_names=existing_tensor_names,
                )
                new_nodes.extend(slice_nodes)
                replaced_count += 1
            else:
                # 不应该发生
                new_nodes.append(node)
        else:
            new_nodes.append(node)

    # 更新图的节点列表
    while len(graph.node) > 0:
        graph.node.pop()
    for n in new_nodes:
        graph.node.append(n)

    # 校验模型
    print("正在校验替换后的模型...")
    if not _validate_model_after_replace(model, model):
        print("模型校验失败, 请检查替换结果")
        return False

    # 使用 onnxsim 优化模型
    print("正在使用 onnxsim 优化模型...")
    try:
        model_simp, check = onnxsim.simplify(model)
        if check:
            model = model_simp
            print("onnxsim 优化成功")
        else:
            print("onnxsim 优化验证未通过, 使用未优化模型")
    except Exception as e:
        print(f"onnxsim 优化失败: {e}, 使用未优化模型")

    # 保存模型
    print(f"保存模型至: {save_path}")
    onnx.save(model, save_path)
    print(f"替换完成, 共替换 {replaced_count} 个 Split 节点")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="将 ONNX 模型中非均匀分割的 Split 节点替换为 Slice 节点;",
        formatter_class=type(
            "MyFormatterClass",
            (
                argparse.ArgumentDefaultsHelpFormatter,
                argparse.RawTextHelpFormatter,
            ),
            {},
        ),
    )
    parser.add_argument(
        "--onnx_path",
        type=str,
        required=True,
        help="输入 ONNX 模型路径;",
    )
    parser.add_argument(
        "--save_path",
        type=str,
        required=True,
        help="输出 ONNX 模型保存路径;",
    )
    args = parser.parse_args()

    success = split2slice(
        onnx_path=args.onnx_path,
        save_path=args.save_path,
    )

    if success:
        print("处理成功")
    else:
        print("处理失败")
        sys.exit(1)


if __name__ == "__main__":
    main()
