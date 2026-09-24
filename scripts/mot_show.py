"""
Author       : gxs
Date         : 2026-07-28
LastEditors  : gxs
LastEditTime : 2026-07-28
FilePath     : /visionAlgorithm/scripts/mot_show.py
Description  :
    检测 + 跟踪可视化脚本;
    通过 ctypes 调用 C++ 检测器和跟踪器库 (libtracker.so) ;
    支持:
      - 图片目录 (按帧顺序读取并跟踪) ;
      - 视频文件 (逐帧读取并跟踪) ;
    Copyright (c) 2026 by gxs, All Rights Reserved.
"""

import argparse
import ctypes
import os
import sys

import cv2
import numpy as np
from tqdm import tqdm

### =============================================================================================
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)

# 支持的图片格式
IMG_FORMATS = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
    ".dng",
    ".mpo",
    ".tif",
    ".tiff",
    ".webp",
    ".pfm",
    ".heic",
}

# 支持的视频格式
VIDEO_FORMATS = {
    ".h264",
    ".h265",
    ".asf",
    ".avi",
    ".mov",
    ".mp4",
    ".gif",
    ".m4v",
    ".mkv",
    ".mpeg",
    ".mpg",
    ".ts",
    ".wmv",
    ".webm",
}

BAR_FORMAT = "{l_bar}{bar:10}{r_bar}"


def get_color(index, is_rgb=False):
    """根据索引生成区分度高的颜色 (BGR 或 RGB);"""
    h = (index * 137.508) % 360 / 360.0
    l_val, s = 0.6, 0.95

    def hue_to_rgb(p, q, t):
        t = t % 1.0
        if t < 1 / 6:
            return p + (q - p) * 6 * t
        if t < 1 / 2:
            return q
        if t < 2 / 3:
            return p + (q - p) * (2 / 3 - t) * 6
        return p

    if s == 0:
        r = g = b = l_val
    else:
        q = l_val * (1 + s) if l_val < 0.5 else l_val + s - l_val * s
        p = 2 * l_val - q
        r = hue_to_rgb(p, q, h + 1 / 3)
        g = hue_to_rgb(p, q, h)
        b = hue_to_rgb(p, q, h - 1 / 3)

    b = int(max(0, min(b, 1)) * 255)
    g = int(max(0, min(g, 1)) * 255)
    r = int(max(0, min(r, 1)) * 255)
    if is_rgb:
        return (r, g, b)
    else:
        return (b, g, r)


def check_dir(_dir, increment=True):
    """检查并创建目录, 增量模式避免覆盖;"""
    if increment:
        count = 1
        new_dir = _dir
        while os.path.exists(new_dir):
            count += 1
            new_dir = f"{_dir}{count}"
        os.makedirs(new_dir, exist_ok=True)
        return new_dir
    else:
        os.makedirs(_dir, exist_ok=True)
        return _dir


### =============================================================================================
# ctypes 类型别名
# =============================================================================================

# 检测结果格式: [x1, y1, x2, y2, score, cls_id]
# 跟踪结果格式: [track_id, det_index, cls_id, score, l, t, w, h]


class TrackerLib:
    """
    封装 tracker_lib 的 C API, 通过 ctypes 调用;
    提供检测器 + 跟踪器的生命周期管理和推理;
    """

    def __init__(self, lib_path: str):
        """
        @param lib_path: libtracker.so 或 libtracker.dll 的路径;
        """
        if not os.path.exists(lib_path):
            raise FileNotFoundError(f"tracker library not found: {lib_path}")

        self._lib = ctypes.cdll.LoadLibrary(lib_path)

        # ================================================================
        # Detector API
        # ================================================================
        self._lib.detector_create.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        self._lib.detector_create.restype = ctypes.c_void_p

        self._lib.detector_destroy.argtypes = [ctypes.c_void_p]
        self._lib.detector_destroy.restype = None

        self._lib.detector_detect.argtypes = [
            ctypes.c_void_p,  # handle
            ctypes.c_char_p,  # image_path
            ctypes.POINTER(ctypes.c_float),  # out_data
            ctypes.c_int,  # max_dets
            ctypes.POINTER(ctypes.c_int),  # n_dets
        ]
        self._lib.detector_detect.restype = ctypes.c_int

        # ================================================================
        # Tracker API
        # ================================================================
        self._lib.tracker_create.argtypes = []
        self._lib.tracker_create.restype = ctypes.c_void_p

        self._lib.tracker_destroy.argtypes = [ctypes.c_void_p]
        self._lib.tracker_destroy.restype = None

        self._lib.tracker_init.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        self._lib.tracker_init.restype = ctypes.c_int

        self._lib.tracker_update.argtypes = [
            ctypes.c_void_p,  # handle
            ctypes.POINTER(ctypes.c_float),  # dets_data
            ctypes.c_int,  # n_dets
            ctypes.POINTER(ctypes.c_float),  # results_data
            ctypes.c_int,  # max_results
            ctypes.POINTER(ctypes.c_int),  # n_results
        ]
        self._lib.tracker_update.restype = ctypes.c_int

        self._lib.tracker_reset.argtypes = [ctypes.c_void_p]
        self._lib.tracker_reset.restype = ctypes.c_int

        self._lib.tracker_is_initialized.argtypes = [ctypes.c_void_p]
        self._lib.tracker_is_initialized.restype = ctypes.c_int

    # ----------------------------------------------------------------
    # 检测器
    # ----------------------------------------------------------------

    def detector_create(self, ini_path: str, onnx_path: str, device: int = -1):
        handle = self._lib.detector_create(
            ini_path.encode("utf-8"),
            onnx_path.encode("utf-8"),
            device,
        )
        if handle is None:
            raise RuntimeError(
                f"detector_create failed: ini={ini_path}, onnx={onnx_path}"
            )
        return handle

    def detector_destroy(self, handle):
        if handle is not None:
            self._lib.detector_destroy(handle)

    def detector_detect(self, handle, image_path: str, max_dets: int = 300):
        """
        单图检测;
        @return: np.ndarray shape=(n, 6), 每行: [x1, y1, x2, y2, score, cls_id];
        """
        out_data = (ctypes.c_float * (max_dets * 6))()
        n_dets = ctypes.c_int(0)

        ret = self._lib.detector_detect(
            handle,
            image_path.encode("utf-8"),
            out_data,
            max_dets,
            ctypes.byref(n_dets),
        )
        if ret != 0:
            return np.empty((0, 6), dtype=np.float32)

        count = n_dets.value
        arr = (
            np.ctypeslib.as_array(
                (ctypes.c_float * (count * 6)).from_address(ctypes.addressof(out_data))
            )
            .reshape(-1, 6)
            .copy()
        )
        return arr

    # ----------------------------------------------------------------
    # 跟踪器
    # ----------------------------------------------------------------

    def tracker_create(self):
        handle = self._lib.tracker_create()
        if handle is None:
            raise RuntimeError("tracker_create failed")
        return handle

    def tracker_init(self, handle, ini_path: str, max_history: int = 150):
        ret = self._lib.tracker_init(handle, ini_path.encode("utf-8"), max_history)
        if ret != 0:
            raise RuntimeError(f"tracker_init failed: ini={ini_path}")

    def tracker_update(self, handle, dets: np.ndarray, max_results: int = 300):
        """
        单帧跟踪更新;
        @param dets: np.ndarray shape=(n, 6), 每行: [x1, y1, x2, y2, score, cls_id];
        @return: np.ndarray shape=(m, 8), 每行: [track_id, det_index, cls_id, score, l, t, w, h];
        """
        n_dets = len(dets)
        if n_dets == 0:
            # 没有检测框时传空数组
            dets_data = (ctypes.c_float * 0)()
        else:
            dets_data = dets.astype(np.float32).ctypes.data_as(
                ctypes.POINTER(ctypes.c_float)
            )

        results_data = (ctypes.c_float * (max_results * 8))()
        n_results = ctypes.c_int(0)

        ret = self._lib.tracker_update(
            handle,
            dets_data,
            n_dets,
            results_data,
            max_results,
            ctypes.byref(n_results),
        )
        if ret != 0:
            return np.empty((0, 8), dtype=np.float32)

        count = n_results.value
        arr = (
            np.ctypeslib.as_array(
                (ctypes.c_float * (count * 8)).from_address(
                    ctypes.addressof(results_data)
                )
            )
            .reshape(-1, 8)
            .copy()
        )
        return arr

    def tracker_reset(self, handle):
        if handle is not None:
            self._lib.tracker_reset(handle)

    def tracker_destroy(self, handle):
        if handle is not None:
            self._lib.tracker_destroy(handle)


### =============================================================================================
# 可视化
# =============================================================================================


def draw_tracks(image: np.ndarray, dets: np.ndarray, tracks: np.ndarray, names: list):
    """
    在图像上绘制检测框 + 跟踪ID + 类别名 + 置信度;
    @param image: BGR image (h, w, 3);
    @param dets:  检测结果 (n, 6) [x1, y1, x2, y2, score, cls_id];
    @param tracks: 跟踪结果 (m, 8) [track_id, det_index, cls_id, score, l, t, w, h];
    @param names:  类别名列表;
    """
    img_h, img_w = image.shape[:2]
    # 自适应线宽和字体
    box_thickness = max(round(sum((img_h, img_w)) / 2 * 0.003), 2)
    text_thickness = max(box_thickness - 1, 1)
    font_scale = box_thickness / 4.0
    font_face = 0

    # 构建 (det_index -> track_id) 映射
    track_map = {}  # det_index -> (track_id, track_score)
    if len(tracks) > 0:
        for t in tracks:
            tid = int(t[0])
            didx = int(t[1])
            tscore = t[3]
            track_map[didx] = (tid, tscore)

    # 逐个绘制检测框
    for i, det in enumerate(dets):
        x1, y1, x2, y2, conf, cls_id = det
        x1, y1, x2, y2, cls_id = int(x1), int(y1), int(x2), int(y2), int(cls_id)
        cls_id = int(cls_id)

        # 获取该检测框对应的 track_id
        track_id = track_map.get(i, (None, None))[0]

        # 用 track_id 或 cls_id 决定颜色
        if track_id is not None:
            color = get_color(track_id)
        else:
            color = get_color(cls_id)

        # 画边界框
        cv2.rectangle(
            image,
            pt1=(x1, y1),
            pt2=(x2, y2),
            color=color,
            thickness=box_thickness,
            lineType=cv2.LINE_AA,
        )

        # 构造显示文本
        cls_name = names[cls_id] if names and cls_id < len(names) else f"cls_{cls_id}"

        if track_id is not None:
            text = f"#{track_id} {cls_name} {conf:.2f}"
        else:
            text = f"{cls_name} {conf:.2f}"

        # 计算文本尺寸
        (text_w, text_h), _ = cv2.getTextSize(
            text=text,
            fontFace=font_face,
            fontScale=font_scale,
            thickness=text_thickness,
        )

        # 自适应文本位置 (优先显示在框上方, 否则显示在框内下方)
        inside_x = (img_w - x1 - text_w) >= 5
        inside_y = (y1 - text_h) >= 8
        text_l = x1 if inside_x else x1 - (x1 + text_w + 3 - img_w)
        text_b = (y1 - 4) if inside_y else y1 + text_h + 5

        # 白色背景框
        cv2.rectangle(
            image,
            pt1=(text_l, text_b - text_h),
            pt2=(text_l + text_w, text_b + 4),
            color=(255, 255, 255),
            thickness=-1,
        )
        # 文字
        cv2.putText(
            image,
            text=text,
            org=(text_l, text_b),
            fontFace=font_face,
            fontScale=font_scale,
            thickness=text_thickness,
            color=color,
        )

    return image


### =============================================================================================
# 工具函数
# =============================================================================================


def _merge_to_video(image_paths: list, output_path: str, fps: float = 15.0):
    """
    将多张图片按顺序合成为视频;
    @param image_paths: 已排序的图片路径列表;
    @param output_path: 输出视频路径;
    @param fps:         帧率;
    """
    if not image_paths:
        return

    # 读取第一张图获取尺寸
    first = cv2.imread(image_paths[0])
    if first is None:
        return
    height, width = first.shape[:2]

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fourcc = cv2.VideoWriter.fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    for img_path in image_paths:
        img = cv2.imread(img_path)
        if img is not None:
            writer.write(img)

    writer.release()


### =============================================================================================
# 主逻辑 - 图片目录模式
# =============================================================================================


def run_image_dir(
    tracker_lib: TrackerLib,
    det_handle,
    trk_handle,
    data_path: str,
    save_root: str,
    names: list,
    max_dets: int = 300,
):
    """
    遍历图片目录, 逐张检测 + 跟踪 + 可视化保存;
    输出结构:
      {save_root}/{data_dir_name}/img/                 -- 可视化图片
      {save_root}/{data_dir_name}/{data_dir_name}.mp4  -- 合并视频(15fps);
    """
    # 收集所有图片 (递归所有子目录, 按文件名排序)
    img_paths = []
    for root, _, files in os.walk(data_path):
        for f in sorted(files):
            _, suffix = os.path.splitext(f)
            if suffix.lower() not in IMG_FORMATS:
                continue
            img_paths.append(os.path.join(root, f))

    if not img_paths:
        print(f"[WARNING] no images found in {data_path}")
        return

    # 输出目录结构
    data_dir_name = os.path.basename(os.path.normpath(data_path))
    img_save_root = os.path.join(save_root, data_dir_name, "img")
    os.makedirs(img_save_root, exist_ok=True)

    print(f"[INFO] found {len(img_paths)} images, start tracking...")

    # 存储可视化后的图片路径, 用于后续合成为视频
    vis_paths = []

    bar = tqdm(img_paths, desc="Tracking", bar_format=BAR_FORMAT)
    for img_path in bar:
        # 1. 检测
        dets = tracker_lib.detector_detect(det_handle, img_path, max_dets=max_dets)

        # 2. 跟踪
        tracks = tracker_lib.tracker_update(trk_handle, dets, max_results=max_dets)

        # 3. 可视化
        image = cv2.imread(img_path)
        if image is None:
            continue
        image = draw_tracks(image, dets, tracks, names)

        # 4. 保存图片到 {save_root}/{data_dir_name}/img/
        save_path = img_path.replace(data_path, img_save_root)
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        cv2.imwrite(save_path, image)
        vis_paths.append(save_path)

    # 5. 将所有可视化图片合并为视频 (15fps)
    if len(vis_paths) > 0:
        video_path = os.path.join(save_root, data_dir_name, f"{data_dir_name}.mp4")
        _merge_to_video(vis_paths, video_path, fps=15)
        print(f"[INFO] video saved to: {video_path}")

    print(f"[INFO] results saved to: {os.path.join(save_root, data_dir_name)}")


### =============================================================================================
# 主逻辑 - 视频模式
# =============================================================================================


def run_video(
    tracker_lib: TrackerLib,
    det_handle,
    trk_handle,
    video_path: str,
    save_root: str,
    names: list,
    max_dets: int = 300,
):
    """
    读取视频文件, 逐帧检测 + 跟踪 + 可视化保存;
    输出结构:
      {save_root}/{video_name}/              -- 可视化结果目录
      {save_root}/{video_name}/img/          -- 逐帧可视化图片
      {save_root}/{video_name}/{video_name}.mp4  -- 合并视频(15fps);
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[ERROR] cannot open video: {video_path}")
        return

    # 获取视频参数
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # 输出目录结构
    video_name = os.path.splitext(os.path.basename(video_path))[0]
    out_dir = os.path.join(save_root, video_name)
    img_save_dir = os.path.join(out_dir, "img")
    os.makedirs(img_save_dir, exist_ok=True)

    out_video_path = os.path.join(out_dir, f"{video_name}.mp4")

    print(f"[INFO] video: {video_path}, {total_frames} frames")

    bar = tqdm(total=total_frames, desc="Tracking", bar_format=BAR_FORMAT)
    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 保存帧图像到临时路径供检测 (C API 接受的是文件路径)
        temp_img_path = os.path.join(img_save_dir, f"_{frame_idx:08d}.jpg")
        cv2.imwrite(temp_img_path, frame)

        # 1. 检测
        dets = tracker_lib.detector_detect(det_handle, temp_img_path, max_dets=max_dets)

        # 2. 跟踪
        tracks = tracker_lib.tracker_update(trk_handle, dets, max_results=max_dets)

        # 3. 可视化
        frame = draw_tracks(frame, dets, tracks, names)

        # 4. 保存帧图片 (覆盖原临时文件)
        cv2.imwrite(temp_img_path, frame)

        frame_idx += 1
        bar.update(1)

    cap.release()

    # 5. 将所有帧合成为视频 (15fps)
    frame_paths = sorted(
        [
            os.path.join(img_save_dir, f)
            for f in os.listdir(img_save_dir)
            if os.path.splitext(f)[1].lower() in IMG_FORMATS
        ]
    )
    if frame_paths:
        _merge_to_video(frame_paths, out_video_path, fps=15)
        print(f"[INFO] video saved to: {out_video_path}")

    print(f"[INFO] results saved to: {out_dir}")


### =============================================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="检测 + 跟踪可视化 (MOT Show)",
        formatter_class=type(
            "MyFormatterClass",
            (argparse.ArgumentDefaultsHelpFormatter, argparse.RawTextHelpFormatter),
            {},
        ),
    )

    parser.add_argument(
        "--det_ini",
        required=True,
        type=str,
        help="检测器 ini 配置文件路径 (如 config/detection/yolov8nDetVOC-bn.ini)",
    )
    parser.add_argument(
        "--onnx_path",
        required=True,
        type=str,
        help="检测模型 ONNX 文件路径",
    )
    parser.add_argument(
        "--trk_ini",
        required=True,
        type=str,
        help="跟踪器 ini 配置文件路径 (如 config/tracker/bytetrack.ini)",
    )
    parser.add_argument(
        "--data_path",
        required=True,
        type=str,
        help="输入: 图片目录或视频文件路径",
    )
    parser.add_argument(
        "--save_root",
        type=str,
        default=os.path.join(CURRENT_DIR, "mot-show"),
        help="保存可视化结果的根目录, default=%(default)s",
    )
    parser.add_argument(
        "--max_dets",
        type=int,
        default=300,
        help="每帧最大检测/跟踪目标数, default=%(default)s",
    )

    try:
        import torch

        parser.add_argument(
            "--device",
            type=int,
            default=-1,
            choices=list(range(-1, torch.cuda.device_count())),
            help=f"指定运行设备, CPU: -1, GPU: 0 ~ {torch.cuda.device_count() - 1};",
        )
    except Exception:
        parser.add_argument(
            "--device",
            type=int,
            default=-1,
            choices=[-1],
            help="指定运行设备, CPU: -1;",
        )

    parser.add_argument(
        "--names",
        type=str,
        default=None,
        help="类别名 (逗号分隔), 不指定则用索引显示",
    )

    parser.add_argument(
        "--lib_path",
        type=str,
        default=None,
        help="libtracker.so 路径, 默认自动搜索",
    )

    return parser.parse_args()


### =============================================================================================
def find_lib_path():
    """
    自动搜索 libtracker.so;
    搜索顺序:
      1. 环境变量 TRACKER_LIB_PATH
      2. tracker_lib/build/lib/libtracker.so
      3. tracker_lib/build/lib/libtracker.dll
    """
    env_path = os.environ.get("TRACKER_LIB_PATH")
    if env_path and os.path.exists(env_path):
        return env_path

    # 尝试 .so (Linux)
    so_path = os.path.join(PROJECT_ROOT, "tracker_lib", "build", "lib", "libtracker.so")
    if os.path.exists(so_path):
        return so_path

    # 尝试 .dll (Windows)
    dll_path = os.path.join(
        PROJECT_ROOT, "tracker_lib", "build", "lib", "libtracker.dll"
    )
    if os.path.exists(dll_path):
        return dll_path

    raise FileNotFoundError(
        "cannot find libtracker.so/.dll. "
        "Set TRACKER_LIB_PATH environment variable or build the tracker library first."
    )


### =============================================================================================
if __name__ == "__main__":
    args = parse_args()

    # 路径检查
    for pname, pval in [
        ("--det_ini", args.det_ini),
        ("--onnx_path", args.onnx_path),
        ("--trk_ini", args.trk_ini),
        ("--data_path", args.data_path),
    ]:
        if not os.path.exists(pval):
            print(f"[ERROR] {pname} not found: {pval}")
            sys.exit(1)

    data_path = os.path.abspath(args.data_path)
    save_root = os.path.abspath(args.save_root)
    os.makedirs(save_root, exist_ok=True)

    # 解析类别名
    names = []
    if args.names:
        names = [name.strip() for name in args.names.split(",")]
    else:
        # 尝试从配置文件中读取 names
        try:
            import re

            config_path = args.det_ini
            with open(config_path, "r", encoding="utf-8") as f:
                content = f.read()
            match = re.search(r"names\s*=\s*\{([^}]+)\}", content)
            if match:
                names_str = match.group(1)
                names = [n.strip().strip("\"'") for n in names_str.split(",")]
            print(f"[INFO] parsed names from ini: {names}")
        except Exception:
            print("[INFO] names not specified, will use class indices")

    # 查找 tracker library
    lib_path = args.lib_path if args.lib_path else find_lib_path()
    print(f"[INFO] loading tracker library: {lib_path}")

    tracker_lib = TrackerLib(lib_path)

    # 创建检测器
    print(
        f"[INFO] creating detector: ini={args.det_ini}, onnx={args.onnx_path}, device={args.device}"
    )
    det_handle = tracker_lib.detector_create(
        ini_path=args.det_ini,
        onnx_path=args.onnx_path,
        device=args.device,
    )

    # 创建并初始化跟踪器
    print(f"[INFO] creating tracker: ini={args.trk_ini}")
    trk_handle = tracker_lib.tracker_create()
    tracker_lib.tracker_init(trk_handle, args.trk_ini)

    try:
        # 判断输入是目录还是视频
        if os.path.isdir(data_path):
            run_image_dir(
                tracker_lib=tracker_lib,
                det_handle=det_handle,
                trk_handle=trk_handle,
                data_path=data_path,
                save_root=save_root,
                names=names,
                max_dets=args.max_dets,
            )
        else:
            run_video(
                tracker_lib=tracker_lib,
                det_handle=det_handle,
                trk_handle=trk_handle,
                video_path=data_path,
                save_root=save_root,
                names=names,
                max_dets=args.max_dets,
            )
    finally:
        # 清理资源
        tracker_lib.detector_destroy(det_handle)
        tracker_lib.tracker_destroy(trk_handle)
        print("[INFO] resources cleaned up")
