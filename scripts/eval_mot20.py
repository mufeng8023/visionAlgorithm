#!/usr/bin/env python3
"""
MOT20 评估流水线;

流程:
  1. 读取 MOT20 数据集序列 (seqinfo.ini + img1/*.jpg + gt/gt.txt)
  2. 逐帧: YOLO 检测 -> 跟踪器跟踪
  3. 保存 MOT Challenge 格式结果
  4. 使用 TrackEval 评估精度 (MOTA, IDF1, HOTA 等)

MOT20 目录结构:
  <MOT20_ROOT>/
    train/
      MOT20-01/
        seqinfo.ini     # 序列信息 (帧率, 分辨率, 帧数)
        img1/000001.jpg
        gt/gt.txt       # Ground Truth
      MOT20-02/ ...
      MOT20-03/ ...
      MOT20-05/ ...
    test/
      ...

用法:
  # 使用指定检测模型 + 跟踪器, 保存结果
  python scripts/eval_mot20.py --det_ini config/detection/yolov8nDetPerson-bn.ini --det_onnx onnx/yolov8nDetPerson-bn.onnx --trackers deepsort

  # 完整评估 (检测+跟踪+TrackEval评估)
  python scripts/eval_mot20.py --det_ini xxx.ini --det_onnx xxx.onnx --trackers deepsort bytetrack --eval

  # 指定序列
  python scripts/eval_mot20.py --det_ini xxx.ini --det_onnx xxx.onnx --trackers deepsort --seqs MOT20-01 MOT20-02

  # 指定 MOT20 数据集根目录
  python scripts/eval_mot20.py --det_ini xxx.ini --det_onnx xxx.onnx --trackers deepsort --mot20_root /path/to/MOT20
"""

import argparse
import configparser
import ctypes
import os
import sys
import time

import numpy as np

try:
    # TrackEval GitHub: https://github.com/JonathonLuiten/TrackEval.git
    import trackeval
except ImportError:
    print("[WARN] TrackEval not available. Install with command:")
    print(
        "\033[31m pip install git+https://github.com/JonathonLuiten/TrackEval.git \033[0m"
    )
    sys.exit(1)


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ================================================================
# MOT20 序列名称常量 (不依赖具体路径)
# ================================================================
MOT20_SEQS_TRAIN = ["MOT20-01", "MOT20-02", "MOT20-03", "MOT20-05"]
MOT20_SEQS_TEST = ["MOT20-04", "MOT20-06", "MOT20-07", "MOT20-08"]

# ================================================================
# C 库加载 (检测器 + 跟踪器)
# ================================================================


def load_lib(lib_path=None):
    if lib_path is None:
        lib_path = os.path.join(
            PROJECT_ROOT,
            "tracker_lib",
            "build",
            "lib",
            "libtracker.so",
        )
    if not os.path.isfile(lib_path):
        raise FileNotFoundError(f"Library not found: {lib_path}")
    lib = ctypes.cdll.LoadLibrary(lib_path)

    # 检测器 API
    lib.detector_create.restype = ctypes.c_void_p
    lib.detector_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
    lib.detector_destroy.argtypes = [ctypes.c_void_p]
    lib.detector_destroy.restype = None
    lib.detector_detect.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
    ]
    lib.detector_detect.restype = ctypes.c_int

    # 跟踪器 API
    lib.tracker_create.restype = ctypes.c_void_p
    lib.tracker_destroy.argtypes = [ctypes.c_void_p]
    lib.tracker_destroy.restype = None
    lib.tracker_init.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.tracker_init.restype = ctypes.c_int
    lib.tracker_update.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
    ]
    lib.tracker_update.restype = ctypes.c_int
    lib.tracker_reset.argtypes = [ctypes.c_void_p]
    lib.tracker_reset.restype = ctypes.c_int
    lib.tracker_version.restype = ctypes.c_char_p

    return lib


class Detector:
    """检测器包装 (单例, 全局共用)"""

    def __init__(self, lib, ini_path, onnx_path, device=-1):
        self._lib = lib
        self._handle = lib.detector_create(
            ini_path.encode("utf-8"), onnx_path.encode("utf-8"), device
        )
        if self._handle is None:
            raise RuntimeError("detector_create failed")

    def __del__(self):
        if hasattr(self, "_handle") and self._handle is not None:
            self._lib.detector_destroy(self._handle)
            self._handle = None

    def detect(self, image_path):
        """单图检测 -> np.ndarray (N, 6) [x1,y1,x2,y2,score,cls]"""
        max_dets = 300
        out = (ctypes.c_float * (max_dets * 6))()
        n_out = ctypes.c_int(0)
        ret = self._lib.detector_detect(
            self._handle, image_path.encode("utf-8"), out, max_dets, ctypes.byref(n_out)
        )
        if ret != 0 or n_out.value == 0:
            return np.empty((0, 6), dtype=np.float32)
        return np.array(
            [[out[i * 6 + j] for j in range(6)] for i in range(n_out.value)],
            dtype=np.float32,
        )


class Tracker:
    """跟踪器包装 (每个序列新建)"""

    def __init__(self, lib, ini_path):
        self._lib = lib
        self._handle = lib.tracker_create()
        if self._handle is None:
            raise RuntimeError("tracker_create failed")
        ret = lib.tracker_init(self._handle, ini_path.encode("utf-8"), 150)
        if ret != 0:
            raise RuntimeError(f"tracker_init failed: {ini_path}")

    def __del__(self):
        if hasattr(self, "_handle") and self._handle is not None:
            self._lib.tracker_destroy(self._handle)
            self._handle = None

    def update(self, dets):
        """dets: (N,6) -> list of {track_id, ltwh, score, cls_id}"""
        if dets is None or len(dets) == 0:
            flat_arr = (ctypes.c_float * 1)(0)
            n_in = 0
        else:
            flat = dets.flatten().astype(np.float32)
            flat_arr = (ctypes.c_float * len(flat))(*flat)
            n_in = len(dets)

        max_res = 1024
        out = (ctypes.c_float * (max_res * 8))()
        n_out = ctypes.c_int(0)
        ret = self._lib.tracker_update(
            self._handle, flat_arr, n_in, out, max_res, ctypes.byref(n_out)
        )
        if ret != 0:
            return []

        results = []
        for i in range(n_out.value):
            b = i * 8
            results.append(
                {
                    "track_id": int(out[b]),
                    "cls_id": int(out[b + 2]),
                    "score": float(out[b + 3]),
                    "ltwh": [
                        float(out[b + 4]),
                        float(out[b + 5]),
                        float(out[b + 6]),
                        float(out[b + 7]),
                    ],
                }
            )
        return results

    def reset(self):
        self._lib.tracker_reset(self._handle)


# ================================================================
# MOT20 序列处理
# ================================================================


def read_seqinfo(seq_dir):
    """读取 seqinfo.ini 获取序列信息"""
    path = os.path.join(seq_dir, "seqinfo.ini")
    if not os.path.isfile(path):
        return None

    cfg = configparser.ConfigParser()
    cfg.read(path)
    sec = cfg["Sequence"]
    return {
        "name": sec.get("name", ""),
        "imDir": sec.get("imDir", "img1"),
        "frameRate": sec.getint("frameRate", 30),
        "seqLength": sec.getint("seqLength", 0),
        "imWidth": sec.getint("imWidth", 1920),
        "imHeight": sec.getint("imHeight", 1080),
        "imExt": sec.get("imExt", ".jpg"),
    }


def list_image_paths(seq_dir, seqinfo):
    """列出序列的所有图片路径"""
    img_dir = os.path.join(seq_dir, seqinfo["imDir"])
    ext = seqinfo["imExt"]
    n = seqinfo["seqLength"]

    paths = []
    for i in range(1, n + 1):
        path = os.path.join(img_dir, f"{i:06d}{ext}")
        if os.path.isfile(path):
            paths.append(path)
    return paths


def to_mot_line(frame_id, track_id, ltwh, score, cls_id):
    """MOT Challenge 格式: frame_id, track_id, x1, y1, w, h, score, cls_id, visibility, -1"""
    x, y, w, h = ltwh
    x = max(0, x)
    y = max(0, y)
    w = max(1, w)
    h = max(1, h)
    return f"{frame_id},{track_id},{x:.2f},{y:.2f},{w:.2f},{h:.2f},{score:.4f},{cls_id},1,-1\n"


def process_sequence(
    detector,
    tracker,
    seq_dir,
    seqinfo,
    seq_name,
    tracker_type,
    model_name,
):
    """处理单个 MOT20 序列"""
    img_paths = list_image_paths(seq_dir, seqinfo)
    total = len(img_paths)
    print(
        f"  [Seq {seq_name}] {total} frames, {seqinfo['imWidth']}x{seqinfo['imHeight']}"
    )

    # 用于 TrackEval 的输出
    out_path = os.path.join(
        PROJECT_ROOT, "scripts", "mot-res", tracker_type, "data", f"{seq_name}.txt"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    lines = []
    t_start = time.time()

    for idx, img_path in enumerate(img_paths):
        frame_id = idx + 1

        # 检测
        dets = detector.detect(img_path)

        # 跟踪
        tracks = tracker.update(dets)

        # MOT 格式输出
        for t in tracks:
            lines.append(
                to_mot_line(frame_id, t["track_id"], t["ltwh"], t["score"], t["cls_id"])
            )

        # 进度
        if (frame_id % 50 == 0) or (frame_id == total):
            elapsed = time.time() - t_start
            fps = frame_id / elapsed if elapsed > 0 else 0
            print(
                f"    frame {frame_id}/{total} | "
                f"{len(dets)} dets -> {len(tracks)} tracks | "
                f"{fps:.1f} fps",
                end="\r",
            )

    elapsed = time.time() - t_start
    fps = total / elapsed if elapsed > 0 else 0
    print(f"    Done: {len(lines)} tracks, {elapsed:.1f}s ({fps:.1f} fps)")

    # 保存
    with open(out_path, "w") as f:
        f.writelines(lines)

    return out_path, lines


# ================================================================
# TrackEval 评估
# ================================================================
# TrackEval GitHub: https://github.com/JonathonLuiten/TrackEval.git


def run_trackeval(tracker_names, seq_list, split, mot20_root):
    """使用 TrackEval 评估跟踪结果"""

    # 评估配置 - 完全匹配 TrackEval 默认参数
    eval_config = trackeval.Evaluator.get_default_eval_config()
    eval_config["DISPLAY_LESS_PROGRESS"] = False
    eval_config["PRINT_ONLY_COMBINED"] = True
    eval_config["TIME_PROGRESS"] = False

    dataset_config = trackeval.datasets.MotChallenge2DBox.get_default_dataset_config()

    # GT_FOLDER 应为 train/ 或 test/ 的直接父目录 (不含 split 子目录)
    # TrackEval 拼装路径: GT_FOLDER/seq/gt/gt.txt
    # 我们的数据: <mot20_root>/train/MOT20-01/gt/gt.txt
    # 因此 GT_FOLDER = <mot20_root>/<split>
    gt_folder = os.path.join(mot20_root, split)
    dataset_config["GT_FOLDER"] = gt_folder
    dataset_config["TRACKERS_FOLDER"] = os.path.join(PROJECT_ROOT, "scripts", "mot-res")
    dataset_config["TRACKERS_TO_EVAL"] = tracker_names
    dataset_config["BENCHMARK"] = "MOT20"
    dataset_config["SPLIT_TO_EVAL"] = split
    dataset_config["TRACKER_SUB_FOLDER"] = "data"
    dataset_config["OUTPUT_FOLDER"] = os.path.join(PROJECT_ROOT, "scripts", "mot-res")
    dataset_config["OUTPUT_SUB_FOLDER"] = "eval_results"
    dataset_config["SEQMAP_FOLDER"] = os.path.join(
        os.path.dirname(mot20_root), "MOTChallengeEvalKit", "seqmaps"
    )
    dataset_config["SKIP_SPLIT_FOL"] = True
    dataset_config["DO_PREPROC"] = True
    # 限制只评估已处理的序列
    dataset_config["SEQ_INFO"] = {seq: None for seq in seq_list}

    metrics_config = {
        "METRICS": ["HOTA", "CLEAR", "Identity"],
        "THRESHOLD": 0.5,
    }

    # 运行评估
    evaluator = trackeval.Evaluator(eval_config)
    dataset = trackeval.datasets.MotChallenge2DBox(dataset_config)
    # build metric list (每个类型只创建一次)
    metric_list = []
    for metric_name in metrics_config["METRICS"]:
        if metric_name == "HOTA":
            metric_list.append(trackeval.metrics.hota.HOTA(metrics_config))
        elif metric_name == "CLEAR":
            metric_list.append(trackeval.metrics.clear.CLEAR(metrics_config))
        elif metric_name == "Identity":
            metric_list.append(trackeval.metrics.identity.Identity(metrics_config))

    output_res, output_msg = evaluator.evaluate([dataset], metric_list)

    return output_res


# ================================================================
# 主入口
# ================================================================


def main():
    parser = argparse.ArgumentParser(
        description="MOT20 评估流水线: YOLO 检测 + 多目标跟踪 + TrackEval 评估",
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
        "--mot20_root",
        type=str,
        default="/mnt/Z/Datasets/MOT/MOT20/MOT20",
        help="MOT20 数据集根目录路径;\n包含 train/ 和 test/ 子目录;\n默认: %(default)s",
    )
    parser.add_argument(
        "--det_ini",
        type=str,
        required=True,
        help="检测模型 ini 配置文件路径 (相对于项目根目录或绝对路径);\n"
        "例如: config/detection/yolov8nDetPerson-bn.ini",
    )
    parser.add_argument(
        "--det_onnx",
        type=str,
        required=True,
        help="检测模型 ONNX 文件路径 (相对于项目根目录或绝对路径);\n"
        "例如: onnx/yolov8nPose-bn.onnx",
    )
    parser.add_argument(
        "--device",
        type=int,
        default=-1,
        help="指定运行设备; CPU: -1, GPU: >=0; 例如: --device -1 使用CPU, --device 0 使用第1块GPU",
    )
    parser.add_argument(
        "--trackers",
        type=str,
        nargs="+",
        required=True,
        choices=["deepsort", "bytetrack"],
        help="要运行的跟踪器类型; 可选: deepsort, bytetrack; 可同时指定多个",
    )
    parser.add_argument(
        "--no_reid",
        action="store_true",
        help="DeepSORT 不使用 ReID 特征 (仅用运动模型关联)",
    )
    parser.add_argument(
        "--eval",
        action="store_true",
        help="跟踪完成后运行 TrackEval 评估 (需要安装 TrackEval)",
    )
    parser.add_argument(
        "--split",
        type=str,
        default="train",
        choices=["train", "test"],
        help="评估数据集划分; train 或 test",
    )
    parser.add_argument(
        "--seqs",
        type=str,
        nargs="+",
        default=None,
        help="指定要处理的序列名称; 例如: MOT20-01 MOT20-02;\n"
        "不指定则处理 split 对应的所有序列",
    )
    args = parser.parse_args()

    # 路径
    def resolve(p):
        if not os.path.isabs(p):
            p = os.path.join(PROJECT_ROOT, p)
        if not os.path.exists(p):
            raise FileNotFoundError(f"Not found: {p}")
        return p

    lib_path = os.path.join(
        PROJECT_ROOT,
        "tracker_lib",
        "build",
        "lib",
        "libtracker.so",
    )
    if not os.path.isfile(lib_path):
        print("[ERROR] Build first: bash build_tracker_lib.sh")
        sys.exit(1)

    det_ini = os.path.abspath(args.det_ini)
    det_onnx = os.path.abspath(args.det_onnx)
    model_name = os.path.splitext(os.path.basename(det_onnx))[0]

    # 加载库
    print(f"[INFO] Loading library: {lib_path}")
    lib = load_lib(lib_path)
    print(f"[INFO] Version: {lib.tracker_version().decode()}")

    # 创建检测器
    print(f"[INFO] Creating detector: {model_name}")
    detector = Detector(lib, det_ini, det_onnx, args.device)

    # 测试检测器
    print("[INFO] Testing detector ...")
    test_img = os.path.join(args.mot20_root, "train", "MOT20-01", "img1", "000001.jpg")
    if os.path.isfile(test_img):
        test = detector.detect(test_img)
        print(f"[INFO] Detector OK: {len(test)} objects on sample")
    else:
        print("[INFO] Detector loaded (no test image)")

    # 确定序列列表
    if args.seqs:
        seq_list = args.seqs
    elif args.split == "train":
        seq_list = MOT20_SEQS_TRAIN
    else:
        seq_list = MOT20_SEQS_TEST

    split_dir = (
        os.path.join(args.mot20_root, "train")
        if args.split == "train"
        else os.path.join(args.mot20_root, "test")
    )

    # 处理每个跟踪器
    for ttype in args.trackers:
        print(f"\n{'=' * 60}")
        print(f"[{ttype.upper()}] Running on MOT20 {args.split}")
        print(f"{'=' * 60}")

        # 跟踪配置
        if ttype == "deepsort" and args.no_reid:
            ini = os.path.join(
                PROJECT_ROOT, "config", "tracker", "deepsort_no_reid.ini"
            )
        elif ttype == "deepsort":
            ini = os.path.join(PROJECT_ROOT, "config", "tracker", "deepsort.ini")
        else:
            ini = os.path.join(PROJECT_ROOT, "config", "tracker", "bytetrack.ini")

        if not os.path.isfile(ini):
            print(f"[WARN] Config not found: {ini}, skip")
            continue

        total_lines = 0
        time_total = time.time()

        for seq_name in seq_list:
            seq_dir = os.path.join(split_dir, seq_name)
            if not os.path.isdir(seq_dir):
                print(f"[WARN] Sequence not found: {seq_dir}")
                continue

            seqinfo = read_seqinfo(seq_dir)
            if seqinfo is None:
                print(f"[WARN] No seqinfo.ini in {seq_dir}")
                continue

            # 每个序列新建跟踪器 (reset 状态)
            tracker = Tracker(lib, ini)

            out_path, lines = process_sequence(
                detector,
                tracker,
                seq_dir,
                seqinfo,
                seq_name,
                ttype,
                model_name,
            )
            total_lines += len(lines)

            del tracker

        elapsed = time.time() - time_total
        print(f"\n[INFO] {ttype}: {total_lines} total tracks, {elapsed:.1f}s")
        print(f"[INFO] Results in: {os.path.join(PROJECT_ROOT, 'reports', ttype)}")

    # TrackEval 评估
    if args.eval:
        print(f"\n{'=' * 60}")
        print("[EVAL] Running TrackEval ...")
        print(f"{'=' * 60}")
        run_trackeval(args.trackers, seq_list, args.split, args.mot20_root)

    print("\n[INFO] All done!")


if __name__ == "__main__":
    main()
