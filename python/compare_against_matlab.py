from __future__ import annotations

import json
import runpy
import shutil
import subprocess
from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parent
MATLAB_EXPORTER = ROOT / "export_matlab_workspace.m"
OUTPUT_DIR = ROOT / "comparison_outputs"
MATLAB_DIR = OUTPUT_DIR / "matlab"
REPORT_PATH = OUTPUT_DIR / "comparison_report.json"
STUB_DIR = ROOT / "comparison_stubs"

EXAMPLES = [
    ("Example_basic.m", "Example_basic.py"),
    ("Example1_paper.m", "Example1_paper.py"),
    ("Example2_paper.m", "Example2_paper.py"),
    ("Example3_paper.m", "Example3_paper.py"),
    ("Example4_paper.m", "Example4_paper.py"),
    ("Example5_paper.m", "Example5_paper.py"),
    ("Example6_paper.m", "Example6_paper.py"),
    ("Example7_paper.m", "Example7_paper.py"),
]

COMPARE_KEYS = {
    "uc",
    "yc",
    "rc",
    "noise_signal",
    "uc1",
    "uc2",
    "uc3",
    "uc4",
    "yc1",
    "yc2",
    "yc3",
    "yc4",
    "y1",
    "y2",
    "y3",
    "y4",
    "yd",
    "ud",
    "uff",
    "modes",
    "ucs",
    "ycs",
    "rcs",
    "modess",
    "v",
    "rc1",
    "rc2",
    "r",
    "r1",
    "r2",
    "dt",
    "tsim",
    "lp",
    "ap",
    "bp",
}


def _ensure_dirs() -> None:
    MATLAB_DIR.mkdir(parents=True, exist_ok=True)


def _matlab_executable() -> str:
    matlab = shutil.which("matlab")
    if not matlab:
        raise RuntimeError("MATLAB no esta disponible en PATH")
    return matlab


def _normalize_matlab_value(value: Any) -> Any:
    arr = np.asarray(value)
    if arr.dtype.kind in {"U", "S"}:
        return str(arr.squeeze())
    arr = np.squeeze(arr)
    if arr.shape == ():
        return float(arr) if np.issubdtype(arr.dtype, np.floating) else arr.item()
    return np.ravel(arr.astype(float))


def _load_matlab_results(mat_path: Path) -> dict[str, Any]:
    raw = loadmat(mat_path)
    result: dict[str, Any] = {}
    for key, value in raw.items():
        if key.startswith("__"):
            continue
        try:
            result[key] = _normalize_matlab_value(value)
        except Exception:
            continue
    return result


def _patch_and_run_python(script_path: Path, matlab_data: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    import process_and_simulation_data as psd
    import show_graphical_results as sgr

    captured: dict[str, Any] = {}
    original_loader = psd.load_process_and_simulation_data
    original_show = sgr.show_graphical_results

    def patched_loader(example_id: int, base_path: str | Path | None = None) -> dict[str, Any]:
        data = original_loader(example_id, base_path)
        for key in ("noise_signal", "v"):
            if key in matlab_data and key in data:
                data[key] = np.asarray(matlab_data[key], dtype=float).copy()
        return data

    def capture_show(example_id: int, data: dict[str, Any]) -> None:
        captured["example_id"] = example_id
        captured["data"] = data

    psd.load_process_and_simulation_data = patched_loader
    sgr.show_graphical_results = capture_show
    try:
        runpy.run_path(str(script_path), run_name="__main__")
    finally:
        psd.load_process_and_simulation_data = original_loader
        sgr.show_graphical_results = original_show

    return int(captured.get("example_id", -1)), captured.get("data", {})


def _normalize_python_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        arr = np.squeeze(value)
        if arr.shape == ():
            return float(arr) if np.issubdtype(arr.dtype, np.floating) else arr.item()
        return np.ravel(arr.astype(float))
    if isinstance(value, (float, int, np.floating, np.integer)):
        return float(value)
    return value


def _compare_values(py_value: Any, ml_value: Any) -> dict[str, Any]:
    py_norm = _normalize_python_value(py_value)
    ml_norm = ml_value

    if isinstance(py_norm, np.ndarray) and isinstance(ml_norm, np.ndarray):
        n = min(py_norm.size, ml_norm.size)
        diff = np.abs(py_norm[:n] - ml_norm[:n])
        return {
            "type": "array",
            "python_len": int(py_norm.size),
            "matlab_len": int(ml_norm.size),
            "compared_len": int(n),
            "max_abs_error": float(diff.max()) if n else 0.0,
            "mean_abs_error": float(diff.mean()) if n else 0.0,
            "allclose": bool(np.allclose(py_norm[:n], ml_norm[:n], atol=1e-6, rtol=1e-6)),
        }

    if isinstance(py_norm, (float, int)) and isinstance(ml_norm, (float, int)):
        err = abs(float(py_norm) - float(ml_norm))
        return {
            "type": "scalar",
            "python": float(py_norm),
            "matlab": float(ml_norm),
            "abs_error": float(err),
            "allclose": bool(np.isclose(py_norm, ml_norm, atol=1e-9, rtol=1e-9)),
        }

    return {
        "type": "unsupported",
        "python": str(type(py_norm)),
        "matlab": str(type(ml_norm)),
    }


def _run_matlab_export(matlab_exe: str, matlab_script: str, out_file: Path) -> None:
    cwd = ROOT.as_posix()
    stub = STUB_DIR.as_posix()
    script_path = (ROOT / matlab_script).as_posix()
    script = script_path.replace("'", "''")
    out = out_file.as_posix().replace("'", "''")
    cmd = [
        matlab_exe,
        "-batch",
        f"cd('{stub}'); addpath(genpath('{cwd}')); export_matlab_workspace('{script}','{out}');",
    ]
    subprocess.run(cmd, check=True, cwd=ROOT)


def main() -> None:
    _ensure_dirs()
    matlab_exe = _matlab_executable()
    report: dict[str, Any] = {}

    for matlab_script, python_script in EXAMPLES:
        base_name = Path(python_script).stem
        out_file = MATLAB_DIR / f"{base_name}.mat"
        _run_matlab_export(matlab_exe, matlab_script, out_file)

        matlab_data = _load_matlab_results(out_file)
        _, python_data = _patch_and_run_python(ROOT / python_script, matlab_data)

        example_report: dict[str, Any] = {
            "matlab_script": matlab_script,
            "python_script": python_script,
            "comparisons": {},
        }

        common_keys = sorted((COMPARE_KEYS & set(matlab_data.keys()) & set(python_data.keys())))
        for key in common_keys:
            example_report["comparisons"][key] = _compare_values(python_data[key], matlab_data[key])

        report[base_name] = example_report

    REPORT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
