#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Portable loader for old and V5 sweep_2d Excel schemas."""
from __future__ import annotations

import glob
import os
from pathlib import Path
from typing import Optional, Tuple

import pandas as pd

ROOT = Path(__file__).resolve().parent
DEFAULT_SWEEP = ROOT / "output_5zones" / "sweep_2d_time_fuel_v5_3_1.xlsx"
DEFAULT_OUTDIR = ROOT / "analysis_figures"

# V5 Chinese export columns -> compact names used by the existing analysis code.
COLUMN_MAP = {
    "权重_w_time": "wt", "权重_w_risk": "wr", "权重_w_fuel": "wf",
    "飞行距离_km": "L", "飞行时间_min": "Tm", "累积暴露_C": "C",
    "燃油消耗_kg": "F", "绕行增加_%": "detour",
    "最大高度_km": "altkm", "最大高度_m": "altm", "航点数": "npts",
    "归一化时间": "nT", "归一化暴露": "nC", "归一化燃油": "nF",
    "统一目标_J": "J", "迭代次数": "iter", "扩展节点数": "nodes",
    "计算耗时_s": "ct", "飞行时间_s": "T", "时间戳": "ts",
    "最小转弯半径_m": "minR", "最大曲率_1_m": "maxK",
    "最大爬升角_deg": "maxClimb", "平滑评价_Js": "Js",
    "曲率变化积分_JkappaPrime": "JkappaPrime",
    "末端航向误差_deg": "terminalHeadingErr",
    "速度变化率最大_m_s2": "maxSpeedRate", "平滑成功": "smoothOK",
    "选中_s_m2": "sSelected",
}

LEGACY_18 = [
    "wt", "wr", "wf", "L", "Tm", "C", "F", "detour", "altkm", "npts",
    "nT", "nC", "iter", "nodes", "ct", "T", "altm", "ts",
]


def resolve_sweep_file(path: Optional[str] = None) -> Path:
    if path:
        p = Path(path).expanduser()
        if p.exists():
            return p
        raise FileNotFoundError(p)
    env = os.environ.get("SWEEP_FILE")
    if env:
        p = Path(env).expanduser()
        if p.exists():
            return p
        raise FileNotFoundError(p)
    if DEFAULT_SWEEP.exists():
        return DEFAULT_SWEEP
    candidates = sorted(
        (Path(x) for x in glob.glob(str(ROOT / "output_5zones" / "sweep_2d_time_fuel*.xlsx"))
         if not x.endswith('.tmp.xlsx')),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    if candidates:
        return candidates[0]
    raise FileNotFoundError(
        f"No sweep Excel found. Run sweep_2d.py first or set SWEEP_FILE. Expected {DEFAULT_SWEEP}")


def analysis_output_dir(path: Optional[str] = None) -> Path:
    p = Path(path or os.environ.get("ANALYSIS_OUTDIR") or DEFAULT_OUTDIR).expanduser()
    p.mkdir(parents=True, exist_ok=True)
    return p


def load_sweep_dataframe(path: Optional[str] = None) -> Tuple[pd.DataFrame, Path]:
    source = resolve_sweep_file(path)
    df = pd.read_excel(source, sheet_name="全部数据")

    # New schema: map by names, never by column count.
    mapped = {c: COLUMN_MAP[c] for c in df.columns if c in COLUMN_MAP}
    if {"权重_w_time", "权重_w_risk", "权重_w_fuel"}.issubset(df.columns):
        df = df.rename(columns=mapped)
    elif {"wt", "wr", "wf"}.issubset(df.columns):
        pass
    elif len(df.columns) >= 18:
        # Last-resort compatibility for the historical 18-column workbook.
        rename = {old: new for old, new in zip(df.columns[:18], LEGACY_18)}
        df = df.rename(columns=rename)
    else:
        raise ValueError(f"Unrecognized sweep schema in {source}: {list(df.columns)}")

    # Stable aliases used inconsistently by older plotting scripts.
    if "altkm" in df.columns:
        df["alt"] = df["altkm"]
    elif "altm" in df.columns:
        df["altkm"] = pd.to_numeric(df["altm"], errors="coerce") / 1000.0
        df["alt"] = df["altkm"]
    if "altm" not in df.columns and "altkm" in df.columns:
        df["altm"] = pd.to_numeric(df["altkm"], errors="coerce") * 1000.0

    for c in df.columns:
        if c not in {"ts", "schema_version", "run_signature", "case_file", "错误", "平滑版本"}:
            try:
                df[c] = pd.to_numeric(df[c], errors="coerce")
            except Exception:
                pass
    return df, source
