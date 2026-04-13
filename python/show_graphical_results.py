from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from tightfig import tightfig


def _crop_common(signal: np.ndarray, lp: int, start: int = 2) -> np.ndarray:
    return signal[lp + start - 1 :]


def show_graphical_results(example_id: int, d: dict) -> None:
    dt = d["dt"]
    tsim = d["tsim"]

    if example_id == 0:
        t = np.arange(dt, tsim + 1e-12, dt)
        lp = d["lp"]
        uc = _crop_common(d["uc"], lp)
        yc = _crop_common(d["yc"], lp)
        rc = _crop_common(d["rc"], lp)
        t = t[: len(uc)]

        fig, ax = plt.subplots(2, 1, num=1, figsize=(10, 7), sharex=True)
        ax[0].plot(t, rc + d["y0"], "r", linewidth=2, label="Setpoint")
        ax[0].plot(t, yc + d["y0"], "k", linewidth=2, label="Process output")
        ax[0].grid(True)
        ax[0].legend(loc="lower right")
        ax[0].set_ylabel("Process output")

        ax[1].plot(t, uc, "k", linewidth=2, label="Control signal")
        ax[1].grid(True)
        ax[1].legend(loc="lower right")
        ax[1].set_ylabel("Controller output")
        ax[1].set_xlabel("Time (s)")

    elif example_id == 1:
        t = np.arange(dt, tsim + 1e-12, dt)
        lp = d["lp"]
        uc = _crop_common(d["uc"], lp)
        yc = _crop_common(d["yc"], lp)
        rc = _crop_common(d["rc"], lp)
        t = t[: len(uc)]

        fig, ax = plt.subplots(2, 1, num=1, figsize=(10, 7), sharex=True)
        ax[0].plot(t, rc + d["y0"], "r", linewidth=2, label="Setpoint")
        ax[0].plot(t, yc + d["y0"], "k", linewidth=2, label="y")
        ax[0].legend(loc="lower right")
        ax[0].grid(True)
        ax[0].set_ylabel("Process output")

        ax[1].plot(t, uc, "k", linewidth=2, label="u")
        ax[1].legend(loc="lower right")
        ax[1].grid(True)
        ax[1].set_ylabel("Controller output")
        ax[1].set_xlabel("Time (s)")

    elif example_id == 2:
        t = np.arange(dt, tsim + 1e-12, dt)
        lp = d["lp"]
        uc = _crop_common(d["uc"], lp)
        yc = _crop_common(d["yc"], lp)
        rc = _crop_common(d["rc"], lp)
        t = t[: len(uc)]

        fig, ax = plt.subplots(2, 1, num=1, figsize=(10, 7), sharex=True)
        ax[0].plot(t, rc + d["y0"], "r", linewidth=2, label="Setpoint")
        ax[0].plot(t, yc + d["y0"], "k", linewidth=2, label="Process output")
        ax[0].legend(loc="upper right")
        ax[0].grid(True)
        ax[0].set_ylabel("Process output")

        ax[1].plot(t, uc, "k-", linewidth=2, label="Control signal")
        ax[1].legend(loc="upper right")
        ax[1].grid(True)
        ax[1].set_ylabel("Controller output")
        ax[1].set_xlabel("Time (s)")

    elif example_id == 3:
        t = np.arange(dt, tsim + 1e-12, dt)
        lp = d["lp"]
        uc1 = _crop_common(d["uc1"], lp)
        uc2 = _crop_common(d["uc2"], lp)
        uc3 = _crop_common(d["uc3"], lp)
        yc1 = _crop_common(d["yc1"], lp)
        yc2 = _crop_common(d["yc2"], lp)
        yc3 = _crop_common(d["yc3"], lp)
        rc = _crop_common(d["rc"], lp)
        t = t[: len(uc1)]

        fig, ax = plt.subplots(2, 1, num=1, figsize=(10, 7), sharex=True)
        ax[0].plot(t, rc + d["y0"], "r", linewidth=2, label="Setpoint")
        ax[0].plot(t, yc1 + d["y0"], "k--", linewidth=2, label="b=1")
        ax[0].plot(t, yc2 + d["y0"], "k:", linewidth=2, label="b=0.5")
        ax[0].plot(t, yc3 + d["y0"], "k", linewidth=2, label="b=0")
        ax[0].legend(loc="lower right")
        ax[0].grid(True)
        ax[0].set_ylabel("Process output")

        ax[1].plot(t, uc1, "k--", linewidth=2, label="b=1")
        ax[1].plot(t, uc2, "k:", linewidth=2, label="b=0.5")
        ax[1].plot(t, uc3, "k", linewidth=2, label="b=0")
        ax[1].legend(loc="lower right")
        ax[1].grid(True)
        ax[1].set_ylabel("Controller output")
        ax[1].set_xlabel("Time (s)")

    elif example_id == 4:
        t = np.arange(dt, tsim + 1e-12, dt)
        lp = d["lp"]
        uc1 = _crop_common(d["uc1"], lp)
        uc2 = _crop_common(d["uc2"], lp)
        uc3 = _crop_common(d["uc3"], lp)
        uc4 = _crop_common(d["uc4"], lp)
        y1 = _crop_common(d["y1"], lp)
        y2 = _crop_common(d["y2"], lp)
        y3 = _crop_common(d["y3"], lp)
        y4 = _crop_common(d["y4"], lp)
        rc = _crop_common(d["rc"], lp)
        t = t[: len(uc1)]

        fig, ax = plt.subplots(2, 1, num=1, figsize=(10, 7), sharex=True)
        ax[0].plot(t, rc + d["y0"], "r", linewidth=2, label="Setpoint")
        ax[0].plot(t, y1 + d["y0"], "b", linewidth=2, label="No saturation")
        ax[0].plot(t, y2 + d["y0"], "k:", linewidth=2, label="No anti-windup")
        ax[0].plot(t, y3 + d["y0"], "k--", linewidth=2, label="Control signal clamping")
        ax[0].plot(t, y4 + d["y0"], "k", linewidth=2, label="Back-calculation")
        ax[0].legend(loc="upper right")
        ax[0].grid(True)
        ax[0].set_ylabel("Process output")

        ax[1].plot(t, uc1, "b", linewidth=2, label="No saturation")
        ax[1].plot(t, uc2, "k:", linewidth=2, label="No anti-windup")
        ax[1].plot(t, uc3, "k--", linewidth=2, label="Control signal clamping")
        ax[1].plot(t, uc4, "k", linewidth=2, label="Back-calculation")
        ax[1].legend(loc="upper right")
        ax[1].grid(True)
        ax[1].set_ylabel("Control signal")
        ax[1].set_xlabel("Time (s)")

    elif example_id == 5:
        t = np.arange(dt, tsim + 1e-12, dt)
        lp = d["lp"]
        uc1 = _crop_common(d["uc1"], lp)
        uc2 = _crop_common(d["uc2"], lp)
        uc3 = _crop_common(d["uc3"], lp)
        yc1 = _crop_common(d["yc1"], lp)
        yc2 = _crop_common(d["yc2"], lp)
        yc3 = _crop_common(d["yc3"], lp)
        rc = _crop_common(d["rc"], lp)
        t = t[: len(uc1)]

        fig, ax = plt.subplots(2, 1, num=1, figsize=(10, 7), sharex=True)
        ax[0].plot(t, rc + d["y0"], "r", linewidth=2, label="Setpoint")
        ax[0].plot(t, yc1 + d["y0"], "k", linewidth=2, label="No filter")
        ax[0].plot(t, yc2 + d["y0"], "b", linewidth=2, label="Filter T_f=0.1*T_i")
        ax[0].plot(t, yc3 + d["y0"], "g", linewidth=2, label="Filter T_f=0.01*T_i")
        ax[0].legend(loc="lower right")
        ax[0].grid(True)
        ax[0].set_ylabel("Process output")

        ax[1].plot(t, uc1, "k", linewidth=2, label="No filter")
        ax[1].plot(t, uc2, "b", linewidth=2, label="Filter T_f=0.1*T_i")
        ax[1].plot(t, uc3, "g", linewidth=2, label="Filter T_f=0.01*T_i")
        ax[1].legend(loc="lower right")
        ax[1].grid(True)
        ax[1].set_ylabel("Controller output")
        ax[1].set_xlabel("Time (s)")

    elif example_id == 6:
        t = np.arange(dt, tsim + 1e-12, dt)
        lp = d["lp"]

        uc = d["uc"][lp:-1]
        yc = d["yc"][lp + 1 :]
        rc = d["rc"][lp:-1]
        ucs = d["ucs"][lp:-1]
        ycs = d["ycs"][lp + 1 :]
        modess = d["modess"][lp:-1]

        n = min(len(uc), len(yc), len(rc), len(ucs), len(ycs), len(modess), len(t))
        t = t[:n]
        uc = uc[:n]
        yc = yc[:n]
        rc = rc[:n]
        ucs = ucs[:n]
        ycs = ycs[:n]
        modess = modess[:n]

        fig, ax = plt.subplots(3, 1, num=1, figsize=(10, 9), sharex=True)
        ax[0].step(t, rc, "r", linewidth=2, where="post", label="Setpoint")
        ax[0].plot(t, yc, "k", linewidth=2, label="Multiple controllers")
        ax[0].plot(t, ycs, "k:", linewidth=2, label="Single controller")
        ax[0].legend(loc="lower right")
        ax[0].grid(True)
        ax[0].set_ylabel("Tank level (cm)")

        ax[1].plot(t, uc, "k", linewidth=2, label="Multiple controllers")
        ax[1].plot(t, ucs, "k:", linewidth=2, label="Single controller")
        ax[1].legend(loc="lower right")
        ax[1].grid(True)
        ax[1].set_ylabel("Flow rate (cm^3/s)")

        ax[2].step(t, modess, "k", linewidth=2, where="post", label="Controller active")
        ax[2].legend(loc="lower right")
        ax[2].grid(True)
        ax[2].set_ylabel("N active controller")
        ax[2].set_xlabel("Time (s)")

    elif example_id == 7:
        t = np.arange(dt, tsim + 1e-12, dt)
        lp = d["lp"]

        uc = d["uc"][lp:]
        yc1 = d["yc1"][lp:]
        yc2 = d["yc2"][lp:]
        rc1 = d["rc1"][lp:]
        rc2 = d["rc2"][lp:]
        uc1 = d["uc1"][lp:]
        uc2 = d["uc2"][lp:]
        v = d["v"][lp:-1]
        t = t[: len(uc)]

        n = min(len(t), len(yc1), len(yc2), len(rc1), len(rc2), len(uc1), len(uc2), len(v))
        t = t[:n]
        uc = uc[:n]
        yc1 = yc1[:n]
        yc2 = yc2[:n]
        rc1 = rc1[:n]
        rc2 = rc2[:n]
        uc1 = uc1[:n]
        uc2 = uc2[:n]
        v = v[:n]

        aux = uc1 - uc2
        ii = np.where(np.abs(aux) < 1e-6)[0]

        fig, ax = plt.subplots(4, 1, num=1, figsize=(11, 10), sharex=True)

        ax[0].plot(t, rc1, "r", linewidth=2, label="r1")
        ax[0].plot(t, rc2, "r:", linewidth=2, label="r2")
        ax[0].plot(t, yc1, "b", linewidth=2, label="y1")
        ax[0].plot(t, yc2, "b:", linewidth=2, label="y2")
        ax[0].set_ylabel("Process outputs")
        ax[0].grid(True)
        ax[0].legend(loc="lower right")

        ax[1].plot(t, uc1, "b", linewidth=2, label="uc1")
        ax[1].plot(t, uc2, "b:", linewidth=2, label="uc2")
        ax[1].plot(t, uc, "m--", linewidth=2, label="u")
        ax[1].set_ylabel("Control signals")
        ax[1].grid(True)
        ax[1].legend(loc="lower right")

        ax[2].plot(t, uc1 - uc2, "b", linewidth=2, label="uc1-uc2")
        ax[2].axhline(0, color="k", linestyle=":", linewidth=2)
        ax[2].set_ylabel("uc1-uc2")
        ax[2].grid(True)
        ax[2].legend(loc="lower right")

        ax[3].plot(t, v, linewidth=2, label="v")
        ax[3].set_ylabel("v")
        ax[3].set_xlabel("Time (s)")
        ax[3].grid(True)
        ax[3].legend(loc="lower right")

        if len(ii) >= 3:
            x_a = t[ii[1]]
            x_b = t[ii[2]]
            for axi in [ax[0], ax[1], ax[2]]:
                y0, y1 = axi.get_ylim()
                axi.axvspan(0.0, x_a, color="0.9", alpha=0.9)
                axi.axvspan(x_a, x_b, color="0.9", alpha=0.5)
                axi.axvspan(x_b, t[-1], color="0.9", alpha=0.9)
                axi.axvline(x_a, color="k", linestyle=":")
                axi.axvline(x_b, color="k", linestyle=":")

    else:
        raise ValueError("Wrong Example ID")

    tightfig()
    plt.show()
