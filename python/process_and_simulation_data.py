from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import loadmat


def _n_samples(tsim: float, dt: float) -> int:
    return int(round(tsim / dt))


def _extract_seed_from_mat_s(s_obj: np.ndarray) -> int:
    # semilla.mat guarda el estado de rng de MATLAB como struct.
    try:
        return int(np.asarray(s_obj[0, 0]["Seed"]).squeeze())
    except Exception:
        return int(np.asarray(s_obj).squeeze())


def load_process_and_simulation_data(example_id: int, base_path: str | Path | None = None) -> dict[str, Any]:
    base = Path(base_path) if base_path is not None else Path(__file__).resolve().parent
    data: dict[str, Any] = {"exampleID": example_id}

    if example_id == 0:
        k = 1.0
        t = 1.0
        l = 0.5
        y0 = 0.0
        u0 = 0.0
        tsim = 20.0 + l
        dt = 0.01

        ap = np.exp(-dt / t)
        bp = k * (1.0 - ap)
        lp = int(round(l / dt))
        n = _n_samples(tsim, dt)

        uc = np.zeros(n)
        yc = np.zeros(n)
        rc = np.zeros(n)
        r = 0.0

        snr_db = 5.0
        variance = 1.0 / (dt * (10.0 ** (snr_db / 10.0)))
        semilla = loadmat(base / "semilla.mat")
        seed = _extract_seed_from_mat_s(semilla["s"])
        rng = np.random.default_rng(seed)
        w = np.sqrt(variance) * rng.standard_normal(n)
        noise_signal = 0.01 * w

        data.update(locals())

    elif example_id == 1:
        k = 1.0
        t = 1.0
        l = 0.5
        y0 = 0.0
        u0 = 0.0
        tsim = 20.0 + l
        dt = 0.01

        ap = np.exp(-dt / t)
        bp = k * (1.0 - ap)
        lp = int(round(l / dt))
        n = _n_samples(tsim, dt)

        uc = np.zeros(n)
        yc = np.zeros(n)
        rc = np.zeros(n)
        r = 3.0

        data.update(locals())

    elif example_id == 2:
        k = 1.0
        t = 1.0
        l = 0.5
        y0 = 0.0
        u0 = 1.0
        tsim = 30.0 + l
        dt = 0.1

        ap = np.exp(-dt / t)
        bp = k * (1.0 - ap)
        lp = int(round(l / dt))
        n = _n_samples(tsim, dt)

        uc = np.ones(n)
        yc = np.ones(n)
        rc = np.ones(n)
        r = 1.0

        data.update(locals())

    elif example_id == 3:
        k = 1.0
        t = 1.0
        l = 0.5
        y0 = 0.0
        u0 = 0.0
        tsim = 20.0 + l
        dt = 0.01

        ap = np.exp(-dt / t)
        bp = k * (1.0 - ap)
        lp = int(round(l / dt))
        n = _n_samples(tsim, dt)

        uc1 = np.zeros(n)
        uc2 = np.zeros(n)
        uc3 = np.zeros(n)
        yc1 = np.zeros(n)
        yc2 = np.zeros(n)
        yc3 = np.zeros(n)
        rc = np.zeros(n)
        r = 0.0
        dist = 0.0

        data.update(locals())

    elif example_id == 4:
        k = 1.0
        t = 3.0
        l = 0.5

        kd = 2.0
        td = 1.0
        ld = 0.5

        y0 = 0.0
        u0 = 0.0

        umax = 7.0
        umin = 0.0
        dumax = np.inf
        dumin = -np.inf

        dt = 0.01
        tsim = 70.0 + l

        ap = np.exp(-dt / t)
        bp = k * (1.0 - ap)
        lp = int(round(l / dt))

        apd = np.exp(-dt / td)
        bpd = kd * (1.0 - apd)
        lpd = int(round(ld / dt))

        n = _n_samples(tsim, dt)

        uff = np.zeros(n)
        uc1 = np.zeros(n)
        uc2 = np.zeros(n)
        uc3 = np.zeros(n)
        uc4 = np.zeros(n)
        utotal1 = np.zeros(n)
        utotal2 = np.zeros(n)
        utotal3 = np.zeros(n)
        utotal4 = np.zeros(n)
        yc1 = np.zeros(n)
        yc2 = np.zeros(n)
        yc3 = np.zeros(n)
        yc4 = np.zeros(n)
        y1 = np.zeros(n)
        y2 = np.zeros(n)
        y3 = np.zeros(n)
        y4 = np.zeros(n)
        yd = np.zeros(n)
        ud = np.zeros(n)
        rc = np.ones(n)
        r = 0.0

        data.update(locals())

    elif example_id == 5:
        k = 1.0
        t = 1.0
        l = 0.5
        y0 = 0.0
        u0 = 0.0
        tsim = 20.0 + l
        dt = 0.01

        ap = np.exp(-dt / t)
        bp = k * (1.0 - ap)
        lp = int(round(l / dt))
        n = _n_samples(tsim, dt)

        uc1 = np.zeros(n)
        uc2 = np.zeros(n)
        uc3 = np.zeros(n)
        yc1 = np.zeros(n)
        yc2 = np.zeros(n)
        yc3 = np.zeros(n)
        rc = np.zeros(n)
        r = 0.0

        snr_db = 5.0
        variance = 1.0 / (dt * (10.0 ** (snr_db / 10.0)))
        semilla = loadmat(base / "semilla.mat")
        seed = _extract_seed_from_mat_s(semilla["s"])
        rng = np.random.default_rng(seed)
        w = np.sqrt(variance) * rng.standard_normal(n)
        noise_signal = 0.01 * w

        data.update(locals())

    elif example_id == 6:
        a_tank = 390.0
        a_orifice = 2.15
        g = 983.0

        h10 = 4.0
        t1 = (a_tank / a_orifice) * np.sqrt((2.0 * h10 / g))
        k1 = t1 / a_tank
        l = 0.0
        q10 = a_orifice * np.sqrt(2.0 * g * h10)

        h20 = 12.0
        t2 = (a_tank / a_orifice) * np.sqrt((2.0 * h20 / g))
        k2 = t2 / a_tank

        h30 = 20.0
        t3 = (a_tank / a_orifice) * np.sqrt((2.0 * h30 / g))
        k3 = t3 / a_tank

        tsim = 600.0
        r = h10
        dt = 0.01
        lp = int(np.ceil(l / dt))
        n = _n_samples(tsim, dt)

        uc = np.ones(n) * q10
        yc = np.ones(n) * h10
        rc = np.ones(n) * h10
        uc1 = np.ones(n) * q10
        uc2 = np.ones(n) * q10
        uc3 = np.ones(n) * q10
        modes = np.ones(n)

        ucs = np.ones(n) * q10
        ycs = np.ones(n) * h10
        rcs = np.ones(n) * h10
        modess = np.ones(n)

        tc = (t1 + t2 + t3) / 3.0

        data.update(locals())

    elif example_id == 7:
        k = 1.0
        t = 1.0
        l = 0.5
        tsim = 150.0
        dt = 0.01

        ap = np.exp(-dt / t)
        bp = k * (1.0 - ap)
        lp = int(round(l / dt))

        n = _n_samples(tsim, dt)

        r1 = 0.3
        r2 = 0.5
        uc1 = np.zeros(n)
        uc2 = np.zeros(n)
        uc = np.zeros(n)
        yc1 = np.ones(n) * r1
        yc2 = np.ones(n) * r2
        rc1 = np.ones(n) * r1
        rc2 = np.ones(n) * r2

        v_mat = loadmat(base / "v.mat")
        v = np.asarray(v_mat["v"]).squeeze()
        if v.shape[0] < n:
            pad = np.zeros(n - v.shape[0])
            v = np.concatenate([v, pad])
        else:
            v = v[:n]

        data.update(locals())

    else:
        raise ValueError("Wrong Example ID")

    for key in list(data.keys()):
        if key.startswith("__"):
            data.pop(key, None)
    return data
