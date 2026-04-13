from __future__ import annotations

from dataclasses import dataclass
import copy


@dataclass
class PID:
    """Controlador PID en forma combinada con filtro de segundo orden."""

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0

    b: float = 1.0
    c: float = 1.0
    u0: float = 0.0

    Dt: float = 0.01
    Tf: float = 0.0
    Tt: float = 0.1

    minlim: float = float("-inf")
    maxlim: float = float("inf")
    dumin: float = float("-inf")
    dumax: float = float("inf")

    def __post_init__(self) -> None:
        self.xu = 0.0
        self.xus = 0.0
        self.xr = 0.0
        self.xdr = 0.0
        self.xy = 0.0
        self.xdy = 0.0
        self.xuff = 0.0
        self.xf1 = 0.0
        self.xf2 = 0.0
        self.is_initialized = False

    def copy(self) -> "PID":
        return copy.deepcopy(self)

    def init(self, r0: float = 0.0, y0: float = 0.0, u0: float | None = None, uff0: float = 0.0) -> None:
        if u0 is None:
            u0 = self.u0

        self.xr = float(r0)
        self.xy = float(y0)

        self.xdr = 0.0
        self.xdy = 0.0

        self.xu = float(u0)
        self.xus = float(u0)

        self.xuff = float(uff0)

        self.xf1 = float(y0)
        self.xf2 = float(y0)

        self.is_initialized = True

    def reset_integrator_like_states(self) -> None:
        self.xdr = 0.0
        self.xdy = 0.0
        self.xuff = 0.0

    def control(
        self,
        r: float,
        y: float,
        uff: float = 0.0,
        uman: float = 0.0,
        utrack: float = 0.0,
        mode: str = "AUTO",
    ) -> float:
        if not self.is_initialized:
            self.init(r, y, self.u0, uff)

        umin = max(self.minlim, self.xus + self.Dt * self.dumin)
        umax = min(self.maxlim, self.xus + self.Dt * self.dumax)

        dr = (r - self.xr) / self.Dt
        dy = (y - self.xy) / self.Dt

        if mode == "TRACK":
            self.xu = float(utrack)

        if mode == "MAN":
            u = float(uman)
        elif self.ki == 0.0:
            u = (
                self.u0
                + self.kp * (r - y)
                + self.kd * (self.c * dr - dy)
                + uff
            )
        else:
            d_r = r - self.xr
            d_y = y - self.xy
            d_dr = dr - self.xdr
            d_dy = dy - self.xdy

            du_p = self.kp * (self.b * d_r - d_y)
            du_i = self.ki * self.Dt * (r - y)
            du_d = self.kd * (self.c * d_dr - d_dy)
            du_ff = uff - self.xuff

            u = self.xu + du_p + du_i + du_d + du_ff

            us = max(min(u, umax), umin)
            u = u - self.Dt / self.Tt * (u - us)

        self.xu = float(u)

        u = max(min(u, umax), umin)
        self.xus = float(u)

        self.xr = float(r)
        self.xdr = float(dr)
        self.xy = float(y)
        self.xdy = float(dy)
        self.xuff = float(uff)

        return float(u)

    def filter(self, y: float) -> float:
        if not self.is_initialized:
            self.init(0.0, y, self.u0, 0.0)

        a = self.Dt / (self.Tf + 0.5 * self.Dt)

        self.xf1 = self.xf1 + a * (y - self.xf1)
        self.xf2 = self.xf2 + a * (self.xf1 - self.xf2)

        return float(self.xf2)
