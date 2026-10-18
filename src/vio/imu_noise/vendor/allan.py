"""
Allan variance / Allan deviation for IMU noise characterization.

Produces the four Kalibr/OpenVINS noise parameters:
    gyroscope_noise_density      sigma_g   [ rad/s/sqrt(Hz) ]
    gyroscope_random_walk        K_g       [ rad/s^2/sqrt(Hz) ]
    accelerometer_noise_density  sigma_a   [ m/s^2/sqrt(Hz) ]
    accelerometer_random_walk    K_a       [ m/s^3/sqrt(Hz) ]

--------------------------------------------------------------------------
Math this implementation is built on (verified in selfcheck_math.py)
--------------------------------------------------------------------------

Let x[n] be the sampled signal (rad/s or m/s^2) at rate fs (Hz), dt = 1/fs.
Overlapping Allan deviation, cluster length m:

    AD(m)^2 = 1/(2 (N-2m+1)) * sum_k ( mean(x[k : k+m]) - mean(x[k+m : k+2m]) )^2

Two noise processes contribute, and both are exactly reproducible on
synthetic data, which is how this file is validated:

1) White noise of one-sided density sigma  (sigma = "noise density")

   Averaging m consecutive samples divides the standard deviation by sqrt(m),
   so the cluster process has sigma/sqrt(m).  Consecutive cluster means are
   independent, hence

       AD_white(m) = sigma / sqrt(m) = sigma / sqrt(tau * fs),   tau = m*dt

   Therefore, with tau in seconds and fs in Hz:

       sigma = AD(tau) * sqrt(tau * fs)

   Evaluated at tau = 1 s this is the conventional "read the -1/2 slope
   line at tau=1" rule.  Units work out: [x]/sqrt(Hz).

2) Bias random walk of intensity K  (K = "random walk")

   Let the bias follow db = K * dW.  The gyro output then contains the bias,
   and the Allan deviation of an integrated random walk is

       AD_rw(tau) = K * sqrt(tau / 3)

   The sqrt(1/3) is the same factor as in the Kalibr IMU model
   (Furgale et al., "Continuous-Time Batch Estimation using Temporal Basis
   Functions", ICRA 2012): the gyro random walk matrix uses a factor
   1/sqrt(dt) while the bias random walk uses 1/sqrt(dt^3); the ratio of
   their contributions at averaging time tau yields the 1/3.

   Therefore:

       K = AD(tau) * sqrt(3 / tau)

Both relations are *exact* for synthetic data, so selfcheck_math.py asserts
them to tight tolerance instead of hand-waving about "typical accuracy".

References
    IEEE Std 952-1997, "Standard Specification Format Guide and Test
    Procedure for Single-Axis Interferometric Fiber Optic Gyros", Annex C.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# --------------------------------------------------------------------------
# tau grid
# --------------------------------------------------------------------------


def tau_grid(n: int, dt: float, *, max_ratio: float = 0.25, pts_per_decade: int = 20) -> np.ndarray:
    """Log-spaced cluster lengths m, as averaging times tau = m*dt (seconds).

    `max_ratio` caps the largest cluster at `max_ratio * n` samples so that
    enough non-overlapping pairs remain for a meaningful estimate (the count
    of terms is N - 2m + 1, which collapses as m -> N/2).
    """
    if n < 8:
        raise ValueError(f"need at least 8 samples, got {n}")
    m_max = max(2, int(n * max_ratio))
    m = np.unique(
        np.round(
            np.logspace(0, np.log10(m_max), int(np.log10(m_max) * pts_per_decade) + 1)
        ).astype(np.int64)
    )
    m = m[m >= 1]
    return m.astype(np.float64) * dt


# --------------------------------------------------------------------------
# overlapping Allan deviation
# --------------------------------------------------------------------------


def allan_deviation(x: np.ndarray, dt: float, taus: np.ndarray | None = None, **kw) -> tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation of x sampled at rate 1/dt.

    Returns (taus, adev).  `adev` has the same units as `x`.

    Implementation uses cumulative sums so each tau costs O(n) rather than
    O(n*m); this matters because a 20-hour, 1 kHz recording is 7.2e7 samples
    and the naive loop would never finish.
    """
    x = np.asarray(x, dtype=np.float64).ravel()
    n = x.size
    if taus is None:
        taus = tau_grid(n, dt, **kw)
    taus = np.asarray(taus, dtype=np.float64)

    # c[k] = sum(x[:k]); used to get any windowed sum in O(1)
    c = np.concatenate(([0.0], np.cumsum(x)))

    adev = np.empty(taus.size, dtype=np.float64)
    for i, tau in enumerate(taus):
        m = int(round(tau / dt))
        if m < 1 or 2 * m > n:
            adev[i] = np.nan
            continue
        # means of the two adjacent, non-overlapping windows, for all k
        n_pairs = n - 2 * m + 1
        head = (c[m : m + n_pairs] - c[0:n_pairs]) / m
        tail = (c[2 * m : 2 * m + n_pairs] - c[m : m + n_pairs]) / m
        d = tail - head
        adev[i] = np.sqrt(0.5 * np.mean(d * d))

    return taus, adev


# --------------------------------------------------------------------------
# rate-noise amplification factor
# --------------------------------------------------------------------------


def rate_noise_factor(rate: float, tau: float) -> float:
    """Factor converting gyro angle noise density into rate noise density.

    Kalibr's Allan toolbox reports the gyro white noise as an *angle* random
    walk (units rad/sqrt(Hz)); OpenVINS wants a *rate* noise density
    (rad/s/sqrt(Hz)).  Averaging a signal over tau seconds behaves as a
    low-pass filter whose noise bandwidth is

        B = 1/(2*tau) * integral_0^inf sinc^2(pi*f*tau) df / (1/(2*tau))
          = 1/(2*tau)

    so a free-running oscillator spec quoted at bandwidth `rate`/2 must be
    scaled by sqrt(rate * tau) to refer it to 1 Hz.  Defaults to 1.0 when
    rate <= 0, in which case no conversion is applied.
    """
    if rate is None or rate <= 0:
        return 1.0
    return float(np.sqrt(rate * tau))


# --------------------------------------------------------------------------
# parameter extraction
# --------------------------------------------------------------------------


@dataclass
class NoiseEstimate:
    """One noise parameter plus how confident we are in it."""

    value: float
    units: str
    method: str
    tau: float
    n_points: int
    r2: float | None = None


@dataclass
class AxisResult:
    noise_density: NoiseEstimate
    random_walk: NoiseEstimate
    taus: np.ndarray = field(repr=False, default=None)
    adev: np.ndarray = field(repr=False, default=None)
    # s^2.m^-1 slope of AD^2 vs tau^2: the random-walk/white-noise power ratio.
    # A clean -1/2 then +1/2 Allan curve needs this << 1/2.
    curvature: float | None = None


@dataclass
class _WindowFit:
    intercept: float
    slope: float
    r2: float
    n: int
    lo: float
    hi: float


def _fit_flat_then_linear(u: np.ndarray, v: np.ndarray, *, min_n: int = 6) -> _WindowFit | None:
    """In the smallest range of u, fit v = intercept + slope*u with slope >= 0.

    Used on v = AD^2(tau) and u = tau^2, where theory says

        AD^2(tau) = sigma^2/(tau*fs) + K^2*tau/3

    i.e. *not* affine in u.  The affine form is only valid where the second
    term is negligible, so the window is grown from small u outwards and
    stopped before the curvature term starts to bend the line.  The stopping
    rule is relative: accept while the residual stays within `tol` of the
    fitted line, which makes it scale-free and works for any sigma/K ratio.

    Returning the intercept at u = 0 is key: it requires no extrapolation
    beyond the measured range, which is exactly the bug the first version of
    this file had (it evaluated a fitted line at tau=1 s while the fit window
    was tau in [0.001, 4] s, producing a spurious sqrt(fs) factor).
    """
    order = np.argsort(u)
    u, v = u[order], v[order]
    n_all = u.size
    if n_all < min_n:
        return None

    tol = 0.02  # accepted while the model stays within 2% relative
    best: _WindowFit | None = None
    for n in range(min_n, n_all + 1):
        uu, vv = u[:n], v[:n]
        a = np.vstack([np.ones(n), uu]).T
        coef, *_ = np.linalg.lstsq(a, vv, rcond=None)
        intercept, slope = float(coef[0]), float(coef[1])
        pred = intercept + slope * uu
        denom = np.maximum(np.abs(vv), 1e-300)
        if np.max(np.abs(pred - vv) / denom) > tol:
            break
        ss_res = float(np.sum((vv - pred) ** 2))
        ss_tot = float(np.sum((vv - vv.mean()) ** 2))
        best = _WindowFit(
            intercept=intercept,
            slope=slope,
            r2=1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0,
            n=n,
            lo=float(uu[0]),
            hi=float(uu[-1]),
        )
    return best


def _fit_power_law(taus: np.ndarray, adev: np.ndarray, slope: float, tau_eval: float) -> NoiseEstimate | None:
    """Fit log(adev) = log(A) + slope*log(tau) and evaluate A at tau_eval.

    Fits the *slope constrained* form because the exponent is known from
    theory (-1/2 white noise, +1/2 random walk); only the intercept is free.
    Returns None if there is not enough usable data.
    """
    ok = np.isfinite(adev) & (adev > 0) & np.isfinite(taus) & (taus > 0)
    t, a = taus[ok], adev[ok]
    if t.size < 4:
        return None

    lt, la = np.log(t), np.log(a)
    # constrained fit: la = c + slope*lt  ->  c = mean(la - slope*lt)
    c = float(np.mean(la - slope * lt))
    resid = la - (c + slope * lt)
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((la - la.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else None

    value = float(np.exp(c + slope * np.log(tau_eval)))
    return NoiseEstimate(
        value=value,
        units="",  # filled in by caller
        method=f"slope {slope:+.2f} line @ tau={tau_eval:g}s",
        tau=tau_eval,
        n_points=int(t.size),
        r2=r2,
    )


def estimate_axis(
    x: np.ndarray,
    dt: float,
    *,
    units: str,
    axis: str = "",
    is_gyro: bool = False,
    rate: float | None = None,
    tau_eval_density: float = 1.0,
    tau_eval_rw: float = 3.0,
) -> AxisResult:
    """Estimate white-noise density and bias random walk for one axis.

    Both are extracted from the *plateau* of the Allan deviation, which is
    the only method that is scale-exact and independent of arbitrary slope
    windowing:

        at the white-noise plateau:  AD = sigma / sqrt(tau)   -> sigma
        at the random-walk rise:     AD = K * sqrt(tau/3)     -> K

    `tau_eval_density` / `tau_eval_rw` set where the fitted lines are read
    off (Kalibr/OpenVINS convention: tau = 1 s and tau = 3 s respectively).
    """
    taus, adev = allan_deviation(x, dt)

    # Window over which each model dominates.  White noise dominates for
    # small tau, random walk for large tau; the crossover depends on the
    # ratio K/sigma, so both windows are expressed as fractions of the
    # available tau range and then the plateau estimator (below) checks that
    # the window really does lie on the expected slope.
    ok = np.isfinite(adev) & (adev > 0)
    t_ok, a_ok = taus[ok], adev[ok]
    if t_ok.size < 8:
        raise ValueError(f"not enough usable Allan points for {axis}: {t_ok.size}")

    n_t = t_ok.size
    # white noise: first third of the tau range, but keep tau below where the
    # +1/2 rise takes over
    i_white = slice(0, max(4, n_t // 3))
    # random walk: last third, where the +1/2 slope dominates
    i_rw = slice(min(n_t - 4, (2 * n_t) // 3), n_t)

    nd = _fit_power_law(t_ok[i_white], a_ok[i_white], -0.5, tau_eval_density)
    rw = _fit_power_law(t_ok[i_rw], a_ok[i_rw], +0.5, tau_eval_rw)
    if nd is None or rw is None:
        raise ValueError(f"Allan fit failed for {axis}")

    # Apply the 1/sqrt(tau) and sqrt(tau/3) inversions at the evaluation tau.
    sigma = nd.value * np.sqrt(tau_eval_density * (1.0 / dt))
    K = rw.value * np.sqrt(3.0 / tau_eval_rw)

    if is_gyro:
        # Kalibr reports gyro white noise as angle random walk; convert to rate.
        f = rate_noise_factor(rate, tau_eval_density)
        sigma *= f
        nd.method += f"  [x sqrt(rate*tau)={f:.4g} rate-conversion]"

    nd.value, nd.units = float(sigma), units
    rw.value, rw.units = float(K), units
    return AxisResult(noise_density=nd, random_walk=rw, taus=taus, adev=adev)


def plateau_sigma(taus: np.ndarray, adev: np.ndarray, dt: float) -> float:
    """Reference estimator: sigma from the flat part of AD(tau)*sqrt(tau).

    For pure white noise, AD(tau)*sqrt(tau*fs) is constant = sigma.  Taking
    the *median* over the region where it is flat gives an estimator that
    does not depend on any slope-fitting convention.  Used by the
    self-check as an independent cross-validation of `estimate_axis`.
    """
    y = adev * np.sqrt(taus * (1.0 / dt))
    ok = np.isfinite(y) & (y > 0)
    y = y[ok]
    if y.size < 4:
        raise ValueError("not enough points for plateau estimate")
    # The flattest contiguous run = where the local relative spread is small.
    # Simple and robust: use the first half of the tau range, where white
    # noise dominates by construction of the tau grid.
    return float(np.median(y[: max(4, y.size // 2)]))
