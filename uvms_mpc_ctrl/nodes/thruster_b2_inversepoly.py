import json
import numpy as np



def _poly_eval_forward(a, u):
    # Polynom Auswertung mit Horner-Verfahren
    # polynom Koeffizienten sind in steigender Ordnung: a[0] + a[1]*u + a[2]*u^2 + ...
    u = np.asarray(u, dtype=float)
    y = np.zeros_like(u) + a[-1]
    for k in range(len(a)-2, -1, -1):
        y = y*u + a[k]
    return y


def _invert_forward_coeffs(a, f_star, side, u_edge, u_min=None, u_max=None):
    # Inverts the polynomial f(u) with coefficients a at f_star, to get the PWM command u.
    # u_edge: PWM at deadband edge for mininal non-zero force f_eps
    a = np.asarray(a).reshape(-1)
    deg = len(a)-1
    f_star = float(f_star)

    p = a.copy()
    p[0] = p[0] - f_star
    coeff_desc = p[::-1]

    if deg == 1:
        b1, b0 = a[1], a[0]
        if abs(b1) < 1e-12:
            u = u_edge
        else:
            u = (f_star - b0)/b1
    elif deg == 2:
        A, B, C = a[2], a[1], a[0]-f_star
        if abs(A) < 1e-12:
            if abs(B) < 1e-12:
                u = u_edge
            else:
                u = -C/B
        else:
            D = B*B - 4*A*C
            if D < 0:
                u = u_edge
            else:
                sqrtD = np.sqrt(D)
                u1 = (-B + sqrtD)/(2*A)
                u2 = (-B - sqrtD)/(2*A)
                cand = []
                if side == "fwd":
                    if u1 >= u_edge: cand.append(u1)
                    if u2 >= u_edge: cand.append(u2)
                    u = max(cand) if cand else u_edge
                else:
                    if u1 <= u_edge: cand.append(u1)
                    if u2 <= u_edge: cand.append(u2)
                    u = min(cand) if cand else u_edge
    else:
        roots = np.roots(coeff_desc)
        roots = roots[np.isreal(roots)].real
        if side == "fwd":
            roots = roots[roots >= u_edge - 1e-9]
            u = float(np.min(roots)) if roots.size else u_edge
        else:
            roots = roots[roots <= u_edge + 1e-9]
            u = float(np.max(roots)) if roots.size else u_edge

    # clip
    # if u_min is not None: u = max(u, u_min)
    # if u_max is not None: u = min(u, u_max)
    return float(u)


class ThrusterInversePoly:
    def __init__(self, voltages, f_eps, f_dz, deg, L, a_fwd, a_rev,
                 u0_map, uminus_map, uplus_map,
                 tau_volt=0.02, hysteresis=1.2, u_min=None, u_max=None,
                 f_max_all=None, f_min_all=None,
                 rmse_fwd=None, rmse_rev=None):
        self.voltages = np.array(sorted([float(v) for v in voltages]))
        self.f_eps = float(f_eps)
        self.f_dz = float(f_dz)
        self.deg = int(deg)
        self.L = float(L)
        self.a_fwd = {float(v): np.array(c) for v,c in a_fwd.items()}
        self.a_rev = {float(v): np.array(c) for v,c in a_rev.items()}
        self.u0_map = {float(k): float(v) for k, v in u0_map.items()}
        self.uminus_map = {float(k): float(v) for k, v in uminus_map.items()}
        self.uplus_map = {float(k): float(v) for k, v in uplus_map.items()}
        self.tau_volt = float(tau_volt)
        self.hysteresis = float(hysteresis)
        self.u_min = u_min
        self.u_max = u_max
        self.f_max_all = f_max_all
        self.f_min_all = f_min_all
        self.rmse_fwd = np.mean(list(rmse_fwd.values())) if rmse_fwd else None
        self.rmse_rev = np.mean(list(rmse_rev.values())) if rmse_rev else None
        self._Vf = None
        self._in_deadband = True
        self._last_u = None

    def get_force_limits(self, V):
        """
        Return (f_min, f_max, f_dz_minus, f_dz_plus) for a given battery voltage V.
        Simpler: evaluate the forward/reverse polynomials at the PWM endpoints
        (deadband edge and u_min/u_max) instead of sampling.
        """
        # require PWM endpoints to be defined
        if self.u_max is None or self.u_min is None:
            raise ValueError("Model must define both u_min and u_max before computing force limits.")
        V = float(V)
        # blended coeffs for this voltage
        a_fwd = self._blend_forward_coeffs(V, side="fwd")
        a_rev = self._blend_forward_coeffs(V, side="rev")

        # evaluate polynomials at endpoints
        f_max = float(_poly_eval_forward(a_fwd, self.u_max))
        f_min = float(_poly_eval_forward(a_rev, self.u_min))

        return f_min, f_max, -float(self.f_dz), float(self.f_dz)
    
    def get_pwm_limits(self, V):
        """
        Return (u_min, u_max, u_dz_minus, u_dz_plus) for a given battery voltage V.
        u_dz_plus is the PWM corresponding to +f_dz, u_dz_minus corresponds to -f_dz.
        """
        # require PWM endpoints to be defined
        if self.u_max is None or self.u_min is None:
            raise ValueError("Model must define both u_min and u_max before computing PWM limits.")
        V = float(V)

        # deadband edge PWMs (interpolated)
        u_dz_plus = float(self._interp_scalar(V, self.uplus_map))
        u_dz_minus = float(self._interp_scalar(V, self.uminus_map))

        return float(self.u_min), float(self.u_max), u_dz_minus, u_dz_plus

    def _interp_scalar(self, V, value_map):
        # Die Funktion liefert den PWM Wert an der Grenze zur Dead Zone für eine gegebene Spannung V
        # also den PWM Wert, um die kleinstmögliche Kraft zu erzeugen, bevor die Dead Zone betreten wird.
        # Liegt V außerhalb des Spannungsbereichs, wird der PWM -Grenz-Wert der
        # nächstgelegenen Spannung zurückgegeben
        # Liegt V innerhalb des Spannungsbereichs, wird der PWM-Grenz-Wert linear interpoliert
        # zwischen den Werten der beiden nächstgelegenen Spannungen
        V = float(V); arrV = self.voltages
        if V <= arrV[0]: return value_map[arrV[0]]
        if V >= arrV[-1]: return value_map[arrV[-1]]
        j = np.searchsorted(arrV, V)
        V0, V1 = arrV[j-1], arrV[j]
        t = (V - V0)/(V1 - V0 + 1e-12)
        return (1-t)*value_map[V0] + t*value_map[V1]

    def _blend_forward_coeffs(self, V, side="fwd"):
        # Die Funktion liefert Koeffizienten der gefitteten Polynome für eine gegebene Spannung V, 
        # entweder für die Vorwärts- oder Rückwärtscharakteristik
        # liegt V außerhalb des Spannungsbereichs, werden die Koeffizienten der 
        # nächstgelegenen Spannung zurückgegeben
        # liegt V innerhalb des Spannungsbereichs, werden die Koeffizienten linear interpoliert
        # zwischen den beiden nächstgelegenen Spannungen
        arrV = self.voltages
        V = float(V)
        if V <= arrV[0]:
            return self.a_fwd[arrV[0]] if side=="fwd" else self.a_rev[arrV[0]]
        if V >= arrV[-1]:
            return self.a_fwd[arrV[-1]] if side=="fwd" else self.a_rev[arrV[-1]]
        j = np.searchsorted(arrV, V)
        V0, V1 = arrV[j-1], arrV[j]
        t = (V - V0)/(V1 - V0 + 1e-12)
        c0 = self.a_fwd[V0] if side=="fwd" else self.a_rev[V0]
        c1 = self.a_fwd[V1] if side=="fwd" else self.a_rev[V1]
        return (1-t)*c0 + t*c1

    def command(self, f_des, V_meas, dt, rate_limit=None):
        if self._Vf is None:
            self._Vf = float(V_meas)
        alpha = float(np.exp(-dt / max(self.tau_volt, 1e-6)))
        self._Vf = alpha*self._Vf + (1-alpha)*float(V_meas)
        V = float(self._Vf)

        f = float(f_des); af = abs(f)
        if self._in_deadband:
            if af > self.hysteresis*self.f_dz:
                self._in_deadband = False
        else:
            if af < self.f_dz/self.hysteresis:
                self._in_deadband = True

        if self._in_deadband:
            u = self._interp_scalar(V, self.u0_map)
        else:
            if f > 0:
                a = self._blend_forward_coeffs(V, side="fwd")
                u_edge = self._interp_scalar(V, self.uplus_map)
                u = _invert_forward_coeffs(a, f_star=f, side="fwd",
                                           u_edge=u_edge, u_min=self.u_min, u_max=self.u_max)
                u = max(u, u_edge)
            else:
                a = self._blend_forward_coeffs(V, side="rev")
                u_edge = self._interp_scalar(V, self.uminus_map)
                u = _invert_forward_coeffs(a, f_star=f, side="rev",
                                           u_edge=u_edge, u_min=self.u_min, u_max=self.u_max)
                u = min(u, u_edge)

        if rate_limit is not None and rate_limit > 0 and self._last_u is not None:
            step = rate_limit * dt
            u = float(np.clip(u, self._last_u - step, self._last_u + step))

        self._last_u = float(u)
        return float(u)
    
    def command_simple(self, f_des, V_meas):
        # return the PWM command [1100, 1900] for the desired thrust f_des at voltage V_meas
        f = float(f_des); af = abs(f)
        # when the desired thrust is lower than the deadband, return the neutral PWM = 1500 (=u0_map)
        if af < self.f_dz:
            u = self._interp_scalar(V_meas, self.u0_map)
        else:
            if f > 0:
                a = self._blend_forward_coeffs(V_meas, side="fwd") # return the coefficients for the polynomial at voltage V_meas, for the function thrust = a0 + a1*u + a2*u^2 + ...
                u_edge = self._interp_scalar(V_meas, self.uplus_map) # return the PWM value at the edge of the deadband for positive thrust at voltage V_meas, uplus_map are the deadzone borders for all voltages in the test data
                u = _invert_forward_coeffs(a, f_star=f, side="fwd",
                                           u_edge=u_edge, u_min=self.u_min, u_max=self.u_max)
                u = max(u, u_edge)
            else:
                a = self._blend_forward_coeffs(V_meas, side="rev")
                u_edge = self._interp_scalar(V_meas, self.uminus_map)
                u = _invert_forward_coeffs(a, f_star=f, side="rev",
                                           u_edge=u_edge, u_min=self.u_min, u_max=self.u_max)
                u = min(u, u_edge)
        return u

    def pwm_to_force(self, u, V_meas):
        """
        Evaluate fitted forward polynomials to map PWM -> thrust at given voltage.
        - u may be scalar or array-like (PWM in [u_min,u_max])
        - V_meas: battery voltage used for blending coefficients
        - returns scalar or numpy array of forces (same shape as input)
        Deadband between interpolated u_minus and u_plus yields zero force.
        """
        V = float(V_meas)
        u_arr = np.asarray(u, dtype=float)

        # clip to saturation if defined
        if self.u_min is not None or self.u_max is not None:
            lo = -np.inf if self.u_min is None else float(self.u_min)
            hi =  np.inf if self.u_max is None else float(self.u_max)
            u_arr = np.clip(u_arr, lo, hi)

        u0 = float(self._interp_scalar(V, self.u0_map))
        u_minus = float(self._interp_scalar(V, self.uminus_map))
        u_plus  = float(self._interp_scalar(V, self.uplus_map))

        # prepare output
        out = np.zeros_like(u_arr, dtype=float)

        # forward region (above deadband)
        mask_fwd = (u_arr > u_plus)
        if np.any(mask_fwd):
            a_fwd = self._blend_forward_coeffs(V, side="fwd")
            out[mask_fwd] = _poly_eval_forward(a_fwd, u_arr[mask_fwd])

        # reverse region (below deadband)
        mask_rev = (u_arr < u_minus)
        if np.any(mask_rev):
            a_rev = self._blend_forward_coeffs(V, side="rev")
            out[mask_rev] = _poly_eval_forward(a_rev, u_arr[mask_rev])

        # deadband (u in [u_minus, u_plus]) remains zero
        # preserve scalar return if input was scalar
        if np.isscalar(u):
            return float(out)
        return out

    def clip_pwm_saturation(self, u_cmd):
        u_cmd = np.asarray(u_cmd).astype(float)
        if self.u_min is not None:
            u_cmd = np.maximum(u_cmd, self.u_min)
        if self.u_max is not None:
            u_cmd = np.minimum(u_cmd, self.u_max)
        return u_cmd
    
    def map_mpc_pwm_to_force(self, u_cmd, V_meas):
        # The MPC PWM ouput must be the normalized command in [-1,1]
        desired_thrust = self.L * V_meas * u_cmd
        return desired_thrust

    def normalize_pwm(self, pwm):
        return 2 * (pwm - self.u_min) / (self.u_max - self.u_min) - 1

    def denormalize_pwm(self, pwm_norm):
        return ((pwm_norm + 1) * (self.u_max - self.u_min) / 2) + self.u_min
    
    # def _interval_intersection(self, a, b):
    #     lo = max(a[0], b[0])
    #     hi = min(a[1], b[1])
    #     return (lo, hi) if lo <= hi else None

    # def _union_of_open_intervals(self, intervals):
    #     if not intervals:
    #         return []
    #     segs = sorted(intervals, key=lambda x: (x[0], x[1]))
    #     merged = []
    #     cur_l, cur_r = segs[0]
    #     for l, r in segs[1:]:
    #         if l <= cur_r:
    #             cur_r = max(cur_r, r)
    #         else:
    #             merged.append((cur_l, cur_r))
    #             cur_l, cur_r = l, r
    #     merged.append((cur_l, cur_r))
    #     return merged

    # def _complement_of_open_union(self, merged_open, box=(-np.inf, np.inf)):
    #     L, U = box
    #     if L > U:
    #         return []
    #     if not merged_open:
    #         return [(L, U)]
    #     out = []
    #     cur = L
    #     for (l, r) in merged_open:
    #         if r <= L or l >= U:
    #             continue
    #         l_clip = max(l, L)
    #         r_clip = min(r, U)
    #         if cur < l_clip:
    #             out.append((cur, l_clip))
    #         cur = max(cur, r_clip)
    #     if cur <= U:
    #         out.append((cur, U))
    #     return [(a, b) for a, b in out if b >= a]

    def _project_scalar_onto_union(self, x, intervals):
        # Project scalar x onto union of CLOSED intervals. thus find optimal x
        # as cost function quadratic, the optimal colustion is the closest point to
        # the unconstraint solution x, that is in one of the intervals.
        # only for 1D, scalar x
        if not intervals:
            return None, np.inf
        for (L, U) in intervals:
            if L <= x <= U:
                return x, 0.0
        best_x, best_d2 = None, np.inf
        for (L, U) in intervals:
            cand = U if abs(U - x) < abs(L - x) else L
            d2 = (cand - x) ** 2
            if d2 < best_d2:
                best_x, best_d2 = cand, d2
        return best_x, best_d2

    def _intersect_box(self, intervals, box):
        """
        Intersect a union of CLOSED intervals with one CLOSED box [L, U].
        intervals: list of (L, U), closed.
        box: (Lbox, Ubox), closed.
        """
        if not intervals:
            return []
        Lb, Ub = box
        out = []
        for L, U in intervals:
            L2 = max(L, Lb)
            U2 = min(U, Ub)
            if L2 <= U2:
                out.append((L2, U2))
        return out

    def _subtract_open(self, intervals, open_seg):
        """
        Subtract one OPEN interval (l, r) from a union of CLOSED intervals.
        Returns union of CLOSED intervals.
        """
        l, r = open_seg
        if not intervals or l >= r:
            return intervals
        out = []
        for L, U in intervals:
            # disjoint → unchanged
            if U <= l or r <= L:
                out.append((L, U))
                continue
            # overlap → keep closed remainders (left includes l, right includes r)
            if L <= l:
                out.append((L, min(U, l)))   # left remainder (closed)
            if r <= U:
                out.append((max(L, r), U))   # right remainder (closed)
        # drop degenerates
        return [(a, b) for (a, b) in out if a <= b]

    def _intersect_outside_deadzone(self, intervals, a, b):
        """
        Intersect existing CLOSED intervals with the set (-inf, a] ∪ [b, +inf),
        implemented as subtracting the OPEN gap (a, b).
        """
        return self._subtract_open(intervals, (a, b))
    
    def _build_feasible_w1(self, f_alt, fmin, fmax, fdz_min, fdz_max):
        c1 = np.array([+0.5, +0.5, -0.5, -0.5])
        # start with all real line as one closed interval
        intervals = [(-np.inf, +np.inf)]
        # 1) intersect saturation (closed box per thruster)
        for i in range(4):
            fi, ci = f_alt[i], c1[i]
            lo = (fmin[i] - fi) / ci
            hi = (fmax[i] - fi) / ci
            if lo > hi: lo, hi = hi, lo
            intervals = self._intersect_box(intervals, (lo, hi))
            if not intervals:
                return []
        # 2) subtract each deadzone open gap
        for i in range(4):
            fi, ci = f_alt[i], c1[i]
            a = (fdz_min[i] - fi) / ci
            b = (fdz_max[i] - fi) / ci
            if a > b: a, b = b, a
            intervals = self._intersect_outside_deadzone(intervals, a, b)
            if not intervals:
                return []
        return intervals

    def _build_feasible_w2(self, f_alt, fmin, fmax, fdz_min, fdz_max):
        c2 = 0.5
        intervals = [(-np.inf, +np.inf)]
        # 1) saturation
        for i in range(4, 8):
            fi = f_alt[i]
            lo = (fmin[i] - fi) / c2
            hi = (fmax[i] - fi) / c2
            if lo > hi: lo, hi = hi, lo
            intervals = self._intersect_box(intervals, (lo, hi))
            if not intervals:
                return []
        # 2) deadzone gaps
        for i in range(4, 8):
            fi = f_alt[i]
            a = (fdz_min[i] - fi) / c2
            b = (fdz_max[i] - fi) / c2
            if a > b: a, b = b, a
            intervals = self._intersect_outside_deadzone(intervals, a, b)
            if not intervals:
                return []
        return intervals
    
    def nullspace_adaption_fast(self, f_alt, fdz_min, fdz_max, fmin=None, fmax=None, objective="Nw"):
        inflation = 0.5  # keep your current margin if you want

        f_alt   = np.asarray(f_alt, float).reshape(-1)
        n = f_alt.size
        assert n == 8, "Expect 8 thrusters."

        # inflate/deflate as you already do
        fdz_min = np.asarray(fdz_min - inflation, float).reshape(-1)
        fdz_max = np.asarray(fdz_max + inflation, float).reshape(-1)
        if fmin is None: fmin = -np.inf * np.ones(n)
        if fmax is None: fmax =  np.inf * np.ones(n)
        fmin = np.asarray(fmin + inflation, float).reshape(-1)
        fmax = np.asarray(fmax - inflation, float).reshape(-1)

        # --- build feasible unions robustly
        W1 = self._build_feasible_w1(f_alt, fmin, fmax, fdz_min, fdz_max)
        W2 = self._build_feasible_w2(f_alt, fmin, fmax, fdz_min, fdz_max)
        if not W1 or not W2:
            return dict(status="infeasible", w=None, f=None, w1_intervals=W1, w2_intervals=W2)

        # --- unconstrained minimizers (same as before)
        c1 = np.array([+0.5, +0.5, -0.5, -0.5]); c2 = 0.5
        if objective == "Nw":
            w1_free, w2_free = 0.0, 0.0
        elif objective == "f":
            w1_free = -np.sum(c1 * f_alt[:4])
            w2_free = -0.5 * np.sum(f_alt[4:8])
        else:
            raise ValueError("objective must be 'Nw' or 'f'.")

        # --- project (your projector is fine)
        w1, _ = self._project_scalar_onto_union(w1_free, W1)
        w2, _ = self._project_scalar_onto_union(w2_free, W2)

        # --- build f and assert
        f = np.empty(8, float)
        f[:4] = f_alt[:4] + c1 * w1
        f[4:] = f_alt[4:] + c2 * w2

        eps = 1e-12
        ok = np.all((f <= fdz_min + eps) | (f >= fdz_max - eps))
        if not ok:
            print("Deadzone violation detected after adaptation. f =", f)
            raise AssertionError("Deadzone violation detected after adaptation")

        return dict(status="optimal", w=np.array([w1, w2]),
                    f=f, w1_intervals=W1, w2_intervals=W2)

    def mpc_thruster_command_adaption(self, u_mpc, V_batt=16.0): #f_alt, fdz_min, fdz_max, fmin=None, fmax=None, objective="Nw", V_batt=16.0):
        # receive the MPC PWM commands (normalized in [-1,1])
        u_mpc   = np.asarray(u_mpc, float).reshape(-1)
        n_thruster = u_mpc.size
        assert n_thruster == 8, "Expect 8 thrusters."

        # f: minimize force norm
        # Nw: minimize norm of nullspace component
        objective = "Nw"
        
        f_mpc = self.map_mpc_pwm_to_force(u_mpc, V_meas=V_batt)  # V_meas is a placeholder here
        f_min, f_max, f_dz_minus, f_dz_plus = self.get_force_limits(V_batt)

        f_min = np.full(n_thruster, f_min, dtype=float)
        f_max = np.full(n_thruster, f_max, dtype=float)
        f_dz_minus = np.full(n_thruster, f_dz_minus, dtype=float)
        f_dz_plus = np.full(n_thruster, f_dz_plus, dtype=float)


        result = self.nullspace_adaption_fast(f_mpc, f_dz_minus, f_dz_plus, fmin=f_min, fmax=f_max, objective=objective)
        if result['status'] != "optimal":
            result = self.nullspace_adaption_fast(f_mpc, f_dz_minus, f_dz_plus, fmin=None, fmax=None, objective=objective)
            if result['status'] != "optimal":
                raise RuntimeError("Thruster command adaption failed: no solution found.")
        u_adapted = np.array([self.command_simple(f, V_batt) for f in result['f']])
        # u_adapted = np.array([self.command_simple(f, V_batt) for f in f_mpc])

        u_adapted = self.clip_pwm_saturation(u_adapted)
        return u_adapted


    def thruster_adaption(self, u_MPC, V_batt):
        u_MPC = np.asarray(u_MPC, dtype=float).reshape(1, -1)
        f_alt = np.array(self.map_mpc_pwm_to_force(u_MPC, V_batt)).reshape(-1)

        f_min, f_max, f_dz_minus, f_dz_plus = self.get_force_limits(V_batt)

        n_thruster = int(u_MPC.shape[1])
        f_min = np.full(n_thruster, f_min, dtype=float)
        f_max = np.full(n_thruster, f_max, dtype=float)
        f_dz_minus = np.full(n_thruster, f_dz_minus, dtype=float)
        f_dz_plus = np.full(n_thruster, f_dz_plus, dtype=float)

        best = self.nullspace_adaption_fast(
            f_alt, f_dz_minus, f_dz_plus, fmin=f_min, fmax=f_max, objective="f"
        )
        if best["status"] == "infeasible":
            # print("No feasible pattern (deadzone + saturation). Solving without saturation")
            best = self.nullspace_adaption_fast(f_alt, f_dz_minus, f_dz_plus, fmin=None, fmax=None, objective="f")
            if best["status"] == "infeasible":
                raise RuntimeError("No feasible pattern even without saturation.")
        f_new = best.get("f")
        u_new = np.array([self.command_simple(f, V_batt) for f in f_new])
        u_new = self.clip_pwm_saturation(u_new)
        return u_new
    
    def save(self, path):
        arrV = self.voltages
        def _pack(m): return json.dumps({str(v): m[v].tolist() for v in arrV})
        np.savez(path,
                 voltages=arrV, f_eps=self.f_eps, f_dz=self.f_dz, deg=self.deg, L=self.L,
                 a_fwd=_pack(self.a_fwd), a_rev=_pack(self.a_rev),
                 u0_vals=np.array([self.u0_map[v] for v in arrV]),
                 uminus_vals=np.array([self.uminus_map[v] for v in arrV]),
                 uplus_vals=np.array([self.uplus_map[v] for v in arrV]),
                 tau_volt=self.tau_volt, hysteresis=self.hysteresis,
                 u_min=self.u_min if self.u_min is not None else np.array([np.nan]),
                 u_max=self.u_max if self.u_max is not None else np.array([np.nan]))

    @classmethod
    def load(cls, path):
        d = np.load(path, allow_pickle=True)
        arrV = d['voltages'].astype(float)
        deg = int(d['deg'])
        def _unpack(key):
            return {float(k): np.array(v) for k,v in json.loads(d[key].item()).items()}
        a_fwd = _unpack('a_fwd'); a_rev = _unpack('a_rev')
        u0_map = {float(v): float(u) for v,u in zip(arrV, d['u0_vals'].astype(float))}
        uminus_map = {float(v): float(u) for v,u in zip(arrV, d['uminus_vals'].astype(float))}
        uplus_map  = {float(v): float(u) for v,u in zip(arrV, d['uplus_vals'].astype(float))}
        u_min = None if np.isnan(d["u_min"]).any() else float(d["u_min"])
        u_max = None if np.isnan(d["u_max"]).any() else float(d["u_max"])
        return cls(voltages=arrV, f_eps=float(d['f_eps']), f_dz=float(d['f_dz']), deg=deg, L = float(d['L']),
                   a_fwd=a_fwd, a_rev=a_rev,
                   u0_map=u0_map, uminus_map=uminus_map, uplus_map=uplus_map,
                   tau_volt=float(d['tau_volt']), hysteresis=float(d['hysteresis']),
                   u_min=u_min, u_max=u_max)
