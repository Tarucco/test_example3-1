#!/usr/bin/env python3
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _repo_root(start):
    d = start
    for _ in range(8):
        if (d / "common" / "__init__.py").is_file():
            return d
        if d.parent == d:
            break
        d = d.parent
    return start


REPO = _repo_root(HERE)
sys.path[:0] = [str(REPO), "/opt"]

from common.harness_base import (  # noqa: E402
    Harness,
    clamp01,
    score_error,
    score_ratio,
)

ALL_CRITERIA = {
    "executes and builds geometry": 0,
    "geometry matches ground truth": 1,
    "pantograph joints mated": 1,
    "mechanism articulates": 1,
    "carries dimensional constraints": 1,
    "dimensions drive the geometry": 1,
    "signature dimensions present": 1,
}

REF_VOL = 966576.8
REF_DX = 449.99
REF_DZ = 701.7


class SlingLiftHarness(Harness):
    MUST_PASS = ("executes and builds geometry",)
    WEIGHTS = ALL_CRITERIA
    BUILD_TIMEOUT_S = 600

    def build_state(self, candidate_path):
        try:
            measurement, error = self.run_freecad_stage(candidate_path)
        except SystemExit as exc:
            measurement, error = None, str(exc)
        except Exception as exc:  # noqa: BLE001
            measurement, error = None, f"{type(exc).__name__}: {exc}"
        return {
            "candidate": candidate_path,
            "measurement": measurement or {},
            "error": error,
        }

    def checks(self, state):
        err = state.get("error")
        m = state.get("measurement", {})
        out = {}

        # 1. gate
        ok_gate = (
            err is None
            and m.get("ok", False)
            and m.get("solids_count", 0) > 0
            and m.get("assembly_volume", 0) > 0
        )
        desc_gate = (
            "Document recomputed and produced solid geometry"
            if ok_gate
            else f"Measurement stage failed: {err or m.get('error', 'no solids')}"
        )
        out["executes and builds geometry"] = (
            "executes and builds geometry",
            1.0 if ok_gate else 0.0,
            desc_gate,
        )

        # 2. geometry
        vol = m.get("assembly_volume", 0.0)
        bounds = m.get("assembly_bounds") or {}
        dx = bounds.get("dx", 0.0)
        dz = bounds.get("dz", 0.0)
        com = m.get("center_of_mass") or [0.0, 0.0, 0.0]
        com_y = com[1]

        vol_err = abs(vol - REF_VOL) / REF_VOL
        score_vol = score_error(vol_err, 0.01, 0.06)

        dx_err = abs(dx - REF_DX) / REF_DX
        dz_err = abs(dz - REF_DZ) / REF_DZ
        score_dx = score_error(dx_err, 0.02, 0.15)
        score_dz = score_error(dz_err, 0.02, 0.05)
        score_bb = min(score_dx, score_dz)

        mirror_ok = 1.0 if com_y >= 0.0 else 0.0

        score_geom = clamp01(score_vol * score_bb * mirror_ok)
        desc_geom = (
            f"Volume {vol:.0f} mm^3 (err {vol_err*100:.1f}%), "
            f"bounds [{dx:.1f} x {dz:.1f} mm], CoM Y={com_y:.2f} mm"
        )
        out["geometry matches ground truth"] = (
            "geometry matches ground truth",
            score_geom,
            desc_geom,
        )

        # 3. joints
        jt = m.get("joint_types", {})
        revolute_cnt = jt.get("Revolute", 0)
        slider_cnt = jt.get("Slider", 0)
        bridge_gap = m.get("bridge_gap_detected", False)
        solve_code = m.get("initial_solve_code", -99)
        tot_joints = m.get("joint_count", 0)

        if bridge_gap:
            score_joints = 0.0
            desc_joints = "Broken topology: joint bridges gap directly to shackle, skipping link"
        elif solve_code != 0:
            score_joints = 0.0
            desc_joints = f"Joints fail kinematic assembly solver (exit code {solve_code})"
        elif tot_joints > 80 and revolute_cnt > 16:
            score_joints = 0.0
            desc_joints = f"Inflated dummy joints ({tot_joints} joints) with conflicting constraints"
        elif revolute_cnt >= 8 and slider_cnt >= 1:
            score_joints = 1.0
            desc_joints = f"Pantograph scissor joints mated: {revolute_cnt} revolute, {slider_cnt} slider"
        else:
            score_joints = score_ratio(revolute_cnt, 8, 2)
            desc_joints = f"Incomplete pantograph joints: {revolute_cnt} revolute, {slider_cnt} slider"
        out["pantograph joints mated"] = (
            "pantograph joints mated",
            clamp01(score_joints),
            desc_joints,
        )

        # 4. articulation
        articulates = m.get("articulates", False)
        delta_z = m.get("delta_z", 0.0)

        if bridge_gap or solve_code != 0:
            score_art = 0.0
            desc_art = "Mechanism cannot articulate due to broken topology or solver failure"
        elif articulates and delta_z >= 2.0:
            score_art = 1.0
            desc_art = f"Mechanism articulates under joint drive (height displacement dz={delta_z:.1f} mm)"
        elif delta_z > 1.8:
            score_art = score_ratio(delta_z, 2.0, 1.8)
            desc_art = f"Mechanism articulation restricted (dz={delta_z:.1f} mm)"
        else:
            score_art = 0.0
            desc_art = f"Mechanism is rigid or overconstrained (dz={delta_z:.1f} mm)"
        out["mechanism articulates"] = (
            "mechanism articulates",
            clamp01(score_art),
            desc_art,
        )

        # 5. constraints
        dim_cs = m.get("dimensional_constraints", 0)
        score_cs = score_ratio(dim_cs, full=50, zero=5)
        desc_cs = f"Model carries {dim_cs} dimensional constraints across {m.get('sketches_count', 0)} sketches"
        out["carries dimensional constraints"] = (
            "carries dimensional constraints",
            clamp01(score_cs),
            desc_cs,
        )

        # 6. parametric drive
        drives = m.get("drives_geometry", False)
        d_vol = m.get("drive_delta_volume", 0.0)

        if solve_code != 0:
            score_drive = 0.0
            desc_drive = "Dimensions fail to drive assembly: kinematic solver fails during update"
        elif drives and d_vol > 10.0:
            score_drive = 1.0
            desc_drive = f"Parametric dimensions actively drive solid geometry (delta volume = {d_vol:.1f} mm^3)"
        elif d_vol > 0.0:
            score_drive = score_ratio(d_vol, 10.0, 0.0)
            desc_drive = f"Weak parametric response (delta volume = {d_vol:.1f} mm^3)"
        else:
            score_drive = 0.0
            desc_drive = "Dead geometry: sketch constraints do not rebuild solids"
        out["dimensions drive the geometry"] = (
            "dimensions drive the geometry",
            clamp01(score_drive),
            desc_drive,
        )

        # 7. signature dimensions
        sig_found = m.get("signature_dimensions_count", 0)
        sig_targets = m.get("signature_targets_count", 19)
        score_sig = score_ratio(sig_found, full=10, zero=3)
        desc_sig = f"Found {sig_found}/{sig_targets} signature dimensions from technical drawing"
        out["signature dimensions present"] = (
            "signature dimensions present",
            clamp01(score_sig),
            desc_sig,
        )

        return out


main = SlingLiftHarness.as_main()

if __name__ == "__main__":
    SlingLiftHarness.cli()
