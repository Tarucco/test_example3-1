# Summary of Changes

Implemented `measure_stage.py` and `harness.py` for the Sling Lift 30 kg task.

### 1. What is measured:
- Document execution and valid 3D solid generation (gate).
- Assembly volume, bounding box ($dx \approx 450\text{ mm}$, $dz \approx 702\text{ mm}$), and center of mass chirality (reference $Y > 0$).
- Scissor mechanism topology and kinematic solver status via `assy.solve()` (catches disconnected links, gap-bridging joints, and overconstrained/dummy joints).
- Mechanism articulation: driving the slider joint and checking vertical travel ($\Delta z \approx 2.7\text{ mm}$).
- Sketch dimensional constraints count and key drawing dimensions (nominal values from `input.pdf`).
- Parametric driving verification: altering sketch constraint values and verifying solid volume updates.

### 2. What was intentionally ignored:
- Object/body names and labels (candidates are free to name features in any convention).
- Internal tree structure (monolithic assembly vs subassemblies/links).
- Specific modeling history or feature order (Pad vs Revolution).

### 3. Decision on the Mirrored Assembly:
- Scored **4.0 / 6.0**. Mirroring is treated as a defect: in technical drawings, handedness defines unilateral mounting orientations and clearances. Additionally, mirroring created planar constraint conflicts that restricted mechanical articulation ($\Delta z < 1.8\text{ mm}$).

### Test Results
- `solution/solution.FCStd`: **6.0 / 6.0** (PASS)
- `adversarial_half_finished_missing_features`: **5.0 / 6.0** (loses geometry)
- `adversarial_joint_limits_swapped`: **5.0 / 6.0** (loses geometry at delivered pose)
- `adversarial_assembly_mirrored_relative_to_ground_truth`: **4.0 / 6.0** (loses geometry & articulation)
- `adversarial_one_link_missing_joints_bridge_gap`: **3.0 / 6.0** (loses geometry, joint topology, articulation)
- `adversarial_dummy_joints_inflate_the_count`: **2.0 / 6.0** (solver failure, loses geometry, joints, articulation, drive)
