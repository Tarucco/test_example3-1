import os
import sys

sys.path.insert(0, os.environ["FC_STAGE_COMMON"])

from common.freecad_stage import Stage  # noqa: E402


class SlingLiftStage(Stage):
    def measure(self, doc):
        result = {}

        # assembly object
        assy = getattr(doc, "Assembly", None)
        if not assy or assy.TypeId != "Assembly::AssemblyObject":
            for obj in doc.Objects:
                if obj.TypeId == "Assembly::AssemblyObject":
                    assy = obj
                    break

        result["has_assembly"] = assy is not None

        # shape and bounding box
        assy_shape = getattr(assy, "Shape", None) if assy else None
        if assy_shape and hasattr(assy_shape, "Solids") and len(assy_shape.Solids) > 0:
            vol = float(assy_shape.Volume)
            bb = assy_shape.BoundBox
            solids = assy_shape.Solids
            n_solids = len(solids)
            cx = sum(s.CenterOfMass.x * s.Volume for s in solids) / (vol if vol > 0 else 1.0)
            cy = sum(s.CenterOfMass.y * s.Volume for s in solids) / (vol if vol > 0 else 1.0)
            cz = sum(s.CenterOfMass.z * s.Volume for s in solids) / (vol if vol > 0 else 1.0)

            result["assembly_volume"] = vol
            result["assembly_bounds"] = {
                "xmin": float(bb.XMin), "xmax": float(bb.XMax),
                "ymin": float(bb.YMin), "ymax": float(bb.YMax),
                "zmin": float(bb.ZMin), "zmax": float(bb.ZMax),
                "dx": float(bb.XLength),
                "dy": float(bb.YLength),
                "dz": float(bb.ZLength),
            }
            result["solids_count"] = n_solids
            result["center_of_mass"] = [float(cx), float(cy), float(cz)]
        else:
            all_solids = [o for o in doc.Objects if hasattr(o, "Shape") and hasattr(o.Shape, "Solids") and len(o.Shape.Solids) > 0]
            tot_vol = sum(float(o.Shape.Volume) for o in all_solids)
            result["assembly_volume"] = tot_vol
            result["assembly_bounds"] = None
            result["solids_count"] = len(all_solids)
            result["center_of_mass"] = [0.0, 0.0, 0.0]

        # body dims
        bodies = [o for o in doc.Objects if o.TypeId == "PartDesign::Body"]
        body_dims_list = []
        body_vols_list = []
        for b in bodies:
            if hasattr(b, "Shape") and b.Shape:
                bb = b.Shape.BoundBox
                dims = sorted([round(float(bb.XLength), 1), round(float(bb.YLength), 1), round(float(bb.ZLength), 1)])
                body_dims_list.append(dims)
                body_vols_list.append(round(float(b.Shape.Volume), 1))
        result["body_count"] = len(bodies)
        result["body_dims"] = body_dims_list
        result["body_volumes"] = body_vols_list

        # joints
        all_joints = [o for o in doc.Objects if hasattr(o, "JointType")]
        joint_types = {}
        joints_detail = []
        bridge_gap_detected = False

        for j in all_joints:
            jt = str(getattr(j, "JointType", "Unknown"))
            joint_types[jt] = joint_types.get(jt, 0) + 1

            r1 = getattr(j, "Reference1", None)
            r2 = getattr(j, "Reference2", None)
            lbl1 = str(r1[0].Label) if r1 and len(r1) >= 1 and r1[0] else ""
            lbl2 = str(r2[0].Label) if r2 and len(r2) >= 1 and r2[0] else ""

            if jt in ("Slider", "Revolute"):
                if ("Shackle" in lbl1 or "Shackle" in lbl2) and ("Washer" in lbl1 or "Washer" in lbl2 or "Bar_10x30" in lbl1 or "Bar_10x30" in lbl2):
                    if jt == "Slider":
                        bridge_gap_detected = True

            joints_detail.append({
                "name": str(j.Name),
                "type": jt,
                "part1": lbl1,
                "part2": lbl2,
            })

        result["joint_count"] = len(all_joints)
        result["joint_types"] = joint_types
        result["bridge_gap_detected"] = bridge_gap_detected

        if assy:
            links = [o for o in assy.Group if o.TypeId in ("App::Link", "Assembly::AssemblyLink")]
            connected = sum(1 for p in links if hasattr(assy, "isPartConnected") and assy.isPartConnected(p))
            grounded = sum(1 for p in links if hasattr(assy, "isPartGrounded") and assy.isPartGrounded(p))
            result["links_total"] = len(links)
            result["links_connected"] = connected
            result["links_grounded"] = grounded
        else:
            result["links_total"] = 0
            result["links_connected"] = 0
            result["links_grounded"] = 0

        # articulation
        solve_code = -99
        articulates = False
        delta_z = 0.0

        if assy:
            try:
                solve_code = int(assy.solve())
            except Exception:
                solve_code = -1

            result["initial_solve_code"] = solve_code

            if solve_code == 0:
                sliders = [j for j in all_joints if j.JointType == "Slider"]
                revolutes = [j for j in all_joints if j.JointType == "Revolute"]

                test_joints = sliders if sliders else revolutes
                if test_joints and assy_shape:
                    tj = test_joints[0]
                    bb0 = assy.Shape.BoundBox
                    z0 = float(bb0.ZLength)
                    old_off = tj.Offset1

                    probe_success = False
                    for dx in [10.0, -10.0, 5.0, -5.0]:
                        try:
                            import FreeCAD
                            if tj.JointType == "Slider":
                                tj.Offset1 = FreeCAD.Placement(FreeCAD.Vector(dx, 0, 0), FreeCAD.Rotation())
                            else:
                                tj.Offset1 = FreeCAD.Placement(FreeCAD.Vector(0, 0, 0), FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), dx))
                            s_res = assy.solve()
                            if s_res == 0:
                                doc.recompute()
                                bb1 = assy.Shape.BoundBox
                                diff = abs(float(bb1.ZLength) - z0)
                                if diff > 1.0:
                                    delta_z = diff
                                    probe_success = True
                                    break
                        except Exception:
                            pass
                        finally:
                            tj.Offset1 = old_off
                            assy.solve()
                            doc.recompute()

                    articulates = probe_success
        else:
            result["initial_solve_code"] = -99

        result["articulates"] = articulates
        result["delta_z"] = float(delta_z)

        # constraints
        sketches = [o for o in doc.Objects if "Sketch" in o.TypeId]
        total_constraints = 0
        dimensional_constraints = 0
        constraint_values = set()

        for s in sketches:
            cs = getattr(s, "Constraints", [])
            total_constraints += len(cs)
            for c in cs:
                if hasattr(c, "Value") and c.Value is not None:
                    val = round(float(c.Value), 1)
                    if val > 0:
                        dimensional_constraints += 1
                        constraint_values.add(val)

        result["sketches_count"] = len(sketches)
        result["total_constraints"] = total_constraints
        result["dimensional_constraints"] = dimensional_constraints

        # drawing dimensions
        drawing_targets = [321.0, 255.0, 220.0, 215.0, 141.0, 121.0, 110.0, 120.0,
                           55.0, 50.0, 45.0, 42.0, 35.0, 33.7, 30.0, 22.0, 15.0, 12.0, 10.0]

        all_flattened_dims = set(constraint_values)
        for dims in body_dims_list:
            for d in dims:
                all_flattened_dims.add(d)

        found_signatures = []
        for t in drawing_targets:
            for v in all_flattened_dims:
                if abs(v - t) < 0.5:
                    found_signatures.append(t)
                    break

        result["signature_dimensions_found"] = found_signatures
        result["signature_dimensions_count"] = len(found_signatures)
        result["signature_targets_count"] = len(drawing_targets)

        # parametric test
        drives_geometry = False
        delta_vol = 0.0

        for s in sketches:
            if not hasattr(s, "Constraints"):
                continue
            for idx, c in enumerate(s.Constraints):
                if hasattr(c, "Value") and 50.0 < float(c.Value) < 350.0:
                    old_val = float(c.Value)
                    v0 = sum(float(b.Shape.Volume) for b in doc.Objects if b.TypeId == "PartDesign::Body" and hasattr(b, "Shape") and b.Shape)
                    try:
                        s.setDatum(idx, old_val + 10.0)
                        doc.recompute()
                        v1 = sum(float(b.Shape.Volume) for b in doc.Objects if b.TypeId == "PartDesign::Body" and hasattr(b, "Shape") and b.Shape)
                        diff_vol = abs(v1 - v0)
                        if diff_vol > 5.0:
                            drives_geometry = True
                            delta_vol = diff_vol
                            break
                    except Exception:
                        pass
                    finally:
                        try:
                            s.setDatum(idx, old_val)
                            doc.recompute()
                        except Exception:
                            pass
            if drives_geometry:
                break

        result["drives_geometry"] = drives_geometry
        result["drive_delta_volume"] = float(delta_vol)

        return result


SlingLiftStage.run()
