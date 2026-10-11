"""Interactive Isaac Sim TRON1 stopping laboratory (tested runtime: 4.5)."""
from pathlib import Path
import argparse
import csv
import json
import math
import time
import hashlib
import traceback
from dataclasses import asdict

from build_scene import build_scene, ROOT, DEFAULT_ASSET
from stopping_core import Settings, reference, motor_torque, slip_metrics, summarize
from stopping_controllers import CONTROLLERS, Observation, Reference, create_controller, torque_to_command

_app = None


def main():
    global _app
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset", type=Path, default=DEFAULT_ASSET)
    parser.add_argument("--config", type=Path, default=ROOT / "config/stopping.json")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--auto-run", action="store_true")
    parser.add_argument("--exit-after-trial", action="store_true")
    parser.add_argument("--suite", type=Path, help="Run sequential parameter overrides in one Isaac session")
    parser.add_argument("--controller", choices=CONTROLLERS, default="lqr_tracking")
    parser.add_argument("--ui-check", action="store_true", help="Exercise pause/resume and manual brake callbacks in the GUI")
    args = parser.parse_args()
    cfg = Settings(**json.loads(args.config.read_text()))
    base_settings = asdict(cfg)
    cases = json.loads(args.suite.read_text()) if args.suite else []
    for case in cases:
        Settings(**(base_settings | case["settings"]))
        if case.get("controller", args.controller) not in CONTROLLERS:
            raise ValueError("Unknown suite controller")
    output = args.output_dir or ROOT / "outputs" / time.strftime("stopping-%Y%m%d-%H%M%S")
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError("Choose a new/empty output directory")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "width": 1280, "height": 800,
                         "renderer": "RayTracedLighting", "anti_aliasing": 0,
                         "multi_gpu": False, "sync_loads": True})
    _app = app
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade, PhysicsSchemaTools
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, RigidPrim
    from isaacsim.core.utils.stage import open_stage, get_current_stage
    from isaacsim.core.utils.types import ArticulationAction
    from omni.kit.viewport.utility import get_active_viewport
    from balance_lqr import WIPParameters

    physical = json.loads((ROOT / "config/balance_wf.json").read_text())
    p = WIPParameters(**physical["model"])
    dt = .005
    wheel_controller = create_controller(args.controller, p, dt)
    selected_controller = args.controller
    controller_selector = None
    rotation = Rotation.from_euler("y", physical["equilibrium_base_pitch_rad"])
    initial_pos = np.array([0., 0., p.radius + .001]) - rotation.apply(physical["axle_in_base"])
    q = rotation.as_quat()[[3, 0, 1, 2]]
    scene = build_scene(args.asset, output / "scene.usda", free_base=True)
    if not open_stage(str(scene)):
        raise RuntimeError("Cannot open stopping scene")
    stage = get_current_stage()
    xform = UsdGeom.Xformable(stage.GetPrimAtPath("/World/TRON1"))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*initial_pos.tolist()))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:].tolist())))
    stage.RemovePrim("/World/Ground")
    PhysicsSchemaTools.addGroundPlane(stage, "/World/Ground", "Z", 100., Gf.Vec3f(0,0,0), Gf.Vec3f(.18,.21,.25))
    material = UsdShade.Material.Define(stage, "/World/StoppingMaterial")
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    mat.CreateStaticFrictionAttr(cfg.static_friction)
    mat.CreateDynamicFrictionAttr(cfg.dynamic_friction)
    mat.CreateRestitutionAttr(0.)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("average")
    collision_paths = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(.002)
            collision.CreateRestOffsetAttr(0.)
            collision_paths.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rigid.CreateMaxAngularVelocityAttr(math.degrees(100.))
            rigid.CreateLinearDampingAttr(0.)
            rigid.CreateAngularDampingAttr(0.)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.)
            drive.CreateDampingAttr(0.)
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateJointFrictionAttr(0.)
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/TRON1/base_Link"))
    art.CreateSolverPositionIterationCountAttr(8)
    art.CreateSolverVelocityIterationCountAttr(4)
    for _ in range(5):
        app.update()
    stage.RemovePrim("/World/PhysicsScene")
    app.update()
    world = World(physics_prim_path="/World/PhysicsScene", stage_units_in_meters=1., physics_dt=dt,
                  rendering_dt=1/60, device="cpu")
    world.get_physics_context().set_solver_type("TGS")
    robot = world.scene.add(SingleArticulation(prim_path="/World/TRON1/base_Link", name="tron1"))
    paths = [str(prim.GetPath()) for prim in stage.Traverse()
             if str(prim.GetPath()).startswith("/World/TRON1/") and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    bodies = RigidPrim(prim_paths_expr=paths, name="measurements", reset_xform_properties=False,
                       prepare_contact_sensors=False)
    contacts = RigidPrim(prim_paths_expr="/World/TRON1/wheel_.*_Link", name="wheel_contacts",
                        reset_xform_properties=False, track_contact_forces=True, disable_stablization=False)
    world.reset()
    bodies.initialize()
    contacts.initialize()
    names = list(robot.dof_names)
    wheels = np.array([names.index(f"wheel_{side}_Joint") for side in ("L", "R")])
    legs = np.array([i for i in range(8) if i not in wheels])
    paths = list(bodies.prim_paths)
    wi = np.array([next(i for i,s in enumerate(paths) if s.endswith(f"/wheel_{side}_Link")) for side in ("L", "R")])
    ci = np.array([next(i for i,s in enumerate(contacts.prim_paths) if s.endswith(f"/wheel_{side}_Link")) for side in ("L", "R")])
    bi = np.array([i for i in range(len(paths)) if i not in wi])
    masses = np.asarray(bodies.get_masses()).reshape(-1)
    local_com = np.asarray(bodies.get_coms()[0]).reshape(len(paths),3)
    assert len(names) == 8 and np.isclose(masses[bi].sum(), p.body_mass, rtol=1e-4)
    print("STOPPING_READY", names, flush=True)
    world.pause()
    viewport = get_active_viewport()
    if viewport:
        viewport.set_active_camera("/World/Camera")
    controller = robot.get_articulation_controller()
    state = {"running": False, "reset": False, "paused": False, "brake": None,
             "rows": [], "index": 0, "status": "Ready: edit settings, then Reset + Run", "previous": None}
    models = {}
    label = None

    def finish(reason):
        state["running"] = False
        world.pause()
        rows = state["rows"]
        if not rows:
            return
        folder = output / f"trial-{state['index']:03d}"
        folder.mkdir(exist_ok=False)
        with (folder / "telemetry.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        report = summarize(rows, cfg, reason)
        (folder / "settings.json").write_text(json.dumps(asdict(cfg), indent=2))
        report.update({"joint_names": names,
                       "controller": wheel_controller.metadata(),
                       "controller_sha256": hashlib.sha256((ROOT/'scripts/stopping_controllers.py').read_bytes()).hexdigest(),
                       "case_name": state.get("case_name", "interactive"),
                       "ui_checks": state.get("ui_checks", {}),
                       "wheel_material_runtime": state.get("wheel_material_runtime"),
                       "ground_collision": "native infinite PhysX plane at z=0",
                       "brake_trigger_time_s": state["brake"],
                       "collision_material_paths": collision_paths,
                       "material_readback": {"static": mat.GetStaticFrictionAttr().Get(), "dynamic": mat.GetDynamicFrictionAttr().Get()},
                       "asset_sha256": hashlib.sha256(args.asset.read_bytes()).hexdigest(),
                       "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                       "core_sha256": hashlib.sha256((ROOT/'scripts/stopping_core.py').read_bytes()).hexdigest(),
                       "effective_physics_dt": world.get_physics_dt()})
        (folder / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
        state["rows"] = []
        distance = report['signed_stop_distance_m']
        distance_text = f"{distance:.3f} m" if distance is not None else "not reached"
        state["status"] = (f"{reason}: stable stop={report['stable_stop']}\n"
                           f"Confirmed-stop displacement: {distance_text}\n"
                           f"Peak excursion: {report['max_excursion_after_brake_m']} m\n"
                           f"Peak supported braking slip: {report['max_supported_slip_after_brake_mps']:.3f} m/s\n"
                           f"Saved CSV + JSON: {folder.name}")
        print("STOPPING_RESULT", json.dumps(report), flush=True)

    def reset():
        nonlocal cfg, wheel_controller
        new = Settings(**{k: m.as_float for k,m in models.items()}) if models else cfg
        name = (CONTROLLERS[controller_selector.model.get_item_value_model().as_int]
                if controller_selector is not None else selected_controller)
        new_controller = create_controller(name, p, dt)
        if state["rows"]:
            finish("reset_by_user")
        cfg = new
        wheel_controller = new_controller
        wheel_controller.reset()
        world.stop()
        mat.GetStaticFrictionAttr().Set(cfg.static_friction)
        mat.GetDynamicFrictionAttr().Set(cfg.dynamic_friction)
        world.reset()
        world.set_simulation_dt(physics_dt=dt, rendering_dt=1/60)
        bodies.initialize()
        contacts.initialize()
        robot.set_world_pose(position=initial_pos.astype(np.float32), orientation=q.astype(np.float32))
        robot.set_joint_positions(np.zeros(8, np.float32))
        dq = np.zeros(8, np.float32)
        dq[wheels] = cfg.initial_speed / p.radius
        robot.set_joint_velocities(dq)
        robot.set_linear_velocity(np.array([cfg.initial_speed, 0., 0.], np.float32))
        robot.set_angular_velocity(np.zeros(3, np.float32))
        kp, kd = np.full(8,500.,np.float32), np.full(8,30.,np.float32)
        kp[wheels] = kd[wheels] = 0.
        controller.set_gains(kps=kp, kds=kd, save_to_usd=True)
        controller.set_gains(kps=kp, kds=kd, save_to_usd=False)
        controller.set_max_efforts(np.full(8, 80., np.float32))
        view = robot._articulation_view
        view.set_friction_coefficients(np.zeros((1,8), np.float32))
        view.set_armatures(np.zeros((1,8), np.float32))
        if not all(np.allclose(g, wanted) for g,wanted in zip(controller.get_gains(),(kp,kd))):
            raise RuntimeError("Expected leg PD / wheel effort gains")
        robot.apply_action(ArticulationAction(joint_positions=np.zeros(6), joint_velocities=np.zeros(6), joint_indices=legs))
        world.physics_sim_view.update_articulations_kinematic()
        material_values = np.asarray(contacts._physics_view.get_material_properties())
        expected = np.array([cfg.static_friction, cfg.dynamic_friction, 0.])
        if not np.allclose(material_values, expected, atol=1e-6):
            raise RuntimeError(f"Runtime wheel contact materials differ: {material_values}")
        state["wheel_material_runtime"] = material_values.tolist()
        assert np.isclose(world.get_physics_dt(), dt)
        state.update(running=True, paused=False, brake=cfg.brake_at, rows=[], previous=None,
                     tick=0, index=state["index"]+1, status="Running", ui_checks={}, brake_x=None)
        stage.GetRootLayer().Save()

    def brake():
        if state["running"]:
            state["brake"] = min(state["brake"], state["tick"]*dt)

    def toggle_pause():
        state["paused"] = not state["paused"]
        if state["paused"]:
            world.pause()
        elif state["running"]:
            world.play()

    if not args.headless:
        import omni.ui as ui
        window = ui.Window("TRON1 Stopping Lab", width=430, height=760)
        fields = [("initial_speed", "Initial speed (m/s)"), ("static_friction", "Static friction coefficient"),
                  ("dynamic_friction", "Dynamic friction coefficient"), ("torque_conversion", "Torque conversion (Nm / unit)"),
                  ("torque_limit", "Wheel torque ceiling (Nm)"), ("no_load_speed", "No-load speed (rad/s)"),
                  ("shaft_resistance", "Wheel shaft resistance (Nm)"), ("brake_at", "Automatic brake time (s)"),
                  ("deceleration", "Reference deceleration (m/s2)"), ("duration", "Trial duration (s)")]
        with window.frame:
            with ui.VStack(spacing=8):
                ui.Label("TRON1 WF | physical stopping + slip", height=25)
                ui.Label("Edit values, then Reset + Run to apply.\nBrake now acts immediately; Pause retains trial state.", height=40)
                with ui.HStack(height=25):
                    ui.Label("Wheel controller", width=180)
                    controller_selector = ui.ComboBox(CONTROLLERS.index(args.controller), *CONTROLLERS)
                for key, title in fields:
                    with ui.HStack(height=25):
                        ui.Label(title, width=285)
                        models[key] = ui.SimpleFloatModel(getattr(cfg,key))
                        ui.FloatDrag(models[key], step=.01)
                with ui.HStack(height=35):
                    ui.Button("Reset + Run", clicked_fn=lambda: state.update(reset=True))
                    ui.Button("Brake now", clicked_fn=brake)
                    ui.Button("Pause / Resume", clicked_fn=toggle_pause)
                ui.Button("End trial + save CSV", height=30, clicked_fn=lambda: finish("ended_by_user"))
                label = ui.Label(state["status"], height=135, word_wrap=True)
                ui.Label("Slip = axle vx - R * world wheel omega_y\nForce display: net contact Z proxy.\nCSV + JSON saved after every trial.", height=65)
        window.position_x = 840
        window.position_y = 65
    try:
        if args.headless or args.auto_run or cases:
            state["reset"] = True
        while app.is_running():
            if state["reset"]:
                state["reset"] = False
                try:
                    if cases:
                        case = cases.pop(0)
                        cfg = Settings(**(base_settings | case["settings"]))
                        state["case_name"] = case["name"]
                        selected_controller = case.get("controller", args.controller)
                        if controller_selector is not None:
                            controller_selector.model.get_item_value_model().set_value(CONTROLLERS.index(selected_controller))
                        for key, model in models.items():
                            model.set_value(getattr(cfg, key))
                    reset()
                except ValueError as exc:
                    state["status"] = str(exc)
                    print("SETTINGS_OR_RESET_ERROR", str(exc), flush=True)
                    if args.headless:
                        raise
            if not state["running"] or state["paused"]:
                world.pause()
                if label:
                    label.text = state["status"] + ("\nPAUSED" if state["paused"] else "")
                app.update()
                if cases and not state["paused"]:
                    state["reset"] = True
                    continue
                if args.headless or (args.exit_after_trial and state["index"]):
                    break
                time.sleep(.01)
                continue
            t = state["tick"] * dt
            if args.ui_check and state["tick"] == 50:
                pose_before = robot.get_world_pose()[0].copy()
                toggle_pause()
                for _ in range(5):
                    app.update()
                np.testing.assert_allclose(robot.get_world_pose()[0], pose_before, atol=1e-8)
                toggle_pause()
                state["ui_checks"]["pause_keeps_pose"] = True
            if args.ui_check and state["tick"] == 100:
                brake()
                assert state["brake"] <= t
                state["ui_checks"]["brake_callback"] = True
            positions, quats = bodies.get_world_poses()
            rotations = Rotation.from_quat(np.asarray(quats)[:,[1,2,3,0]])
            coms = np.asarray(positions) + rotations.apply(local_com)
            com = np.average(coms[bi], axis=0, weights=masses[bi])
            axle = np.asarray(positions)[wi].mean(0)
            theta = math.atan2(com[0]-axle[0], com[2]-axle[2])
            angular = np.asarray(bodies.get_angular_velocities())
            origin_v = np.asarray(bodies.get_linear_velocities()) - np.cross(angular, rotations.apply(local_com))
            roll, _, yaw = Rotation.from_quat(np.asarray(robot.get_world_pose()[1])[[1,2,3,0]]).as_euler("xyz")
            vx = float(origin_v[wi,0].mean())
            if state["previous"] is None:
                rate = 0.
            else:
                old_x, old_theta = state["previous"]
                vx, rate = (axle[0]-old_x)/dt, (theta-old_theta)/dt
            state["previous"] = (axle[0], theta)
            xref, vref, accel = reference(t, cfg.initial_speed, state["brake"], cfg.deceleration)
            requested = wheel_controller.step(Observation(float(axle[0]), vx, theta, rate),
                                              Reference(xref, vref, accel))
            jp, jv = robot.get_joint_positions(), robot.get_joint_velocities()
            command = torque_to_command(requested)
            torque = motor_torque(command, jv[wheels], cfg)
            robot.apply_action(ArticulationAction(joint_efforts=torque, joint_indices=wheels))
            rolling, slip, ratio = slip_metrics(origin_v[wi], angular[wi], p.radius)
            normal = np.asarray(contacts.get_net_contact_forces(dt=dt))[ci,2]
            row = dict(t=t,x=float(axle[0]),axle_z=float(axle[2]),max_leg_error=float(np.max(np.abs(jp[legs]))),vx=float(vx),tensor_vx=float(origin_v[wi,0].mean()),pitch=theta,
                       roll=float(roll),yaw=float(yaw),reference_x=xref,reference_v=vref,
                       braking=int(t>=state["brake"]),omega_l=float(jv[wheels[0]]),omega_r=float(jv[wheels[1]]),
                       rolling_l=float(rolling[0]),rolling_r=float(rolling[1]),slip_l=float(slip[0]),slip_r=float(slip[1]),
                       slip_ratio_l=float(ratio[0]),slip_ratio_r=float(ratio[1]),normal_l=float(normal[0]),normal_r=float(normal[1]),
                       requested_torque_l=float(requested[0]),requested_torque_r=float(requested[1]),
                       command_l=float(command[0]),command_r=float(command[1]),torque_l=float(torque[0]),torque_r=float(torque[1]))
            if not all(math.isfinite(v) for v in row.values()):
                finish("nonfinite_state")
                raise RuntimeError("Nonfinite simulator measurement")
            state["rows"].append(row)
            if row["braking"] and state["brake_x"] is None:
                state["brake_x"] = float(axle[0])
            travel = float(axle[0])-state["brake_x"] if state["brake_x"] is not None else 0.
            state["status"] = (f"t={t:.2f} s | {'BRAKING' if row['braking'] else 'ROLLING'}\n"
                               f"x={axle[0]:.3f} m   v={vx:.3f} m/s   pitch={math.degrees(theta):.1f} deg\n"
                               f"slip L/R={slip[0]:.3f} / {slip[1]:.3f} m/s\n"
                               f"torque L/R={torque[0]:.2f} / {torque[1]:.2f} Nm\n"
                               f"normal L/R={normal[0]:.1f} / {normal[1]:.1f} N\n"
                               f"Travel since brake: {travel:.3f} m")
            if label and state["tick"] % 20 == 0:
                label.text = state["status"]
            if abs(theta)>math.radians(45) or abs(roll)>math.radians(35) or axle[2]<.06:
                finish("fallen")
            elif t>=cfg.duration-dt/2:
                finish("completed")
            else:
                before = world.current_time
                world.step(render=False)
                if not np.isclose(world.current_time-before, dt, atol=1e-6):
                    raise RuntimeError("Physics step did not advance by exactly the configured dt")
                state["tick"] += 1
                if not args.headless and state["tick"]%4==0:
                    world.render()
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        if state["rows"]:
            finish("window_closed")
        app.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        traceback.print_exc()
        if _app is not None:
            _app.close()
        raise
