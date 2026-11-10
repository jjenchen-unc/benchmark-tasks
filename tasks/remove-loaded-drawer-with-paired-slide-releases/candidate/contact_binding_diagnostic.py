"""Failure-only USD/handle observation. Never changes the stage or selections."""
import json
from pathlib import Path

def record(simulation_view, view, sensors, filters, error):
    report = {'error': repr(error), 'sensors': list(sensors), 'filters': list(filters),
              'filter_patterns': [list(filters) for _ in sensors],
              'simulation_view_type': type(simulation_view).__name__,
              'simulation_backend_none': getattr(simulation_view, '_backend', None) is None,
              'contact_view_type': type(view).__name__,
              'contact_backend_none': getattr(view, '_backend', None) is None,
              'prims': [], 'observational_only': True}
    try:
        import omni.usd
        from pxr import Usd, UsdPhysics
        stage = omni.usd.get_context().get_stage()
        wanted = set(sensors) | set(filters)
        # Actual43633 scene config names a global floor; inspect rather than guess its leaf.
        wanted.add('/World/GroundPlane')
        report['floor_root_from_actual43633_scene_cfg'] = '/World/GroundPlane'
        report['required_coverage'] = ['floor', 'table', 'robot', 'task']
        report['missing_exact_paths'] = sorted(p for p in wanted if not stage.GetPrimAtPath(p).IsValid())
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path not in wanted and not any(path.startswith(p + '/') for p in wanted):
                continue
            attrs = {}
            for attr in prim.GetAttributes():
                name = attr.GetName()
                if name.startswith(('physics:', 'physx', 'physxContactReport:')):
                    value = attr.Get()
                    attrs[name] = value if isinstance(value, (type(None), bool, int, float, str)) else str(value)
            report['prims'].append({'path': path, 'type': prim.GetTypeName(),
                'applied_schemas': list(prim.GetAppliedSchemas()),
                'rigid_body_api': prim.HasAPI(UsdPhysics.RigidBodyAPI),
                'collision_api': prim.HasAPI(UsdPhysics.CollisionAPI), 'attributes': attrs})
    except Exception as diagnostic_error:
        report['diagnostic_error'] = repr(diagnostic_error)
    target = Path(__file__).resolve().parent.parent / 'contact-binding-diagnostic.json'
    target.write_text(json.dumps(report, indent=2) + '\n')
    return str(target)
