"""Read-only USD static shape binding. No guessed collider suffix or physics writes."""
def resolve_roles(rows, roots):
    by={r['path']:r for r in rows}
    if len(by)!=len(rows):raise ValueError('duplicate prim records')
    result={}
    for role,root in roots.items():
        if root not in by:raise ValueError('missing role root: '+role)
        shapes=[r for p,r in by.items() if (p==root or p.startswith(root+'/')) and r['collision_api']]
        if len(shapes)!=1:raise ValueError('missing/ambiguous collider: '+role)
        shape=shapes[0];path=shape['path'];cursor=path;owner=None
        while cursor and cursor!='/':
            if cursor not in by:raise ValueError('incomplete ancestor coverage: '+cursor)
            r=by[cursor]
            if r.get('active',True) is not True:raise ValueError('inactive ancestor: '+cursor)
            if r['rigid_body_api']:
                # Even a disabled/kinematic body is not an implicit static shape.
                owner=cursor
            cursor=cursor.rsplit('/',1)[0] or '/'
        if owner is not None:raise ValueError('body-owned collider in static role: '+role)
        if shape['attributes'].get('physics:collisionEnabled') is not True:raise ValueError('disabled/unknown collider: '+role)
        result[role]={'shape_path':path,'body_owner':None,'filter_path':path,'static':True}
    if len({r['shape_path'] for r in result.values()})!=len(result):raise ValueError('overlapping roles')
    return result

def stage_rows(stage):
    from pxr import UsdPhysics
    return [{'path':str(p.GetPath()),'active':bool(p.IsActive()),
             'rigid_body_api':bool(p.HasAPI(UsdPhysics.RigidBodyAPI)),
             'collision_api':bool(p.HasAPI(UsdPhysics.CollisionAPI)),
             'attributes':{'physics:collisionEnabled':bool(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get())} if p.HasAPI(UsdPhysics.CollisionAPI) else {}}
            for p in stage.Traverse()]

def bind(stage,env,fixed):
    roots={'floor':'/World/GroundPlane','table':env+'/table'}
    roots.update({'task_static_'+str(i):path for i,path in enumerate(fixed)})
    return resolve_roles(stage_rows(stage),roots)
