"""Read-only static collider selection. Preserve body paths for dynamic actors."""
def external_colliders(stage,envroot,excluded,collision_api,rigid_api):
 from pxr import Usd
 selected={}
 for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
  path=str(prim.GetPath())
  if not path.startswith(envroot+'/') or not prim.HasAPI(collision_api):continue
  if collision_api(prim).GetCollisionEnabledAttr().Get() is False:continue
  root=envroot+'/'+path[len(envroot)+1:].split('/')[0]
  if root in excluded:continue
  # The static root Xform has no rigid actor. Select its actual collider,
  # not merely an existing ancestor. Dynamic actors retain their rigid root.
  ancestor=prim;actor=None
  while ancestor and str(ancestor.GetPath()).startswith(root):
   if ancestor.HasAPI(rigid_api):actor=str(ancestor.GetPath());break
   ancestor=ancestor.GetParent()
  selected[actor or path]=root
 return dict(sorted(selected.items()))
