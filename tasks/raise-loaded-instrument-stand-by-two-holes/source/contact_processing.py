"""Raw-view observer initialization equivalent to installed PhysX ContactSensor.
Process-local reporting switch only; no solver/scene/goal/controller writes.
"""
KEY='/physics/disableContactProcessing'
def enable(settings=None):
 if settings is None:
  from isaaclab.app.settings_manager import get_settings_manager
  settings=get_settings_manager()
 before=settings.get(KEY)
 if before is not None and type(before) is not bool:raise RuntimeError('contact processing setting is not boolean')
 settings.set_bool(KEY,False)
 after=settings.get(KEY)
 if after is not False:raise RuntimeError('contact processing failed to enable')
 return dict(setting=KEY,before=before,after=after,scope='process-only observer initialization')
def require_enabled():
 from isaaclab.app.settings_manager import get_settings_manager
 if get_settings_manager().get(KEY) is not False:raise RuntimeError('contact processing disabled after reader initialization')
