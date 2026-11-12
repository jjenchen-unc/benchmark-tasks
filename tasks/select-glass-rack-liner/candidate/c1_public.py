"""Explicit fixed public fixture; generic seed semantics remain unchanged."""
INSTRUCTION = "Select a suitable liner, lay it on the fixed 20 degree rack, stand one glass on it, release both glass and liner, and leave both hands off for 20 seconds after 0.5 seconds settling, with at most 5 mm displacement and 15 degree normal misalignment."
def select(module, trial_seed, target, jitter):
 public_seed=getattr(module,'C1_PHYSICAL_SEED',None)
 public_target=getattr(module,'C1_PUBLIC_TARGET',None)
 if not hasattr(module,'C1_PHYSICAL_SEED') and not hasattr(module,'C1_PUBLIC_TARGET'):
  return 42 if trial_seed is None else int(trial_seed)
 if type(public_seed) is not int or public_seed!=42 or public_target!='C1-T1':
  raise ValueError('Invalid C1 public case metadata')
 if target!=public_target or jitter!=0:
  raise ValueError('Contradictory C1 public target or jitter')
 return public_seed
