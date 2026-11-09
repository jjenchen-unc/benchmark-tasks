"""Fail-closed bridge to the unchanged pure numeric rubric.

Runtime channels must be installed and verified before this exposes any credit.
No synthetic telemetry is substituted for absent PhysX contact/wrench history.
"""
from numeric_score import score as numeric_score
REQUIRED_CHANNELS=frozenset(('joint_state','geometric_pose','robot_stand_contacts','panel_furniture_contacts','robot_box_contacts','lock_integrity','base_reaction_wrench','wrist_force','physics_substeps'))

def evaluate(measurement,verified_channels=frozenset()):
    missing=sorted(REQUIRED_CHANNELS-set(verified_channels))
    if missing:return {'score':0.0,'gate':'missing_runtime_measurement_channels','missing':missing,'backend':'Arena/PhysX preparation','full_credit':False}
    result=numeric_score(measurement);return dict(result,backend='Arena/PhysX',gate=None)
