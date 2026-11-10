"""Public API stop guard: runs before the SDK can advance any physics."""
class EpisodeStopped(RuntimeError):pass
class EpisodeGuard:
    def __init__(self,limit):self.limit=limit;self.reset()
    def reset(self):self.actions=0;self.stopped=False;self.reason=None
    def stop(self,reason):self.stopped=True;self.reason=reason
    def invoke(self,advance):
        if self.stopped:raise EpisodeStopped('Episode already stopped: '+str(self.reason))
        if self.actions>=self.limit:
            self.stop('time_limit');raise EpisodeStopped('Control budget exhausted')
        # Reserve this action before entering SDK physics. A failed SDK call
        # cannot be retried against an ambiguously partially advanced scene.
        self.actions+=1
        try:result=advance()
        except BaseException:
            self.stop('step_exception');raise
        if result.terminated:self.stop(result.termination_reason or 'task_termination')
        if self.actions>=self.limit:self.stop(self.reason or 'time_limit')
        return result
