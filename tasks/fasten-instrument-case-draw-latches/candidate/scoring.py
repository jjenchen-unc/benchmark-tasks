"""DL-T2 numeric final-state rubric. Synthetic states test logic, never physics."""
import math
class History:
 def __init__(self):
  self.seconds=0.;self.samples=0;self.hold=0.;self.failed=None;self.last=None;self.abort=[0.,0.];self.guard=[0.,0.];self.guard_stopped=[False,False];self.link_window=[];self.peak_wrist=[0.,0.];self.peak_instrument_tilt=0.;self.peak_case_tilt=0.
 def update(self,row,dt):
  if self.failed:return
  if not math.isfinite(dt) or dt<=0 or dt>.01:raise ValueError('Invalid physical sample interval')
  values=[row['lid'],*row['lever'],*row['slide'],*row['catch'],*row['case_tilt_deg'],*row['instrument_tilt_deg'],*row['wrist_n'],*row['nonfinger_n']]
  if not all(math.isfinite(float(x)) for x in values):raise ValueError('Nonfinite measurement')
  if not(len(row['lever'])==len(row['slide'])==len(row['catch'])==len(row['engaged'])==2):raise ValueError('Two target latches required')
  self.seconds+=dt;self.samples+=1;self.last=row
  self.peak_instrument_tilt=max(self.peak_instrument_tilt,*row['instrument_tilt_deg']);self.peak_case_tilt=max(self.peak_case_tilt,*row['case_tilt_deg'])
  if self.peak_instrument_tilt>20.:self.failed='instrument_tipped'
  if self.peak_case_tilt>15.:self.failed='case_tipped'
  for a,f in enumerate(row['wrist_n']):
   self.peak_wrist[a]=max(self.peak_wrist[a],f);self.abort[a]=self.abort[a]+dt if f>80. else 0.;self.guard[a]=self.guard[a]+dt if f>60. else 0.
   if self.abort[a]>=.1-1e-10:self.failed='wrist_force_abort'
   if self.guard[a]>=.02-1e-10:self.guard_stopped[a]=True
  self.link_window.append((self.seconds,dt,row['nonfinger_n']))
  while self.link_window and self.link_window[0][0]<=self.seconds-.02+1e-10:self.link_window.pop(0)
  for a in range(2):
   if sum(d*forces[a] for _,d,forces in self.link_window)/.02>50.:self.guard_stopped[a]=True
  self.hold=self.hold+dt if self.goal(row) and not self.failed else 0.
 @staticmethod
 def fastened(row,j):return row['engaged'][j] in (0,1,2) and row['lever'][j]<=.05 and row['catch'][j]<=.0015 and row['slide'][j]+.004>=.0015-1e-12
 def goal(self,row):return not row['robot_contact'] and row['lid']<=math.radians(2.) and all(self.fastened(row,j) for j in range(2))
 def score(self):
  if self.failed or self.last is None or self.last['robot_contact']:return 0.
  if self.hold>=20.-1e-9 and self.goal(self.last):return 1.
  if self.last['lid']>math.radians(2.):return 0.
  return .25+.25*sum(self.fastened(self.last,j) for j in range(2))
 def status(self):return dict(seconds=self.seconds,samples=self.samples,hold_s=self.hold,failure=self.failed,measured_score=self.score(),guard_stopped=self.guard_stopped,peak_wrist_n=self.peak_wrist,peak_instrument_tilt_deg=self.peak_instrument_tilt,peak_case_tilt_deg=self.peak_case_tilt)
