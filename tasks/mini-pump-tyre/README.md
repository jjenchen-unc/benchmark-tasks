# Pump up a tyre with a gauge-less mini pump, holding the push-on head on the valve

A small pram wheel lies flat and fixed on the station table, its tyre soft at 0.8 bar. Its brass valve stem is bent 90
degrees, so its tip points at the right arm, 6 cm above the table. A mini bike pump with no gauge lies in front of the
arms. The robot has to push the pump's head fully onto the stem, keep it there with one hand while the other strokes the
handle in and out (about 10 full strokes to reach 1.5 bar), then pull the pump off and lay it on the table.

The head is a plain push-on fit, not a lever clamp: if the holding hand lets it slip back, the next stroke's air goes out
of the head. With no gauge, the pressure is judged by counting full strokes, and too many over-inflate the tyre.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: wheel, pump, the air model, `success()` and `score()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with sliding joints, for yam_arena task files |
| `demo.py` | The scripted full-score demo and the cheap-strategy variants |

`partkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pump the tyre up to 1.5 bar with the mini pump, then take the pump off the valve and lay it on the table. The tyre starts at 0.8 bar; the pump has no gauge, and each full stroke adds about 0.08 bar." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mini-pump-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out --max-steps 2400            # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=no_seat python demo.py task.py out --no-record   # holds the head 1 cm short of the stem and strokes
```

Other variants: `on_table` (strokes the pump where it lies), `swat` (sweeps the pump into the wheel), `over` (24
strokes), `stay` (pumps but leaves the pump on the valve), `nothing`.

1. The left arm grips the barrel just behind the head, lifts it, carries the head's opening to 3 cm short of the stem
   on the stem's line, corrects twice from the pump's measured pose, and pushes the head on.
2. The right arm grips the handle knob where the pump's measured pose puts it and gives 10 full strokes. The count
   comes from the published gain per stroke; nothing reads the tyre pressure.
3. The right hand lets go; the left arm pulls the pump straight off the valve and lays it on the table.

The plan uses only the published layout, the pump's pose and the handle's position (both visible to a camera), and the
arms' own joints. The demo scores 1.0 (success) at step 1,041 with the tyre at 1.59 bar.

## How the pressure can be judged without a gauge

The pressure is never shown. A policy can judge it by counting full strokes, because each full stroke adds a steady,
known amount. Measured in the demo run (tyre gauge pressure after each full stroke, from 0.80 bar):
0.89, 0.97, 1.06, 1.14, 1.22, 1.30, 1.38, 1.45, 1.52, 1.59 (10 strokes), and with the `over` variant 1.65, 1.71, 1.76,
1.80, 1.84, 1.87 (16 strokes). Strokes 6 to 11 land in the 1.3 to 1.7 bar window. The handle also pushes back harder as
the pressure rises (about 5 N at 1.0 bar, 8 N at 1.6 bar), which shows as the handle springing back out if it is let go
part way in.

## Cheap strategies (measured, same scene and scorer)

| Run | Tyre (bar) | Score |
|---|---|---|
| Full demo, 10 strokes | 1.59 | 1.0 (success, step 1,041) |
| `nothing` | 0.80 | 0.0 |
| `swat`: sweep the pump into the wheel | 0.80 | 0.0 |
| `on_table`: stroke the pump where it lies, head not on the valve | 0.80 | 0.0 |
| `no_seat`: hold the head 1 cm short of the stem and stroke | 0.80 | 0.0 |
| `over`: 24 strokes, then pull off and lay down | 1.87 | 0.25 |
| `stay`: 10 strokes, pump left on the valve | 1.59 | 0.5 |

In `over`, the demo's grip on the handle knob starts to slip after about 16 strokes, so the tyre stops at 1.87 bar; a
firmer grip would push it higher, and every pressure above 1.7 bar scores 0.25.

## Scoring

Evaluated every control step; the score at the end of the episode counts.

- 0: tyre below 1.0 bar (no air got in).
- 0.25: tyre at 1.0 bar or more but outside 1.3 to 1.7 bar (under- or over-inflated).
- 0.5: tyre between 1.3 and 1.7 bar.
- 0.75: also the pump off the valve (its mouth at least 2 cm clear of the stem tip).
- 1.0 (success, held for 1 s): also the pump lying still on the table.

A judge reads the tyre with a tyre gauge and checks the pump's position on a photo.

## Runtime and seeds

About 8 minutes per trial for the full 2,400 steps on an RTX A6000, plus about 2 minutes for Isaac Sim to start. The
layout is fixed (`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

The parts are rigid bodies built from boxes and cylinders; the wheel and valve are fixed. The head's opening is a real
collider (12 mm square around the 8 mm stem), so seating is done by contact. The handle is a prismatic joint. The air
is an isothermal model of the 8 mm bore, 12 cm stroke cylinder (1 ml dead volume) and the 60 ml tube: the stroke
compresses the cylinder air; it flows into the tyre when the head is seated and its pressure exceeds the tyre's, and
escapes from the head when it is not. The pressure pushes the handle back out through the joint's drive (about 9 N
near 1.5 bar). The tyre's volume is fixed, and a seated head does not leak.

## Real-world build

A 4 inch pram or scooter wheel clamped flat to the table, with a 90 degree bent valve extension on its valve, and any
gauge-less mini pump with a push-on head (most have a Schrader/Presta push-on head with an 8 mm bore). The tube volume
sets the strokes per bar; count strokes for the actual tube and adjust the stroke count in the instruction.
