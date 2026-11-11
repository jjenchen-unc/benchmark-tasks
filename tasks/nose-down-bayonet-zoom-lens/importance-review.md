# Capability and neighboring tasks

The task combines compatibility selection, inverted bayonet mounting with a physical capture/pin lock, and passive 50 mm zoom retention after releasing contact. It models camera servicing rather than optical calibration or image quality. These mechanism and outcome differences support the importance claim; colors, starting layout and different thresholds are not the claim.

A scoped authenticated board review refreshed on 2026-11-11 at07:39UTC (1,572 tasks) examined these neighboring tasks:

- TK-GPLTLT: Lock a bayonet cap into its socket
- TK-SHZASJ: Remove a multistage bayonet sleeve
- TK-64H93S: Block both lenses for edging
- TK-MZ7JMD: Turn a camera turntable without wrapping its cable
- TK-CFKY7E: Set a lens and screen on an optical bench for a sharp, same-size image
- TK-568ZHA: Reattach your detached wrist camera
- TK-U3EVW9: Commission a camera at two inspection depths
- TK-XDB5VZ: Release a camera from a cam-clamped quick-release mount
- TK-4EQ4RS: Set up a camera from its box

These already cover optical-bench placement, camera reattachment, lens setting tabs, quick-release removal and camera/tripod assembly. Generic camera assembly is therefore an existing capability. This task adds compatible camera-lens bayonet locking together with gravity-dependent zoom retention. Bayonet locking itself is explicitly an existing capability: TK-GPLTLT already requires keyed alignment, spring-loaded insertion and a quarter-turn contact lock. TK-SHZASJ already tests a multistage bayonet release track. The distinction claimed here is the combined camera-lens compatibility decision, inverted installation, zoom manipulation and friction-dependent focal retention after release; neither generic bayonet operation nor a longer hold alone is the novelty claim. The camera-turntable and optical-edging tasks likewise share camera/lens context but score cable topology and lens-block decentration, respectively. No exact compatible-lens-plus-passive-zoom-retention target was identified in the inspected current records; this is a scoped comparison, not an acceptance claim.

The inspected suite catalogs provide the following comparison; it is not an exhaustive absence claim.

- LIBERO: Reviewed official task map householdplacement/drawers/stove; no lensbayonet or passivezoomretention in inspected tasknames. [Official source](https://raw.githubusercontent.com/Lifelong-Robot-Learning/LIBERO/master/libero/libero/benchmark/libero_suite_task_map.py).
- SimplerEnv: Reviewed official pick/movenear/drawer/place/stack environment table; no lensmount operation identified there. [Official source](https://github.com/simpler-env/SimplerEnv#current-environments).
- RoboCasa: Kitchenfixture andtoolhandling descriptions share generalmanipulation; no lensbayonet/zoom retention identified in inspected catalogs. [Official source](https://robocasa.ai/docs/build/html/tasks/atomic_tasks.html).
- BEHAVIOR: Explicit neighboring task camera-to-tripod attached predicate. It doesnot specify compatiblelensselection, lensbayonet alignment/contactlock and focal setting retention; generic cameraassembly is not novel. [Official source](https://behavior.stanford.edu/knowledgebase/tasks/attach_a_camera_to_a_tripod-0.html).
- DataCenterBench: Controls andconnector mating share contactalignment; reviewed ten-task catalog differs from lenscompatibility/bayonet+zoomretention. [Official source](https://robocurve.org/datacenterbench/).
- RoboDojo: Fastenscrews/key/connector tasks already coverrotationalmating/precision; B6 distinguishes lensmount pluspassivezoom retention, notgeneric insertion novelty. [Official source](https://robodojo-benchmark.com/doc/).

Canon documents lens index alignment, rotation to lock and zoom-ring operation in its [lens preparation instructions](https://www.canon.com.au/support/sims-content?cid=8B07527D8E08489F9F687436384B6897&pid=009f0ff77a6d43f08979aa293e916ca9). The target 50 mm, 48–52 mm band and 20-second observation period are benchmark choices. Estimated rigid geometry, constraints and friction are disclosed; the benchmark makes no measured real-camera calibration claim.
