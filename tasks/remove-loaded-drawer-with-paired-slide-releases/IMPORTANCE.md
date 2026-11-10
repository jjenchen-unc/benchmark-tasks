# Drawers importance and ownership assessment

Authenticated board refreshed 2026-11-10T19:27:28.523043+00:00: 1,480 tasks,
SHA256 8ff15b02df1c896ff6d6c48252061e5b0fd962c0ed1ad02adde3d7eb5727b336.
This assesses the actual fixed DR-T1/S task; no organizer clearance is claimed.
The six closest reviewed entries remain pending and yours=false.

The proposed capability is removing a loaded drawer by operating two independent
spring-loaded slide disconnects, supporting both sides through extraction,
and placing the drawer level without damaging either jar or losing its box.
The fixed physical layout and privileged scripted reference are disclosed.
The target is furniture service rather than ordinary drawer opening; the red
marking identifies the drawer and is not the novelty claim.

Material current board overlap, all pending and yours=false in this snapshot:

| Task | Shared capability | Substantive distinction |
| --- | --- | --- |
| TK-MG5GRS, remove a drawer past an anti-pull-out stop | Entire drawer removal and released table placement; this is the closest goal | Its empty drawer must lean about 18 degrees to clear a fixed rear tab. DR-T1/S has independently movable paired slide-release paddles and fragile retained contents, so supporting extraction while operating both releases and limiting jar contact/tilt matters. A cosmetic drawer variant would not suffice. |
| TK-DBVGN2, interlocked dual-drawer transfer | Coordinating drawer manipulation and safety | Transfers a block between drawers while avoiding simultaneous opening; drawers stay installed. Here the entire loaded drawer disconnects and moves. |
| TK-37QME4, socket retrieval from interlocked chest | A drawer fails to move until another mechanism is addressed | Removes a tall tool and closes another drawer to unlock socket retrieval. It does not disconnect paired slides or carry the drawer. |
| TK-DSGGLD, spring-return drawer | Sustained bimanual support against a spring | Holds a sliding drawer open while removing a block, then lets it close; no drawer disconnection or fragile loaded carry. |
| TK-8JWZYQ, heavy tin in free-standing chest | Supporting a loaded drawer while managing damage risk | Counterholds the tipping chest and removes a tin, then closes the drawer. DR-T1/S fixes the carcass and instead scores slide release, loaded-drawer transfer and jar integrity. |
| TK-YBZC8L, release trapped clothing | Bimanual recovery from a physical obstruction | Compresses a spring-supported rigid cloth proxy so a drawer can continue opening; no slide disconnection or loaded table placement. |

Six benchmark comparisons use primary catalogs checked 2026-11-10. These are
scoped task-definition comparisons, not exhaustive audits of every collision
asset or proof that no similar task could be constructed in those systems.

- [LIBERO task map](https://raw.githubusercontent.com/Lifelong-Robot-Learning/LIBERO/master/libero/libero/benchmark/libero_suite_task_map.py)
  includes cabinet/drawer opening and household object placement. Those named
  goals overlap access and placement, but do not specify paired slide
  disconnection followed by carrying a fragile loaded drawer.
- [RoboCasa atomic tasks](https://robocasa.ai/docs/build/html/tasks/atomic_tasks.html)
  and [official source](https://github.com/robocasa/robocasa/blob/main/robocasa/__init__.py)
  include opening/closing drawers and sliding a dishwasher rack. Opening an
  articulated drawer shares access mechanics; complete loaded-drawer removal
  through two release paddles with contact-based jar damage is the added goal.
- [SimplerEnv environments](https://raw.githubusercontent.com/simpler-env/SimplerEnv/main/README.md)
  include Google-robot drawer manipulation and object placement. The listed
  drawer tasks concern articulation/retrieval, rather than slide disconnection
  and free loaded-drawer transport.
- [BEHAVIOR task catalog](https://behavior.stanford.edu/knowledgebase/tasks/)
  lists cleaning out drawers, putting laundry in drawers and organizing a file
  cabinet. Those content/household predicates are adjacent to the setting;
  their catalog definitions do not establish the same paired-release and
  quantitative non-pad jar-force/normal-impact constraints.
- [DataCenterBench catalog](https://robocurve.org/datacenterbench/)
  describes control and connection steps such as key interlocks, switches,
  plugs and cable clips. Servicing context overlaps, but the ten retained
  single-step tasks do not define this loaded slide-disconnect operation.
- [RoboDojo task documentation](https://robodojo-benchmark.com/doc/)
  lists packing, tool storage, precision insertion, screws and real LEGO
  disassembly. These overlap manipulation or disassembly skills; the enumerated
  goals do not specify removing a loaded drawer via paired slide releases.

Assessment: the pairing of physically independent disconnects and fragile
loaded-drawer support provides a substantive basis to submit after complete
physical/model/media checks. TK-MG5GRS creates real goal overlap and should be
disclosed to the reviewer. Importance and similarity remain organizer decisions.
