# Task primitive inventory

Dimensions below are physical full extents/radii. For capsule shapes the straight
segment excludes the endcaps. Local positions/quaternions and collision masks
remain exactly in candidate/fixture.json; native quaternions there are WXYZ.
Materials describe the rigid surrogates; no elastic material calibration is claimed.

|Body|Collider|Type|Dimensions|Nominal material|
|---|---|---|---|---|
|case_A|A_floor|box|240 x 160 x 6 mm|rigid plastic shell|
|case_A|A_wall_f|box|240 x 6 x 180 mm|rigid plastic shell|
|case_A|A_wall_b|box|240 x 6 x 180 mm|rigid plastic shell|
|case_A|A_wall_l|box|6 x 148 x 180 mm|rigid plastic shell|
|case_A|A_wall_r|box|6 x 148 x 180 mm|rigid plastic shell|
|case_A|A0_base|box|26 x 4 x 32 mm|steel hardware/instrument surrogate|
|case_A|A0_bracket|box|24 x 10 x 10 mm|steel hardware/instrument surrogate|
|case_A|A1_base|box|26 x 4 x 32 mm|steel hardware/instrument surrogate|
|case_A|A1_bracket|box|24 x 10 x 10 mm|steel hardware/instrument surrogate|
|lid_A|A_lid_top|box|240 x 160 x 6 mm|rigid plastic shell|
|lid_A|A_lid_f|box|240 x 6 x 40 mm|rigid plastic shell|
|lid_A|A_lid_b|box|240 x 6 x 40 mm|rigid plastic shell|
|lid_A|A_lid_l|box|6 x 148 x 40 mm|rigid plastic shell|
|lid_A|A_lid_r|box|6 x 148 x 40 mm|rigid plastic shell|
|lid_A|A_lip|box|44 x 24 x 4 mm|rigid plastic shell|
|lid_A|A0_keeper|box|24 x 4 x 28 mm|steel hardware/instrument surrogate|
|lid_A|A0_bar0|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_A|A0_bar1|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_A|A0_bar2|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_A|A1_keeper|box|24 x 4 x 28 mm|steel hardware/instrument surrogate|
|lid_A|A1_bar0|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_A|A1_bar1|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_A|A1_bar2|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|A0_lever|A0_lever_side0|box|2 x 8 x 62 mm|steel hardware/instrument surrogate|
|A0_lever|A0_lever_side1|box|2 x 8 x 62 mm|steel hardware/instrument surrogate|
|A0_lever|A0_lever_web|box|20 x 2 x 58 mm|steel hardware/instrument surrogate|
|A0_lever|A0_lever_tab|box|24 x 30 x 3 mm|steel hardware/instrument surrogate|
|A0_lever|A0_pivot|cylinder|radius 3 mm; cylinder height 22 mm|steel hardware/instrument surrogate|
|A0_link|A0_link_plate|box|4 x 6 x 21.4776 mm|steel hardware/instrument surrogate|
|A0_hook|A0_hook_shank|box|4 x 8 x 30 mm|steel hardware/instrument surrogate|
|A0_hook|A0_hook_nose|box|10 x 2.5 x 4 mm|steel hardware/instrument surrogate|
|A0_hook|A0_hook_cap|box|12 x 11 x 3 mm|steel hardware/instrument surrogate|
|A1_lever|A1_lever_side0|box|2 x 8 x 62 mm|steel hardware/instrument surrogate|
|A1_lever|A1_lever_side1|box|2 x 8 x 62 mm|steel hardware/instrument surrogate|
|A1_lever|A1_lever_web|box|20 x 2 x 58 mm|steel hardware/instrument surrogate|
|A1_lever|A1_lever_tab|box|24 x 30 x 3 mm|steel hardware/instrument surrogate|
|A1_lever|A1_pivot|cylinder|radius 3 mm; cylinder height 22 mm|steel hardware/instrument surrogate|
|A1_link|A1_link_plate|box|4 x 6 x 21.4776 mm|steel hardware/instrument surrogate|
|A1_hook|A1_hook_shank|box|4 x 8 x 30 mm|steel hardware/instrument surrogate|
|A1_hook|A1_hook_nose|box|10 x 2.5 x 4 mm|steel hardware/instrument surrogate|
|A1_hook|A1_hook_cap|box|12 x 11 x 3 mm|steel hardware/instrument surrogate|
|A0_catch|A0_catch_tab|box|12 x 12 x 14 mm|steel hardware/instrument surrogate|
|A0_catch|A0_catch_ridge|box|10 x 0.8 x 11 mm|steel hardware/instrument surrogate|
|A1_catch|A1_catch_tab|box|12 x 12 x 14 mm|steel hardware/instrument surrogate|
|A1_catch|A1_catch_ridge|box|10 x 0.8 x 11 mm|steel hardware/instrument surrogate|
|inst_A|A_inst|cylinder|radius 20 mm; cylinder height 160 mm|steel hardware/instrument surrogate|
|inst_A|A_inst_cap|cylinder|radius 21 mm; cylinder height 12 mm|steel hardware/instrument surrogate|
|case_B|B_floor|box|240 x 160 x 6 mm|rigid plastic shell|
|case_B|B_wall_f|box|240 x 6 x 180 mm|rigid plastic shell|
|case_B|B_wall_b|box|240 x 6 x 180 mm|rigid plastic shell|
|case_B|B_wall_l|box|6 x 148 x 180 mm|rigid plastic shell|
|case_B|B_wall_r|box|6 x 148 x 180 mm|rigid plastic shell|
|case_B|B0_base|box|26 x 4 x 32 mm|steel hardware/instrument surrogate|
|case_B|B0_bracket|box|24 x 10 x 10 mm|steel hardware/instrument surrogate|
|case_B|B1_base|box|26 x 4 x 32 mm|steel hardware/instrument surrogate|
|case_B|B1_bracket|box|24 x 10 x 10 mm|steel hardware/instrument surrogate|
|lid_B|B_lid_top|box|240 x 160 x 6 mm|rigid plastic shell|
|lid_B|B_lid_f|box|240 x 6 x 40 mm|rigid plastic shell|
|lid_B|B_lid_b|box|240 x 6 x 40 mm|rigid plastic shell|
|lid_B|B_lid_l|box|6 x 148 x 40 mm|rigid plastic shell|
|lid_B|B_lid_r|box|6 x 148 x 40 mm|rigid plastic shell|
|lid_B|B_lip|box|44 x 24 x 4 mm|rigid plastic shell|
|lid_B|B0_keeper|box|24 x 4 x 28 mm|steel hardware/instrument surrogate|
|lid_B|B0_bar0|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_B|B0_bar1|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_B|B0_bar2|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_B|B1_keeper|box|24 x 4 x 28 mm|steel hardware/instrument surrogate|
|lid_B|B1_bar0|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_B|B1_bar1|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|lid_B|B1_bar2|cylinder|radius 2.5 mm; cylinder height 18 mm|steel hardware/instrument surrogate|
|B0_lever|B0_lever_side0|box|2 x 8 x 62 mm|steel hardware/instrument surrogate|
|B0_lever|B0_lever_side1|box|2 x 8 x 62 mm|steel hardware/instrument surrogate|
|B0_lever|B0_lever_web|box|20 x 2 x 58 mm|steel hardware/instrument surrogate|
|B0_lever|B0_lever_tab|box|24 x 30 x 3 mm|steel hardware/instrument surrogate|
|B0_lever|B0_pivot|cylinder|radius 3 mm; cylinder height 22 mm|steel hardware/instrument surrogate|
|B0_link|B0_link_plate|box|4 x 6 x 33.4466 mm|steel hardware/instrument surrogate|
|B0_hook|B0_hook_shank|box|4 x 8 x 30 mm|steel hardware/instrument surrogate|
|B0_hook|B0_hook_nose|box|10 x 2.5 x 4 mm|steel hardware/instrument surrogate|
|B0_hook|B0_hook_cap|box|12 x 11 x 3 mm|steel hardware/instrument surrogate|
|B1_lever|B1_lever_side0|box|2 x 8 x 62 mm|steel hardware/instrument surrogate|
|B1_lever|B1_lever_side1|box|2 x 8 x 62 mm|steel hardware/instrument surrogate|
|B1_lever|B1_lever_web|box|20 x 2 x 58 mm|steel hardware/instrument surrogate|
|B1_lever|B1_lever_tab|box|24 x 30 x 3 mm|steel hardware/instrument surrogate|
|B1_lever|B1_pivot|cylinder|radius 3 mm; cylinder height 22 mm|steel hardware/instrument surrogate|
|B1_link|B1_link_plate|box|4 x 6 x 33.4466 mm|steel hardware/instrument surrogate|
|B1_hook|B1_hook_shank|box|4 x 8 x 30 mm|steel hardware/instrument surrogate|
|B1_hook|B1_hook_nose|box|10 x 2.5 x 4 mm|steel hardware/instrument surrogate|
|B1_hook|B1_hook_cap|box|12 x 11 x 3 mm|steel hardware/instrument surrogate|
|B0_catch|B0_catch_tab|box|12 x 12 x 14 mm|steel hardware/instrument surrogate|
|B0_catch|B0_catch_ridge|box|10 x 0.8 x 11 mm|steel hardware/instrument surrogate|
|B1_catch|B1_catch_tab|box|12 x 12 x 14 mm|steel hardware/instrument surrogate|
|B1_catch|B1_catch_ridge|box|10 x 0.8 x 11 mm|steel hardware/instrument surrogate|
|inst_B|B_inst|cylinder|radius 20 mm; cylinder height 160 mm|steel hardware/instrument surrogate|
|inst_B|B_inst_cap|cylinder|radius 21 mm; cylinder height 12 mm|steel hardware/instrument surrogate|
|world|plinth|box|200 x 600 x 60 mm|wood surrogate|
