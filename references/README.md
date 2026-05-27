# Gripper Refactor References

This folder is the curated input set for the next gripper-control refactor. The
full pre-refactor workspace is moved to `old/`; this folder keeps the files and
notes that should be read first so requirements and hardware knowledge are not
lost.

## Layout

- `00_context/`: handoff notes, workspace inventory, and current verified state.
- `01_requirements/`: user-facing requirement and design documents.
- `02_mechanics/`: mechanical calculations, diagrams, motion curves, and scripts.
- `03_control/`: verified UI/control implementation and source-code snapshot.
- `04_hardware/`: Damiao motor, USB2FDCAN, DeviceSDK, and protocol references.
- `05_build_test/`: build, package, and test notes from the current prototype.
- `06_implementation/`: implementation rollout plans, sub-agent work packets,
  and review rules for the refactor.
- `07_ui/`: Web UI test guide and operator-facing UI notes.
- `07_standards/`: project coding, naming, unit, configuration, and dependency
  rules.
- `08_review/`: human design review checklist, review scope, and review pack.
- `archive/`: legacy design snapshots kept for traceability after V2
  architecture baseline changes.

## Root Layout After Archiving

The repository root is intentionally reduced to:

- `.venv/`: existing Python environment.
- `dlls/`: runtime DLLs used by the current Windows UI prototype.
- `references/`: curated refactor references.
- `old/`: full previous workspace, including original assets, build outputs,
  source tree, tests, and discussion files.

`dlls/` is kept at the root for this archive because the final root does not keep
`src/`. During the new project layout, it is reasonable to move or copy these
DLLs into a clearer runtime dependency location such as `src/libs/`,
`third_party/damiao/bin/`, or an installer/package resource directory.

## Read First

1. `00_context/2026-05-15_context_handoff.md`
2. `00_context/current_workspace_inventory.md`
3. `01_requirements/夹爪控制系统设计.md`
4. `01_requirements/夹爪控制系统联调归档.md`
5. `03_control/gripper_control_architecture_v2.md`
6. `06_implementation/v2_incremental_implementation_plan.md`
7. `06_implementation/sub_agent_execution_plan.md`
8. `07_standards/project_coding_standards.md`
9. `03_control/verified_ui/gripper_control_ui.py`
10. `04_hardware/protocol_refs/dmcan.h`

## Control Refactor Notes

- `03_control/gripper_control_architecture_v2.md` is the active control-system
  architecture baseline. It keeps the state-machine approach, redefines the
  encoder foundation as a virtual nut-position encoder, and adds explicit
  debug-constraint policy.
- `06_implementation/v2_incremental_implementation_plan.md` is the active
  implementation plan. It requires design, placeholder/interface, implementation,
  and test to proceed one function at a time.
- `archive/2026-05-22_control_architecture_design_v1_legacy.md` and
  `archive/2026-05-22_controller_interface_first_implementation_plan_v1_legacy.md`
  preserve the V1 design and implementation plan as historical references.
- `03_control/control_architecture_design.md` and
  `06_implementation/controller_interface_first_implementation_plan.md` remain
  in place for immediate traceability, but new control-system work should use
  the V2 files above unless the user explicitly asks to inspect V1 history.
- `06_implementation/sub_agent_execution_plan.md` defines how to split work
  across sub agents without losing requirements or creating overlapping write
  scopes.
- `06_implementation/full_implementation_pass_2026-05-16.md` records the first
  full implementation pass, verified build commands, current behavior boundary,
  review fixes, remaining hardware bring-up tasks, and the 2026-05-20
  low-energy `MotorBringup` real-hardware step-test entry.
- `06_implementation/control_function_completion_2026-05-20.md` records the
  controller completion pass for PreSelfCheck, homing, travel learning, motion
  health check, target-force clamp, release, UI gating, and the workflow test.
- `06_implementation/pre_self_check_implementation_2026-05-20.md` records the
  PreSelfCheck implementation aligned with section 10 of the control design,
  including limited-speed/current probing, preliminary limit search, progress
  logs, and verification results.
- `06_implementation/pre_self_check_final_archive_2026-05-26.md` records the
  final PreSelfCheck/lead-screw self-learning design after the 2026-05-22 to
  2026-05-26 real-hardware tuning loop, including PreA/PreB behavior,
  friction-anomaly mapping, downstream homing/travel/health/clamp usage,
  research/open-source references, and a reusable control-library plan.
- `03_control/generic_dual_hard_stop_axis_control_library_architecture.md`
  defines the proposed reusable two-hard-stop linear-axis control library
  architecture, module boundaries, core data structures, application bindings,
  and migration route from the current gripper project.
- `04_hardware/hardware_reference_notes.md` records the Damiao hardware path
  and the current console command sequence for real hardware step testing.
- `04_hardware/cpp_bringup_can_probe_update_2026-05-20.md` records the
  C++ `bringup_can_probe` command ported from the verified Python UI and how to
  interpret raw TX/RX results before enable or jog.
- `04_hardware/real_hardware_test_issue_log_2026-05-20.md` records the first
  C++ real-hardware test problems, fixes, residual feedback timeout, and next
  bench checks.
- `06_implementation/clamp_speed_mode_update_2026-05-16.md` records the force
  clamp speed-mode interface update and the current nonlinear kinematics
  boundary.
- `05_build_test/current_build_requirements.md` records the current CMake preset
  and PowerShell script based build/test requirements for the active C++
  refactor.
- `07_ui/web_ui_test_guide_2026-05-20.md` records how to start the browser UI,
  the suggested low-energy manual test order, and current verification status.
- `07_standards/project_coding_standards.md` defines naming, physical-unit,
  configuration-source, dependency, and human-review-friendly comment rules for
  implementation work.
- `08_review/human_design_review_pack.md` defines the current manual design
  review scope, checklist, required reading order, and conclusion template.
- `08_review/ai_preliminary_review_2026-05-16.md` records the first AI
  design/code review findings, fixes, residual risks, and manual review focus.
- `08_review/stage1_architecture_interface_review_2026-05-19.md` records the
  current architecture/interface review conclusion and why no additional broad
  tests are required before the next PreSelfCheck implementation stage.
- `08_review/source_comment_improvement_tasks.md` lists source files that need
  stronger comments before a deep manual review.
- `03_control/verified_ui/gripper_control_ui.py` remains the behavioral hardware
  bring-up reference, but its monolithic structure should not be copied directly
  into the new architecture.

## Important State

The Python UI prototype has already reached basic hardware bring-up with the
DM-USB2FDCAN_Dual adapter and DM-J4310P-2EC motor. It should be treated as the
behavioral reference for the next implementation, even though it is currently
monolithic and should be split into hardware abstraction, protocol, controller,
safety, and UI modules during refactor.
