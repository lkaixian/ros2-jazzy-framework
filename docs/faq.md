# FAQ

**Do I need a GPU?** No for building, the tutorial, the validator, the rule generator and CI. A GPU helps the LLM (Gemma E2B runs on a 4 GB GTX 1650) and SLM training (needs about 4 GB with Adafactor and gradient checkpointing). The SLM itself runs on CPU.

**Do I need an LLM to try it?** No. Publish a task spec to `bt/task_spec` yourself ([Tutorial](tutorial-first-plan.md)) or submit XML with `submit_bt`. The LLM only turns language into a spec.

**Why two models (LLM and SLM)?** The LLM understands open language but is slow and approximate. The rule generator renders a spec exactly in 0.06 ms; the 230M SLM covers retries and specs the rules cannot render. Both are checked by the same validator.

**Can the LLM make the robot do something dangerous?** It can only choose from catalog nodes and whitelisted macros (a JSON schema enforces it), values are range-checked by the validator and the extensions, motion passes the mux (clamps, watchdog, latch), and the executor is dry-run by default. It is still a software layer: keep a hardware e-stop within reach.

**Why is my plan rejected?** Read `BtPlan.errors`; every rule is in [Validator rules](validator-rules.md). The same message is what the LLM sees on a retry.

**How do I add a robot skill?** [Writing an extension](writing-an-extension.md): a YAML manifest and a handler. Add a macro if the LLM should use it.

**How do I change what the robot says it can do?** Edit descriptions in the manifests, macros in `param/macros.yaml`, and examples in `param/llm_knowledge.md`; re-run [Evaluation](evaluation.md).

**How do I use my own object detector?** [Perception](perception.md): train with `train_detector`, set `object_detection_node.model`; the validator then rejects labels the detector cannot produce.

**Does it work on Humble?** It builds and the unit tests pass in a `ros:humble` container (CI job `build-test-humble`). Integration sims run on Jazzy only, and nothing was run on a real Humble machine. Do not mix Jazzy and Humble machines on one DDS domain.

**Can I use several computers?** Yes, in simulation: [Cluster](cluster.md) schedules jobs and fails them over; [Workspaces and security](workspaces-security.md) secures the link. Untested on a real second machine or WiFi.

**Is the SLM or the dataset public?** The fine-tuned SLM is on Hugging Face as `larmkaixian/bt-sim-slm`, and a dataset as `larmkaixian/bt-sim-dataset`. Both cards are neutral.

**Which robot is this for?** Developed for a Kobuki base with an RPLIDAR A1, a RealSense D435 and a Dynamixel arm on OpenCR (arm driver undecided; see [Status](status.md)). The pieces are generic ROS 2 and can be adapted.

**What is not done?** Real robot bring-up, a real arm, real microphone audio, a second machine. See [Status](status.md).

**Where do I start contributing?** Read [Testing and CI](testing-and-ci.md), then `notes/BACKLOG.md` in the repository for the open items.
