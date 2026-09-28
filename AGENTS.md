# AGENTS.md — Instructions for AI Agents Working in This Repo

## Documentation is mandatory

Every code change MUST come with proper documentation. If you write or
modify code and leave it undocumented, the change is incomplete.

### Module level (top of every `.py` file)

- Start with a **module docstring** (`"""..."""`) describing:
  - what the module does, in 1–3 sentences,
  - the domain/task tag it belongs to (e.g. `Domain D1 — Pose & Sensor
    Clean`, `TASK-xx` markers used in this codebase),
  - design notes or invariants worth knowing before reading the code.

### Functions and methods

Every public function/method gets a docstring with:

```python
def foo(a: int, b: str) -> bool:
    """One-line summary of what it does.

    Longer description / why it exists (optional but encouraged for
    non-obvious logic).

    Args:
        a: What `a` means and its units.
        b: What `b` means.

    Returns:
        What the return value means (include edge cases: None, -1, ...).

    Algorithm:
        1. Numbered steps of the implementation when the logic is more
           than a couple of lines.
    """
```

- Private helpers (leading `_`) still get at least a short docstring;
  include `Args:`/`Returns:` for anything non-trivial.
- Classes: docstring with a summary, then an `Attributes:` section
  listing every field and what it represents (see `RobotContext` in
  `swarm_controller.py`).

### Comments inside code

- **Preserve all existing comments**, especially:
  - `TASK-01` … `TASK-12` markers,
  - `FIX:` / `bugfix:` comments,
  - Hindi/Roman-Urdu author comments — do not translate or delete them.
- Add inline comments only for non-obvious logic (why, not what).
- Never delete a comment because code moved; move it with the code.

### What documentation must reflect

- Docstrings/comments must match **current behavior**, not historical
  behavior. When you change logic (new parameter, new branch, changed
  threshold, new state), update the module docstring, class/function
  docstrings, and any `TASK-xx` comments that describe it.

## Code conventions in this project

- Python 3.10+, type hints on all new signatures.
- No new third-party dependencies without updating `requirements.txt`.
- Comments/docstrings never alter behavior — keep them truthful and in
  the same language style as the surrounding file.
- Existing structure: single-robot brain = `main_controller.py`,
  dual-robot brain = `swarm_controller.py`, shared logic = `utils/`.
- Firmware lives in `esp32/` (Arduino `.ino` + `config.h`); keep Python
  telemetry key names in sync with firmware JSON fields.

## Before finishing a task

1. Syntax-check every file you touched:
   `python -m py_compile <file1.py> <file2.py> ...`
2. If `human_detector_robot/tests/test_navigation_tasks.py` applies, run
   it with pytest from `human_detector_robot/`.
3. Re-read your diff: for every changed line, confirm the corresponding
   documentation was updated too.
