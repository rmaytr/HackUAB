# AGENTS.md

Instructions for any AI coding agent (OpenAI Codex/ChatGPT, Cursor, Gemini, Copilot, Claude...).

## Read before doing anything, in this order

1. [CLAUDE.md](CLAUDE.md): the project, scoring, simulator API, harness rules, AWD vehicle, module contracts,
   folder ownership and team rules. It is the source of truth even though it's named for Claude.
2. [docs/STATUS.md](docs/STATUS.md): what's on `main` right now, the latest results, and the next tasks.
   **Start from its "next tasks" list.**
3. [docs/ROADMAP.md](docs/ROADMAP.md): timeline, who owns what, and the "how" for each task.
4. The code you're about to touch in `hackuab/`, plus `simulator/autocross/api.py` for the harness types.

Shared agent skills (markdown how-to guides) are in `.agents/skills/`.

## Hard rules

- Edit only the files your human owns (see the ownership table in CLAUDE.md). **Never edit `simulator/`**:
  it's a pinned git submodule.
- Work on a branch named `<owner>/<topic>` (e.g. `rodion/verifier`), never directly on `main`.
- Controller code in `hackuab/` is **numpy + Python stdlib only**: no file writes, network, subprocesses,
  `ctypes` or CARLA imports (the harness sandbox treats any of those as a controller error).
- Before you say you're done: run the checks in docs/STATUS.md ("How to verify any change"), report a results
  table vs. the STATUS.md table, **update docs/STATUS.md**, and commit + push your branch.

## Environment

- The simulator needs Python 3.10–3.12. Set it up once: `git submodule update --init --recursive`, then from
  `simulator/` run `scripts/setup.sh` (Linux/macOS) or `scripts\setup.ps1` (Windows). That creates `simulator/.venv`.
- Use the bicycle backend (`--backend bicycle`) for development. CARLA needs an NVIDIA GPU, and only Viraj runs it.
