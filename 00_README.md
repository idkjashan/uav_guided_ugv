# DRDO UAV-guided UGV plan pack

## Files
- 00_README.md: wiring, budget, start, resume. orchestrator_prompt.md, worker_prompt.md: system prompts.
- 01_storage_cleanup_plan.md: Plan 1, 01_storage_report.md. 02_setup_and_repo_fixes_plan.md: Plan 2, 02_setup_report.md. 03_ps_solution_plan.md: Plan 3, ~/drdo_ws.
- 04_free_model_sources.md: free endpoints per role. 05_gemini_cli_harness.md: harness comparison. 06_gemini_setup_ubuntu.md: install + first run (START HERE). Reports, STATE.md: ~/drdo_setup/reports/

## Protocol
ORCHESTRATOR (reasoning model, never runs commands) reads per turn orchestrator_prompt.md, the plan's "## Orchestrator" section, STATE, new reports; updates STATE, picks the next tree node, dispatches filled TASK CARDS (independent ones batched), batches user questions (sudo, GUI, decisions). WORKER (small model, persistent shell on Ubuntu) sees worker_prompt.md plus ONE card, runs it literally, stops at the first "Stop if", returns ONE WORKER REPORT, never decides or asks (NEEDS_USER). 12 orchestrator turns per plan maximum.

## Formats
TASK CARD:
```
### T<n> <short title>  |  tier: flash|pro  |  needs: none|sudo|gui
Goal: one line.
Inputs: {{PLACEHOLDER}} tokens the orchestrator fills before dispatch, or "none".
Run: a fenced bash block, copy-pasteable, no leading $, one command per line, in order.
Expect: one line - what success output looks like.
Stop if: one line - when the worker must stop and return BLOCKED.
Return: extra key: value fields the worker must report (beyond the standard ones).
```
Delete/remove/overwrite cards: needs: sudo, a dry-run line before the real line, a Stop if for the dangerous case. Cards hold no decision; branches live in the orchestrator's tree.
WORKER REPORT:
```
card: T<n>
status: DONE | FAILED | BLOCKED | NEEDS_USER
result: one line
values: the key: value lines the card asked for
raw: last 20 lines of the most relevant command output, in a fenced block
```
STATE (top of ~/drdo_setup/reports/STATE.md, under 40 lines):
```
plan: 1|2|3
node: current decision-tree node id
facts: key: value lines gathered so far
done: T1 T3 ...
pending: T4 T5
blocked: card: reason
ask_user: open questions
```

## Wiring (chosen): Gemini CLI with Google AI Pro
- Orchestrator: `gemini -m pro` started in `~/drdo_setup`; `orchestrator_prompt.md` copied to `~/drdo_setup/GEMINI.md`. It writes STATE.md and reports itself (write_file) but runs no shell commands.
- Worker: subagent `~/.gemini/agents/drdo-worker.md` = YAML header (tools run_shell_command/read_file/write_file/replace/list_directory, flash-class model, temperature 0) + worker_prompt.md as body. Install per 06_gemini_setup_ubuntu.md.
- Dispatch: the orchestrator sends `@drdo-worker` followed by ONE complete filled card; the subagent returns the WORKER REPORT. One card at a time (parallel subagents unverified). `tier: pro` cards: same subagent, or the orchestrator runs the card itself if the worker fails twice.
- Approval: `--approval-mode default` (CLI prompts before sudo/delete). No Docker sandbox for Plans 1-2 (they must touch /opt/ros, apt, ~/PX4-Autopilot).
- sudo: type `!sudo -v` in the CLI (shell passthrough) before a needs: sudo card; it caches ~15 min for that terminal.

## Wiring (alternative): any OpenAI-compatible endpoint
- Orchestrator call: system = orchestrator_prompt.md; user = plan "## Orchestrator" section, next cards, STATE.md, new reports, user answers; no tools.
- Worker call: system = worker_prompt.md; user = one filled card; shell tool on Ubuntu, cwd ~. DeepSeek Harness dsh-crew fits here (tier flash|pro = card tier).

## Budget
- Gemini CLI: 1,500 requests/day per AI Pro account (1,000 on a plain account); second account = reserve (`HOME=~/gemini-b gemini`). No hard turn cap; still batch user questions and keep STATE under 40 lines.
- Fallback orchestrator: OpenRouter `z-ai/glm-5.2:free`, 50 requests/day - then the turn budgets in the plan headers apply.

## Start
1. `mkdir -p ~/drdo_setup/reports`
2. Plan 1, 2, 3 in order; first turn STATE: `plan: N`, `node: start`, facts from the previous report.
3. Before a needs: sudo card type `!sudo -v` in the CLI (cached about 15 min); passwords never enter chat, cards, reports, STATE.md.
4. Each turn: the orchestrator fills the DISPATCH card from the plan, sends it to `@drdo-worker`, saves the STATE block to STATE.md and the report into reports/ (Gemini CLI: it does this itself with write_file; other wiring: your driver script).

## Resume
- Read STATE.md; continue at `node`; never re-dispatch `done`.
- `pending`: ask the user whether it started; deleting/removing/resizing/appending cards only after a check card (ls, dpkg -l, lvs) shows no change yet, plus a fresh yes.
- No STATE.md but reports: rebuild facts and done from the newest; nothing: `node: start`.
