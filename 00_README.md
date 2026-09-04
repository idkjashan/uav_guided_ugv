# DRDO UAV-guided UGV plan pack

## Files
- 00_README.md: wiring, budget, start, resume. orchestrator_prompt.md, worker_prompt.md: system prompts.
- 01_storage_cleanup_plan.md: Plan 1, 01_storage_report.md. 02_setup_and_repo_fixes_plan.md: Plan 2, 02_setup_report.md. 03_ps_solution_plan.md: Plan 3, ~/uav_guided_ugv/ws.
- 04_free_model_sources.md: free endpoints per role. 05_gemini_cli_harness.md: harness comparison. 06_gemini_setup_ubuntu.md: install + first run (START HERE). Reports, STATE.md: ~/uav_guided_ugv/setup/reports/

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
STATE (top of ~/uav_guided_ugv/setup/reports/STATE.md, under 40 lines):
```
plan: 1|2|3
node: current decision-tree node id
facts: key: value lines gathered so far
done: T1 T3 ...
pending: T4 T5
blocked: card: reason
ask_user: open questions
```

## Wiring (chosen): Antigravity CLI (`agy`) with Google AI Pro
- Gemini CLI no longer serves individual accounts (since 18 June 2026); Antigravity CLI is its successor and takes the same AI Pro account.
- Orchestrator: `agy` started in `~/uav_guided_ugv/setup` with a Gemini Pro model (`/model`); `orchestrator_prompt.md` copied to `~/uav_guided_ugv/setup/AGENTS.md`. It writes STATE.md and reports itself but runs no shell commands.
- Worker: subagent `~/.gemini/config/agents/drdo-worker.md` = YAML header (name, description, model: flash, subagent: true, commandExecutionPolicy: auto) + worker_prompt.md as body. Install per 06_gemini_setup_ubuntu.md.
- Dispatch: the orchestrator invokes the `drdo-worker` subagent with ONE complete filled card; the subagent returns the WORKER REPORT. One card at a time. `tier: pro` cards: the orchestrator may run the card itself if the worker fails twice.
- Permissions: `/permissions` = request-review (CLI asks before each command). No sandbox for Plans 1-2.
- sudo: before a needs: sudo card the orchestrator asks you to run `sudo -v` in the same terminal (caches ~15 min).

## Wiring (alternative): any OpenAI-compatible endpoint
- Orchestrator call: system = orchestrator_prompt.md; user = plan "## Orchestrator" section, next cards, STATE.md, new reports, user answers; no tools.
- Worker call: system = worker_prompt.md; user = one filled card; shell tool on Ubuntu, cwd ~. DeepSeek Harness dsh-crew fits here (tier flash|pro = card tier).

## Budget
- Antigravity AI Pro: a 5-hour window plus a WEEKLY cap (not per-day); heavy sessions can lock out for days. Worker on `flash`, one card at a time, check `/usage`; second account = separate weekly bucket (06, step 9). Batch user questions; keep STATE under 40 lines.
- Fallback orchestrator: OpenRouter `z-ai/glm-5.2:free`, 50 requests/day - then the turn budgets in the plan headers apply.

## Start
1. `mkdir -p ~/uav_guided_ugv/setup/reports`
2. Plan 1, 2, 3 in order; first turn STATE: `plan: N`, `node: start`, facts from the previous report.
3. Before a needs: sudo card run `sudo -v` in the terminal when the orchestrator asks (cached about 15 min); passwords never enter chat, cards, reports, STATE.md.
4. Each turn: the orchestrator fills the DISPATCH card from the plan, invokes the `drdo-worker` subagent with it, saves the STATE block to STATE.md and the report into reports/ (Antigravity: it does this itself; other wiring: your driver script).

## Resume
- Read STATE.md; continue at `node`; never re-dispatch `done`.
- `pending`: ask the user whether it started; deleting/removing/resizing/appending cards only after a check card (ls, dpkg -l, lvs) shows no change yet, plus a fresh yes.
- No STATE.md but reports: rebuild facts and done from the newest; nothing: `node: start`.
