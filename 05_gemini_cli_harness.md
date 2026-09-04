# Gemini CLI / Antigravity CLI as the harness (instead of, or alongside, DeepSeek Harness)

**Update:** Gemini CLI stopped serving individual accounts on 18 June 2026. Everything below applies to its successor **Antigravity CLI (`agy`)**, which keeps the same shape: `AGENTS.md` instructions, subagents in `~/.gemini/config/agents/<name>.md` (fields: name, description, model inherit|flash|pro, commandExecutionPolicy off|auto|eager|sandbox, tools, mcpServers), parent invokes them via `invoke_subagent`, concurrent, isolated context, result message back. Permissions: `/permissions` request-review | always-proceed | strict. MCP via `mcp_config.json`. Setup: `06_gemini_setup_ubuntu.md`. dsh-crew already ships an Antigravity MCP installer (`node src/install/cli.mjs agy`), so DeepSeek workers plug in natively if ever wanted.

Original comparison (Gemini CLI, Sept 2026) kept for reference:

## What Gemini CLI has

| Capability | Gemini CLI | DeepSeek Harness (dsh) |
|---|---|---|
| Agent loop with shell + file tools | yes (shell, read/write/edit, glob, grep, web fetch, web search, memory) | yes |
| Approval gates | `--approval-mode default|auto_edit|yolo|plan` | yes |
| Sandbox | Docker/Podman, gVisor (`GEMINI_SANDBOX=docker`, `-s`, settings `tools.sandbox`) | yes |
| Subagents with own prompt / tools / model / context | yes - `~/.gemini/agents/*.md` (experimental) | yes - presets + `spawnTeammate` |
| MCP servers | yes - `~/.gemini/settings.json` -> `mcpServers` | yes (plugins) |
| Headless / scriptable | `gemini -p "..." --output-format json` | yes (jsonrpc agent) |
| Project instructions file | `GEMINI.md` | preset persona |
| Other model providers (DeepSeek, Groq, OpenRouter...) | **no - Gemini only** | yes, any |
| Web UI / session hub | no | yes |

So: Gemini CLI is a full harness for ONE model family. It can be the orchestrator AND run workers as subagents on the same account's 1,500/day. It cannot natively call DeepSeek/Groq workers - for that, add dsh-crew as an MCP server (below).

## Mapping the pack onto Gemini CLI (single account, no DSH)

Step-by-step install and first run: see `06_gemini_setup_ubuntu.md`. The reference below is the same subagent definition.

Create `~/.gemini/agents/drdo-worker.md`:

```markdown
---
name: drdo-worker
description: Executes exactly one DRDO task card literally and returns a WORKER REPORT. Use for every task card.
kind: local
tools:
  - run_shell_command
  - read_file
  - write_file
  - replace
model: gemini-3-flash-preview
temperature: 0
max_turns: 15
timeout_mins: 30
---
<paste the full contents of worker_prompt.md here>
```

Then, in `~/uav_guided_ugv/setup`:

1. Copy `orchestrator_prompt.md` to `~/uav_guided_ugv/setup/GEMINI.md` (Gemini CLI loads it as project instructions).
2. `cd ~/uav_guided_ugv/setup && gemini -m pro`
3. First message: `Run Plan 1 from ~/uav_guided_ugv/plans/01_storage_cleanup_plan.md. Dispatch every task card to @drdo-worker with placeholders filled in. Keep ~/uav_guided_ugv/setup/reports/STATE.md updated. Ask me before any Tier-2 card.`

Notes:
- Check tool names with `/tools` inside the CLI before trusting the list above - they have been renamed before. `/mcp` lists MCP tools, `/about` shows the account.
- If `model:` is rejected, delete the line; the subagent inherits the parent model (costs the same quota).
- Keep `--approval-mode default` for Plans 1-2: the CLI will still prompt you on `sudo`/delete commands, which is a second safety net under the plan's own `[ASK USER]`. Never use `yolo` on Plan 1.
- Subagents cannot nest and return only their final result - exactly the WORKER REPORT contract.
- Subagents are experimental: if `@drdo-worker` is not recognised, check `settings.json` for `"experimental": {"enableAgents": false}`.
- `run_shell_command` takes `is_background: true` - use it for `gz sim`, `px4` and `MicroXRCEAgent` instead of `nohup ... &` so the CLI tracks the process.
- Hard allowlist for a worker: `"tools": {"core": ["run_shell_command(df)", "run_shell_command(du)", ...]}` in `settings.json` limits the shell to those command prefixes. Too strict for most cards, but useful for a sense-only worker. (`tools.exclude` is deprecated in favour of the Policy Engine.)

## Adding DSH workers to Gemini CLI (optional, unverified)

dsh-crew's Antigravity installer (`node src/install/cli.mjs agy`) writes an MCP entry to `~/.gemini/config/mcp_config.json`. Copy that server entry (`command`, `args`, `env`) into `~/.gemini/settings.json` under `"mcpServers"`, restart `gemini`, and `/mcp` should list `dsh_run_worker`, `dsh_spawn_worker`, `dsh_worker_status`, `dsh_worker_result`, `dsh_worker_cancel`, `dsh_worker_config`. The orchestrator can then dispatch cards with `tier: flash|pro` to DeepSeek workers. Gemini CLI is not one of dsh-crew's documented hosts - test with one harmless card (a `sense` card) first.

## Two AI Pro accounts

Terminal A: `gemini` (account A = orchestrator + its subagents, 1,500/day). Terminal B: `HOME=~/gemini-b gemini` (account B) as the reserve when A's quota runs out, or as a second worker pool fed through `~/uav_guided_ugv/setup/queue/` files. See `04_free_model_sources.md` for login details.
