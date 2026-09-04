# Gemini CLI setup on Ubuntu 22.04 for the DRDO plan pack

Do this once, on the laptop's own screen (not over SSH). ~20 minutes.

## 1. Get the pack onto Ubuntu

Copy the `drdo_plans` folder to `~/drdo_plans` (OneDrive web download, USB, or `git clone` if you push it to a repo). Then:

```bash
mkdir -p ~/drdo_setup/reports ~/drdo_setup/logs ~/.gemini/agents
ls ~/drdo_plans
```

Expect: `00_README.md 01_... 02_... 03_... orchestrator_prompt.md worker_prompt.md` plus the 04/05/06 files.

## 2. Node 22 and Gemini CLI

Ubuntu's apt Node is 12 - too old. Use nvm:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
```

```bash
source ~/.bashrc && nvm install 22 && node -v
```

Expect: `v22.x`.

```bash
npm install -g @google/gemini-cli && gemini --version
```

## 3. Sign in with account A (the Jio AI Pro one)

```bash
cd ~/drdo_setup && gemini
```

- Choose **Sign in with Google**. A browser opens; pick the Gmail that redeemed the Jio offer; approve.
- Back in the CLI type `/about` - it should show the auth method and that account.
- Do NOT set `GOOGLE_CLOUD_PROJECT`; that is for Workspace accounts.
- `/quit`.

If the browser does not open: `NO_BROWSER=true gemini` prints a URL to open on any device and asks for the code. (This flow has had bugs; the normal browser flow is preferred.)

## 4. Install the worker subagent

Create `~/.gemini/agents/drdo-worker.md` with exactly this header, then paste the whole of `~/drdo_plans/worker_prompt.md` below the second `---`:

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
  - list_directory
model: gemini-3-flash-preview
temperature: 0
max_turns: 15
timeout_mins: 30
---
```

One command does it:

```bash
{ cat <<'EOF'
---
name: drdo-worker
description: Executes exactly one DRDO task card literally and returns a WORKER REPORT. Use for every task card.
kind: local
tools:
  - run_shell_command
  - read_file
  - write_file
  - replace
  - list_directory
model: gemini-3-flash-preview
temperature: 0
max_turns: 15
timeout_mins: 30
---
EOF
cat ~/drdo_plans/worker_prompt.md; } > ~/.gemini/agents/drdo-worker.md
```

## 5. Make the orchestrator prompt the project instructions

```bash
cp ~/drdo_plans/orchestrator_prompt.md ~/drdo_setup/GEMINI.md
```

Gemini CLI loads `GEMINI.md` from the directory you start it in.

## 6. Settings

`~/.gemini/settings.json` - create if missing:

```json
{
  "experimental": { "enableAgents": true }
}
```

Do NOT enable the Docker sandbox for Plans 1 and 2: it confines writes to the project folder, and those plans must touch `/opt/ros`, apt, `~/PX4-Autopilot`. Rely on the approval prompts instead (below). The sandbox is fine for Plan 3 coding cards if you want it.

## 7. Smoke test

```bash
cd ~/drdo_setup && gemini -m pro
```

Inside the CLI:

- `/tools` - confirm `run_shell_command`, `read_file`, `write_file`, `replace` exist (names have changed before; fix the subagent file if they differ).
- `@drdo-worker run: df -h and return a WORKER REPORT with card T0` - you should get back the report format from worker_prompt.md. If `@drdo-worker` is not recognised, re-check step 4 and 6.
- If the CLI complains about `model:` in the subagent file, delete that line (the worker then uses the parent model).

## 8. Run Plan 1

Still in `gemini -m pro`, with `--approval-mode` left at `default` (the CLI will prompt you before sudo/delete commands - say yes only when the card and the report make sense):

```
Run Plan 1 from ~/drdo_plans/01_storage_cleanup_plan.md. Dispatch every task card to @drdo-worker with placeholders filled in, one card at a time. Keep ~/drdo_setup/reports/STATE.md updated after every report. Ask me before any Tier-2 card. Start with the sense cards.
```

Then Plan 2, then Plan 3, the same way. Resume after a crash: start `gemini` in `~/drdo_setup`, say `Read ~/drdo_setup/reports/STATE.md and continue Plan N`.

## 9. Account B (reserve)

Only when account A returns quota errors:

```bash
mkdir -p ~/gemini-b/.gemini && cp -r ~/.gemini/agents ~/.gemini/settings.json ~/gemini-b/.gemini/
HOME=~/gemini-b gemini
```

Sign in with the second Jio account. Run it from `~/drdo_setup` too - it sees the same `GEMINI.md` and `STATE.md`. Do not run both accounts on the same plan at the same time; they would race on STATE.md.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `/auth` keeps logging the same account | quit; `rm ~/.gemini/oauth_creds.json` (and `google_accounts.json` if present); start again |
| Worker times out on `make px4_sitl` | that card must run the build with `is_background: true` / `nohup` and a later card reads the log - check the card, do not raise the timeout to an hour |
| Worker "improves" a command | it must not; paste the WORKER REPORT rules back and re-dispatch; if it repeats, lower `max_turns` to 10 |
| 429 / quota exhausted | switch to account B (step 9) or wait for the daily reset |
| Plan says `[ASK USER]` but nothing was asked | the orchestrator skipped a rule - tell it: `You must ask me before Tier-2 cards. Show STATE.md.` |
