# Antigravity CLI setup on Ubuntu 22.04 for the DRDO plan pack

Gemini CLI stopped serving individual (free / AI Pro / AI Ultra) accounts on 18 June 2026. Its replacement is Antigravity CLI (`agy`). Your Jio Google AI Pro is a normal AI Pro subscription on that Google account and gets Antigravity's Pro tier. Do this once, at the laptop's own screen. ~15 minutes.

## 1. Get the pack onto Ubuntu

Copy this whole `uav_guided_ugv` folder (the one holding these .md files) to `~/uav_guided_ugv/plans` on Ubuntu (OneDrive web, USB, or git). Everything the project creates lives beside it: `~/uav_guided_ugv/setup` (env.sh, bringup.sh, reports, logs) and `~/uav_guided_ugv/ws` (the ROS workspace). Then:

```bash
mkdir -p ~/uav_guided_ugv/setup/reports ~/uav_guided_ugv/setup/logs ~/.gemini/config/agents
ls ~/uav_guided_ugv/plans
```

## 2. Install Antigravity CLI

```bash
curl -fsSL https://antigravity.google/cli/install.sh | bash
```

```bash
source ~/.bashrc; agy --version
```

## 3. Sign in with account A (the Jio AI Pro Gmail)

```bash
cd ~/uav_guided_ugv/setup && agy
```

A browser opens; pick the Gmail that redeemed the Jio offer; approve. Credentials go into the system keyring (Secret Service on Ubuntu), so it stays signed in. Inside the CLI: `/usage` shows "Five Hour Limit Remaining" and "Weekly Limit Remaining" - if you see both, the Pro tier is active. `/model` picks the default reasoning model: choose a Gemini Pro model for the orchestrator. `/quit`.

Over SSH with no screen it prints a URL + code instead of opening a browser.

## 4. Install the worker subagent

Antigravity discovers subagents in `~/.gemini/config/agents/<name>.md` (global) or `.agents/agents/<name>.md` (per workspace). Build it from worker_prompt.md:

```bash
{ cat <<'EOF'
---
name: drdo-worker
description: Executes exactly ONE DRDO task card literally (shell commands in order, stop at Stop if) and returns ONE WORKER REPORT. Use for every task card the orchestrator dispatches.
model: flash
subagent: true
mainAgent: false
commandExecutionPolicy: auto
---
EOF
cat ~/uav_guided_ugv/plans/worker_prompt.md; } > ~/.gemini/config/agents/drdo-worker.md
```

`model: flash` keeps worker turns cheap against the weekly cap. `commandExecutionPolicy: auto` lets it run shell commands but the CLI's permission mode (step 6) still gates sudo/delete. If `agy` rejects a field, remove that line; only `name` and `description` are required.

## 5. Orchestrator instructions

Antigravity reads `AGENTS.md` from the directory you start it in:

```bash
cp ~/uav_guided_ugv/plans/orchestrator_prompt.md ~/uav_guided_ugv/setup/AGENTS.md
```

## 6. Permissions

Inside `agy`, run `/permissions` and choose **request-review** (asks before each command). Do NOT use always-proceed or `--dangerously-skip-permissions` on Plans 1-2. Optional hard denies in `~/.gemini/antigravity-cli/settings.json`:

```json
{ "permissions": { "deny": ["command(rm -rf /)", "command(mkfs)", "command(dd )", "command(parted)", "command(gparted)"] } }
```

Do not enable a sandbox for Plans 1-2 (they must touch /opt/ros, apt, ~/PX4-Autopilot).

## 7. Smoke test

```bash
cd ~/uav_guided_ugv/setup && agy
```

- `/agents` - confirm `drdo-worker` is listed.
- Say: `Use the drdo-worker subagent to run this card and return only its WORKER REPORT:` then paste a trivial card (`### T0 test | tier: flash | needs: none`, Run: `df -h /`, Expect: one / row, Return: free_gb). You should get the report format from worker_prompt.md back.
- `/usage` to see what that cost.

## 8. Run Plan 1

Still in `agy` (request-review on):

```
Read ~/uav_guided_ugv/plans/01_storage_cleanup_plan.md. Run Plan 1: dispatch every task card to the drdo-worker subagent with placeholders filled in, one card at a time, and keep ~/uav_guided_ugv/setup/reports/STATE.md updated after each report. Ask me before any Tier-2 card. Before a needs: sudo card, ask me to run `sudo -v` in this terminal. Start with the sense cards.
```

Then Plan 2, then Plan 3 the same way. Resume after a crash: `cd ~/uav_guided_ugv/setup && agy`, then `Read ~/uav_guided_ugv/setup/reports/STATE.md and continue Plan N`.

## 9. Account B (reserve, separate weekly bucket)

Credentials live in the OS keyring, so the clean way to hold a second account is a second Linux user:

```bash
sudo adduser drdo2 && sudo usermod -aG sudo drdo2
```

Log in as `drdo2` (or `su - drdo2` in a terminal), install `agy` again (step 2), sign in with the second Jio Gmail, and repeat steps 4-6 there. Give it read access to the shared folders: keep `~/uav_guided_ugv/setup` and the workspaces under the first user and run account B only for Plan 3 coding cards, or when account A shows 0% weekly. Simpler fallback: in account A's CLI run `/logout`, restart `agy`, sign in as B, `/logout` again later - one account at a time, no second Linux user.

## Quota reality

Pro has a 5-hour window and a weekly cap; users report multi-day lockouts after heavy sessions. Habits that matter: worker on `flash`; one card at a time; never `-a`-style "do everything" prompts; check `/usage` before a long PX4-build day; switch to account B at ~20% weekly remaining, not 0%.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Browser never opens at sign-in | copy the printed URL to any browser, paste the code back |
| `/agents` does not list drdo-worker | check the file path (`~/.gemini/config/agents/drdo-worker.md`) and that frontmatter has `name` + `description`; remove unknown fields |
| Worker asks the user in prose instead of NEEDS_USER | re-send: "Return only the WORKER REPORT format"; if it repeats, put `Return only the WORKER REPORT` at the top of the card |
| Worker times out on `make px4_sitl` | the card must background the build with a log; a later card reads the log |
| Quota exhausted mid-plan | STATE.md is on disk; switch account (step 9) and say `continue Plan N from STATE.md` |
| Old Gemini CLI still installed | `npm uninstall -g @google/gemini-cli`; it no longer serves personal accounts |
