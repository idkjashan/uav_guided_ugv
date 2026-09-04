# Free model sources for running this pack

Verified Sept 2026. Limits change monthly - re-check before relying on one.

## Orchestrator candidates (strong reasoning), ranked

| Source | Model(s) | Budget | Wiring | Notes |
|---|---|---|---|---|
| **Gemini CLI + Jio "Google AI Pro" (18 months free)** | Gemini flagship (Pro) | **120 req/min, 1,500 req/day** | Gemini CLI is its own agent harness (shell, MCP, `GEMINI.md` instructions). Run the orchestrator prompt inside it. | Jio SIM, 18+, active unlimited 5G plan >= Rs 349. Also gives $10/month Google Cloud credit (usable on Vertex AI Gemini API). |
| **Gemini CLI, plain personal Google account** | Gemini Pro/Flash | 60 req/min, 1,000 req/day | Same as above | No Jio needed. 20x the GLM budget. |
| **GitHub Copilot Student plan** (Student Developer Pack) | Included models unlimited (fair use); 300 premium req/month | Copilot CLI runs locally with shell + `AGENTS.md`/instructions | Frontier premium models are NOT hand-selectable on the student plan. Not an API - cannot be a DSH provider. |
| OpenRouter `z-ai/glm-5.2:free` | GLM 5.2 | 20 req/min, **50 req/day** (1,000/day after a one-time $10 top-up) | OpenAI-compatible: `https://openrouter.ai/api/v1`, model `z-ai/glm-5.2:free` | What the plans were budgeted for (<= 12 turns/plan). |
| Z.ai direct | GLM-4.7-Flash / 4.5-Flash free; daily trial quota on flagship | unpublished | OpenAI-compatible | Check quota in the Z.ai console. |

**Single-agent mode.** With Gemini CLI or Copilot CLI (1,000+ turns/day) you do not need two tiers: give the agent `orchestrator_prompt.md` + the plan, and let it execute the task cards itself, one at a time, still producing the WORKER REPORT after each card and keeping `STATE.md`. The card format is what keeps a strong model from improvising; the tiering was only ever about GLM's 50/day.

## Worker candidates (mechanical cards, high limits), ranked

| Source | Model(s) | Budget | Wiring |
|---|---|---|---|
| **Azure for Students** (institute Microsoft account) | DeepSeek V3.x, Llama 3.3 70B, Phi, (GPT if quota allows) via Azure AI Foundry | **$100/year credit, no card, renewable yearly while a student** = tens of millions of tokens at DeepSeek/Llama prices | OpenAI-compatible endpoint from the Foundry project. Azure OpenAI GPT deployments are sometimes quota-blocked on student subscriptions - deploy DeepSeek/Llama first. |
| **NVIDIA Build (build.nvidia.com)** | 100+ models: DeepSeek, Llama, Qwen, Nemotron | ~40 req/min, no card, free developer account | OpenAI-compatible `https://integrate.api.nvidia.com/v1` |
| Groq | Llama 3.3 70B, Llama 4 Scout, Qwen 3, GPT-OSS 120B | 30 req/min, 6K tok/min, 14,400 req/day, no card | OpenAI-compatible |
| Google AI Studio | Gemini 2.5 Flash / Flash-Lite | 15 req/min, 1,500 req/day, no card | OpenAI-compatible or Gemini SDK. Separate quota from Gemini CLI. |
| SambaNova Cloud | Llama, DeepSeek | free tier, high RPM, no card | OpenAI-compatible |
| Codestral (Mistral) | Codestral | 30 req/min free coding key | OpenAI-compatible; good for Plan 3 `tier: pro` coding cards |
| DeepSeek platform | V4 Flash / V4 Pro | 5M-token sign-up grant, then paid (V4 Flash $0.14/M in) | Native `dsh-crew` flash/pro tiers |
| Cloudflare Workers AI | small open models | 10,000 neurons/day | marginal; skip unless nothing else |

Keep the OpenRouter key OFF the worker side: `:free` models share one 50/day pool per account.

## Trials and near-zero-cost upgrades

| Offer | What it buys | Card? |
|---|---|---|
| OpenRouter one-time $10 top-up | 1,000/day on all `:free` models + $10 of paid tokens | yes |
| Google Cloud $300 / 90-day trial | Vertex AI Gemini Pro at full rate limits | yes (no auto-charge) |
| Jio AI Pro's $10/month Google Cloud credit | same, without a card | no |
| DeepSeek 5M-token grant | ~all the `tier: pro` cards in Plans 2-3 | no |
| Kaggle 30 GPU-h/week | self-host Qwen2.5-Coder-7B with vLLM + a tunnel as a private worker | no (fiddly; last resort) |

## Setup: Gemini CLI with the AI Pro accounts (on the Ubuntu machine)

The AI Pro limit (1,500/day) applies ONLY to "Sign in with Google" inside Gemini CLI. An AI Studio API key is a different, smaller quota (~250/day, Flash only) and does not get the subscription benefit.

```bash
# Node 20+ (Ubuntu 22.04's apt node is too old) - nvm is the standard route
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
source ~/.bashrc
nvm install 22
npm install -g @google/gemini-cli
gemini            # choose "Sign in with Google", pick the account that redeemed the Jio AI Pro offer
```

Check it took: `/about` shows the auth method and account. Personal Gmail accounts must NOT set GOOGLE_CLOUD_PROJECT (that path is for Workspace accounts).

Two AI Pro accounts at once = two independent credential stores. The CLI keeps credentials in `~/.gemini/oauth_creds.json`, resolved from `$HOME`, so give the second account its own home:

```bash
mkdir -p ~/gemini-b
HOME=~/gemini-b gemini      # sign in with the SECOND account; run this in its own terminal
```

Terminal 1 (`gemini`) = account A, terminal 2 (`HOME=~/gemini-b gemini`) = account B; both live, 3,000 requests/day combined. Known bug: `/auth` on an already-signed-in CLI logs the same account back in - to change accounts in place, quit, delete `~/.gemini/oauth_creds.json` (and `google_accounts.json` if present), start again. Alternative: the [Gemini-Account-Switcher](https://github.com/kranthik123/Gemini-Account-Switcher) extension saves profiles under `~/.gemini/accounts/<name>/` and switches (one active per CLI, restart after switching).

No browser (SSH into the laptop): `NO_BROWSER=true gemini` prints a URL to open elsewhere and asks for the code; this flow has had regressions - if it hangs, log in once at the laptop's own screen and the cached credential is reused headlessly.

## Recommended stack for this pack

1. Orchestrator: Gemini CLI (Jio AI Pro if eligible, else the plain free tier). Fallback: GLM 5.2 on OpenRouter.
2. Workers (flash): NVIDIA Build or Groq. Workers (pro): Azure for Students DeepSeek, or the DeepSeek sign-up grant.
3. Never mix roles on one key; log which key each report came from in `STATE.md`.
