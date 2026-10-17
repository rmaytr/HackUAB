# CLAUDE.md — HackBama_01 team project

Read this first. Three of us run AI coding agents in parallel on separate branches,
so these rules exist to keep merges painless and `main` always demo-ready.

**Timeline:** hacking started 9:30 AM · **code freeze 5:00 PM** · demo is 3–5 min.

## Project & track

- **Idea:** _TBD_
- **Track / prize targets:** _TBD_
- **One-sentence pitch:** _TBD_
- **The 3-minute demo path** (the exact clicks a judge will see): _TBD_

## Stack

Default unless changed here:

- Next.js (App Router) + TypeScript
- Tailwind CSS + shadcn/ui
- Deployed on Vercel (preview deploy per branch, production from `main`)
- Data / auth: _TBD_ (e.g. Supabase)
- AI / LLM provider: _TBD_ (keys live only in `.env.local` and Vercel env vars)

## Folder ownership

Edit only inside your own folders. Touching a shared file (layout, config,
`package.json`, `lib/types.ts`) means telling the team first.

| Teammate | Owns | Branch prefix |
| -------- | ---- | ------------- |
| Rodion   | _TBD_ | `rodion/*` |
| _TBD_    | _TBD_ | `_tbd_/*`  |
| _TBD_    | _TBD_ | `_tbd_/*`  |

**Shared (coordinate before editing):** _TBD_

## Data shapes & API contracts

Agree on these before building so the frontend can work against mocks.
The single source of truth for types: `lib/types.ts` (TBD).

```ts
// TBD: core types, e.g.
// type Thing = { id: string; ... }
```

| Route | Method | Request | Response | Owner |
| ----- | ------ | ------- | -------- | ----- |
| _TBD_ | | | | |

## Design brief

Make it look deliberate. **No generic AI-dashboard look**: no default purple
gradients, no grid of identical stat cards, no emoji-as-icons.

- **Palette:** _TBD_ (background, surface, text, accent, 1–2 semantic colors)
- **Fonts:** _TBD_ (one display face, one body face)
- **Signature element:** _TBD_ (the one visual thing judges will remember)
- **Tone of copy:** _TBD_

## Rules

1. **Small commits** with clear messages; push often.
2. **Stay in your own folders** (see ownership above).
3. **Never commit `.env*`, API keys, or tokens.** Use `.env.local` locally and Vercel env vars for deploys. Keep `.env.example` updated with variable *names* only.
4. **Keep `main` deployable.** Work on your branch, open a PR, merge only when it builds (`npm run build`) and the Vercel preview works.
5. Pull/rebase from `main` before opening a PR, and especially before 5:00 PM.
6. Mock data beats a blocked teammate: stub the API contract and keep moving.
7. Feature freeze target ~4:15 PM; after that, only bug fixes, polish, and the demo.
