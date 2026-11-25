# Git workflow

- Feature branches + pull requests. **No direct pushes to `main`.**
- Branch naming: `feature/<topic>`, `fix/<topic>`.
- Every PR needs a review from a teammate.

## Which repo gets which change
| Change | Repo |
|---|---|
| MRM nodes, sim tools, MRM launch files | `src/custom/autoware_custom` |
| Ioniq 5 vehicle/sensor packages, MABX bridge, Autoware param changes | `src/launcher/autoware_launch` (our fork) |
| Dockerfile, `repositories/*.repos`, `config/`, docs, `CLAUDE.md` | this repo (root) |

Rules:
- Don't modify other upstream repos in `src/`. If an upstream change is needed, do it as a parameter override in our `autoware_launch` fork.
- The sub-repos under `src/` are separate git repos. Commit and open PRs inside each one.
- When a sub-repo changes in a way the root needs, update the version in `repositories/custom.repos` (and `autoware.repos` for the fork) in a root PR.
