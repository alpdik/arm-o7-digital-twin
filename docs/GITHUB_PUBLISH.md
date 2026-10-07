# Publishing the project to GitHub for the team

## Why a private repository?

LinkerHand O7 sources are licensed under Apache-2.0. The ARM1.5 URDF and STL files came from a user archive with no explicit license. Therefore, create the repository as **private** until your organization verifies its right to share the model files. A private repository does not replace licensing permission; it only prevents unwanted public access.

## 1. Create an empty repository on GitHub

Select `New repository` on GitHub:

- Repository name: `arm-o7-digital-twin`
- Visibility: `Private`
- Add README: off
- Add `.gitignore`: off
- Add license: off

The project already contains these files. Recreating them on GitHub can cause unnecessary conflicts during the first push.

## 2. Make the local project a Git repository

Ubuntu terminalinde:

```bash
cd ~/arm_o7_digital_twin

git init -b main
git config user.name "AD SOYAD"
git config user.email "GITHUB_EPOSTA"

git status --short
git add .
git status --short
git diff --cached --stat
```

The second `git status` output must not contain:

- `build/`, `install/`, `log/`
- `*.log`, video, or rosbag files
- ZIP/PDF outputs under `output/`
- passwords, tokens, Wi-Fi passwords, or private keys

If the list is correct, create the first checkpoint:

```bash
git commit -m "Initial ARM1.5 and LinkerHand O7 digital twin"
```

## 3. Connect and push to GitHub

Use the URL shown on the empty GitHub repository page:

```bash
git remote add origin https://github.com/GITHUB_KULLANICI_ADI/arm-o7-digital-twin.git
git remote -v
git push -u origin main
```

Instead of accepting a GitHub password, GitHub may require browser sign-in, GitHub CLI, or a personal access token. Never write the token to a file or paste it into conversations.

## 4. Add a teammate

On the repository page, invite the teammate's GitHub username under `Settings` → `Collaborators`. After the invitation is accepted:

```bash
git clone https://github.com/GITHUB_KULLANICI_ADI/arm-o7-digital-twin.git
cd arm-o7-digital-twin
```

The simulation computer performs the full setup. If the sender is only a Raspberry Pi, follow `docs/NETWORK_TWO_HOSTS.md` first.

## Daily workflow

Before a new change:

```bash
git pull --ff-only
```

After a change:

```bash
git status --short
git add DOSYA_VEYA_KLASOR
git commit -m "Short and descriptive change summary"
git push
```

`commit` is like a snapshot saved in the project history. `push` sends that snapshot to GitHub. `pull` brings teammates' new snapshots onto the computer.
