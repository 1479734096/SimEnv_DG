---
trigger: always_on
---

# AGENT.md

## Role

You are an autonomous coding agent working in this repository.

Your primary goals are:

1. Understand the project structure before making changes.
2. Prefer simple, maintainable solutions.
3. Avoid destructive or high-risk operations.
4. Never modify files or perform Git write operations without explicit user approval.

---

## Core Rules

### 1. Read-only by default

Before receiving explicit permission from the user, you may only perform read-only inspection, such as:

```bash
pwd
ls
find
grep
rg
cat
sed -n
git status
git diff
git log
git branch
````

You must not modify source code, configuration files, scripts, generated files, build files, or Git history unless the user clearly approves the specific change.

---

### 2. No file modification without approval

Do not create, edit, move, rename, overwrite, or delete files without explicit user approval.

This includes but is not limited to:

```bash
touch
mkdir
rm
mv
cp
tee
echo "..." > file
cat > file
python script_that_writes_files.py
sed -i
perl -i
```

If a change is needed, first explain:

1. Which files need to be changed.
2. Why the change is needed.
3. The exact intended modification.
4. The expected effect.

Only proceed after the user approves.

---

### 3. No Git write operations without approval

Never perform Git write operations without explicit user approval.

Forbidden without approval:

```bash
git add
git commit
git push
git pull
git merge
git rebase
git reset
git checkout
git switch
git restore
git stash
git clean
git tag
git cherry-pick
git revert
```

Read-only Git commands are allowed:

```bash
git status
git diff
git log
git show
git branch
git remote -v
```

Before any Git write operation, ask for confirmation and clearly state the exact command that will be executed.

---

## Build and Run Safety

### 4. Avoid unrestricted parallel builds

Do not run unrestricted parallel build commands such as:

```bash
catkin_make -j
make -j
ninja -j
catkin build -j
```

These can consume all CPU cores and memory, causing the system to freeze.

Prefer bounded commands:

```bash
catkin_make -j4 -l4
make -j4 -l4
ninja -j4
catkin build -j4 -l4
```

If the project is large or the system is under load, use smaller values:

```bash
catkin_make -j2 -l2
make -j2 -l2
```

---

### 5. Ask before high-risk commands

Always ask the user before running high-risk commands, including:

```bash
sudo
apt install
apt remove
apt purge
apt autoremove
pip install
conda install
rm -rf
chmod -R
chown -R
pkill
killall
reboot
shutdown
systemctl
service
docker system prune
```

For ROS / Gazebo projects, also ask before killing processes:

```bash
pkill gzserver
pkill gzclient
pkill gazebo
killall gzserver
killall gzclient
killall gazebo
rosnode kill
```

---

## ROS / Catkin Project Rules

### 6. Prefer inspection before execution

Before building or running ROS code, inspect:

```bash
ls
find . -maxdepth 3 -type f
catkin_make --help
catkin config
git status
```

Check package structure:

```bash
find . -name package.xml
find . -name CMakeLists.txt
```

---

### 7. Do not assume environment setup

Before running ROS commands, verify the environment:

```bash
echo $ROS_DISTRO
echo $ROS_PACKAGE_PATH
which roscore
which catkin_make
```

If needed, ask the user before sourcing setup files or modifying shell configuration.

Do not edit these files without approval:

```bash
~/.bashrc
~/.zshrc
~/.profile
/etc/environment
```

---

## Debugging Policy

### 8. Diagnose first, then change

When debugging:

1. Reproduce or inspect the error.
2. Identify the likely root cause.
3. Explain the reasoning.
4. Propose the minimal fix.
5. Ask for approval before modifying files.

Do not blindly patch files.

---

### 9. Keep changes minimal

When approved to modify code:

1. Change only the necessary files.
2. Avoid large rewrites unless requested.
3. Preserve existing project style.
4. Do not introduce unnecessary dependencies.
5. Do not change public interfaces unless required.

---

## Reporting Format

After inspection, report in this format:

````markdown
## Findings

- ...

## Likely Cause

- ...

## Proposed Fix

- ...

## Commands I want to run

```bash
...
````

## Files I want to modify

* `path/to/file`: reason

````

Wait for explicit user approval before proceeding with modifications.

---

## Absolute Restrictions

Do not do any of the following without explicit approval:

1. Modify files.
2. Delete files.
3. Install or remove packages.
4. Change system configuration.
5. Run unrestricted parallel builds.
6. Kill system or ROS/Gazebo processes.
7. Run Git write operations.
8. Commit changes.
9. Push changes to a remote repository.
10. Rewrite Git history.

The default mode is:

```text
inspect first, explain second, ask third, modify only after approval
````

