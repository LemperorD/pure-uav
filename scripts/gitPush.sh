#!/bin/bash

# ===============================
# 配置工作空间根目录和代理
# ===============================
WORKSPACE_DIR=~/pure-uav
HTTPS_PROXY=http://127.0.0.1:7897
HTTP_PROXY=http://127.0.0.1:7897
ALL_PROXY=socks5://127.0.0.1:7897

cd "$WORKSPACE_DIR" || { echo "无法进入工作目录 $WORKSPACE_DIR"; exit 1; }

# ===============================
# 扫描 Git 仓库（主仓库 + 子模块）
# ===============================
echo "正在扫描 Git 仓库..."
repos=()
repo_names=()

# 检查主仓库
if [ -d ".git" ]; then
    repos+=(".")
    # 仓库名用 git rev-parse 获取根目录名
    name=$(basename "$(git rev-parse --show-toplevel)")
    repo_names+=("$name")
fi

# 扫描子模块
while IFS= read -r line; do
    path=$(echo "$line" | awk '{print $2}')
    if [ -d "$path/.git" ]; then
        repos+=("$path")
        # 进入子模块获取仓库名
        name=$(cd "$path" && basename "$(git rev-parse --show-toplevel)")
        repo_names+=("$name")
    fi
done < <(git submodule status)

# 显示仓库表格
echo -e "\e[1;32m找到以下 Git 仓库：\e[0m"
for i in "${!repos[@]}"; do
    echo "$i) ${repo_names[$i]} -> ${repos[$i]}"
done

# ===============================
# 用户选择仓库
# ===============================
echo "-------------------------------"
read -rp "$(echo -e "\e[1;32m请输入要 push 的仓库序号（多个用空格分隔）：\e[0m")" -a choices

# ===============================
# 对选择的仓库执行 push
# ===============================
for index in "${choices[@]}"; do
    repo="${repos[$index]}"
    repo_name="${repo_names[$index]}"
    echo "-------------------------------"
    echo -e "\e[1;32m正在处理仓库: $repo_name (路径: $repo)\e[0m"
    cd "$WORKSPACE_DIR/$repo" || continue

    # 设置代理
    export https_proxy=$HTTPS_PROXY
    export http_proxy=$HTTP_PROXY
    export all_proxy=$ALL_PROXY

    # 获取当前分支
    current_branch=$(git symbolic-ref --short HEAD)
    echo "当前分支: $current_branch"

    # 检查是否有未提交的修改
    if git diff --quiet && git diff --cached --quiet; then
        echo -e "\e[1;32m仓库 $repo_name 没有未提交的修改，执行 push...\e[0m"
    else
        echo -e "\e[1;31m仓库 $repo_name 有未提交的修改，请先在 VSCode 完成 commit!\e[0m"
        continue
    fi

    # push 到远程同名分支
    git push origin "$current_branch"
done

echo -e "\e[1;32m所有选择的仓库已 push 完成!\e[0m"
