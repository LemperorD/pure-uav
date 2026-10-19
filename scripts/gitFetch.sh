#!/usr/bin/env bash

set -e

# ============================================================
# Git Fetch Helper
#
# 功能：
#   1. 自动检测 Git remote
#   2. 显示远端分支
#   3. 选择 fetch 单个分支
#   4. fetch 全部远端分支
#   5. 不执行 git pull
# ============================================================

# ---------- 颜色 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

export https_proxy=http://127.0.0.1:7897 http_proxy=http://127.0.0.1:7897 all_proxy=socks5://127.0.0.1:7897

# ============================================================
# 检查当前目录是不是 Git 仓库
# ============================================================

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo -e "${RED}错误：当前目录不是 Git 仓库${NC}"
    exit 1
fi


# ============================================================
# 获取 remote
# ============================================================

remotes=($(git remote))

if [ ${#remotes[@]} -eq 0 ]; then
    echo -e "${RED}错误：当前仓库没有配置远端仓库${NC}"
    exit 1
fi


echo
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}        Git Fetch Helper${NC}"
echo -e "${CYAN}========================================${NC}"
echo

echo -e "${BLUE}当前仓库：${NC}"
git rev-parse --show-toplevel

echo

echo -e "${BLUE}远端仓库：${NC}"

for i in "${!remotes[@]}"; do
    remote="${remotes[$i]}"
    url=$(git remote get-url "$remote")

    echo "  [$((i + 1))] $remote"
    echo "      $url"
done

echo


# ============================================================
# 选择 remote
# ============================================================

if [ ${#remotes[@]} -eq 1 ]; then

    remote="${remotes[0]}"

    echo -e "${GREEN}自动选择 remote: $remote${NC}"

else

    echo "请选择 remote："

    read -p "输入编号 [1-${#remotes[@]}]: " remote_index

    if ! [[ "$remote_index" =~ ^[0-9]+$ ]] ||
       [ "$remote_index" -lt 1 ] ||
       [ "$remote_index" -gt "${#remotes[@]}" ]; then

        echo -e "${RED}无效选择${NC}"
        exit 1
    fi

    remote="${remotes[$((remote_index - 1))]}"

fi


echo
echo -e "${CYAN}当前 remote: ${GREEN}$remote${NC}"
echo


# ============================================================
# 更新远端分支信息
#
# 注意：
# 这里只更新 remote-tracking branch
# 不会修改当前本地分支
# ============================================================

echo -e "${YELLOW}正在获取远端分支信息...${NC}"

git fetch "$remote" --prune


echo
echo -e "${GREEN}远端分支获取完成${NC}"
echo


# ============================================================
# 获取远端分支
# ============================================================

mapfile -t branches < <(
    git for-each-ref \
        --format='%(refname:strip=3)' \
        "refs/remotes/$remote" |
    grep -v "^HEAD$" |
    sort
)


if [ ${#branches[@]} -eq 0 ]; then

    echo -e "${RED}没有检测到远端分支${NC}"
    exit 1

fi


# ============================================================
# 显示远端分支
# ============================================================

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}远端分支${NC}"
echo -e "${CYAN}========================================${NC}"

for i in "${!branches[@]}"; do
    branch="${branches[$i]}"

    echo "  [$((i + 1))] $branch"
done

echo
echo "  [A] Fetch 全部远端分支"
echo "  [Q] 退出"
echo


# ============================================================
# 用户选择
# ============================================================

read -p "请选择: " choice


# ============================================================
# Fetch 全部
# ============================================================

if [[ "$choice" =~ ^[Aa]$ ]]; then

    echo
    echo -e "${YELLOW}正在 fetch 全部远端分支...${NC}"
    echo

    git fetch "$remote" --prune

    echo
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}全部 fetch 完成${NC}"
    echo -e "${GREEN}========================================${NC}"

    echo
    echo "当前远端分支："
    git branch -r

    exit 0
fi


# ============================================================
# 退出
# ============================================================

if [[ "$choice" =~ ^[Qq]$ ]]; then

    echo "退出。"
    exit 0

fi


# ============================================================
# Fetch 单个分支
# ============================================================

if ! [[ "$choice" =~ ^[0-9]+$ ]]; then

    echo -e "${RED}无效选择${NC}"
    exit 1

fi


if [ "$choice" -lt 1 ] || [ "$choice" -gt "${#branches[@]}" ]; then

    echo -e "${RED}无效选择${NC}"
    exit 1

fi


branch="${branches[$((choice - 1))]}"


echo
echo -e "${YELLOW}准备 fetch：${NC}"
echo
echo "  remote : $remote"
echo "  branch : $branch"
echo


# ============================================================
# Fetch 指定分支
#
# 这里使用：
#
#   git fetch remote branch
#
# 只获取指定分支
# ============================================================

git fetch "$remote" "$branch"


echo
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Fetch 完成${NC}"
echo -e "${GREEN}========================================${NC}"

echo
echo "远端分支状态："
git branch -r