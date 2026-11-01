# 注释与文档规范

- 状态：现行
- 适用：`src/**`（第三方/驱动 SDK 除外）与 `assets/docs/**`

---

## 1 总原则

> **代码注释回答“这一行为什么这么写”；文档回答“系统为什么这么设计、接口是什么、怎么验”。**

- 注释跟代码一起改；
- 文档可以独立演进；
- 同一段长文只在一处维护；
- 代码注释不要变成“第二份 README”。

---

## 2 代码注释

### 2.1 文件头（≤ 5 行）

只写：

- 这个文件负责什么；
- 边界/不变量（如“不依赖 app”）；
- 详细文档链接（可选）。

不要写：日期、迁移历史、“合并自 …”、30 行 banner。

### 2.2 公开 API（Doxygen）

接口、类、公开函数用：

```cpp
/// 一句话说明做什么。
/// @param[out] out 仅在返回 true 时写入。
/// @return 有新帧为 true。
/// @note 非阻塞、零分配、可每拍调用。
```

至少写清楚：

- 做什么；
- 参数/返回；
- 前置条件、单位、所有权、线程安全。

### 2.3 实现注释（1–3 行）

只写“为什么”：

- 不变量；
- 单位/量纲；
- 所有权/生命周期；
- 线程/实时性；
- 非显然的取舍、坑。

不要逐行复述代码。

### 2.4 TODO

```text
// TODO(name): 原因 / 触发条件 / 计划
```

不要写无主 TODO。

---

## 3 不要写进代码的内容

| 内容 | 放到 |
|---|---|
| 日期、版本、迁移/合并历史 | `assets/docs/changelog.md` 或 git |
| 长设计推导/架构说明 | `assets/docs/architecture.md`、`decisions/` |
| 坐标/时间/单位契约 | `assets/docs/interfaces.md` |
| CSV 格式/回放边界 | `assets/docs/app/csv-format.md` |
| CLI/YAML/跑法 | `assets/docs/app/usage.md` |
| 黑匣子 | `assets/docs/app/blackbox.md` |
| 测试矩阵/基线 | `assets/docs/testing.md` |
| emoji/装饰性 banner | 删除；用 `NOTE:` / `WARNING:` |

---

## 4 文档位置

```text
assets/docs/
├── README.md          # 索引
├── architecture.md    # 分层/数据流/线程
├── interfaces.md      # 坐标/姿态/时间/单位/端口
├── testing.md         # 基线/跑法
├── changelog.md       # 迁移/合并历史
├── comment-style.md   # 本文
├── decisions/         # ADR：为什么这么定
├── app/               # app 层：README/usage/csv-format/blackbox
└── guidance/          # 制导：README/任务书/后续注意问题
```

代码里用 1 行相对路径指过去，例如：

```text
// 设计见 assets/docs/decisions/0003-los-rate-source.md
```

---

## 5 评审清单

- [ ] 文件头 ≤ 5 行，无日期/历史/emoji
- [ ] 公开 API 有 Doxygen（做什么/参数/返回/边界）
- [ ] 实现注释只解释 why，不解释 what
- [ ] 长文已移到 `assets/docs/`，代码里只留链接
- [ ] 没有重复 README 的段落
- [ ] 单位/坐标系/时间基准写清楚
- [ ] 改接口时同步 `interfaces.md`
- [ ] 改测试基线时同步 `testing.md` 和 `changelog.md`

---

## 6 当前状态

- 已拆分：quat、Pure 前缀、ω、CSV、接口/时间、架构、测试、冻结判据、目标模型、传感器墙；
- 仍待清理：测试段标题里的日期/emoji、部分 `★/⚠`、公开 API 的 Doxygen。
