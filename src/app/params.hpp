#pragma once
// =============================================================================
//  params.hpp —— 参数加载 / 校验（YAML 子集 + CLI 覆盖）
//
//  【为什么单独一层】
//    机载主循环里 `GuidanceParams` / `LoopConfig` 的字段会随着真机联调不断变；
//    如果它们散在 main() 的命令行解析里，每加一个参数就要改三处（解析/帮助/赋值），
//    而且**没有范围校验**（N=0、a_max=-1、loop_hz=0 都能启动，直到飞起来才炸）。
//    ⇒ 把"默认值 / 文件 / 命令行 / 校验 / 打印"收在一个 OOP 模块里，
//      main 只负责把 CLI 显式给出的项交给它。
//
//  【依赖】
//    只依赖 C++17 标准库 + 本层的 guidance_loop.hpp / guidance_types.hpp。
//    **不引入 yaml-cpp**：机载目标（香橙派）和 Windows 构建都不必为几个标量装库；
//    这里的 YAML 子集只支持本项目参数需要的语法：
//      · 扁平点号键：`guidance.N: 4`
//      · 缩进嵌套：`guidance:` 下一行缩进 `N: 4`
//      · `#` 注释（引号外）、空行、字符串两端引号
//      · 标量：整数 / 浮点 / 布尔 / 枚举名（大小写不敏感）
//    ⚠ 不支持列表、锚点、多文档、行内映射 —— 参数本来就是标量。
//
//  【CLI 优先级】
//    默认值 < YAML 文件 < 命令行（只覆盖**显式给出**的项，见 main.cpp 的 *_given）
// =============================================================================

#include <string>

#include "guidance_loop.hpp"    // LoopConfig / CameraModel
#include "guidance_types.hpp"   // GuidanceParams

namespace pure_uav {
namespace app {

// 一次加载所需的全部参数（**机载只认这一份**，不要再散落其它配置结构）。
struct AppParams {
    GuidanceParams guidance;
    LoopConfig     loop;
};

// 生产默认值：与 2026-10-25 之前 main.cpp 里硬编码的一致（N=4 / a_max=10 /
// PPN / 50 Hz / max_sample_age=0.2），保证加载器接入后**行为不变**。
// ⚠ 相机模型故意留空（valid()==false）：它必须由配置给出；
//   只有合成世界 demo（main.cpp）才允许回退到 demo_camera()。
AppParams default_params();

// 加载 / 校验结果。失败时 error 带 `来源:行号: 原因`（文件加载）或键名（校验）。
struct ParamResult {
    bool        ok = false;
    std::string error;

    static ParamResult success() { return ParamResult{true, {}}; }
    static ParamResult fail(std::string e) { return ParamResult{false, std::move(e)}; }
};

// 从 YAML 文本加载（**在 *out 的现有值上覆盖**，便于"默认值 → 文件 → CLI"叠加）。
// `source` 只用于报错信息（例如文件名）。
ParamResult load_yaml_text(const std::string &text, AppParams *out,
                           const char *source = "<yaml>");

// 从文件加载（读不出文件时返回失败，不抛异常）。
ParamResult load_yaml_file(const std::string &path, AppParams *out);

// 范围 / 一致性校验。**加载后、启动前必须调用**（main.cpp 已接）。
ParamResult validate(const AppParams &p);

// 打印成 YAML 子集（`--print-params` 用；顺序固定，便于 diff 两次配置）。
std::string to_string(const AppParams &p);

}  // namespace app
}  // namespace pure_uav
