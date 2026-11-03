#pragma once
// =============================================================================
//  guidance_application.hpp —— 启动层：参数加载 / 校验 + GuidanceApplication 总控台
//
//  职责：默认值 / YAML / CLI 合成 GuidanceAppParams → 校验 → 分发。
//  本层是组合根；guidance 模块不依赖它。
//  CLI/YAML/跑法见 assets/docs/app/usage.md。
// =============================================================================

#include <string>
#include <utility>

#include "guidance_app_params.hpp"   // GuidanceAppParams（宿主侧配置）

namespace pure_uav {
namespace app {

struct GuidanceParamResult {
    bool        ok = false;
    std::string error;

    static GuidanceParamResult success() { return GuidanceParamResult{true, {}}; }
    static GuidanceParamResult fail(std::string e) { return GuidanceParamResult{false, std::move(e)}; }
};

/// @brief 生产默认值（相机留空，真机必须配置）。
GuidanceAppParams guidance_default_params();

/// @brief 从 YAML 文本加载，在 *out 的现有值上覆盖。
/// @param source 仅用于报错信息（文件名）。
GuidanceParamResult guidance_load_yaml_text(const std::string &text, GuidanceAppParams *out,
                           const char *source = "<yaml>");

/// @brief 从文件加载；读不出文件时返回失败，不抛异常。
GuidanceParamResult guidance_load_yaml_file(const std::string &path, GuidanceAppParams *out);

/// @brief 范围/一致性校验；加载后、启动前必须调用。
GuidanceParamResult guidance_validate_params(const GuidanceAppParams &p);

/// @brief 打印成 YAML 子集（--print-params 用，顺序固定）。
std::string guidance_params_to_string(const GuidanceAppParams &p);

/// 总控台：main 只认识它。
class GuidanceApplication {
public:
    GuidanceApplication() = default;

    /// @brief 解析 argc/argv → 加载参数 → 分发到仿真/回放/真机。
    /// @return 0=成功/help/print，1=运行失败，2=参数错误。
    int run(int argc, char **argv);
};

}  // namespace app
}  // namespace pure_uav
