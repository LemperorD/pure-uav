#pragma once
// =============================================================================
//  application.hpp —— 启动层：参数加载 / 校验 + Application 总控台
//
//  职责：默认值 / YAML / CLI 合成 AppParams → 校验 → 分发。
//  本层是组合根；guidance 模块不依赖它。
//  CLI/YAML/跑法见 assets/docs/app/usage.md。
// =============================================================================

#include <string>
#include <utility>

#include "flight_types.hpp"   // AppParams

namespace pure_uav {
namespace app {

struct ParamResult {
    bool        ok = false;
    std::string error;

    static ParamResult success() { return ParamResult{true, {}}; }
    static ParamResult fail(std::string e) { return ParamResult{false, std::move(e)}; }
};

/// @brief 生产默认值（相机留空，真机必须配置）。
AppParams default_params();

/// @brief 从 YAML 文本加载，在 *out 的现有值上覆盖。
/// @param source 仅用于报错信息（文件名）。
ParamResult load_yaml_text(const std::string &text, AppParams *out,
                           const char *source = "<yaml>");

/// @brief 从文件加载；读不出文件时返回失败，不抛异常。
ParamResult load_yaml_file(const std::string &path, AppParams *out);

/// @brief 范围/一致性校验；加载后、启动前必须调用。
ParamResult validate(const AppParams &p);

/// @brief 打印成 YAML 子集（--print-params 用，顺序固定）。
std::string to_string(const AppParams &p);

/// 总控台：main 只认识它。
class Application {
public:
    Application() = default;

    /// @brief 解析 argc/argv → 加载参数 → 分发到仿真/回放/真机。
    /// @return 0=成功/help/print，1=运行失败，2=参数错误。
    int run(int argc, char **argv);
};

}  // namespace app
}  // namespace pure_uav
