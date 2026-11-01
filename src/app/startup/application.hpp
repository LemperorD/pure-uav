#pragma once
// =============================================================================
//  application.hpp —— 启动层：参数加载 / 校验 + Application 总控台
//
//  职责：默认值 / YAML / CLI 合成 AppParams → 校验 → 分发。
//  本层是组合根；guidance 模块不依赖它。
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

AppParams default_params();

ParamResult load_yaml_text(const std::string &text, AppParams *out,
                           const char *source = "<yaml>");

ParamResult load_yaml_file(const std::string &path, AppParams *out);

ParamResult validate(const AppParams &p);

std::string to_string(const AppParams &p);

// 总控台：main 只认识它。
class Application {
public:
    Application() = default;

    // 解析 argc/argv → 加载参数 → 分发到 仿真 / 回放 / 将来的真机。
    int run(int argc, char **argv);
};

}  // namespace app
}  // namespace pure_uav
