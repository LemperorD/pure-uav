#pragma once
// =============================================================================
//  application.hpp —— 启动层：参数加载 / 校验 + Application 总控台
//
//  合并自：
//    · params.hpp    AppParams 的加载/校验/打印（AppParams 本体已移到 flight_types.hpp）
//    · main.cpp 的 CLI 解析、模式分发
//
//  【职责】
//    · 把“默认值 / YAML / 命令行”合成一份 AppParams；
//    · 校验参数；
//    · 按模式调用 tools（仿真 / 回放）或未来的真机路径。
//  本层是 app 的**组合根**，知道所有模式；guidance 模块不依赖它。
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
