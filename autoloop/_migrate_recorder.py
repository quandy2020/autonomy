#!/usr/bin/env python3
"""Copy autonomy/system/recorder into autoloop/autoloop/robot/recorder."""
from pathlib import Path
import re
import shutil

ROOT = Path("/Users/quandy/Workspace/github/autonomy")
SRC = ROOT / "autonomy/system/recorder"
DST = ROOT / "autoloop/autoloop/robot/recorder"
LATCH_H = ROOT / "autonomy/system/safety/safety_latch.hpp"
LATCH_CC = ROOT / "autonomy/system/safety/safety_latch.cpp"
CONF = ROOT / "autonomy/system/conf/recorder.pb.txt"
DOCS = ROOT / "autonomy/system/docs/recorder"

if DST.exists():
    shutil.rmtree(DST)
DST.mkdir(parents=True)

repls = [
    ("#include \"autonomy/system/recorder/proto/", '#include "robot/recorder/proto/'),
    ("#include \"autonomy/system/recorder/", '#include "robot/recorder/'),
    ("#include \"autonomy/system/safety/safety_latch.hpp\"",
     '#include "robot/recorder/safety_latch.hpp"'),
    ("autonomy::system::safety::", "autoloop::robot::recorder::"),
    ("autonomy::system::recorder::", "autoloop::robot::recorder::"),
    ("namespace autonomy {\nnamespace system {\nnamespace recorder {",
     "namespace autoloop {\nnamespace robot {\nnamespace recorder {"),
    ("}  // namespace recorder\n}  // namespace system\n}  // namespace autonomy",
     "}  // namespace recorder\n}  // namespace robot\n}  // namespace autoloop"),
    ("package autonomy.system.recorder.proto;", "package autoloop.robot.recorder.proto;"),
]


def xform(text: str) -> str:
    for a, b in repls:
        text = text.replace(a, b)
    return text


for p in SRC.rglob("*"):
    if not p.is_file():
        continue
    rel = p.relative_to(SRC)
    out = DST / rel
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = p.read_bytes()
    try:
        s = raw.decode("utf-8")
    except UnicodeDecodeError:
        out.write_bytes(raw)
        continue
    out.write_text(xform(s), encoding="utf-8")

# safety latch
for src, name in ((LATCH_H, "safety_latch.hpp"), (LATCH_CC, "safety_latch.cpp")):
    t = src.read_text(encoding="utf-8")
    t = t.replace('#include "autonomy/system/safety/safety_latch.hpp"',
                  '#include "robot/recorder/safety_latch.hpp"')
    t = t.replace("namespace autonomy {\nnamespace system {\nnamespace safety {",
                  "namespace autoloop {\nnamespace robot {\nnamespace recorder {")
    t = t.replace("}  // namespace safety\n}  // namespace system\n}  // namespace autonomy",
                  "}  // namespace recorder\n}  // namespace robot\n}  // namespace autoloop")
    t = t.replace("autonomy::system::safety", "autoloop::robot::recorder")
    t = t.replace("/autonomy", "/autoloop")
    t = t.replace("AUTONOMY_SAFETY_LATCH_PATH", "AUTOLOOP_SAFETY_LATCH_PATH")
    t = t.replace(".autonomy/runtime", ".autoloop/runtime")
    (DST / name).write_text(t, encoding="utf-8")

# Drop autonomy conf_loader / version / cli
rec_cc = DST / "recorder.cpp"
t = rec_cc.read_text()
t = t.replace('#include "autonomy/common/conf_loader.hpp"\n', "")
t = t.replace(
    """    if (!common::LoadModuleConf("system", file, &options)) {
        AWARN << "recorder: using defaults, failed to load " << file;
    }""",
    """    if (autolink::common::PathExists(file) &&
        autolink::common::GetProtoFromFile(file, &options)) {
        ApplyDefaults(&options);
        return options;
    }
    AWARN << "recorder: using defaults, failed to load " << file;""",
)
t = t.replace("AUTONOMY_RECORD_ROOT", "AUTOLOOP_RECORD_ROOT")
t = t.replace("AUTONOMY_LOG_ROOT", "AUTOLOOP_LOG_ROOT")
t = t.replace("/.autonomy/records", "/.autoloop/records")
t = t.replace("/tmp/autonomy/records", "/tmp/autoloop/records")
rec_cc.write_text(t)

ver_cc = DST / "version_tuple.cpp"
t = ver_cc.read_text()
t = t.replace('#include "autonomy/common/version.hpp"\n', "")
t = t.replace(
    """    const std::string raw = common::GetGitCommitID();
    const std::string marker = "ID: ";
    const auto pos = raw.find(marker);
    if (pos == std::string::npos) {
        return {};
    }
    auto start = pos + marker.size();
    const auto on = raw.find(" on ", start);
    std::string sha = on == std::string::npos ? raw.substr(start)
                                             : raw.substr(start, on - start);
    if (sha == "unknown" || sha.empty()) {
        return {};
    }
    return sha;""",
    """    const std::string from_loop = EnvOr("AUTOLOOP_GIT_SHA", "");
    if (!from_loop.empty()) {
        return from_loop;
    }
#ifdef AUTOLOOP_GIT_SHA
    const char* compiled = AUTOLOOP_GIT_SHA;
    if (compiled != nullptr && compiled[0] != '\\0' &&
        std::string(compiled) != "unknown") {
        return compiled;
    }
#endif
    return {};""",
)
t = t.replace('EnvOr(\n        "AUTONOMY_POLICY_VERSION"',
              'EnvOr("AUTOLOOP_POLICY_VERSION", EnvOr(\n        "AUTONOMY_POLICY_VERSION"')
# fix MakeVersionTuple policy line more carefully
t = ver_cc.read_text() if False else t
ver_cc.write_text(t)

# version_tuple policy: AUTOLOOP first
t = ver_cc.read_text()
old_pol = '''    tuple->set_policy_version(EnvOr(
        "AUTONOMY_POLICY_VERSION",
        EnvOr("AUTONOMY_SOFTWARE_VERSION", options.policy_version())));
    tuple->set_env_version(
        EnvOr("AUTONOMY_ENV_VERSION", options.env_version()));
    tuple->set_seed(EnvOr("AUTONOMY_SEED", options.seed()));'''
new_pol = '''    tuple->set_policy_version(EnvOr(
        "AUTOLOOP_POLICY_VERSION",
        EnvOr("AUTONOMY_POLICY_VERSION",
              EnvOr("AUTONOMY_SOFTWARE_VERSION", options.policy_version()))));
    tuple->set_env_version(EnvOr(
        "AUTOLOOP_ENV_VERSION", EnvOr("AUTONOMY_ENV_VERSION", options.env_version())));
    tuple->set_seed(EnvOr("AUTOLOOP_SEED", EnvOr("AUTONOMY_SEED", options.seed())));'''
if old_pol not in t:
    raise SystemExit("version_tuple policy block not found")
ver_cc.write_text(t.replace(old_pol, new_pol))

# main: drop autonomy CLI, rename binary
main = DST / "recorder_main.cpp"
t = main.read_text()
t = t.replace('#include "autonomy/common/cli_options.hpp"\n', "")
t = t.replace("autonomy.recorder", "autoloop.recorder")
t = t.replace("/autonomy/system/recorder", "/autoloop/robot/recorder")
t = t.replace("autonomy::common::BindCommonOptions(app);",
              "app.add_option(\"--conf\", FLAGS_conf, \"RecorderOptions text proto\");")
t = t.replace("autonomy::common::ParseOrExit(app, argc, argv);",
              """try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }""")
t = t.replace(
    """    const std::string conf = autonomy::common::FLAGS_conf.empty()
                                 ? std::string("recorder.pb.txt")
                                 : autonomy::common::FLAGS_conf;""",
    """    const std::string conf =
        FLAGS_conf.empty() ? std::string("recorder.pb.txt") : FLAGS_conf;""",
)
# inject FLAGS_conf
t = t.replace("int main(int argc, char** argv) {\n    CLI::App app{\"autoloop.recorder\"};",
              "int main(int argc, char** argv) {\n    std::string FLAGS_conf;\n"
              "    CLI::App app{\"autoloop.recorder\"};")
main.write_text(t)

# conf
conf_dst = ROOT / "autoloop/autoloop/robot/config/data_agent/recorder.pb.txt"
conf_dst.parent.mkdir(parents=True, exist_ok=True)
c = CONF.read_text()
c = c.replace("autonomy.recorder", "autoloop.recorder")
c = c.replace("autonomy/system/recorder/proto", "robot/recorder/proto")
conf_dst.write_text(c)

# docs
docs_dst = ROOT / "autoloop/docs/recorder"
if docs_dst.exists():
    shutil.rmtree(docs_dst)
shutil.copytree(DOCS, docs_dst)
for p in docs_dst.rglob("*.md"):
    s = p.read_text()
    s = s.replace("autonomy.recorder", "autoloop.recorder")
    s = s.replace("autonomy/system/recorder", "autoloop/autoloop/robot/recorder")
    s = s.replace("`autonomy/system/recorder/", "`robot/recorder/")
    s = s.replace("AUTONOMY_BUILD_RECORDER", "AUTOLOOP_BUILD_RECORDER")
    s = s.replace("//autonomy/system:autonomy.recorder", "autoloop.recorder")
    s = s.replace("//autonomy/system:recorder_test", "autoloop_robot_recorder_test")
    p.write_text(s)

print("copied", len(list(DST.rglob("*"))), "paths to", DST)
