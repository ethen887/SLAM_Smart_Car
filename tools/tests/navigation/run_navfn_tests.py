"""Compare the C planner to the local, unmodified Nav2 NavFn source."""

from pathlib import Path
import subprocess
import tempfile


root = Path(__file__).resolve().parents[3]
ra = root / "Renesas/TinySLAM/3.LSLidar/src"
reference = root / "Reference_Project/navigation2-main/nav2_navfn_planner"
nav = ra / "Navigation"
app = ra / "Application"
includes = [
    nav, app, ra / "Device",
    root / "Renesas/TinySLAM/4.LSLidar-stm32/src/Algorithm",
    reference / "include",
]

if not (reference / "src/navfn.cpp").is_file():
    raise SystemExit(f"The local Nav2 reference required for the comparison is missing: {reference}")

with tempfile.TemporaryDirectory(prefix="navfn-tests-") as directory:
    build = Path(directory)
    # NavFn's algorithm only uses ROS for logging and the cancel exception.
    stubs = {
        "rclcpp/logger.hpp": "#pragma once\n",
        "rclcpp/logging.hpp": "#pragma once\n#define RCLCPP_DEBUG(...) ((void)0)\n"
                              "#define RCLCPP_WARN(...) ((void)0)\n",
        "nav2_core/planner_exceptions.hpp": "#pragma once\n#include <stdexcept>\n"
            "namespace nav2_core { struct PlannerCancelled : std::runtime_error "
            "{ using std::runtime_error::runtime_error; }; }\n",
    }
    for name, content in stubs.items():
        target = build / name
        target.parent.mkdir(exist_ok=True)
        target.write_text(content, encoding="utf-8")

    flags = ["-I" + str(path) for path in [build, *includes]]
    objects = []
    for source in [nav / "nav_costmap.c", nav / "nav_navfn.c", app / "app_global_plan.c"]:
        output = build / (source.stem + ".o")
        subprocess.run(["gcc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                        *flags, "-c", str(source), "-o", str(output)], check=True)
        objects.append(str(output))

    output = build / "navfn_compare.exe"
    subprocess.run(["g++", "-std=c++17", "-O2", *flags, *objects,
                    str(reference / "src/navfn.cpp"),
                    str(Path(__file__).with_name("test_navfn_compare.cpp")),
                    "-o", str(output)], check=True)
    subprocess.run([str(output)], check=True)
