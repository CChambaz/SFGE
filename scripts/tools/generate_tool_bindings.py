import sys
import re
import os
from pathlib import Path
from license_generator import get_license_str


def generate_bindings(tools: list):
    include_dir = Path("include")

    tools_dir_name = "tools"
    try:
        os.mkdir(include_dir / tools_dir_name)
    except FileExistsError:
        pass
    tool_include_file = open(include_dir / tools_dir_name / "tools_pch.h", "w")
    tool_include_file.write("/*\n" + get_license_str() + "\n*/")
    tool_include_file.write("""
#ifndef SFGE_TOOL_PCH_H
#define SFGE_TOOL_PCH_H

#include <utility/python_utility.h>

namespace sfge::tools
{
void ExtendPythonTools(py::module& m);
}

#endif"""
                            )
    tool_include_file.close()

    src_dir = Path("src")
    try:
        os.mkdir(src_dir / tools_dir_name)
    except FileExistsError:
        pass
    tool_src_file = open(src_dir / tools_dir_name / "tools_pch.cpp", "w")
    tool_src_file.write("/*\n" + get_license_str() + "\n*/\n")
    tool_src_file.write("#include <tools/tools_pch.h>\n")
    tool_src_file.write("#include <engine/engine.h>\n")
    for tool_name in tools:
        tool_name_lowercase = tool_name[0].lower() + re.sub(r"([A-Z])", r"_\1", tool_name[1:]).lower()
        tool_src_file.write("#include <" + tool_name_lowercase + ".h>\n")
    tool_src_file.write("""
namespace sfge::tools
{
void ExtendPythonTools(py::module& m)
{""")
    for tool_name in tools:
        tool_name_lowercase = tool_name[0].lower() + re.sub(r"([A-Z])", r"_\1", tool_name[1:]).lower()
        tool_src_file.write("""
    py::class_<%s, System> %s(m, "%s");
    %s
		.def(py::init<Engine&>());""" % (
        tool_name, tool_name[0].lower() + tool_name[1:], tool_name, tool_name[0].lower() + tool_name[1:]))
    tool_src_file.write("\n}\n}")
    tool_src_file.close()


if __name__ == "__main__":
    generate_bindings(sys.argv[1:])
