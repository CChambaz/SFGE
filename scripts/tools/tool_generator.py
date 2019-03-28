"""
MIT License

Copyright (c) 2018 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import json
import sys
import os
import re


from pathlib import Path

from scripts.tools.license_generator import get_license_str


def open_tools_config_json(tools_config_filename="data/tools_config.json"):
    try:
        file = open(tools_config_filename, "r")
        file_data = file.read()
        file.close()
        tools_config = json.loads(file_data)
        return tools_config
    except FileNotFoundError:
        sys.stderr.write("[Error] Could not open {0} did you run CMake?\n".format(tools_config_filename))
        return None


def generate_new_tool(tool_name: str):
    tools_config_json = open_tools_config_json()
    if tools_config_json is not None:
        src_dir = Path(tools_config_json["src_dir"])

        tools_dir = src_dir / "tools"
        new_tool_dir = tools_dir / tool_name

        tool_name_lowercase = tool_name[0].lower() + re.sub(r"([A-Z])", r"_\1", tool_name[1:]).lower()

        if tools_dir.is_dir() and not new_tool_dir.exists():
            os.mkdir(new_tool_dir)
            os.mkdir(new_tool_dir / "include")
            tool_include_file = open(new_tool_dir / "include" / (tool_name_lowercase + ".h"), "w")
            tool_include_file.write("/*\n" + get_license_str() + "\n*/")
            tool_include_file.write("""
#ifndef SFGE_TOOLS_""" + tool_name_lowercase.upper() + """_H
#define SFGE_TOOLS_""" + tool_name_lowercase.upper() + """_H

#include <engine/system.h>
namespace sfge::tools
{
class """ + tool_name + """ : public System
{
    using System::System;
    /*
    * \\brief Called at scene init (a good place to link to other Systems
    */
    void Init() override;
    /*
    * \\brief Called every graphic frame (dt depends on the use of VSync or not, controllable in the Configuration) 
    */
    void Update(float dt) override;
     /*
    * \\brief Called every graphic frame after Update
    */
    void Draw() override;
};
}
#endif"""
                                    )
            tool_include_file.close()
            os.mkdir(new_tool_dir / "src")
            tool_src_file = open(new_tool_dir / "src" / (tool_name_lowercase + ".cpp"), "w")
            tool_src_file.write("/*\n" + get_license_str() + "\n*/\n")
            tool_src_file.write("""
#include <""" + tool_name_lowercase + """.h>

namespace sfge::tools
{
void """+tool_name+"""::Init()
{
}

void """+tool_name+"""::Update(float dt)
{
}


void """+tool_name+"""::Draw()
{
}
}
""")
            tool_src_file.close()
            print("Create tool: " + tool_name)
        else:
            sys.stderr.write("[Error] Tool {0} already exists\n".format(tool_name))


def main():
    generate_new_tool("ProutTool")


if __name__ == "__main__":
    main()
