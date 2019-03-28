/*
 MIT License

 Copyright (c) 2017 SAE Institute Switzerland AG

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
 */

#include <cctype>
#include <sstream>

#include <utility/python_utility.h>
#include <utility/log.h>

namespace sfge
{
py::object import(const std::string& module, const std::string& path, py::object& globals)
{
	/*{
		std::ostringstream oss;
		oss << "Loading python module: " << module << " with globals: " << py::str(globals).cast<std::string>();
		sfge::Log::GetInstance()->Msg(oss.str());
	}*/

	const py::dict locals;
	locals["module_name"] = py::cast(module); // have to cast the std::string first
	locals["file_path"] = py::cast(path);

	/*
	 *
	 * "import imp\n"
		"new_module = imp.load_module(module_name, open(path), path, ('py', 'U', imp.PY_SOURCE))\n",
	 */
	py::eval<py::eval_statements>(            // tell eval we're passing multiple statements
		"import importlib.util\n"
		"spec = importlib.util.spec_from_file_location(module_name, file_path)\n"
		"module = importlib.util.module_from_spec(spec)\n"
		"spec.loader.exec_module(module)\n",
		globals,
		locals);

	return locals["module"];
}

std::string module2class(std::string& module_name)
{

	std::istringstream iss(module_name);
	std::string token;
	std::string class_name;
	while (std::getline(iss, token, '_'))
	{
		if (!token.empty())
		{
			const char first = token.at(0);
			class_name += std::toupper(first);
			class_name += token.substr(1, token.size());
		}
	}
	return class_name;
}
}
