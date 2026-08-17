/*
 * Tests.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#ifndef _TESTS_H_
#define _TESTS_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "../../libs/Common/Common.h"

#include <filesystem>


// S T R U C T S ///////////////////////////////////////////////////

// uniquely named temporary directory, removed with all its content when going out of scope;
// the test is expected to abort if IsValid() returns false
class ScopedTempDir {
public:
	// testName: identifies the test in the directory name and in the error message
	ScopedTempDir(const SEACAVE::String& testName) {
		const std::filesystem::path dir(std::filesystem::temp_directory_path() /
			(_T("openmvs_") + testName + _T("_") + SEACAVE::Util::getUniqueName(0)).c_str());
		std::error_code ec;
		if (std::filesystem::create_directories(dir, ec))
			path = dir;
		else
			VERBOSE("%s FAILED: cannot create temp dir '%s': %s", testName.c_str(), dir.string().c_str(), ec.message().c_str());
	}
	~ScopedTempDir() {
		if (IsValid()) {
			std::error_code ec;
			std::filesystem::remove_all(path, ec);
		}
	}

	bool IsValid() const { return !path.empty(); }
	// path of this directory
	SEACAVE::String Path() const { ASSERT(IsValid()); return SEACAVE::String(path.string()); }
	// path of the given file inside this directory
	SEACAVE::String operator()(const SEACAVE::String& fileName) const {
		ASSERT(IsValid());
		return SEACAVE::String((path / fileName.c_str()).string());
	}

private:
	std::filesystem::path path;
};
/*----------------------------------------------------------------*/

#endif // _TESTS_H_
