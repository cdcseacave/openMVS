---
description: "Use when editing C++ in OpenMVS (libs/MVS, libs/Common, apps). Applies project naming, formatting, logging, and assertion conventions inferred from copilot instructions, .clang-format, and existing source files."
name: "OpenMVS C++ Style"
---
# OpenMVS C++ Style

- Keep formatting aligned with `.clang-format` and nearby code; avoid unrelated formatting churn.
- Indentation uses tabs for indentation width 4, with continuation indentation width 4.
- Bracing style in this codebase is mixed by construct:
  - Classes/structs/functions: opening brace on the next line.
  - Control statements (`if`, `for`, `while`): opening brace on the same line.
- Keep include order stable and do not auto-sort includes (`SortIncludes: false`).
- In `.cpp` files, prefer local `"Common.h"` and the matching header first, then project headers, then third-party headers.
- Use left pointer/reference alignment (`Type* p`, `const Type& v`).
- Naming conventions:
  - Types and functions: `CamelCase`
  - Variables: `lowerCamelCase`
  - Common variable prefixes: `n` numeric, `f` float, `b` bool, `p` pointer
  - Constants/enums/macros: uppercase style used by existing code
- Use `ASSERT(...)` for internal invariants and debug-time correctness checks.
- Use project logging macros (`VERBOSE`, `DEBUG`, `DEBUG_EXTRA`, `DEBUG_ULTIMATE`) instead of ad-hoc logging.
- Convert OpenMVS math types (`Point2/3`, `Matrix3x3`, `RMatrix`, `KMatrix`, `PMatrix`, `Normal`, ...) to and from Eigen through their built-in conversions (`EVec`/`EMat`, `CEVecMap`/`CEMatMap`, Eigen constructors, `operator=`), never through `Eigen::Map` over `.val`/`.ptr()` or component-wise copies; see "Eigen Interop of the OpenMVS Math Types" in the root `AGENTS.md`.
- Use project idioms such as `FOREACH`/`RFOREACH`, custom container types (`cList`), and section banners where appropriate.
- Prefer early returns on invalid states or failed preconditions.
- For associative containers (`std::map`, `std::unordered_map`), prefer single-lookup insertion (`emplace`/`try_emplace` with insertion-result check) instead of `find` + `emplace` when both existence check and insert are needed.
- For update-or-insert flows on associative containers, prefer `try_emplace` when constructing only on miss, or `insert_or_assign` when replacement semantics are intended; choose the API that matches the intended behavior clearly.
- For loops over map-like containers, prefer explicit key/value variables (for example structured bindings) instead of generic iterator/item names when it improves readability.
- Preserve long lines where needed (`ColumnLimit: 0`) unless local readability clearly improves.
- Create comment blocks for complex logic, and maintain existing comment style and formatting.
