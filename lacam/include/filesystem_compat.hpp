#pragma once

#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ < 8
#include <experimental/filesystem>
namespace lacam_filesystem = std::experimental::filesystem;
#else
#include <filesystem>
namespace lacam_filesystem = std::filesystem;
#endif
