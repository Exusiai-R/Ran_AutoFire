#pragma once
#include <string>
#include <map>
namespace base { namespace webview_info {
struct Entry { std::string value; std::string& get() { return value; } };
struct Group { std::map<std::string, Entry> entries; Entry& sub(const std::string& name) { return entries[name]; } };
struct Page { std::map<std::string, Group> groups; Group& sub(const std::string& name) { return groups[name]; } };
}}
