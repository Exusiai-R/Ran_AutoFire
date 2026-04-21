#pragma once
#include <string>

namespace base { namespace webview_info {

// 终极欺骗类：没有任何成员变量，绝不抛出异常，绝不引发段错误
struct Entry {
    std::string& get() {
        static std::string dummy_value;
        return dummy_value;
    }
};

struct Group {
    Entry& sub(const std::string& /*name*/) {
        static Entry dummy_entry;
        return dummy_entry;
    }
};

struct Page {
    Group& sub(const std::string& /*name*/) {
        static Group dummy_group;
        return dummy_group;
    }
};

}}