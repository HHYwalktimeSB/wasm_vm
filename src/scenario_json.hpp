#pragma once

#include <charconv>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace scenario_json {
struct Value {
    using Object = std::map<std::string, Value, std::less<>>;
    using Array = std::vector<Value>;
    std::variant<uint64_t, Object, Array> data;
};

class Parser {
public:
    explicit Parser(std::string_view text) : text_(text) {}
    Value parse() {
        auto result = value(0);
        whitespace();
        if (!text_.empty()) fail();
        return result;
    }

private:
    [[noreturn]] static void fail() { throw std::invalid_argument("invalid scenario JSON"); }
    void whitespace() {
        const auto offset = text_.find_first_not_of(" \t\r\n");
        text_.remove_prefix(offset == text_.npos ? text_.size() : offset);
    }
    bool take(char token) {
        whitespace();
        if (text_.empty() || text_.front() != token) return false;
        text_.remove_prefix(1);
        return true;
    }
    std::string key() {
        if (!take('"')) fail();
        std::string result;
        while (!text_.empty()) {
            const auto ch = text_.front();
            text_.remove_prefix(1);
            if (ch == '"') return result;
            if (ch == '\\') {
                if (text_.empty() || (text_.front() != '"' && text_.front() != '\\')) fail();
                result += text_.front();
                text_.remove_prefix(1);
            } else {
                if (static_cast<unsigned char>(ch) < 32) fail();
                result += ch;
            }
        }
        fail();
    }
    Value value(unsigned depth) {
        if (depth > 12) fail();
        if (take('{')) {
            Value::Object result;
            if (!take('}')) {
                do {
                    auto name = key();
                    if (!take(':')) fail();
                    if (!result.emplace(std::move(name), value(depth + 1)).second) fail();
                } while (take(','));
                if (!take('}')) fail();
            }
            return {std::move(result)};
        }
        if (take('[')) {
            Value::Array result;
            if (!take(']')) {
                do { result.push_back(value(depth + 1)); } while (take(','));
                if (!take(']')) fail();
            }
            return {std::move(result)};
        }
        uint64_t number = 0;
        if (text_.empty()) fail();
        const auto [last, error] = std::from_chars(text_.data(), text_.data() + text_.size(), number);
        if (error != std::errc{} || (last - text_.data() > 1 && text_.front() == '0')) fail();
        text_.remove_prefix(last - text_.data());
        return {number};
    }
    std::string_view text_;
};

inline Value::Object read(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    const auto size = file.tellg();
    if (size < 0 || size > 1024 * 1024) throw std::invalid_argument("cannot open JSON or file exceeds 1 MiB");
    std::string text(static_cast<size_t>(size), '\0');
    file.seekg(0);
    if (!file.read(text.data(), size)) throw std::invalid_argument("cannot read JSON file");
    auto root = Parser(text).parse();
    auto* object = std::get_if<Value::Object>(&root.data);
    if (!object) throw std::invalid_argument("expected JSON object");
    const auto version = object->find("version");
    if (version == object->end() || std::get_if<uint64_t>(&version->second.data) == nullptr ||
        std::get<uint64_t>(version->second.data) != 1)
        throw std::invalid_argument("expected JSON schema version 1");
    object->erase(version);
    return std::move(*object);
}

inline uint64_t number(const Value& value, uint64_t maximum = UINT64_MAX) {
    const auto* result = std::get_if<uint64_t>(&value.data);
    if (!result || *result > maximum) throw std::invalid_argument("invalid numeric JSON field");
    return *result;
}

inline uint64_t field(const Value::Object& object, std::string_view key, uint64_t maximum) {
    const auto found = object.find(key);
    if (found == object.end()) throw std::invalid_argument("missing JSON field " + std::string(key));
    return number(found->second, maximum);
}
}

