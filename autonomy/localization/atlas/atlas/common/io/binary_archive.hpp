#pragma once

#include <cstdint>
#include <istream>
#include <list>
#include <map>
#include <ostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autonomy::localization::atlas {

class GeometricCamera;
class PinholeCamera;
class KannalaBrandtCamera;

class ArchiveAccess
{
public:
    template <class Archive, class T>
    static void Serialize(Archive& archive, T& value) {
        value.serialize(archive, 0);
    }
};

template <class T>
struct ArrayView {
    T* data;
    std::size_t size;
};

template <class T>
ArrayView<T> MakeArray(T* data, std::size_t size) {
    return {data, size};
}

class BinaryOutputArchive
{
public:
    using is_saving = std::true_type;
    using is_loading = std::false_type;

    explicit BinaryOutputArchive(std::ostream& stream) : stream_(stream) {}

    template <class T>
    BinaryOutputArchive& operator&(T& value) {
        Write(value);
        return *this;
    }
    template <class T>
    BinaryOutputArchive& operator&(ArrayView<T> value) {
        Write(value);
        return *this;
    }
    template <class T>
    BinaryOutputArchive& operator<<(T& value) {
        return *this & value;
    }
    template <class T>
    BinaryOutputArchive& operator<<(const T& value) {
        auto& mutable_value = const_cast<T&>(value);
        return *this & mutable_value;
    }
    template <class T>
    void register_type() {}

private:
    template <class T>
    std::enable_if_t<std::is_arithmetic_v<T> || std::is_enum_v<T>> Write(
        T& value) {
        stream_.write(reinterpret_cast<const char*>(&value), sizeof(value));
        Check();
    }
    void Write(std::string& value) {
        std::uint64_t size = value.size();
        Write(size);
        stream_.write(value.data(), static_cast<std::streamsize>(size));
        Check();
    }
    template <class T>
    void Write(ArrayView<T> value) {
        stream_.write(reinterpret_cast<const char*>(value.data),
                      static_cast<std::streamsize>(value.size * sizeof(T)));
        Check();
    }
    template <class A, class B>
    void Write(std::pair<A, B>& value) {
        *this& value.first& value.second;
    }
    template <class T, class Alloc>
    void Write(std::vector<T, Alloc>& values) {
        std::uint64_t size = values.size();
        Write(size);
        for (auto& value : values)
            *this& value;
    }
    void Write(std::vector<bool>& values) {
        std::uint64_t size = values.size();
        Write(size);
        for (bool value : values)
            *this& value;
    }
    template <class T, class Alloc>
    void Write(std::list<T, Alloc>& values) {
        std::uint64_t size = values.size();
        Write(size);
        for (auto& value : values)
            *this& value;
    }
    template <class K, class V, class Compare, class Alloc>
    void Write(std::map<K, V, Compare, Alloc>& values) {
        std::uint64_t size = values.size();
        Write(size);
        for (auto& [key, value] : values) {
            auto mutable_key = key;
            *this& mutable_key& value;
        }
    }
    template <class K, class V, class Hash, class Equal, class Alloc>
    void Write(std::unordered_map<K, V, Hash, Equal, Alloc>& values) {
        std::uint64_t size = values.size();
        Write(size);
        for (auto& [key, value] : values) {
            auto mutable_key = key;
            *this& mutable_key& value;
        }
    }
    template <class T>
    void Write(T*& pointer) {
        std::uint64_t id = 0;
        bool is_new = false;
        if (pointer != nullptr) {
            const auto [it, inserted] = pointer_ids_.emplace(pointer, next_id_);
            id = it->second;
            is_new = inserted;
            if (inserted)
                ++next_id_;
        }
        Write(id);
        if (id == 0)
            return;
        Write(is_new);
        if (is_new)
            WritePointedObject(pointer);
    }
    template <class T>
    void WritePointedObject(T* pointer) {
        *this&* pointer;
    }
    void WritePointedObject(GeometricCamera* pointer);
    template <class T>
    std::enable_if_t<!std::is_arithmetic_v<T> && !std::is_enum_v<T>> Write(
        T& value) {
        ArchiveAccess::Serialize(*this, value);
    }
    void Check() {
        if (!stream_)
            throw std::runtime_error("failed to write Atlas archive");
    }

    std::ostream& stream_;
    std::unordered_map<const void*, std::uint64_t> pointer_ids_;
    std::uint64_t next_id_ = 1;
};

class BinaryInputArchive
{
public:
    using is_saving = std::false_type;
    using is_loading = std::true_type;

    explicit BinaryInputArchive(std::istream& stream) : stream_(stream) {}

    template <class T>
    BinaryInputArchive& operator&(T& value) {
        Read(value);
        return *this;
    }
    template <class T>
    BinaryInputArchive& operator&(ArrayView<T> value) {
        Read(value);
        return *this;
    }
    template <class T>
    BinaryInputArchive& operator>>(T& value) {
        return *this & value;
    }
    template <class T>
    void register_type() {}

private:
    template <class T>
    std::enable_if_t<std::is_arithmetic_v<T> || std::is_enum_v<T>> Read(
        T& value) {
        stream_.read(reinterpret_cast<char*>(&value), sizeof(value));
        Check();
    }
    void Read(std::string& value) {
        std::uint64_t size = 0;
        Read(size);
        value.resize(CheckedSize(size));
        stream_.read(value.data(), static_cast<std::streamsize>(size));
        Check();
    }
    template <class T>
    void Read(ArrayView<T> value) {
        stream_.read(reinterpret_cast<char*>(value.data),
                     static_cast<std::streamsize>(value.size * sizeof(T)));
        Check();
    }
    template <class A, class B>
    void Read(std::pair<A, B>& value) {
        *this& value.first& value.second;
    }
    template <class T, class Alloc>
    void Read(std::vector<T, Alloc>& values) {
        std::uint64_t size = 0;
        Read(size);
        values.resize(CheckedSize(size));
        for (auto& value : values)
            *this& value;
    }
    void Read(std::vector<bool>& values) {
        std::uint64_t size = 0;
        Read(size);
        values.assign(CheckedSize(size), false);
        for (std::size_t i = 0; i < values.size(); ++i) {
            bool value = false;
            *this& value;
            values[i] = value;
        }
    }
    template <class T, class Alloc>
    void Read(std::list<T, Alloc>& values) {
        std::uint64_t size = 0;
        Read(size);
        values.clear();
        for (std::uint64_t i = 0; i < size; ++i) {
            values.emplace_back();
            *this& values.back();
        }
    }
    template <class K, class V, class Compare, class Alloc>
    void Read(std::map<K, V, Compare, Alloc>& values) {
        std::uint64_t size = 0;
        Read(size);
        values.clear();
        for (std::uint64_t i = 0; i < size; ++i) {
            K key{};
            V value{};
            *this& key& value;
            values.emplace(std::move(key), std::move(value));
        }
    }
    template <class K, class V, class Hash, class Equal, class Alloc>
    void Read(std::unordered_map<K, V, Hash, Equal, Alloc>& values) {
        std::uint64_t size = 0;
        Read(size);
        values.clear();
        for (std::uint64_t i = 0; i < size; ++i) {
            K key{};
            V value{};
            *this& key& value;
            values.emplace(std::move(key), std::move(value));
        }
    }
    template <class T>
    void Read(T*& pointer) {
        std::uint64_t id = 0;
        Read(id);
        if (id == 0) {
            pointer = nullptr;
            return;
        }
        bool is_new = false;
        Read(is_new);
        if (!is_new) {
            const auto it = pointers_.find(id);
            if (it == pointers_.end())
                throw std::runtime_error("invalid Atlas pointer reference");
            pointer = static_cast<T*>(it->second);
            return;
        }
        ReadNewPointer(id, pointer);
    }
    template <class T>
    void ReadNewPointer(std::uint64_t id, T*& pointer) {
        pointer = new T();
        pointers_[id] = pointer;
        *this&* pointer;
    }
    void ReadNewPointer(std::uint64_t id, GeometricCamera*& pointer);
    template <class T>
    std::enable_if_t<!std::is_arithmetic_v<T> && !std::is_enum_v<T>> Read(
        T& value) {
        ArchiveAccess::Serialize(*this, value);
    }
    std::size_t CheckedSize(std::uint64_t size) const {
        constexpr std::uint64_t kMaximumElements = 100000000;
        if (size > kMaximumElements)
            throw std::runtime_error("invalid Atlas archive size");
        return static_cast<std::size_t>(size);
    }
    void Check() {
        if (!stream_)
            throw std::runtime_error("failed to read Atlas archive");
    }

    std::istream& stream_;
    std::unordered_map<std::uint64_t, void*> pointers_;
};

}  // namespace autonomy::localization::atlas
