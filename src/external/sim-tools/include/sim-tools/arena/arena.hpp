#pragma once
#include "array-view.hpp"

namespace sito {

#define ONE_MB 1024 * 1024

constexpr size_t ARENA_SIZE_DEFAULT = 1024 * ONE_MB; // 1 GB

enum class MemType {
    HOST,
    DEVICE,
    DEVICE_PINNED,
    NONE
};

/*Simple allocator, can only allocate, no freeing*/
class ArenaBump {
public:
    uint capacity_;
    uint offset_;
    MemType mem_type;
    byte *buf;

    // ArenaBump() { offset_ = 1; } // skipping the first byte so that we can check if the offset is 0
    void create(MemType mem_type_, size_t size = ARENA_SIZE_DEFAULT);
    void destroy();

    static void copy_all(ArenaBump &dst, const ArenaBump &src);

    static void copy_slice(ArenaBump &dst, const ArenaBump &src, uint begin, uint end);

    /*Returns the actual pointer in memory and the offset in number of bytes relative to the start of the memory buffer
    offset will be most useful in this application. Pass nullptr to arguments without interest. */

    template <typename T> void allocate(Offset<T> *offset, uint count, T **ptr = nullptr) {
        assert(offset->count == 0 &&
               offset->offset == 0); // safety measures to ensure that it hasnt been previously allocated
        offset->count = count;
        allocate<T>(count, ptr, &offset->offset);
    }

    /*Normal allocation when you just want an adress from a count*/
    template <typename T> void allocate(uint count, T **ptr) {
        uint dummy = 0; // the exisiting api requires this
        // if (*ptr == nullptr) {
        //     *ptr =
        // } else {
        //     assert(false);
        // }
        allocate<T>(count, ptr, &dummy);
    }

  private:
    template <typename T> void allocate(uint count, T **ptr, uint *offset) {
        static_assert(!std::is_same_v<T, bool>, "Allocation of bool is not allowed");
        assert(offset != nullptr);
        assert(*offset == 0); /*A safety measure to ensure that arrays arent doubly allocated.
          The offset expects to be initialized with value 0*/
        uint size_of_new = count * sizeof(T);
        // This is some gpt generated stuff: DOnt't fully understand it, but has something to do about alignment,
        // for instance if T is and int (4 bytes) we want to start at a location which is a multiple of 4 bytes I think.
        uint alignment = alignof(T);
        uint aligned_offset;
        if (offset_ % alignment == 0) {
            aligned_offset = offset_;
        } else {
            aligned_offset = offset_ + alignment - (offset_ % alignment);
        }

        offset_ = aligned_offset + size_of_new;
        if (offset_ > capacity_) {

            THROW_RUNTIME_ERROR("Allocator capacity reached (" + to_string(bytes_to_gigabytes(capacity_)) +
                                " GB). Increase size to solve the current problem");
        }

        *offset = aligned_offset;
        if (ptr != nullptr) {
            *ptr = (T *)(buf + aligned_offset);
        }
    }
    scalar bytes_to_gigabytes(uint bytes);
};
} // namespace sito