#include "sim-tools/arena/arena.hpp"
namespace sito {
void ArenaBump::create(MemType mem_type_, size_t size) {
    mem_type = mem_type_;
    capacity_ = size;
    offset_ = 1;               // skipping the first byte so that we can check if the offset is 0
    assert(size < 4294967296); /*Largest number that an u32 can hold*/
    
    if (mem_type == MemType::HOST) {
        buf = new byte[capacity_];
    } else if (mem_type == MemType::DEVICE) {
        assert(CUDA_ENABLED);        
        //!!REBOOT SYSTEM IF IT FAILS HERE!!
        CUDA_CHECK_ERROR(cudaMalloc((void **)&buf, capacity_)); 
    } else {
        assert(CUDA_ENABLED);
        assert(mem_type == MemType::DEVICE_PINNED);
        CUDA_CHECK_ERROR(cudaMallocHost((void **)&buf, capacity_));
    }
}
void ArenaBump::destroy() {
    if (buf != nullptr) {
        if (mem_type == MemType::HOST) {
            delete[] buf;
        } else if (mem_type == MemType::DEVICE) {
            CUDA_CHECK_ERROR(cudaFree(buf));
        } else {
            assert(mem_type == MemType::DEVICE_PINNED);
            CUDA_CHECK_ERROR(cudaFreeHost(buf));
        }
    }
}

void ArenaBump::copy_slice(ArenaBump &dst, const ArenaBump &src, uint begin, uint end) {
    assert(end > begin);
    assert(dst.capacity_ >= end);
    assert(src.capacity_ >= end);
    const uint size = end - begin;
    if ((dst.mem_type == MemType::DEVICE || dst.mem_type == MemType::DEVICE_PINNED) && src.mem_type == MemType::HOST) {
        CUDA_CHECK_ERROR(cudaMemcpy(dst.buf + begin, src.buf + begin, size, cudaMemcpyHostToDevice));
    } else if (dst.mem_type == MemType::HOST &&
               (src.mem_type == MemType::DEVICE || src.mem_type == MemType::DEVICE_PINNED)) {
        CUDA_CHECK_ERROR(cudaMemcpy(dst.buf + begin, src.buf + begin, size, cudaMemcpyDeviceToHost));
    } else if (dst.mem_type == MemType::HOST && src.mem_type == MemType::HOST) {
        std::memcpy(dst.buf + begin, src.buf + begin, size);
    }
    else 
    assert(false); // device to device copy not implemented
}

void ArenaBump::copy_all(ArenaBump &dst, const ArenaBump &src) {
    assert(dst.capacity_ == src.capacity_); /*They strightly don't have to have the same capacity, but just enforcing
                                               this for now*/
    copy_slice(dst, src, 0, src.offset_);
    dst.offset_ = src.offset_;
}

scalar ArenaBump::bytes_to_gigabytes(uint bytes) {
    constexpr scalar bytes_per_gigabyte = 1024.0 * 1024.0 * 1024.0;
    const scalar GB = bytes / bytes_per_gigabyte;
    return GB;
}
} // namespace sito