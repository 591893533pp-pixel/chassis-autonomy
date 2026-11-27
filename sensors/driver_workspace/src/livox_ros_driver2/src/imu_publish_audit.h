#pragma once
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <mutex>
#include <stdexcept>
// Optional bounded-buffer publication audit. Never alters or synthesizes an IMU.
class ImuPublishAudit {
    FILE* file_=nullptr;
    std::mutex mutex_;
    unsigned rows_=0;
    ImuPublishAudit() {
        const char* path=std::getenv("LIVOX_IMU_AUDIT_FILE");
        if(path && *path) {
            file_=std::fopen(path,"a");
            if(!file_) throw std::runtime_error("Cannot open IMU publication audit");
            std::fprintf(file_,"event,steady_ns,index,stamp_ns\n");
            std::fflush(file_);
        }
    }
    ~ImuPublishAudit(){if(file_)std::fclose(file_);}
public:
    static void record(const char* event,uint8_t index,uint64_t stamp) {
        static ImuPublishAudit audit;
        if(!audit.file_)return;
        auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        std::lock_guard<std::mutex> lock(audit.mutex_);
        if(std::fprintf(audit.file_,"%s,%lld,%u,%llu\n",event,
            static_cast<long long>(now),unsigned(index),static_cast<unsigned long long>(stamp))<0)
            throw std::runtime_error("IMU publication audit write failed");
        if(++audit.rows_%200==0 && std::fflush(audit.file_)!=0)
            throw std::runtime_error("IMU publication audit flush failed");
    }
};
