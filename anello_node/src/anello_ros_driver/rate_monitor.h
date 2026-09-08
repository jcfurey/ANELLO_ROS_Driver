#ifndef ANELLO_RATE_MONITOR_H
#define ANELLO_RATE_MONITOR_H
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <utility>

namespace anello {
// Five-second diagnostic window in 100 ms buckets. Storage is fixed even
// under continuous corrupt input; expiry has at most 100 ms quantization.
// Owned by the decode/diagnostics callback group, with single-threaded access.
class RateMonitor {
public:
    using Clock=std::chrono::steady_clock;
    static constexpr double kWindowSeconds=5.0;
    uint64_t total_ok=0, total_checksum_fail=0, total_parse_fail=0;

    void add_ok(uint64_t count=1, Clock::time_point now=Clock::now()) {
        total_ok+=count; record(count,0,now);
    }
    void add_checksum_fail(uint64_t count=1, Clock::time_point now=Clock::now()) {
        total_checksum_fail+=count; record(0,count,now);
    }
    void add_parse_fail(uint64_t count=1, Clock::time_point now=Clock::now()) {
        total_parse_fail+=count; record(0,count,now);
    }
    double rate_hz(Clock::time_point now=Clock::now()) {
        return static_cast<double>(totals(now).first)/kWindowSeconds;
    }
    double error_percent(Clock::time_point now=Clock::now()) {
        const auto [ok,errors]=totals(now);
        const double total=static_cast<double>(ok)+static_cast<double>(errors);
        return total>0?100.0*static_cast<double>(errors)/total:0;
    }

private:
    static constexpr int64_t kBuckets=50;
    static constexpr int64_t kEmpty=std::numeric_limits<int64_t>::min();
    struct Bucket { int64_t tick=kEmpty; uint64_t ok=0, errors=0; };
    std::array<Bucket,kBuckets> recent_{};
    int64_t latest_tick_=kEmpty;

    int64_t tick(Clock::time_point now) {
        const auto current=std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count()/100;
        if (current<latest_tick_) recent_={};
        latest_tick_=current;
        return current;
    }
    void record(uint64_t ok, uint64_t errors, Clock::time_point now) {
        if (ok==0 && errors==0) return;
        const auto current=tick(now);
        auto &bucket=recent_[(current%kBuckets+kBuckets)%kBuckets];
        if (bucket.tick!=current) bucket={current,0,0};
        bucket.ok+=ok; bucket.errors+=errors;
    }
    std::pair<uint64_t,uint64_t> totals(Clock::time_point now) {
        const auto current=tick(now);
        uint64_t ok=0, errors=0;
        for (const auto &bucket:recent_) {
            if (bucket.tick!=kEmpty && current>=bucket.tick && current-bucket.tick<kBuckets) {
                ok+=bucket.ok; errors+=bucket.errors;
            }
        }
        return {ok,errors};
    }
};
}  // namespace anello
#endif
