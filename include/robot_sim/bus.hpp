#pragma once
#include <cstdint>
#include <stdexcept>
#include <optional>
#include <vector>
#include <span>

namespace robot_sim {
enum class FaultKind { unmapped, bounds, width, alignment, register_access };
struct Fault : std::runtime_error {
    FaultKind kind;
    uint64_t address;
    Fault(FaultKind kind, uint64_t address);
};
enum class Power : uint32_t { active, low_power, sleep };
struct Pins { unsigned clock, data; };
enum class Protocol : uint32_t { abstract = 1 };
struct ControllerConfig {
    Protocol protocol = Protocol::abstract;
    Pins controller{0, 1}, peripheral{0, 1};
    uint32_t address = 7, timeout_ticks = 8;
    uint32_t dma_max_bytes = 0;
    uint32_t bytes_per_second = UINT32_MAX;
};
struct Transfer {
    uint32_t target, command, dma_address = 0, dma_size = 0, direction = 0;
    std::vector<uint8_t> payload;
};

class Bus {
public:
    static constexpr uint32_t page_size = 65536;
    explicit Bus(uint32_t initial, uint32_t maximum, uint32_t base = 0x40000000);
    uint64_t load(uint64_t address, unsigned width) const;
    void store(uint64_t address, unsigned width, uint64_t value);
    uint32_t pages() const;
    uint32_t grow(uint32_t delta);
    void connect(Pins controller, Pins peripheral, uint32_t address);
    void tick();
    void configure(std::optional<ControllerConfig> controller);
    std::optional<Transfer> take_request();
    void complete(uint32_t status, uint32_t response, std::span<const uint8_t> payload = {});
    void boundary(uint64_t time_ns, uint32_t irq, bool check_alarm = true);
    std::optional<uint64_t> alarm_deadline() const {
        if (!alarm_armed_) return {};
        return alarm_;
    }
    uint32_t pending_irq() const { return irq_; }
    uint64_t mmio_accesses() const { return accesses_; }
    uint64_t alarm_count() const { return alarms_; }
    uint64_t routing_errors() const { return routing_errors_; }
    uint32_t controller_status() const { return status_; }
    uint32_t controller_response() const { return response_; }
    uint64_t rtc_ns() const { return time_ns_; }
    std::span<const uint8_t> ram() const { return ram_; }
    uint64_t checksum() const;
    Power power() const { return power_; }
    uint32_t base() const { return base_; }
private:
    uint32_t mmio_offset(uint64_t address, unsigned width) const;
    bool in_ram(uint64_t address, unsigned width) const;
    std::vector<uint8_t> ram_;
    uint32_t maximum_, base_;
    Power power_ = Power::active;
    uint32_t irq_ = 0, wake_mask_ = 0, gpio_ = 0;
    uint64_t ticks_ = 0;
    uint32_t target_ = 0, response_ = 0, status_ = 0;
    std::optional<Transfer> pending_;
    std::optional<uint32_t> peripheral_address_;
    bool wired_ = false;
    bool enabled_ = true, issued_ = false;
    uint32_t dma_limit_ = 0, dma_address_ = 0, dma_size_ = 0;
    uint64_t time_ns_ = 0, alarm_ = 0, alarm_shadow_ = 0, alarms_ = 0;
    uint64_t routing_errors_ = 0;
    bool alarm_armed_ = false;
    mutable uint64_t accesses_ = 0;
};
}

