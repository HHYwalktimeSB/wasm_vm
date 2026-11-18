#include <string>
#include "robot_sim/bus.hpp"
#include "checksum.hpp"

namespace robot_sim {
Fault::Fault(FaultKind kind, uint64_t address)
    : std::runtime_error("memory fault " + std::to_string(static_cast<int>(kind)) +
                         " at " + std::to_string(address)), kind(kind), address(address) {}

Bus::Bus(uint32_t initial, uint32_t maximum, uint32_t base)
    : maximum_(maximum), base_(base) {
    if (initial > maximum || uint64_t(maximum) * page_size > base ||
        base % 4 || uint64_t(base) + 0x200 > (uint64_t{1} << 32))
        throw std::invalid_argument("invalid RAM or controller mapping");
    ram_.resize(size_t(initial) * page_size);
}

uint32_t Bus::pages() const { return static_cast<uint32_t>(ram_.size() / page_size); }

uint32_t Bus::grow(uint32_t delta) {
    const auto previous = pages();
    if (delta > maximum_ - previous) return UINT32_MAX;
    try { ram_.resize(size_t(previous + delta) * page_size); }
    catch (const std::bad_alloc&) { return UINT32_MAX; }
    catch (const std::length_error&) { return UINT32_MAX; }
    return previous;
}

bool Bus::in_ram(uint64_t address, unsigned width) const {
    if (width != 1 && width != 2 && width != 4 && width != 8)
        throw Fault(FaultKind::width, address);
    if (address >= base_) return false;
    if (address >= ram_.size() || width > ram_.size() - address)
        throw Fault(FaultKind::bounds, address);
    return true;
}

uint32_t Bus::mmio_offset(uint64_t address, unsigned width) const {
    if (address - base_ >= 0x200) throw Fault(FaultKind::unmapped, address);
    if (width != 4) throw Fault(FaultKind::width, address);
    if (address % 4) throw Fault(FaultKind::alignment, address);
    const auto offset = static_cast<uint32_t>(address - base_);
    if ((offset >= 0x100 && !enabled_) || (offset >= 0x120 && !dma_limit_))
        throw Fault(FaultKind::register_access, address);
    ++accesses_;
    return offset;
}

uint64_t Bus::load(uint64_t address, unsigned width) const {
    if (in_ram(address, width)) {
        uint64_t result = 0;
        for (unsigned i = 0; i < width; ++i)
            result |= uint64_t(ram_[size_t(address) + i]) << (8 * i);
        return result;
    }
    switch (mmio_offset(address, width)) {
    case 0x00: return static_cast<uint32_t>(power_);
    case 0x10: return static_cast<uint32_t>(time_ns_);
    case 0x14: return time_ns_ >> 32;
    case 0x18: return static_cast<uint32_t>(alarm_shadow_);
    case 0x1c: return alarm_shadow_ >> 32;
    case 0x20: return irq_;
    case 0x24: return wake_mask_;
    case 0x28: return alarm_armed_;
    case 0x30: return gpio_;
    case 0x100: return target_;
    case 0x108: return status_;
    case 0x10c: return response_;
    case 0x120: return dma_address_;
    case 0x124: return dma_size_;
    default: throw Fault(FaultKind::register_access, address);
    }
}

void Bus::store(uint64_t address, unsigned width, uint64_t value) {
    if (in_ram(address, width)) {
        for (unsigned i = 0; i < width; ++i)
            ram_[size_t(address) + i] = static_cast<uint8_t>(value >> (8 * i));
        return;
    }
    const auto word = static_cast<uint32_t>(value);
    switch (mmio_offset(address, width)) {
    case 0x00:
        if (word > 2) break;
        power_ = static_cast<Power>(word);
        return;
    case 0x20: irq_ &= ~word; return;
    case 0x18: alarm_shadow_ = (alarm_shadow_ & 0xffffffff00000000ull) | word; return;
    case 0x1c: alarm_shadow_ = (alarm_shadow_ & 0xffffffffull) | (uint64_t(word) << 32); return;
    case 0x24: wake_mask_ = word; return;
    case 0x28:
        if (word > 1) break;
        alarm_armed_ = word != 0;
        if (alarm_armed_) alarm_ = alarm_shadow_;
        return;
    case 0x30: gpio_ = word; return;
    case 0x100: target_ = word; return;
    case 0x120: dma_address_ = word; return;
    case 0x124: dma_size_ = word; return;
    case 0x104: case 0x128: {
        if (pending_) break;
        Transfer request{target_, word};
        if (address - base_ == 0x128) {
            if (word != 1 && word != 2) break;
            if (!dma_size_ || dma_size_ > dma_limit_ ||
                uint64_t(dma_address_) + dma_size_ > ram_.size())
                throw Fault(FaultKind::bounds, dma_address_);
            request.dma_address = dma_address_;
            request.dma_size = dma_size_;
            request.direction = word;
        }
        pending_ = std::move(request);
        issued_ = false;
        status_ = 1;
        return;
    }
    }
    throw Fault(FaultKind::register_access, address);
}

void Bus::connect(Pins controller, Pins peripheral, uint32_t address) {
    if (controller.clock >= 32 || controller.data >= 32 || peripheral.clock >= 32 ||
        peripheral.data >= 32 || controller.clock == controller.data ||
        peripheral.clock == peripheral.data)
        throw std::invalid_argument("invalid CLK/DATA pin assignment");
    wired_ = controller.clock == peripheral.clock && controller.data == peripheral.data;
    peripheral_address_ = address;
}

void Bus::tick() {
    ++ticks_;
    if (const auto request = take_request())
        complete(2, request->command ^ 0xa5u);
    boundary(ticks_, 0);
}

void Bus::configure(std::optional<ControllerConfig> controller) {
    if (pending_) throw std::logic_error("controller is busy");
    if (controller) {
        if (controller->protocol != Protocol::abstract || !controller->timeout_ticks ||
            !controller->bytes_per_second ||
            controller->dma_max_bytes > 65536)
            throw std::invalid_argument("unsupported controller configuration");
        connect(controller->controller, controller->peripheral, controller->address);
    }
    enabled_ = controller.has_value();
    dma_limit_ = controller ? controller->dma_max_bytes : 0;
}

std::optional<Transfer> Bus::take_request() {
    if (!pending_ || issued_) return {};
    if (!wired_ || peripheral_address_ != pending_->target) {
        ++routing_errors_;
        complete(3, 0);
        return {};
    }
    auto request = *pending_;
    if (request.direction == 2) {
        const auto first = ram_.begin() + request.dma_address;
        request.payload.assign(first, first + request.dma_size);
    }
    issued_ = true;
    return request;
}

void Bus::complete(uint32_t status, uint32_t response, std::span<const uint8_t> payload) {
    if (!pending_) return;
    if (status == 2 && pending_->direction == 1) {
        if (payload.size() == pending_->dma_size) {
            for (size_t i = 0; i < payload.size(); ++i)
                ram_[pending_->dma_address + i] = payload[i];
        } else status = 3;
    }
    status_ = status;
    if (status == 2) response_ = response;
    irq_ |= pending_->direction ? 4u : 1u;
    pending_.reset();
}

void Bus::boundary(uint64_t time_ns, uint32_t irq, bool check_alarm) {
    time_ns_ = time_ns;
    irq_ |= irq;
    if (check_alarm && alarm_armed_ && time_ns_ >= alarm_) {
        alarm_armed_ = false;
        irq_ |= 2;
        ++alarms_;
    }
    if (power_ == Power::sleep && (irq_ & wake_mask_)) power_ = Power::active;
}

uint64_t Bus::checksum() const {
    Checksum hash;
    hash.word(ram_.size());
    for (auto byte : ram_) hash.byte(byte);
    for (auto word : {uint64_t(static_cast<uint32_t>(power_)), time_ns_, alarm_, alarm_shadow_,
                      uint64_t(alarm_armed_), uint64_t(irq_), uint64_t(wake_mask_),
                      uint64_t(gpio_), uint64_t(target_), uint64_t(response_),
                      uint64_t(status_), uint64_t(dma_address_), uint64_t(dma_size_)})
        hash.word(word);
    hash.word(pending_.has_value());
    if (pending_) {
        hash.word(pending_->target);
        hash.word(pending_->command);
        hash.word(pending_->dma_address);
        hash.word(pending_->dma_size);
        hash.word(pending_->direction);
        hash.word(issued_);
    }
    return hash.value();
}
}

