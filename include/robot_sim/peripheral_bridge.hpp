#pragma once
#include "robot_sim/simulation.hpp"
#include <variant>

namespace robot_sim {
using DeviceId = uint64_t;
using TxId = uint64_t;
using Tick = uint64_t;

enum class TransferKind { command, read, write };
struct AbstractParams { uint32_t address; };
using ProtocolParams = std::variant<AbstractParams>;

struct PeripheralDevice {
    Protocol protocol = Protocol::abstract;
    Pins pins{0, 1};
    uint32_t address = 7;
    uint32_t max_payload = 65536;
    uint32_t bytes_per_second = UINT32_MAX;
    bool available = true;
    bool data_ready_irq = false;
};

struct PeripheralRequest {
    TxId id;
    RobotId robot;
    uint32_t controller;
    DeviceId device;
    Protocol protocol;
    TransferKind direction;
    ProtocolParams params;
    uint32_t command, expected_rx_bytes;
    Tick accepted_tick;
    uint64_t source_seq;
    std::vector<uint8_t> tx;
};

enum class HostResult { success, busy, device_error, no_response };
struct PeripheralResponse {
    TxId id;
    HostResult result;
    Tick earliest_ready_tick;
    uint32_t response = 0;
    std::vector<uint8_t> rx;
};

enum class PeripheralEventKind { data_ready, disconnected };
struct PeripheralEvent {
    DeviceId device;
    PeripheralEventKind kind;
    Tick earliest_delivery_tick;
    uint64_t host_seq;
};
struct PeripheralInput {
    std::vector<PeripheralResponse> responses;
    std::vector<PeripheralEvent> events;
};

enum class BridgeError { unknown_transaction, duplicate_response, invalid_response,
                         unknown_device, invalid_event, input_limit };
enum class TransactionOutcome { completed, failed, timed_out, cancelled };
struct RejectedInput { uint64_t id; BridgeError error; };
struct TransactionEnd { TxId id; RobotId robot; DeviceId device; TransactionOutcome outcome; };
struct PeripheralStep {
    Tick tick;
    uint64_t time_ns, checksum;
    std::vector<PeripheralRequest> requests;
    std::vector<TransactionEnd> terminal;
    std::vector<RejectedInput> rejected;
};

// Call from one coordinator thread at quiescent tick boundaries.
class PeripheralBridge {
public:
    static constexpr uint32_t api_version = 1;
    static constexpr uint32_t data_ready_irq = 8;
    explicit PeripheralBridge(SimulatorConfig config);
    RobotId add_robot(const RobotConfig& config, std::shared_ptr<const Module> module);
    bool remove_robot(RobotId id);
    DeviceId register_device(PeripheralDevice device);
    bool remove_device(DeviceId id);
    void connect(RobotId robot, DeviceId device);
    bool cancel(TxId id);
    PeripheralStep step(PeripheralInput input = {});
    RobotSnapshot inspect(RobotId id) const { return simulation_.inspect(id); }
private:
    struct Connection {
        std::optional<ControllerConfig> controller;
        DeviceId device = 0;
    };
    struct Device {
        PeripheralDevice config;
        uint64_t last_event = 0;
        TxId active = 0;
    };
    struct InFlight {
        RobotId robot;
        DeviceId device;
        uint64_t sequence;
        Tick accepted, deadline;
        uint32_t expected_rx, wire_bytes, rate;
        uint64_t credits = 0;
        std::optional<PeripheralResponse> response;
        bool cancelled = false;
    };
    void disconnect(DeviceId id);
    void release(TxId id, const InFlight& transfer);
    Simulation simulation_;
    uint32_t tick_rate_;
    Tick tick_ = 0;
    DeviceId next_device_ = 1;
    TxId next_tx_ = 1;
    std::map<RobotId, Connection> connections_;
    std::map<DeviceId, Device> devices_;
    std::map<TxId, InFlight> pending_;
    std::vector<PeripheralEvent> events_;
    std::vector<TransactionEnd> removed_;
    std::vector<Completion> route_failures_;
};
}

