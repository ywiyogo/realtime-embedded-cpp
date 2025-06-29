#pragma once

#include <memory>
#include <array>
#include <type_traits>
#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <cmath>
#include <utility>

namespace aerospace {

// ============================================================================
// COMPILE-TIME CONSTANTS AND CONFIGURATION
// ============================================================================

// C++23 constexpr configurations for real-time constraints
constexpr std::size_t MAX_SENSORS = 4;
constexpr std::size_t SENSOR_BUFFER_SIZE = 50;
constexpr double CRITICAL_ALTITUDE_THRESHOLD = 1000.0;
constexpr std::uint32_t SENSOR_TIMEOUT_MS = 50;
constexpr std::size_t MAX_TYPE_SIZE_BYTES = 8;  // Embedded constraint for sensor types
constexpr std::size_t MIN_IDENTIFIER_LENGTH = 3;
constexpr std::uint64_t TIME_SIMULATION_INCREMENT_US = 1000;  // 1ms increments
constexpr double MAX_TEMPERATURE_CELSIUS = 85.0;
constexpr double MIN_PRESSURE_PA = 50000.0;

// C++23 constexpr function for compile-time validation
[[nodiscard]] constexpr auto is_valid_sensor_id(std::size_t sensor_id) noexcept -> bool {
    return sensor_id < MAX_SENSORS;
}

// ============================================================================
// TEMPLATE UTILITIES AND TYPE TRAITS (C++23 CONCEPTS)
// ============================================================================

// Template concept for sensor data validation (C++20/23 concept)
template<typename T>
concept SensorDataType = std::is_arithmetic_v<T> &&
                        !std::is_same_v<T, bool> &&
                        sizeof(T) <= MAX_TYPE_SIZE_BYTES;

// Template for compile-time sensor range validation
template<SensorDataType T>
[[nodiscard]] constexpr auto is_in_valid_range(T value, T min_val, T max_val) noexcept -> bool {
    return value >= min_val && value <= max_val;
}

// ============================================================================
// MEMORY POOL ALLOCATOR (NO DYNAMIC ALLOCATION)
// ============================================================================

template<typename T, std::size_t PoolSize>
class StaticMemoryPool {
public:
    static constexpr std::size_t pool_size = PoolSize;

    constexpr StaticMemoryPool() noexcept = default;

    [[nodiscard]] auto allocate() noexcept -> T* {
        if (next_free_ >= PoolSize) {
            return nullptr; // Pool exhausted - handle gracefully
        }
        return &pool_[next_free_++];
    }

    void deallocate(T* ptr) noexcept {
        // Simple deallocation - in real system would have proper free list
        if (ptr == &pool_[next_free_ - 1] && next_free_ > 0) {
            --next_free_;
        }
    }

    [[nodiscard]] constexpr auto available() const noexcept -> std::size_t {
        return PoolSize - next_free_;
    }

private:
    std::array<T, PoolSize> pool_{};
    std::size_t next_free_{0};
};

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

struct SensorData {
private:
    double value_;
    std::uint64_t timestamp_us_;
    std::uint8_t sensor_id_;
    bool is_valid_;

public:
    constexpr SensorData() noexcept
        : value_(0.0), timestamp_us_(0), sensor_id_(0), is_valid_(false) {}

    constexpr SensorData(double sensor_value, std::uint64_t timestamp_microseconds, std::uint8_t sensor_identifier) noexcept
        : value_(sensor_value), timestamp_us_(timestamp_microseconds), sensor_id_(sensor_identifier), is_valid_(true) {}

    [[nodiscard]] constexpr auto value() const noexcept -> double { return value_; }
    [[nodiscard]] constexpr auto timestamp_us() const noexcept -> std::uint64_t { return timestamp_us_; }
    [[nodiscard]] constexpr auto sensor_id() const noexcept -> std::uint8_t { return sensor_id_; }
    [[nodiscard]] constexpr auto is_valid() const noexcept -> bool { return is_valid_; }
};

// ============================================================================
// ABSTRACT INTERFACES (DEPENDENCY INJECTION TARGETS)
// ============================================================================

class ISensor {
public:
    virtual ~ISensor() = default;
    ISensor(const ISensor&) = delete;
    ISensor& operator=(const ISensor&) = delete;
    ISensor(ISensor&&) = default;
    ISensor& operator=(ISensor&&) = default;

    virtual auto read() noexcept -> SensorData = 0;
    [[nodiscard]] virtual auto is_healthy() const noexcept -> bool = 0;
    [[nodiscard]] virtual auto get_id() const noexcept -> std::uint8_t = 0;
    [[nodiscard]] virtual auto get_name() const noexcept -> const char* = 0;

protected:
    ISensor() = default;
};

class IDataProcessor {
public:
    virtual ~IDataProcessor() = default;
    IDataProcessor(const IDataProcessor&) = delete;
    IDataProcessor& operator=(const IDataProcessor&) = delete;
    IDataProcessor(IDataProcessor&&) = default;
    IDataProcessor& operator=(IDataProcessor&&) = default;

    virtual void process(const SensorData& data) noexcept = 0;
    [[nodiscard]] virtual auto is_critical_condition() const noexcept -> bool = 0;

protected:
    IDataProcessor() = default;
};

class ILogger {
public:
    virtual ~ILogger() = default;
    ILogger(const ILogger&) = delete;
    ILogger& operator=(const ILogger&) = delete;
    ILogger(ILogger&&) = default;
    ILogger& operator=(ILogger&&) = default;

    virtual void log_info(const char* message) noexcept = 0;
    virtual void log_warning(const char* message) noexcept = 0;
    virtual void log_error(const char* message) noexcept = 0;

protected:
    ILogger() = default;
};

// ============================================================================
// CONCRETE SENSOR IMPLEMENTATIONS (TEMPLATE CLASSES)
// ============================================================================

// Template sensor class for different sensor types
template<SensorDataType T>
class GenericSensor : public ISensor {
public:
    constexpr GenericSensor(std::uint8_t sensor_id, const char* sensor_name, T minimum_range, T maximum_range) noexcept
        : id_(sensor_id), name_(sensor_name), min_range_(minimum_range), max_range_(maximum_range) {}

    auto read() noexcept -> SensorData override {
        // Simulate sensor reading - in real system, would read from hardware
        T raw_value = simulate_sensor_reading();

        // Validate reading
        bool valid = is_in_valid_range(raw_value, min_range_, max_range_);
        healthy_ = valid;

        if (valid) {
            last_reading_ = static_cast<double>(raw_value);
        }

        return SensorData{last_reading_, get_current_time_us(), id_};
    }

    [[nodiscard]] auto is_healthy() const noexcept -> bool override { return healthy_; }
    [[nodiscard]] auto get_id() const noexcept -> std::uint8_t override { return id_; }
    [[nodiscard]] auto get_name() const noexcept -> const char* override { return name_; }

private:
    std::uint8_t id_;
    const char* name_;
    T min_range_;
    T max_range_;
    bool healthy_{true};
    double last_reading_{0};

    // Simulate sensor reading (in real system, would interface with hardware)
    auto simulate_sensor_reading() noexcept -> T {
        // Simple simulation - in real system would read from ADC, I2C, etc.
        static T counter = static_cast<T>(0);
        T range_diff = max_range_ - min_range_;
        if constexpr (std::is_floating_point_v<T>) {
            counter = static_cast<T>(counter + 1);
            return min_range_ + static_cast<T>(std::fmod(static_cast<double>(counter), static_cast<double>(range_diff)));
        } else {
            counter = static_cast<T>((counter + 1) % range_diff);
            return min_range_ + counter;
        }
    }

    [[nodiscard]] auto get_current_time_us() const noexcept -> std::uint64_t {
        // In real system, would use hardware timer
        static std::uint64_t sim_time = 0;
        return sim_time += TIME_SIMULATION_INCREMENT_US;
    }
};

// Specialized sensors using template aliases
using AltitudeSensor = GenericSensor<double>;
using TemperatureSensor = GenericSensor<float>;
using PressureSensor = GenericSensor<std::uint32_t>;

// ============================================================================
// DATA PROCESSING SYSTEM
// ============================================================================

class FlightDataProcessor : public IDataProcessor {
public:
    constexpr FlightDataProcessor() noexcept = default;

    void process(const SensorData& data) noexcept override {
        // Store data in circular buffer (no dynamic allocation)
        // Real-time safe: buffer_index_ is always < SENSOR_BUFFER_SIZE due to modulo
        buffer_[buffer_index_] = data;
        buffer_index_ = (buffer_index_ + 1) % SENSOR_BUFFER_SIZE;

        // Process based on sensor type
        process_by_sensor_type(data);
    }

    [[nodiscard]] auto is_critical_condition() const noexcept -> bool override {
        return critical_condition_;
    }

    // C++23 lambda with template for flexible processing
    template<typename ProcessorFunc>
    void apply_to_recent_data(ProcessorFunc&& processor_func) const noexcept {
        for (std::size_t i = 0; i < SENSOR_BUFFER_SIZE; ++i) {
            // Real-time safe: i is always < SENSOR_BUFFER_SIZE due to loop bounds
            if (buffer_[i].is_valid()) {
                std::forward<ProcessorFunc>(processor_func)(buffer_[i]);
            }
        }
    }

private:
    std::array<SensorData, SENSOR_BUFFER_SIZE> buffer_{};
    std::size_t buffer_index_{0};
    bool critical_condition_{false};

    void process_by_sensor_type(const SensorData& data) noexcept {
        switch (data.sensor_id()) {
            case 0: // Altitude sensor
                if (data.value() < CRITICAL_ALTITUDE_THRESHOLD) {
                    critical_condition_ = true;
                }
                break;
            case 1: // Temperature sensor
                if (data.value() > MAX_TEMPERATURE_CELSIUS) {
                    critical_condition_ = true;
                }
                break;
            case 2: // Pressure sensor
                if (data.value() < MIN_PRESSURE_PA) {
                    critical_condition_ = true;
                }
                break;
            default:
                break;
        }
    }
};

// ============================================================================
// LOGGING SYSTEM
// ============================================================================

class EmbeddedLogger : public ILogger {
public:
    void log_info(const char* message) noexcept override {
        log_with_level("INFO", message);
    }

    void log_warning(const char* message) noexcept override {
        log_with_level("WARN", message);
    }

    void log_error(const char* message) noexcept override {
        log_with_level("ERROR", message);
    }

private:
    static void log_with_level(const char* level, const char* message) noexcept;
};

// ============================================================================
// MAIN SYSTEM CONTROLLER (DEPENDENCY INJECTION)
// ============================================================================

class FlightControlSystem {
public:
    // Constructor with dependency injection
    FlightControlSystem(std::unique_ptr<IDataProcessor> processor,
                       std::unique_ptr<ILogger> logger) noexcept
        : processor_(std::move(processor)), logger_(std::move(logger)) {}

    // Add sensor with perfect forwarding
    template<typename SensorType>
    auto add_sensor(std::unique_ptr<SensorType> sensor) noexcept -> bool {
        static_assert(std::is_base_of_v<ISensor, SensorType>,
                     "SensorType must inherit from ISensor");

        if (sensor_count_ >= MAX_SENSORS) {
            return false;
        }

        // Real-time safe: sensor_count_ is always < MAX_SENSORS due to check above
        sensors_[sensor_count_++] = std::move(sensor);
        return true;
    }

    // Main execution loop with C++23 lambda
    void execute_cycle() noexcept {
        // Lambda for sensor processing with capture
        auto process_sensor = [this](auto& sensor) noexcept {
            if (!sensor || !sensor->is_healthy()) {
                return;
            }

            auto data = sensor->read();
            if (data.is_valid()) {
                processor_->process(data);

                // Check for critical conditions
                if (processor_->is_critical_condition()) {
                    logger_->log_warning("Critical flight condition detected!");
                    handle_critical_condition();
                }
            }
        };

        // Process all sensors
        for (std::size_t i = 0; i < sensor_count_; ++i) {
            // Real-time safe: i is always < sensor_count_ <= MAX_SENSORS due to loop bounds
            process_sensor(sensors_[i]);
        }
    }

    // Template method for batch sensor operations
    template<typename Operation>
    void for_each_sensor(Operation&& operation) noexcept {
        for (std::size_t i = 0; i < sensor_count_; ++i) {
            // Real-time safe: i is always < sensor_count_ <= MAX_SENSORS due to loop bounds
            if (sensors_[i]) {
                std::forward<Operation>(operation)(*sensors_[i]);
            }
        }
    }

private:
    std::array<std::unique_ptr<ISensor>, MAX_SENSORS> sensors_{};
    std::size_t sensor_count_{0};
    std::unique_ptr<IDataProcessor> processor_;
    std::unique_ptr<ILogger> logger_;

    void handle_critical_condition() noexcept {
        // Implement emergency procedures
        logger_->log_error("Executing emergency protocols");
    }
};

// ============================================================================
// FACTORY FUNCTIONS (SMART POINTER CREATION)
// ============================================================================

// Factory function using perfect forwarding
template<typename T, typename... Args>
auto make_sensor(Args&&... args) noexcept -> std::unique_ptr<T> {
    return std::make_unique<T>(std::forward<Args>(args)...);
}

// System factory function
inline auto create_flight_system() noexcept -> std::unique_ptr<FlightControlSystem> {
    auto processor = std::make_unique<FlightDataProcessor>();
    auto logger = std::make_unique<EmbeddedLogger>();

    return std::make_unique<FlightControlSystem>(std::move(processor),
                                               std::move(logger));
}

// Template function for sensor data validation with concepts
template<SensorDataType T>
constexpr auto validate_sensor_range(T value, T nominal, T tolerance) noexcept -> bool {
    static_assert(std::is_arithmetic_v<T>, "Sensor data must be arithmetic type");

    if constexpr (std::is_floating_point_v<T>) {
        return std::abs(value - nominal) <= tolerance;
    } else {
        T diff = (value > nominal) ? (value - nominal) : (nominal - value);
        return diff <= tolerance;
    }
}

} // namespace aerospace
