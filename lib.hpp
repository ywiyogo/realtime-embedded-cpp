#pragma once

#include <memory>
#include <array>
#include <type_traits>
#include <cstdint>
#include <algorithm>
#include <cmath>

namespace aerospace {

// ============================================================================
// COMPILE-TIME CONSTANTS AND CONFIGURATION
// ============================================================================

// C++23 constexpr configurations for real-time constraints
constexpr std::size_t MAX_SENSORS = 4;
constexpr std::size_t SENSOR_BUFFER_SIZE = 50;
constexpr double CRITICAL_ALTITUDE_THRESHOLD = 1000.0;
constexpr std::uint32_t SENSOR_TIMEOUT_MS = 50;

// C++23 constexpr function for compile-time validation
constexpr bool is_valid_sensor_id(std::size_t id) noexcept {
    return id < MAX_SENSORS;
}

// ============================================================================
// TEMPLATE UTILITIES AND TYPE TRAITS (C++23 CONCEPTS)
// ============================================================================

// Template concept for sensor data validation (C++20/23 concept)
template<typename T>
concept SensorDataType = std::is_arithmetic_v<T> &&
                        !std::is_same_v<T, bool> &&
                        sizeof(T) <= 8; // Embedded constraint

// Template for compile-time sensor range validation
template<SensorDataType T>
constexpr bool is_in_valid_range(T value, T min_val, T max_val) noexcept {
    return value >= min_val && value <= max_val;
}

// ============================================================================
// MEMORY POOL ALLOCATOR (NO DYNAMIC ALLOCATION)
// ============================================================================

template<typename T, std::size_t PoolSize>
class StaticMemoryPool {
public:
    static constexpr std::size_t pool_size = PoolSize;

    constexpr StaticMemoryPool() noexcept : next_free_(0) {}

    [[nodiscard]] T* allocate() noexcept {
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

    constexpr std::size_t available() const noexcept {
        return PoolSize - next_free_;
    }

private:
    std::array<T, PoolSize> pool_{};
    std::size_t next_free_;
};

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

struct SensorData {
    double value;
    std::uint64_t timestamp_us;
    std::uint8_t sensor_id;
    bool is_valid;

    constexpr SensorData() noexcept
        : value(0.0), timestamp_us(0), sensor_id(0), is_valid(false) {}

    constexpr SensorData(double val, std::uint64_t ts, std::uint8_t id) noexcept
        : value(val), timestamp_us(ts), sensor_id(id), is_valid(true) {}
};

// ============================================================================
// ABSTRACT INTERFACES (DEPENDENCY INJECTION TARGETS)
// ============================================================================

class ISensor {
public:
    virtual ~ISensor() = default;
    virtual SensorData read() noexcept = 0;
    virtual bool is_healthy() const noexcept = 0;
    virtual std::uint8_t get_id() const noexcept = 0;
    virtual const char* get_name() const noexcept = 0;
};

class IDataProcessor {
public:
    virtual ~IDataProcessor() = default;
    virtual void process(const SensorData& data) noexcept = 0;
    virtual bool is_critical_condition() const noexcept = 0;
};

class ILogger {
public:
    virtual ~ILogger() = default;
    virtual void log_info(const char* message) noexcept = 0;
    virtual void log_warning(const char* message) noexcept = 0;
    virtual void log_error(const char* message) noexcept = 0;
};

// ============================================================================
// CONCRETE SENSOR IMPLEMENTATIONS (TEMPLATE CLASSES)
// ============================================================================

// Template sensor class for different sensor types
template<SensorDataType T>
class GenericSensor : public ISensor {
public:
    constexpr GenericSensor(std::uint8_t id, const char* name, T min_range, T max_range) noexcept
        : id_(id), name_(name), min_range_(min_range), max_range_(max_range),
          healthy_(true), last_reading_(0) {}

    SensorData read() noexcept override {
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

    bool is_healthy() const noexcept override { return healthy_; }
    std::uint8_t get_id() const noexcept override { return id_; }
    const char* get_name() const noexcept override { return name_; }

private:
    std::uint8_t id_;
    const char* name_;
    T min_range_;
    T max_range_;
    bool healthy_;
    double last_reading_;

    // Simulate sensor reading (in real system, would interface with hardware)
    T simulate_sensor_reading() noexcept {
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

    std::uint64_t get_current_time_us() const noexcept {
        // In real system, would use hardware timer
        static std::uint64_t sim_time = 0;
        return sim_time += 1000; // Simulate 1ms increments
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
    constexpr FlightDataProcessor() noexcept
        : buffer_index_(0), critical_condition_(false) {}

    void process(const SensorData& data) noexcept override {
        // Store data in circular buffer (no dynamic allocation)
        buffer_[buffer_index_] = data;
        buffer_index_ = (buffer_index_ + 1) % SENSOR_BUFFER_SIZE;

        // Process based on sensor type
        process_by_sensor_type(data);
    }

    bool is_critical_condition() const noexcept override {
        return critical_condition_;
    }

    // C++23 lambda with template for flexible processing
    template<typename ProcessorFunc>
    void apply_to_recent_data(ProcessorFunc&& processor) const noexcept {
        for (std::size_t i = 0; i < SENSOR_BUFFER_SIZE; ++i) {
            if (buffer_[i].is_valid) {
                processor(buffer_[i]);
            }
        }
    }

private:
    std::array<SensorData, SENSOR_BUFFER_SIZE> buffer_{};
    std::size_t buffer_index_;
    bool critical_condition_;

    void process_by_sensor_type(const SensorData& data) noexcept {
        switch (data.sensor_id) {
            case 0: // Altitude sensor
                if (data.value < CRITICAL_ALTITUDE_THRESHOLD) {
                    critical_condition_ = true;
                }
                break;
            case 1: // Temperature sensor
                if (data.value > 85.0) { // 85°C max temperature
                    critical_condition_ = true;
                }
                break;
            case 2: // Pressure sensor
                if (data.value < 50000) { // 50kPa minimum pressure
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
    void log_with_level(const char* level, const char* message) noexcept;
};

// ============================================================================
// MAIN SYSTEM CONTROLLER (DEPENDENCY INJECTION)
// ============================================================================

class FlightControlSystem {
public:
    // Constructor with dependency injection
    FlightControlSystem(std::unique_ptr<IDataProcessor> processor,
                       std::unique_ptr<ILogger> logger) noexcept
        : sensor_count_(0), processor_(std::move(processor)), logger_(std::move(logger)) {}

    // Add sensor with perfect forwarding
    template<typename SensorType>
    bool add_sensor(std::unique_ptr<SensorType> sensor) noexcept {
        static_assert(std::is_base_of_v<ISensor, SensorType>,
                     "SensorType must inherit from ISensor");

        if (sensor_count_ >= MAX_SENSORS) {
            return false;
        }

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
            if (data.is_valid) {
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
            process_sensor(sensors_[i]);
        }
    }

    // Template method for batch sensor operations
    template<typename Operation>
    void for_each_sensor(Operation&& op) noexcept {
        for (std::size_t i = 0; i < sensor_count_; ++i) {
            if (sensors_[i]) {
                op(*sensors_[i]);
            }
        }
    }

private:
    std::array<std::unique_ptr<ISensor>, MAX_SENSORS> sensors_{};
    std::size_t sensor_count_;
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
std::unique_ptr<T> make_sensor(Args&&... args) noexcept {
    return std::make_unique<T>(std::forward<Args>(args)...);
}

// System factory function
inline std::unique_ptr<FlightControlSystem> create_flight_system() noexcept {
    auto processor = std::make_unique<FlightDataProcessor>();
    auto logger = std::make_unique<EmbeddedLogger>();

    return std::make_unique<FlightControlSystem>(std::move(processor),
                                               std::move(logger));
}

// Template function for sensor data validation with concepts
template<SensorDataType T>
constexpr bool validate_sensor_range(T value, T nominal, T tolerance) noexcept {
    static_assert(std::is_arithmetic_v<T>, "Sensor data must be arithmetic type");

    if constexpr (std::is_floating_point_v<T>) {
        return std::abs(value - nominal) <= tolerance;
    } else {
        T diff = (value > nominal) ? (value - nominal) : (nominal - value);
        return diff <= tolerance;
    }
}

} // namespace aerospace
