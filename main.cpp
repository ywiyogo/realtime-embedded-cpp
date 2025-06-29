#include "lib.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cassert>

// ============================================================================
// REAL-TIME SYSTEM CONFIGURATION
// ============================================================================

constexpr std::uint32_t SYSTEM_CYCLE_TIME_MS = 10;  // 100Hz execution rate
constexpr std::uint32_t MAX_EXECUTION_CYCLES = 100; // Demonstration cycles
constexpr bool ENABLE_MONITORING = true;

// ============================================================================
// SENSOR FACTORY FUNCTIONS (C++23 TEMPLATE FACTORIES)
// ============================================================================

namespace sensor_factory {
    using namespace aerospace;

    // Template factory functions with compile-time configuration
    template<typename T = double>
    constexpr auto create_altitude_sensor(std::uint8_t id = 0) noexcept {
        return AltitudeSensor{id, "Altitude_Sensor", 0.0, 50000.0}; // 0-50km range
    }

    template<typename T = float>
    constexpr auto create_temperature_sensor(std::uint8_t id = 1) noexcept {
        return TemperatureSensor{id, "Temperature_Sensor", -40.0f, 125.0f}; // -40°C to 125°C
    }

    template<typename T = std::uint32_t>
    constexpr auto create_pressure_sensor(std::uint8_t id = 2) noexcept {
        return PressureSensor{id, "Pressure_Sensor", 10000u, 120000u}; // 10-120 kPa
    }
}

// ============================================================================
// SYSTEM VALIDATION (INTEGRATED)
// ============================================================================

class SystemValidator {
public:
    static bool run_basic_tests() noexcept {
        using namespace aerospace;
        
        std::cout << "Running system validation tests..." << std::endl;
        bool all_passed = true;

        // Test 1: Memory Pool
        {
            StaticMemoryPool<int, 5> pool;
            auto* ptr1 = pool.allocate();
            auto* ptr2 = pool.allocate();
            
            if (!ptr1 || !ptr2 || pool.available() != 3) {
                std::cerr << "❌ Memory pool test failed" << std::endl;
                all_passed = false;
            } else {
                std::cout << "✅ Memory pool test passed" << std::endl;
            }
        }

        // Test 2: Sensor Creation
        {
            auto altitude_sensor = sensor_factory::create_altitude_sensor<double>(0);
            if (!altitude_sensor.is_healthy() || altitude_sensor.get_id() != 0) {
                std::cerr << "❌ Sensor creation test failed" << std::endl;
                all_passed = false;
            } else {
                std::cout << "✅ Sensor creation test passed" << std::endl;
            }
        }

        // Test 3: Constexpr Validation
        {
            static_assert(is_valid_sensor_id(0), "Sensor ID 0 should be valid");
            static_assert(is_valid_sensor_id(MAX_SENSORS - 1), "Last sensor ID should be valid");
            static_assert(!is_valid_sensor_id(MAX_SENSORS), "Out of range sensor ID should be invalid");
            static_assert(is_in_valid_range(50.0, 0.0, 100.0), "Value should be in range");
            
            std::cout << "✅ Constexpr validation test passed" << std::endl;
        }

        return all_passed;
    }
};

// ============================================================================
// SIMPLIFIED SYSTEM MANAGER
// ============================================================================

class FlightSystemManager {
public:
    explicit FlightSystemManager(std::unique_ptr<aerospace::FlightControlSystem> system) noexcept
        : system_(std::move(system)), cycle_count_(0), running_(true) {}

    void run_system() noexcept {
        std::cout << "\n🚀 Starting real-time execution loop..." << std::endl;
        
        auto start_time = std::chrono::steady_clock::now();

        while (running_ && cycle_count_ < MAX_EXECUTION_CYCLES) {
            auto cycle_start = std::chrono::steady_clock::now();

            // Execute main system cycle
            system_->execute_cycle();

            // Real-time scheduling constraint
            auto cycle_end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start);

            if (duration.count() < SYSTEM_CYCLE_TIME_MS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(SYSTEM_CYCLE_TIME_MS - duration.count()));
            } else if constexpr (ENABLE_MONITORING) {
                std::cout << "⚠️  Cycle " << cycle_count_ << " exceeded deadline: " << duration.count() << "ms" << std::endl;
            }

            cycle_count_++;

            // Periodic status
            if (cycle_count_ % 25 == 0) {
                std::cout << "📊 Completed " << cycle_count_ << " cycles..." << std::endl;
            }
        }

        auto total_time = std::chrono::steady_clock::now() - start_time;
        std::cout << "✅ System completed " << cycle_count_ << " cycles in "
                  << std::chrono::duration_cast<std::chrono::seconds>(total_time).count() << " seconds" << std::endl;
    }

    void shutdown() noexcept { running_ = false; }
    std::uint32_t get_cycle_count() const noexcept { return cycle_count_; }

private:
    std::unique_ptr<aerospace::FlightControlSystem> system_;
    std::uint32_t cycle_count_;
    bool running_;
};

// ============================================================================
// TEMPLATE INITIALIZATION WITH FOLD EXPRESSIONS (C++23)
// ============================================================================

template<typename... SensorTypes>
std::unique_ptr<aerospace::FlightControlSystem> initialize_system_with_sensors() {
    using namespace aerospace;

    auto system = create_flight_system();

    // C++23 lambda with template parameter for sensor creation
    auto add_sensor = [&system]<typename T>(T sensor_config) {
        auto sensor = make_sensor<T>(std::move(sensor_config));
        if (!system->add_sensor(std::move(sensor))) {
            std::cerr << "Failed to add sensor to system" << std::endl;
        }
    };

    // Create and add sensors using perfect forwarding
    add_sensor(sensor_factory::create_altitude_sensor<double>(0));
    add_sensor(sensor_factory::create_temperature_sensor<float>(1));
    add_sensor(sensor_factory::create_pressure_sensor<std::uint32_t>(2));

    return system;
}

// ============================================================================
// MAIN APPLICATION ENTRY POINT
// ============================================================================

int main() {
    using namespace aerospace;

    std::cout << "🛩️  Real-Time Aerospace Flight Control System" << std::endl;
    std::cout << "C++23 Implementation with Modern Best Practices" << std::endl;
    std::cout << "=============================================" << std::endl;

    try {
        // Demonstrate compile-time validation
        std::cout << "\n🔍 Compile-time validation:" << std::endl;
        std::cout << "• Max sensors: " << MAX_SENSORS << std::endl;
        std::cout << "• Buffer size: " << SENSOR_BUFFER_SIZE << std::endl;
        std::cout << "• Critical altitude: " << CRITICAL_ALTITUDE_THRESHOLD << "m" << std::endl;

        // Run system validation tests
        if (!SystemValidator::run_basic_tests()) {
            std::cerr << "\n❌ System validation failed! Aborting." << std::endl;
            return -1;
        }

        std::cout << "\n🔧 Initializing flight control system..." << std::endl;

        // Initialize system with template-based sensor creation
        auto flight_system = initialize_system_with_sensors();

        // Demonstrate C++23 lambda usage with sensor analysis
        std::cout << "\n📡 Sensor Analysis:" << std::endl;
        flight_system->for_each_sensor([](const ISensor& sensor) noexcept {
            std::cout << "• " << sensor.get_name() 
                      << " [ID: " << static_cast<int>(sensor.get_id()) 
                      << ", Status: " << (sensor.is_healthy() ? "HEALTHY" : "FAULT") << "]" << std::endl;
        });

        // Create system manager with dependency injection
        FlightSystemManager manager(std::move(flight_system));

        // Emergency handler using C++23 lambda (demonstration of lambda capture)
        [[maybe_unused]] auto emergency_handler = [&manager](const char* reason) noexcept {
            std::cout << "\n🚨 EMERGENCY: " << reason << " - Shutting down safely..." << std::endl;
            manager.shutdown();
        };

        // Demonstrate constexpr computation
        constexpr auto max_safe_cycles = static_cast<std::uint32_t>(MAX_EXECUTION_CYCLES * 0.9);
        std::cout << "\n⚙️  System Configuration:" << std::endl;
        std::cout << "• Cycle time: " << SYSTEM_CYCLE_TIME_MS << "ms (100Hz)" << std::endl;
        std::cout << "• Max safe cycles: " << max_safe_cycles << std::endl;
        std::cout << "• Memory management: Static allocation only" << std::endl;
        std::cout << "• Exception handling: Disabled (noexcept)" << std::endl;

        // Start real-time execution
        manager.run_system();

        std::cout << "\n🎯 Flight Control System Demonstration Complete!" << std::endl;
        std::cout << "Key C++23 features demonstrated:" << std::endl;
        std::cout << "• Concepts for type safety" << std::endl;
        std::cout << "• Template specialization" << std::endl;
        std::cout << "• Smart pointers with RAII" << std::endl;
        std::cout << "• Constexpr compile-time computation" << std::endl;
        std::cout << "• Lambda expressions with perfect forwarding" << std::endl;
        std::cout << "• Static memory pools (no dynamic allocation)" << std::endl;
        std::cout << "• Deterministic real-time execution" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n💥 System exception: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "\n💥 Unknown system exception!" << std::endl;
        return -1;
    }

    return 0;
}

// ============================================================================
// COMPILE-TIME ASSERTIONS FOR SYSTEM VALIDATION
// ============================================================================

// Validate system configuration at compile time
static_assert(aerospace::MAX_SENSORS <= 16, "Maximum sensors exceeded reasonable limit");
static_assert(aerospace::SENSOR_BUFFER_SIZE > 0, "Sensor buffer size must be positive");
static_assert(SYSTEM_CYCLE_TIME_MS >= 1, "Cycle time too short for real-time constraints");
static_assert(sizeof(aerospace::SensorData) <= 32, "SensorData structure too large for embedded systems");

// Validate template instantiations
static_assert(std::is_trivially_copyable_v<aerospace::SensorData>,
              "SensorData must be trivially copyable for efficient embedded operations");

static_assert(std::is_nothrow_destructible_v<aerospace::FlightControlSystem>,
              "FlightControlSystem destructor must not throw");