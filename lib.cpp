#include "lib.hpp"
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <array>

namespace aerospace {

// ============================================================================
// TEMPLATE EXPLICIT INSTANTIATIONS
// ============================================================================

// Explicit template instantiation for common sensor types
template class GenericSensor<double>;
template class GenericSensor<float>;
template class GenericSensor<std::uint32_t>;

// ============================================================================
// ENHANCED LOGGING IMPLEMENTATIONS
// ============================================================================

// ============================================================================
// ENHANCED LOGGING IMPLEMENTATION
// ============================================================================

void EmbeddedLogger::log_with_level(const char* level, const char* message) noexcept {
    // Named constants for better maintainability
    constexpr std::uint64_t TIME_INCREMENT_MS = 10;
    constexpr std::size_t LOG_BUFFER_SIZE = 256;
    
    // Get timestamp (in real system, would use hardware timer)
    static std::uint64_t system_time_ms = 0;
    system_time_ms += TIME_INCREMENT_MS;

    // Format: [TIMESTAMP] LEVEL: MESSAGE
    // In real embedded system, this would go to:
    // - UART for real-time monitoring
    // - Flash memory for persistent logging
    // - Shared memory for other systems

    std::array<char, LOG_BUFFER_SIZE> log_buffer{};

    // Safe string formatting (avoiding printf for embedded safety)
    std::size_t pos = 0;

    // Add timestamp
    log_buffer.at(pos++) = '[';
    pos += static_cast<std::size_t>(std::snprintf(&log_buffer.at(pos), LOG_BUFFER_SIZE - pos - 1, "%llu",
                   static_cast<unsigned long long>(system_time_ms)));
    log_buffer.at(pos++) = ']';
    log_buffer.at(pos++) = ' ';

    // Add level
    if (level != nullptr) {
        const char* level_ptr = level;
        while ((*level_ptr != '\0') && pos < LOG_BUFFER_SIZE - 1) {
            log_buffer.at(pos++) = *level_ptr;
            ++level_ptr;
        }
    }
    log_buffer.at(pos++) = ':';
    log_buffer.at(pos++) = ' ';

    // Add message
    if (message != nullptr) {
        const char* msg_ptr = message;
        while ((*msg_ptr != '\0') && pos < LOG_BUFFER_SIZE - 1) {
            log_buffer.at(pos++) = *msg_ptr;
            ++msg_ptr;
        }
    }

    log_buffer.at(pos) = '\0';

    // In real system: write to hardware interfaces
    // simulate_uart_write(log_buffer, pos);
    // simulate_flash_write(log_buffer, pos);

    // For demonstration only - remove in production embedded code
    #ifdef DEBUG_LOGGING
    std::cout << log_buffer << std::endl;
    #endif
}

// ============================================================================
// TEMPLATE FUNCTION IMPLEMENTATIONS
// ============================================================================

// Explicit instantiation for common types
template bool validate_sensor_range<double>(double, double, double) noexcept;
template bool validate_sensor_range<float>(float, float, float) noexcept;
template bool validate_sensor_range<std::uint32_t>(std::uint32_t, std::uint32_t, std::uint32_t) noexcept;

} // namespace aerospace
