#include "lib.hpp"
#include <cstdio>

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
    // Get timestamp (in real system, would use hardware timer)
    static std::uint64_t system_time_ms = 0;
    system_time_ms += 10; // Simulate 10ms increments

    // Format: [TIMESTAMP] LEVEL: MESSAGE
    // In real embedded system, this would go to:
    // - UART for real-time monitoring
    // - Flash memory for persistent logging
    // - Shared memory for other systems

    constexpr std::size_t log_buffer_size = 256;
    char log_buffer[log_buffer_size];

    // Safe string formatting (avoiding printf for embedded safety)
    std::size_t pos = 0;

    // Add timestamp
    log_buffer[pos++] = '[';
    pos += static_cast<std::size_t>(std::snprintf(&log_buffer[pos], log_buffer_size - pos - 1, "%llu",
                   static_cast<unsigned long long>(system_time_ms)));
    log_buffer[pos++] = ']';
    log_buffer[pos++] = ' ';

    // Add level
    if (level) {
        const char* level_ptr = level;
        while (*level_ptr && pos < log_buffer_size - 1) {
            log_buffer[pos++] = *level_ptr++;
        }
    }
    log_buffer[pos++] = ':';
    log_buffer[pos++] = ' ';

    // Add message
    if (message) {
        const char* msg_ptr = message;
        while (*msg_ptr && pos < log_buffer_size - 1) {
            log_buffer[pos++] = *msg_ptr++;
        }
    }

    log_buffer[pos] = '\0';

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
