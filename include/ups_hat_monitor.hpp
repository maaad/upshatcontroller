#ifndef UPS_HAT_MONITOR_HPP
#define UPS_HAT_MONITOR_HPP

/**
 * @file ups_hat_monitor.hpp
 * @brief Main monitoring and shutdown control logic for UPS HAT Controller
 */

#include "ups_hat_driver.hpp"
#include "config_parser.hpp"
#include <string>
#include <memory>
#include <syslog.h>
#include <chrono>
#include <optional>

/**
 * @brief Main monitoring class for UPS HAT Controller
 *
 * Monitors UPS HAT status, detects power loss and low voltage conditions,
 * and initiates safe system shutdown when necessary.
 */
class UpsHatMonitor {
public:
    UpsHatMonitor(const UpsHatConfig& config);
    ~UpsHatMonitor();

    bool initialize();
    void run();
    void stop();

private:
    UpsHatConfig config_;
    std::unique_ptr<UpsHatDriver> driver_;
    bool running_;
    bool on_mains_;
    std::chrono::steady_clock::time_point power_loss_time_;
    std::chrono::steady_clock::time_point last_battery_status_log_;
    int low_voltage_count_;
    std::optional<int> cooling_state_;
    std::optional<double> cpu_temp_;

    void logMessage(int priority, const std::string& message);

    bool checkLowVoltage(const CellVoltages& cell_voltages, int16_t current_ma);
    void handlePowerLoss();
    void handlePowerRestored();
    void handleLowVoltage();
    void shutdownSystem();

    std::optional<int> readSysfsInt(const std::string& path);
    std::optional<double> readSysfsDouble(const std::string& path, double scale = 1.0);
    void updateThermalData();

    std::string formatBriefMetrics(const VbusData& vbus_data, const BatteryData& battery_data);
    std::string getChargeStateName(uint8_t charge_state);
    void logBatteryStatus(const ChargingStatus& charging_status,
                         const VbusData& vbus_data,
                         const BatteryData& battery_data,
                         const CellVoltages& cell_voltages);
};

#endif // UPS_HAT_MONITOR_HPP

