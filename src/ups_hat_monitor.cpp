/**
 * @file ups_hat_monitor.cpp
 * @brief Implementation of UPS HAT monitoring and shutdown control
 */

#include "ups_hat_monitor.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <cstdlib>

UpsHatMonitor::UpsHatMonitor(const UpsHatConfig& config)
    : config_(config), running_(false), on_mains_(true),
      power_loss_time_(), last_battery_status_log_(), low_voltage_count_(0) {
    driver_ = std::make_unique<UpsHatDriver>(config_.i2c_bus, config_.i2c_addr);
}

UpsHatMonitor::~UpsHatMonitor() {
    stop();
    if (config_.enable_syslog) {
        closelog();
    }
}

bool UpsHatMonitor::initialize() {
    // Initialize syslog
    if (config_.enable_syslog) {
        openlog("ups-hat-controller", LOG_PID | LOG_CONS, LOG_DAEMON);
    }

    // Initialize driver
    if (!driver_->initialize()) {
        logMessage(LOG_ERR, "Failed to initialize I2C driver");
        return false;
    }

    logMessage(LOG_INFO, "UPS HAT monitor initialized successfully");
    return true;
}

void UpsHatMonitor::stop() {
    running_ = false;
}

void UpsHatMonitor::logMessage(int priority, const std::string& message) {
    // Write to syslog (journald)
    // systemd automatically captures syslog output, so no need to write to stderr separately
    if (config_.enable_syslog) {
        syslog(priority, "%s", message.c_str());
    }
}

std::optional<int> UpsHatMonitor::readSysfsInt(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return std::nullopt;
    }

    int value;
    if (file >> value) {
        return value;
    }
    return std::nullopt;
}

std::optional<double> UpsHatMonitor::readSysfsDouble(const std::string& path, double scale) {
    auto value = readSysfsInt(path);
    if (value.has_value()) {
        return static_cast<double>(value.value()) * scale;
    }
    return std::nullopt;
}

void UpsHatMonitor::updateThermalData() {
    cooling_state_ = readSysfsInt("/sys/class/thermal/cooling_device0/cur_state");
    cpu_temp_ = readSysfsDouble("/sys/class/hwmon/hwmon0/temp1_input", 0.001);
}

std::string UpsHatMonitor::formatBriefMetrics(const VbusData& vbus_data, const BatteryData& battery_data) {
    std::ostringstream oss;
    double vbus_v = vbus_data.voltage_mv / 1000.0;
    double batt_v = battery_data.voltage_mv / 1000.0;
    oss << std::fixed << std::setprecision(2);
    oss << "VBUS=" << vbus_v << "V " << vbus_data.current_ma << "mA, ";
    oss << "BAT=" << batt_v << "V " << std::showpos << battery_data.current_ma
        << std::noshowpos << "mA " << battery_data.percent << "%";
    return oss.str();
}

std::string UpsHatMonitor::getChargeStateName(uint8_t charge_state) {
    // Map charge state values to human-readable names
    // Reference: https://www.waveshare.com/wiki/UPS_HAT_(E)_Register
    // Register 0x02, bits 2-0: 000=standby, 001=trickle, 010=CC, 011=CV,
    //                           100=pending, 101=full, 110=timeout
    switch (charge_state) {
        case 0: return "Standby";
        case 1: return "Trickle Charge";
        case 2: return "Constant Current Charge";
        case 3: return "Constant Voltage Charge";
        case 4: return "Charging Pending";
        case 5: return "Full State";
        case 6: return "Charge Timeout";
        default: return "Unknown";
    }
}

void UpsHatMonitor::logBatteryStatus(const ChargingStatus& charging_status,
                                     const VbusData& vbus_data,
                                     const BatteryData& battery_data,
                                     const CellVoltages& cell_voltages) {
    // Log comprehensive battery status for monitoring during battery operation
    std::ostringstream oss;
    oss << "Battery Status: ";
    oss << "VBUS=" << std::fixed << std::setprecision(2) << (vbus_data.voltage_mv / 1000.0)
        << "V " << vbus_data.current_ma << "mA, ";
    oss << "BAT=" << (battery_data.voltage_mv / 1000.0) << "V "
        << std::showpos << battery_data.current_ma << std::noshowpos << "mA "
        << battery_data.percent << "%, ";
    oss << "Cells=" << (cell_voltages.cell1_mv / 1000.0) << "V/"
        << (cell_voltages.cell2_mv / 1000.0) << "V/"
        << (cell_voltages.cell3_mv / 1000.0) << "V/"
        << (cell_voltages.cell4_mv / 1000.0) << "V, ";
    oss << "Capacity=" << battery_data.remaining_capacity_mah << "mAh, ";
    oss << "State=" << getChargeStateName(charging_status.charge_state)
        << " (" << static_cast<int>(charging_status.charge_state) << ")";

    if (cooling_state_.has_value()) {
        oss << ", Cooling=" << cooling_state_.value();
    }
    if (cpu_temp_.has_value()) {
        oss << ", CPU=" << std::fixed << std::setprecision(1) << cpu_temp_.value() << "°C";
    }

    logMessage(LOG_INFO, oss.str());
}

bool UpsHatMonitor::checkLowVoltage(const CellVoltages& cell_voltages, int16_t current_ma) {
    // Only check low voltage when current is low (near idle) to avoid false positives
    // during high current discharge which causes voltage sag
    if (current_ma >= 50) {
        return false;
    }

    // Check if any cell voltage is below threshold
    if (cell_voltages.cell1_mv < config_.low_voltage_threshold_mv ||
        cell_voltages.cell2_mv < config_.low_voltage_threshold_mv ||
        cell_voltages.cell3_mv < config_.low_voltage_threshold_mv ||
        cell_voltages.cell4_mv < config_.low_voltage_threshold_mv) {
        return true;
    }

    return false;
}

void UpsHatMonitor::handlePowerLoss() {
    auto now = std::chrono::steady_clock::now();
    power_loss_time_ = now;
    std::ostringstream oss;
    oss << "Power loss detected. Scheduling shutdown in " << config_.shutdown_delay_sec << " sec";
    std::string message = oss.str();
    logMessage(LOG_WARNING, message);

    // Send wall message to all logged-in users
    std::ostringstream wall_cmd;
    wall_cmd << "echo \"" << message << "\" | wall";
    int result = system(wall_cmd.str().c_str());
    if (result != 0) {
        logMessage(LOG_WARNING, "Failed to send wall message");
    }
}

void UpsHatMonitor::handlePowerRestored() {
    power_loss_time_ = std::chrono::steady_clock::time_point();
    logMessage(LOG_INFO, "Mains power restored. Cancelling pending shutdown");
}

void UpsHatMonitor::handleLowVoltage() {
    low_voltage_count_++;
    if (low_voltage_count_ >= config_.low_voltage_threshold_count) {
        logMessage(LOG_ERR, "Low voltage detected. Shutting down system now.");
        shutdownSystem();
    } else {
        int remaining_time = 60 - (2 * low_voltage_count_);
        std::ostringstream oss;
        oss << "Voltage Low, please charge in time, otherwise it will shut down in "
            << remaining_time << " s";
        logMessage(LOG_WARNING, oss.str());
    }
}

void UpsHatMonitor::shutdownSystem() {
    try {
        logMessage(LOG_INFO, "Sending shutdown command to UPS HAT");
        driver_->shutdown();

        logMessage(LOG_INFO, "Executing system shutdown");
        int result = system("systemctl poweroff");
        if (result != 0) {
            logMessage(LOG_WARNING, "systemctl poweroff returned non-zero exit code");
        }
    } catch (const std::exception& e) {
        logMessage(LOG_ERR, std::string("Failed to initiate shutdown: ") + e.what());
    }
}

void UpsHatMonitor::run() {
    running_ = true;
    auto timer_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / config_.publish_rate_hz));

    bool prev_on_mains = true;
    uint8_t prev_charge_state = 255;

    logMessage(LOG_INFO, "Starting UPS HAT monitoring loop");

    while (running_) {
        try {
            // Update thermal data asynchronously (non-blocking)
            updateThermalData();

            // Read all UPS data
            auto charging_status = driver_->readChargingStatus();
            auto vbus_data = driver_->readVbus();
            auto battery_data = driver_->readBattery();
            auto cell_voltages = driver_->readCells();

            // Determine power status
            bool on_mains_now = charging_status.charging ||
                               charging_status.fast_charging ||
                               charging_status.vbus_powered;

            // Check for power loss
            if (on_mains_now != on_mains_) {
                on_mains_ = on_mains_now;
                if (!on_mains_) {
                    handlePowerLoss();
                } else {
                    handlePowerRestored();
                }
            }

            // Check for low voltage condition
            bool low_voltage_detected = checkLowVoltage(cell_voltages, battery_data.current_ma);
            if (low_voltage_detected) {
                handleLowVoltage();
            } else {
                low_voltage_count_ = 0;
            }

            // Handle power loss timeout
            if (!on_mains_ && power_loss_time_.time_since_epoch().count() > 0) {
                auto elapsed = std::chrono::steady_clock::now() - power_loss_time_;
                auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

                if (elapsed_sec >= config_.shutdown_delay_sec) {
                    logMessage(LOG_ERR, "Power loss persisted. Shutting down system now.");
                    shutdownSystem();
                    break;
                }
            }

            // Log status changes (power and/or charge state) in a single message
            uint8_t current_charge_state = charging_status.charge_state;
            bool power_changed = (prev_on_mains != on_mains_);
            bool charge_changed = (prev_charge_state != current_charge_state);

            if (power_changed || charge_changed) {
                std::ostringstream oss;
                bool first_part = true;

                // Add power status change
                if (power_changed) {
                    std::string power_state = on_mains_ ? "mains" : "battery";
                    std::string prev_state = prev_on_mains ? "mains" : "battery";
                    oss << "Power: " << prev_state << " -> " << power_state;
                    first_part = false;
                }

                // Add charge state change
                if (charge_changed) {
                    if (!first_part) {
                        oss << ", ";
                    }
                    if (prev_charge_state != 255) {
                        oss << "Charge: " << getChargeStateName(prev_charge_state)
                            << " (" << static_cast<int>(prev_charge_state) << ") -> "
                            << getChargeStateName(current_charge_state)
                            << " (" << static_cast<int>(current_charge_state) << ")";
                    } else {
                        oss << "Charge: " << getChargeStateName(current_charge_state)
                            << " (" << static_cast<int>(current_charge_state) << ")";
                    }
                }

                // Add metrics
                oss << " | " << formatBriefMetrics(vbus_data, battery_data);
                logMessage(LOG_INFO, oss.str());

                // Update previous states
                if (power_changed) {
                    prev_on_mains = on_mains_;
                }
                if (charge_changed) {
                    prev_charge_state = current_charge_state;
                }
            }

            // Periodic status logging when on battery (every 60 seconds)
            if (!on_mains_) {
                auto now = std::chrono::steady_clock::now();
                bool should_log = false;

                if (last_battery_status_log_.time_since_epoch().count() == 0) {
                    // First time on battery - log immediately
                    last_battery_status_log_ = now;
                    should_log = true;
                } else {
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        now - last_battery_status_log_).count();
                    if (elapsed >= 60) {
                        last_battery_status_log_ = now;
                        should_log = true;
                    }
                }

                if (should_log) {
                    logBatteryStatus(charging_status, vbus_data, battery_data, cell_voltages);
                }
            } else {
                // Reset timer when mains power is restored
                last_battery_status_log_ = std::chrono::steady_clock::time_point();
            }

        } catch (const std::exception& e) {
            logMessage(LOG_ERR, std::string("UPS read failed: ") + e.what());
        }

        std::this_thread::sleep_for(timer_period);
    }

    logMessage(LOG_INFO, "UPS HAT monitoring loop stopped");
}

