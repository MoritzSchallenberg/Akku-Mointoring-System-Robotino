
/***************************************************************************
 *  sensor_thread.cpp - Robotino sensor thread
 *
 *  Created: Apr 28 22:00:00 2026
 *  Copyright  2026-2029  Moritz Schallenberg
 
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "sensor_thread.h"

#include "com_thread.h"

#include <interfaces/BatteryInterface.h>
#include <interfaces/IMUInterface.h>
#include <interfaces/RobotinoSensorInterface.h>

#include <algorithm>
#include <cmath>
#include <numeric>

using namespace fawkes;

/** @class RobotinoSensorThread "sensor_thread.h"
 * Robotino sensor hook integration thread.
 * This thread integrates into the Fawkes main loop at the SENSOR hook and
 * writes new sensor data.
 * @author Tim Niemueller
 */

/// taken from Robotino API2 DistanceSensorImpl.hpp
const std::vector<std::pair<double, double>> VOLTAGE_TO_DIST_DPS = {{0.0, 1.0},
                                                                    {1.05, 1.0},
                                                                    {1.11, 0.12},
                                                                    {1.3, 0.10},
                                                                    {1.4, 0.09},
                                                                    {1.55, 0.08},
                                                                    {1.8, 0.07},
                                                                    {2.35, 0.05},
                                                                    {10.55, 0.04}};

/** Constructor.
 * @param com_thread communication thread to trigger for writing data
 */
RobotinoSensorThread::RobotinoSensorThread(RobotinoComThread *com_thread)
: Thread("RobotinoSensorThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
	com_ = com_thread;
}

auto now = std::chrono::steady_clock::now();
dt = std::chrono::duration<float>(now - last_update).count();
last_update = now;

if(dt < 0.001f || dt > 1.0f) {
    dt = 0.0f;
}

void
RobotinoSensorThread::init()
{
	cfg_enable_gyro_  = config->get_bool("/hardware/robotino/gyro/enable");
	cfg_imu_iface_id_ = config->get_string("/hardware/robotino/gyro/interface_id");

	batt_if_ = NULL;
	sens_if_ = NULL;
	imu_if_  = NULL;

	std::chrono::duration<int, std::ratio<60 * 60 * 24>> one_day(1);
	last_battery_warning = std::chrono::system_clock::now() - one_day;

	batt_if_ = blackboard->open_for_writing<BatteryInterface>("Robotino");
	sens_if_ = blackboard->open_for_writing<RobotinoSensorInterface>("Robotino");

	if (cfg_enable_gyro_) {
		imu_if_ = blackboard->open_for_writing<IMUInterface>(cfg_imu_iface_id_.c_str());
	}

	last_monitor_update_ = std::chrono::steady_clock::now();
	last_status_log_     = std::chrono::steady_clock::now();
	current_window_.clear();
	voltage_samples_.clear();
	logger->log_info(name(), "Akku-Monitoring Erweiterung initialisiert");
}

// Schließt alle Schnittstellen beim beenden des Programmes
void
RobotinoSensorThread::finalize()
{
	blackboard->close(sens_if_);
	blackboard->close(batt_if_);
	blackboard->close(imu_if_);
}

// Liest  Sensordaten ein, Aktualisiert interne Zustände (Motoren, Sensoren), Berechnet Akkuzustand, gibt Warnungen aus
void
RobotinoSensorThread::loop()
{
	process_sensor_msgs();
	RobotinoComThread::SensorData data;
	if (com_->get_data(data)) {
		sens_if_->set_mot_velocity(data.mot_velocity);
		sens_if_->set_mot_position(data.mot_position);
		sens_if_->set_mot_current(data.mot_current);
		sens_if_->set_bumper(data.bumper);
		sens_if_->set_bumper_estop_enabled(data.bumper_estop_enabled);
		sens_if_->set_digital_in(data.digital_in);
		sens_if_->set_digital_out(data.digital_out);
		sens_if_->set_analog_in(data.analog_in);
		update_distances(data.ir_voltages);
		sens_if_->write();
		batt_if_->set_voltage(data.bat_voltage);
		batt_if_->set_current(data.bat_current);
		batt_if_->set_absolute_soc(data.bat_absolute_soc);
		batt_if_->write();
		update_akku_monitor(data);
		if (data.bat_voltage < 17200) {
			if (battery_counter > 100) {
				battery_counter                           = 0;
				std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
				std::chrono::duration<double> elapsed_seconds = now - last_battery_warning;
				if (elapsed_seconds.count() > 300) {
					last_battery_warning = std::chrono::system_clock::now();
					logger->log_warn(
					  name(),
					  "BATTERY LEVEL ARE LOW. Battery is currently supplying %f mV to the system",
					  data.bat_voltage);
					std::system("notify-send -u critical -i battery-low -t 10000 \"$(hostname)\" \"Battery "
					            "is running low!\"");
				}
			} else {
				battery_counter++;
			}
		} else {
			battery_counter = 0;
		}
		if (cfg_enable_gyro_) {
			if (data.imu_enabled) {
				imu_if_->set_angular_velocity(data.imu_angular_velocity);
				imu_if_->set_angular_velocity_covariance(data.imu_angular_velocity_covariance);
				imu_if_->set_orientation(data.imu_orientation);
				imu_if_->write();
			} else {
				if (fabs(data.imu_angular_velocity[0] + 1.) > 0.00001) {
					imu_if_->set_linear_acceleration(0, -1.);
					imu_if_->set_angular_velocity(0, -1.);
					imu_if_->set_angular_velocity(2, 0.);
					imu_if_->set_orientation(0, -1.);
					imu_if_->write();
				}
			}
		}
	}
}

// Ausführung verschiedener Steuerbefehle
void
RobotinoSensorThread::process_sensor_msgs()
{
	while (!sens_if_->msgq_empty()) {
		if (RobotinoSensorInterface::SetBumperEStopEnabledMessage *msg =
		      sens_if_->msgq_first_safe(msg)) {
			com_->set_bumper_estop_enabled(msg->is_enabled());
		} else if (RobotinoSensorInterface::SetDigitalOutputMessage *msg =
		             sens_if_->msgq_first_safe(msg)) {
			try {
				com_->set_digital_output(msg->digital_out(), msg->is_enabled());
			} catch (Exception &e) {
				logger->log_warn(name(), e);
			}
		}
		sens_if_->msgq_pop();
	}
}

// Read Robotino Spannungsmessung
float
RobotinoSensorThread::clamp01(float value) const
{
	if (value < 0.0f) return 0.0f;
	if (value > 1.0f) return 1.0f;
	return value;
}

// Read Robotino Akku Messung
float
RobotinoSensorThread::battery_voltage_v(const RobotinoComThread::SensorData &data) const
{
	return data.bat_voltage / 1000.0f;
}

// Read ACS758 Daten
float
RobotinoSensorThread::read_acs758_current_a(const RobotinoComThread::SensorData &data) const
{
	const float u_out_v = data.analog_in[ACS758_ANALOG_IDX];
	return (u_out_v - ACS758_ZERO_V) / ACS758_SENS_V_A;
}

// Read Akku Status und speichern Plugged, Temperatur und Maschinen_id
void
RobotinoSensorThread::read_akku_slots(const RobotinoComThread::SensorData &data)
{
	n_packs_ = 0;
	for (size_t i = 0; i < slots_.size(); ++i) {
		slots_[i].plugged = data.digital_in[SLOT_PLUG_DIGITAL_IDX[i]];
		slots_[i].temperature = data.analog_in[SLOT_TEMP_ANALOG_IDX[i]];
		slots_[i].machine_id = static_cast<int>(data.analog_in[SLOT_ID_ANALOG_IDX[i]] * 1000.0f);
		if (slots_[i].plugged) {
			++n_packs_;
		}
	}
	ah_total_ = static_cast<float>(n_packs_) * cfg_ah_pack_;
}

// Berechnung durchschnittliche Temperatur
float
RobotinoSensorThread::average_temperature_c() const
{
	float sum = 0.0f;
	int   cnt = 0;
	for (const auto &slot : slots_) {
		if (slot.plugged) {
			sum += slot.temperature;
			++cnt;
		}
	}
	return (cnt > 0) ? (sum / cnt) : cfg_temp_ref_c_;
}

float
RobotinoSensorThread::estimate_soc_from_voltage(float u_safe_v) const
{
	return clamp01((u_safe_v - cfg_u_min_v_) / (cfg_u_max_v_ - cfg_u_min_v_));
}

// Spannungs Messung bei Stillstand 1.bewegt sich der Roboter 2.Bei Stillstand messen...)
void
RobotinoSensorThread::handle_stillstand_voltage_calibration(const RobotinoComThread::SensorData &data)
{
	const auto  now         = std::chrono::steady_clock::now();
	const float abs_current = std::fabs(i_last_a_);
	if (abs_current < cfg_stillstand_current_a_) {
		if (!stillstand_active_) {
			stillstand_active_ = true;
			voltage_sampling_  = false;
			stillstand_start_  = now;
			voltage_samples_.clear();
		}
		const float stillstand_s = std::chrono::duration<float>(now - stillstand_start_).count();
		if (!voltage_sampling_ && stillstand_s >= cfg_relaxation_wait_s_) {
			voltage_sampling_     = true;
			voltage_sample_start_ = now;
			voltage_samples_.clear();
			logger->log_info(name(), "Akku-Kalibrierung: Stillstand stabil, Spannungsmessung gestartet");
		}
		if (voltage_sampling_) {
			voltage_samples_.push_back(battery_voltage_v(data));
			const float sample_s = std::chrono::duration<float>(now - voltage_sample_start_).count();
			if (sample_s >= cfg_voltage_sample_s_ && !voltage_samples_.empty()) {
				u_avg_v_ = std::accumulate(voltage_samples_.begin(), voltage_samples_.end(), 0.0f)
				           / static_cast<float>(voltage_samples_.size());
				const float temp_c = average_temperature_c();
				u_corr_v_    = u_avg_v_ + cfg_temp_coeff_v_per_c_ * (cfg_temp_ref_c_ - temp_c);
				u_safe_v_    = u_corr_v_ - cfg_voltage_error_v_;
				soc_voltage_ = estimate_soc_from_voltage(u_safe_v_);
				const float ah_voltage_est = soc_voltage_ * ah_total_;
				if (!monitor_initialized_) {
					ah_start_            = ah_voltage_est;
					ah_used_             = 0.0f;
					ah_rest_             = ah_start_;
					monitor_initialized_ = true;
					logger->log_info(name(), "Akku-Monitoring initial kalibriert");
				} else {
					const float ah_cc_rest = std::max(0.0f, ah_start_ - ah_used_);
					ah_rest_ = cfg_voltage_mix_alpha_ * ah_cc_rest
					           + (1.0f - cfg_voltage_mix_alpha_) * ah_voltage_est;
					ah_start_ = ah_rest_ + ah_used_;
					logger->log_info(name(), "Akku-Monitoring im Stillstand rekalibriert");
				}
				logger->log_info(name(),
				                 "U_avg=%.2f V | U_corr=%.2f V | U_safe=%.2f V | T=%.2f | SoC_U=%.1f %%",
				                 u_avg_v_,
				                 u_corr_v_,
				                 u_safe_v_,
				                 temp_c,
				                 soc_voltage_ * 100.0f);
				voltage_sampling_ = false;
				voltage_samples_.clear();
			}
		}
	} else {
		stillstand_active_ = false;
		voltage_sampling_  = false;
		voltage_samples_.clear();
	}
}

// Wie viel Strom schon verbraucht und wie lange Restlaufzeit
void
RobotinoSensorThread::update_coulomb_counting(float dt_s)
{
	if (!monitor_initialized_) {
		return;
	}
	if(dt > 0.0f) {
	    Ah_used += current * dt / 3600.0f;
	}
	current_window_.push_back(std::fabs(i_last_a_));
	const size_t max_samples = static_cast<size_t>(std::max(1.0f, cfg_avg_window_s_ / std::max(dt_s, 0.001f)));
	while (current_window_.size() > max_samples) {
		current_window_.pop_front();
	}
	if (!current_window_.empty()) {
		i_avg_a_ = std::accumulate(current_window_.begin(), current_window_.end(), 0.0f)
		           / static_cast<float>(current_window_.size());
	}
	ah_rest_ = std::max(0.0f, ah_start_ - ah_used_);
	const float i_calc = std::max(i_avg_a_, cfg_min_current_a_);
	float I_safe = std::max(fabs(current), 1.0f);
	t_rest = Ah_rest / I_safe * 60.0f;
	t_safe = (Ah_rest * 0.8f) / (I_safe * 1.2f) * 60.0f;
	t_safe = std::max(0.0f, t_safe);
}

// Status-Anzeige
void
RobotinoSensorThread::log_akku_status(bool force)
{
	const auto  now       = std::chrono::steady_clock::now();
	const float elapsed_s = std::chrono::duration<float>(now - last_status_log_).count();
	if (!force && elapsed_s < 30.0f) {
		return;
	}
	last_status_log_ = now;
	logger->log_info(name(), "===== Akku-Monitoring Gesamtübersicht =====");
	logger->log_info(name(),
	                 "Packs=%d | U_bus=%.2f V | U_avg=%.2f V | U_corr=%.2f V | U_safe=%.2f V",
	                 n_packs_,
	                 u_bus_v_,
	                 u_avg_v_,
	                 u_corr_v_,
	                 u_safe_v_);
	logger->log_info(name(),
	                 "I_last=%.2f A | I_avg=%.2f A | Ah_used=%.3f Ah | Ah_rest=%.3f Ah | t_rest=%.1f min | t_safe=%.1f min",
	                 i_last_a_,
	                 i_avg_a_,
	                 ah_used_,
	                 ah_rest_,
	                 t_rest_min_,
	                 t_safe_min_);
	for (size_t i = 0; i < slots_.size(); ++i) {
		logger->log_info(name(),
		                 "Slot %zu: plug=%d | temp=%.2f | id=%d",
		                 i + 1,
		                 slots_[i].plugged ? 1 : 0,
		                 slots_[i].temperature,
		                 slots_[i].machine_id);
	}
	logger->log_info(name(), "===========================================");
}

// Zentrale Steuerfunktion Akku-Monitoring (Abstände zwischen Zeitmessungen, Aktuallisierung Temperatur usw., Spannungsmessung nur im Stillstand usw.)
void
RobotinoSensorThread::update_akku_monitor(const RobotinoComThread::SensorData &data)
{
	const auto now  = std::chrono::steady_clock::now();
	float      dt_s = std::chrono::duration<float>(now - last_monitor_update_).count();
	last_monitor_update_ = now;
	if (dt_s < 0.0f || dt_s > 5.0f) {
		dt_s = 0.0f;
	}
	u_bus_v_  = battery_voltage_v(data);
	i_last_a_ = read_acs758_current_a(data);
	read_akku_slots(data);
	handle_stillstand_voltage_calibration(data);
	update_coulomb_counting(dt_s);
	log_akku_status(false);
}

// Errechnen von Ausgleichsspannung mit 20% aus Messspannung und 80% aus errechnete Spannung
void
RobotinoSensorThread::update_distances(float *voltages)
{
	float        dist_m[NUM_IR_SENSORS];
	const size_t num_dps = VOLTAGE_TO_DIST_DPS.size();
	for (int i = 0; i < NUM_IR_SENSORS; ++i) {
		dist_m[i] = 1.0;
		for (size_t j = 0; j < num_dps - 1; ++j) {
			const double lv = VOLTAGE_TO_DIST_DPS[j].first;
			const double rv = VOLTAGE_TO_DIST_DPS[j + 1].first;
			if ((voltages[i] >= lv) && (voltages[i] < rv)) {
				const double ld = VOLTAGE_TO_DIST_DPS[j].second;
				const double rd = VOLTAGE_TO_DIST_DPS[j + 1].second;
				double dv = rv - lv;
				double dd = rd - ld;
				dist_m[i] = (dd / dv) * (voltages[i] - lv) + ld;
				break;
			}
		}
	}
	sens_if_->set_distance(dist_m);
}
