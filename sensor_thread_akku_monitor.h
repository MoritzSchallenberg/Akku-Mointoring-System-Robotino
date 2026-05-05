
/***************************************************************************
 *  sensor_thread.h - Robotino sensor thread
 *
 *  Created: Apr 28 22:00:00 2026
 *  Copyright  2026-2029  Moritz Schallenberg
 *
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

#ifndef _PLUGINS_ROBOTINO_SENSOR_THREAD_H_
#define _PLUGINS_ROBOTINO_SENSOR_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>

#include <array>
#include <chrono>
#include <ctime>
#include <deque>
#include <stdlib.h>
#include <string>
#pragma once

#include <vector>
#include <chrono>

class RobotinoComThread;
class RobotinoActThread;

namespace fawkes {
class BatteryInterface;
class RobotinoSensorInterface;
class IMUInterface;
}

struct AkkuSlotInfo
{
	bool  plugged = false;
	float temperature = 0.0f;
	int   machine_id = 0;
};

class RobotinoSensorThread : public fawkes::Thread,
                             public fawkes::BlockedTimingAspect,
                             public fawkes::LoggingAspect,
                             public fawkes::ClockAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::BlackBoardAspect
{
	friend RobotinoActThread;

public:
	RobotinoSensorThread(RobotinoComThread *com_thread);

	virtual void init();
	virtual void loop();
	virtual void finalize();

protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void process_sensor_msgs();
	void update_distances(float *voltages);

	// Akku-Monitoring
	void  update_akku_monitor(const RobotinoComThread::SensorData &data);
	void  read_akku_slots(const RobotinoComThread::SensorData &data);
	float read_acs758_current_a(const RobotinoComThread::SensorData &data) const;
	float battery_voltage_v(const RobotinoComThread::SensorData &data) const;
	float average_temperature_c() const;
	float estimate_soc_from_voltage(float u_safe_v) const;
	void  handle_stillstand_voltage_calibration(const RobotinoComThread::SensorData &data);
	void  update_coulomb_counting(float dt_s);
	void  log_akku_status(bool force = false);
	float clamp01(float value) const;

	// Voltage to distance data points
	static const std::vector<std::pair<double, double>> voltage_to_dist_dps_;

private: 
	RobotinoComThread *com_;

	int                                   battery_counter = 0;
	bool                                  cfg_enable_gyro_;
	std::string                           cfg_imu_iface_id_;
	std::chrono::system_clock::time_point last_battery_warning;

	fawkes::BatteryInterface        *batt_if_;
	fawkes::RobotinoSensorInterface *sens_if_;
	fawkes::IMUInterface            *imu_if_;

// ---------------- Akku-Monitoring Konfiguration ----------------
	static constexpr int   ACS758_ANALOG_IDX = 0;
	static constexpr float ACS758_ZERO_V      = 2.5f;
	static constexpr float ACS758_SENS_V_A    = 0.010f; 

	const std::array<int, 3> SLOT_TEMP_ANALOG_IDX  = {{1, 2, 3}};
	const std::array<int, 3> SLOT_ID_ANALOG_IDX    = {{4, 5, 6}};
	const std::array<int, 3> SLOT_PLUG_DIGITAL_IDX = {{0, 1, 2}};

	float cfg_ah_pack_              = 4.0f;
	float cfg_u_min_v_              = 16.5f;
	float cfg_u_max_v_              = 20.5f;
	float cfg_temp_ref_c_           = 25.0f;
	float cfg_temp_coeff_v_per_c_   = 0.01f;
	float cfg_voltage_error_v_      = 0.30f;
	float cfg_stillstand_current_a_ = 0.50f;
	float cfg_relaxation_wait_s_    = 10.0f;
	float cfg_voltage_sample_s_     = 5.0f;
	float cfg_avg_window_s_         = 30.0f;
	float cfg_min_current_a_        = 1.0f;
	float cfg_k_cap_                = 0.80f;
	float cfg_k_i_                  = 1.20f;
	float cfg_voltage_mix_alpha_    = 0.80f;

	

// ---------------- Akku-Monitoring Zustand ----------------
	std::array<AkkuSlotInfo, 3> slots_;
	std::deque<float>           current_window_;
	std::vector<float>          voltage_samples_;

	bool monitor_initialized_ = false;
	bool stillstand_active_   = false;
	bool voltage_sampling_    = false;

	std::chrono::steady_clock::time_point last_monitor_update_;
	std::chrono::steady_clock::time_point stillstand_start_;
	std::chrono::steady_clock::time_point voltage_sample_start_;
	std::chrono::steady_clock::time_point last_status_log_;

	struct Akku {
	    float temperature;
	    bool plugged;
	    int id;
	};

	std::vector<Akku> akkus;
	int num_akkus = 3;

	std::chrono::steady_clock::time_point last_update;
	float dt;

	int   n_packs_       = 0;
	float u_bus_v_       = 0.0f;
	float u_avg_v_       = 0.0f;
	float u_corr_v_      = 0.0f;
	float u_safe_v_      = 0.0f;
	float i_last_a_      = 0.0f;
	float i_avg_a_       = 0.0f;
	float ah_start_      = 0.0f;
	float ah_used_       = 0.0f;
	float ah_total_      = 0.0f;
	float ah_rest_       = 0.0f;
	float t_rest_min_    = 0.0f;
	float t_safe_min_    = 0.0f;
	float soc_voltage_   = 0.0f;
};

#endif
