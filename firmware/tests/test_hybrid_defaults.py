# SPDX-License-Identifier: MIT
"""The shipped literal header must match the supplied specification exactly."""
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class HybridDefaults(unittest.TestCase):
    def test_generated_header_and_dimensions(self):
        spec = importlib.util.spec_from_file_location(
            "hybrid_defaults_generator", ROOT / "tools/generate-hybrid-defaults.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(module.OUTPUT.read_text(), module.generated())
        data = json.loads(module.SOURCE.read_text())
        clock = data["clock"]
        thermal = data["thermal_outer"]
        self.assertEqual(clock["frequency_time_constant_s"], 0)
        self.assertEqual(clock["phase_natural_time_s"], 10)
        self.assertAlmostEqual(
            thermal["integral_gain_K_per_s_per_unit_duty"],
            thermal["gain_estimate_K_per_unit_duty"] / thermal["nominal_recentering_time_s"],
        )
        self.assertTrue(data["boot"]["autostart_enabled"])
        self.assertTrue(data["boot"]["thermal_outer_enabled"])
        self.assertFalse(data["boot"]["require_host_or_usb_connection"])
        self.assertEqual(data["boot"]["additional_ppp_wait_s"], 0)
        self.assertEqual(data["boot"]["qualification_dwell_s"], 60)
        self.assertEqual(clock["acquisition_average_samples"], 50)
        self.assertEqual(clock["acquisition_verification_samples"], 50)
        self.assertEqual(clock["lock_good_dwell_s"], 10)
        self.assertEqual(clock["led1_basis"], "INDEPENDENT_FREQUENCY_MEASUREMENTS")
        self.assertEqual(clock["fallback_frequency_time_constant_s"], 10)
        self.assertEqual(clock["fallback_phase_natural_time_s"], 100)
        self.assertEqual(clock["frequency_settle_samples"], 50)
        self.assertEqual(clock["frequency_settle_good_dwell_s"], 10)
        self.assertEqual(clock["frequency_settle_mean_drift_limit_ppb"], 0.05)
        self.assertEqual(clock["frequency_settle_mean_error_limit_ppb"], 0.05)
        self.assertEqual(clock["frequency_settle_drift_stddev_limit_ppb"], 0.1)
        self.assertEqual(clock["lock_statistics_window_s"], 100)
        self.assertEqual(clock["tracking_temperature_statistics_window_s"], 120)
        self.assertEqual(data["boot"]["phase_mode"], "RELATIVE_FIXED")
        self.assertEqual(
            data["thermal_startup"]["policy"],
            "FULL_POWER_SLOPE_CAPTURE_THEN_INDEPENDENT_OUTER_CENTERING",
        )
        self.assertFalse(thermal["startup_requires_gnss"])
        self.assertTrue(thermal["qualification_during_centering"])
        self.assertFalse(thermal["holdover_freeze_after_first_acquisition"])
        self.assertNotIn("start_after_healthy_tracking_s", thermal)
        boot = data["thermal_boot"]
        self.assertEqual((boot["reference_delay_s"], boot["reference_window_s"]), (60, 30))
        self.assertEqual(boot["capture_confirm_s"], 20)
        self.assertEqual(boot["capture_slope_mC_per_min"], 400)
        self.assertNotIn("capture_ratio_permille", boot)
        self.assertEqual(boot["boost_duty_permille"], 1000)
        self.assertEqual(boot["handoff_duty_permille"], 0)
        self.assertTrue(boot["manual_pid_default_continuous"])
        qualification = data["qualification"]
        self.assertEqual(qualification["satellite_count_policy"], "DIAGNOSTIC_ONLY")
        self.assertNotIn("minimum_satellites_used", qualification)
        self.assertEqual(qualification["temperature_statistics_window_s"], 60)
        self.assertEqual(qualification["temperature_max_abs_error_C"], 0.25)
        self.assertAlmostEqual(qualification["temperature_error_slope_limit_K_per_s"] * 60, 0.1)
        self.assertTrue(qualification["temperature_spread_is_diagnostic_only"])
        inner = data["thermal_inner"]
        self.assertTrue(data["implementation"]["new_pid_derivative_term"])
        self.assertAlmostEqual(inner["kd_duty_s_per_K"] / inner["kp_duty_per_K"], 6.5)
        self.assertEqual(inner["derivative_filter_s"], 3.0)
        self.assertEqual(
            (
                thermal["setpoint_min_C"],
                thermal["setpoint_max_C"],
                thermal["overtemperature_trip_C"],
            ),
            (0.0, 60.0, 70.0),
        )


if __name__ == "__main__":
    unittest.main()
