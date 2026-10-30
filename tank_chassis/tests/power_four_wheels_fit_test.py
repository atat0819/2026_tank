"""Check four-wheel identification against independent synthetic power data."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "user/core/Alg/PowerControl-TestVersion/PythonProject/PythonScript/Polynomial Method/PowerFourWheels.py"
assert SCRIPT.exists(), "Four-wheel least-squares script is missing"
spec = importlib.util.spec_from_file_location("power_fit", SCRIPT)
fit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fit)


class PowerFitTest(unittest.TestCase):
    def test_recovers_per_motor_coefficients_with_signed_feedback(self):
        rng = np.random.default_rng(27)
        current = rng.uniform(-15, 15, (800, 4))
        speed = rng.uniform(-500, 500, (800, 4))
        expected = np.array([.33, -.014, .00053, .0055, .041, .0000053])
        power = sum(expected[0] + expected[1]*current[:, i]
                    + expected[2]*abs(speed[:, i]) + expected[3]*current[:, i]*speed[:, i]
                    + expected[4]*current[:, i]**2 + expected[5]*speed[:, i]**2
                    for i in range(4))
        matrix = fit.build_features(current, speed)
        result, diagnostics = fit.least_squares(matrix, power)
        np.testing.assert_allclose(result, expected, rtol=1e-9, atol=1e-11)
        assert diagnostics["rank"] == 6
        assert fit.metrics(power, matrix @ result)["rmse_w"] < 1e-10

    def test_rejects_unidentifiable_constant_data(self):
        matrix = fit.build_features(np.ones((20, 4)), np.zeros((20, 4)))
        with self.assertRaisesRegex(ValueError, "rank"):
            fit.least_squares(matrix, np.ones(20))

    def test_weighted_fusion_matches_known_group_objective(self):
        rng = np.random.default_rng(28)
        base = fit.build_features(rng.uniform(-10, 10, (300, 4)),
                                  rng.uniform(-500, 500, (300, 4)))
        ground = np.array([.3, -.01, .0005, .0055, .041, .000005])
        air = np.array([.1, -.2, .006, .0054, .043, -.000003])
        matrix = np.concatenate([base, base])
        power = np.concatenate([base @ ground, base @ air])
        weights = np.r_[np.full(300, .999/300), np.full(300, .001/300)]
        result, _ = fit.least_squares(matrix, power, weights=weights)
        np.testing.assert_allclose(result, .999*ground+.001*air, rtol=1e-9, atol=1e-11)
        scaled, _ = fit.least_squares(matrix, power, weights=weights*1000)
        np.testing.assert_allclose(result, scaled, rtol=1e-9, atol=1e-11)

    def test_invalid_weights_are_rejected(self):
        matrix = np.ones((10, 6))
        for weights in [np.ones(9), np.full(10, -1), np.zeros(10), np.full(10, np.nan)]:
            with self.assertRaisesRegex(ValueError, "weights"):
                fit.least_squares(matrix, np.ones(10), weights=weights)

    def test_load_filters_invalid_rows_and_preserves_regeneration_and_gaps(self):
        data = np.zeros((5, 15))
        data[:, 0] = [-5, 2, 3, 4, 5]
        data[:, 10] = [1, 1.01, 1.02, 1.03, 2]
        data[:, 13] = [15, 15, 7, 15, 15]
        data[:, 14] = [1, 2, 3, 4, 20]
        data[3, 1] = np.nan
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.csv"
            pd.DataFrame(data, columns=[f"I{i}" for i in range(15)]).to_csv(path, index=False)
            capture = fit.load_capture(path)
        assert capture["audit"]["rows_total"] == 5
        assert capture["audit"]["rows_used"] == 3
        assert capture["audit"]["negative_power_rows"] == 1
        assert capture["audit"]["missing_sequence_count"] == 15
        np.testing.assert_array_equal(capture["power"], [-5, 2, 5])


if __name__ == "__main__":
    unittest.main()
