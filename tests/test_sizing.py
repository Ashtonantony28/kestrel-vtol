"""M0: every sizing requirement in spec/vehicle.yaml limits passes on the current design."""
import pytest

from kestrel import sizing


@pytest.fixture(scope="module")
def result():
    return sizing.analyse()


def test_all_checks_pass(result):
    c = sizing.checks(result)
    assert all(c.values()), {k: v for k, v in c.items() if not v}


def test_hand_calc_hover_power(result):
    # momentum theory with the model's figure of merit and efficiencies: P = T^1.5 / sqrt(2 rho A) / (FM eta)
    import math
    T = result["weight_n"] / 4
    A = math.pi * 0.127 ** 2
    p_ideal = 4 * T ** 1.5 / math.sqrt(2 * 1.225 * A)
    assert result["lift"]["hover_power_w"] > p_ideal          # can't beat momentum theory
    assert result["lift"]["hover_power_w"] < 3.0 * p_ideal    # and isn't absurdly inefficient


def test_bom_within_budget():
    b = sizing.bom_totals()
    assert b["aircraft"][1] <= 240 and b["total"][1] <= 300
