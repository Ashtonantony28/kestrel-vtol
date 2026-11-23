"""Independent validation checks (kestrel/validate.py): aero cross-method, hand calcs, structures, numerics."""
from kestrel import validate


def test_all_validation_checks_pass():
    assert validate.main() == 0
