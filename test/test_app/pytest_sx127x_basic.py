import pytest
from pytest_embedded import Dut


@pytest.mark.supported_targets
@pytest.mark.generic
def test_single(dut: Dut) -> None:
    dut.expect('Press ENTER to see the list of tests')
    dut.write('"rx_single"')
    dut.expect_unity_test_output()