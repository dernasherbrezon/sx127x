import pytest
from pytest_embedded import Dut
from typing import Tuple

@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_single(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"lora_rx"')
    dut_rx.expect('(1)')
    dut_rx.write("1")

    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"lora_rx"')
    dut_tx.expect('(2)')
    dut_tx.write("2")
    dut_tx.expect_unity_test_output()

    dut_rx.expect("CAFE")
    dut_rx.expect_unity_test_output()
