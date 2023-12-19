import pytest
from pytest_embedded import Dut
from typing import Tuple

@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_explicit_header(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_lora_rx_explicit_header"')
    dut_rx.expect('RX started')

    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"sx127x_test_lora_tx_explicit_header"')
    dut_tx.expect_unity_test_output()

    dut_rx.expect_unity_test_output()

@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_implicit_header(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_lora_rx_implicit_header"')
    dut_rx.expect('RX started')

    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"sx127x_test_lora_tx_implicit_header"')
    dut_tx.expect_unity_test_output()

    dut_rx.expect_unity_test_output()

@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_rx_after_cad(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_lora_rx_after_cad"')

    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"sx127x_test_lora_tx_explicit_header"')
    dut_tx.expect_unity_test_output()

    dut_rx.expect_unity_test_output()

@pytest.mark.supported_targets
@pytest.mark.generic
@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_rx_deepsleep(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_lora_rx_deepsleep"')

    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"sx127x_test_lora_tx_explicit_header"')
    dut_tx.expect_unity_test_output()

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_lora_rx_deepsleep_verify"')
    dut_rx.expect_unity_test_output()
