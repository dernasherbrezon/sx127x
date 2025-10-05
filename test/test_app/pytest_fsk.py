import pytest
from pytest_embedded import Dut
from typing import Tuple

@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_variable_length(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_fsk_rx_variable_length"')
    dut_rx.expect('RX started')

    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"sx127x_test_fsk_tx_variable_length"')
    dut_tx.expect_unity_test_output()

    dut_rx.expect_unity_test_output()

@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_beacon(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_fsk_rx_beacons"')
    dut_rx.expect('RX started')

    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"sx127x_test_fsk_tx_beacons"')
    dut_tx.expect_unity_test_output()

    dut_rx.expect_unity_test_output()

@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_fixed_length(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_fsk_rx_fixed_small"')
    dut_rx.expect('RX started')
    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"sx127x_test_fsk_tx_fixed_small"')
    dut_tx.expect_unity_test_output()
    dut_rx.expect_unity_test_output()

    dut_rx.expect('Enter next test, or \'enter\' to see menu')
    dut_rx.write('"sx127x_test_fsk_rx_fixed_batch"')
    dut_rx.expect('RX started')
    dut_tx.expect('Enter next test, or \'enter\' to see menu')
    dut_tx.write('"sx127x_test_fsk_tx_fixed_batch"')
    dut_tx.expect_unity_test_output()
    dut_rx.expect_unity_test_output()

    dut_rx.expect('Enter next test, or \'enter\' to see menu')
    dut_rx.write('"sx127x_test_fsk_rx_fixed_max"')
    dut_rx.expect('RX started')
    dut_tx.expect('Enter next test, or \'enter\' to see menu')
    dut_tx.write('"sx127x_test_fsk_tx_fixed_max"')
    dut_tx.expect_unity_test_output()
    dut_rx.expect_unity_test_output()

@pytest.mark.parametrize('count', [
    2,
], indirect=True)
def test_max_baud(dut: Tuple[Dut, Dut]) -> None:
    dut_rx = dut[0]
    dut_tx = dut[1]

    dut_rx.expect('Press ENTER to see the list of tests')
    dut_rx.write('"sx127x_test_fsk_rx_max_baud"')
    dut_rx.expect('RX started')

    dut_tx.expect('Press ENTER to see the list of tests')
    dut_tx.write('"sx127x_test_fsk_tx_max_baud"')
    dut_tx.expect_unity_test_output()

    dut_rx.expect_unity_test_output()
