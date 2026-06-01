*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}
Library         uart_example_keywords.py

*** Test Cases ***
UART example echoes 5 bytes
    UART Example Should Echo 5 Bytes
