*** Settings ***
Suite Setup     Setup
Suite Teardown  Teardown
Test Teardown   Test Teardown
Resource        ${RENODEKEYWORDS}
Library         bus_example_keywords.py

*** Test Cases ***
Protocol Turns LED On
    Verify Serial Protocol LED Control    a    True

Protocol Turns LED Off
    Verify Serial Protocol LED Control    b    False
