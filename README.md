# OSSM-controller

OSSM embedded motion control software.

## Supported Boards

| Framework | Platform | Board |
|-----------|----------|-------|
| arduino | espressif32 | esp32thing |
| arduino | atmelavr | pro16MHzatmega328 |

## PlatformIO Setup

Install PlatformIO IDE as a VS Code Extension or [other integration](https://platformio.org/install/integration).

With PlatformIO installed, the [platformio.ini](platformio.ini) configuration file for the project will be recognized and all you have to do is build. All prerequisite libraries will be installed automatically.

Follow instructions for VS Code integration [here](https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode).

### Enable Tests

Add `test_ignore` option for each board in platformio.ini. For example, to enable all tests for the Arduino Pro Mini, add the following:

```ini
[env:pro16MHzatmega328]
...
test_ignore =
```

## Use Python to Run PlatformIO

First install Python 3

```bash
sudo apt install -y python3 python3-pip python3-venv
```

The create virtual environment, activate, and install platformio

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install platformio
```

<!-- 
### Wiring For Tests

TODO(kescholm): Explain how to wire hardware in order to run tests

There area number of unit tests which can be run with the proper wiring and external hardware components attached to each board.

TODO: Document wiring for board tests

-->
