# BLDC speed governor tools

## Scripts

- adjust_settings.py - GUI tool to connect to running program on your MCU and communicate through serial port in order to adjust the running settings
- pid_simulator.py - small demonstration app, its purpose is just to give you a little perspective on how adjusting the parameters might change the behavior of your control system

## Setup

To run the scripts you need to have working Python installation.

The scripts were developed with Python 3.12.4.

1. Create [virtual environment](https://docs.python.org/3/library/venv.html) that will hold script dependencies:
    
    ```shell
    python -m venv "$HOME/bldc-venv"
    ```

2. Activate created virtual environment:

    ```shell
    . "$HOME/bldc-env/Scripts/activate"
    ```

3. Install dependencies:

    ```shell
    python -m pip install -r requirements.py
    ```

4. Check out the scripts:

    ```shell
    python adjust_settings.py
    ```
