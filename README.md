# LSPE_STRIP_TCS
This is the repository containing the code for the LSPE-STRIP Telescope Control System.

The code is used to move the telescope of the STRIP instruent.

### Installation

To install the code create a virtual envirnment running `python>=3.8`

then append at the end of the `bin/activate` script the following code:
```
MOTORS_PATH="/path/to/drivers/folder"
if ! [[ "$PYTHONPATH" =~ (^|:)$MOTORS_PATH(|/)(:|$) ]]; then
    export PYTHONPATH=$PYTHONPATH:$MOTORS_PATH
    echo drivers path added to PYTHONPATH
else
    echo drivers path already present in PYTHONPATH
fi
```
