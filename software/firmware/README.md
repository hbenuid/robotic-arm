# software/firmware — microcontroller firmware

One subfolder per board; part of neither uv project.

## `stm32/` — archived Nucleo-F446RE test firmware
From before the Python CLI (`software/control/`): self-contained PlatformIO projects (`platform = ststm32`,
`board = nucleo_f446re`, `framework = stm32cube`; open a project folder in PlatformIO, serial monitor at 115200 baud).
The `include/`, `lib/` and `test/` READMEs in each project are PlatformIO's stubs.

| project | what it does |
|---|---|
| `blink_test` | blinks the PA5 user LED and prints over UART2 |
| `servo42d_test` | one-shot MKS SERVO42D CAN test through an SN65HVD230 transceiver (PB8 RX / PB9 TX): status, read, enable, speed and position commands; the servo set to SR_vFOC, CAN 500K, ID 01 |
| `servo42d_interactive_test` | a serial-terminal controller for one motor (CAN ID 0x01): speed, jog, stop, zero, enable and status keys, numeric commands such as `+500` / `p3200` |
| `servo42d_2motor_interactive_test` | the same for two motors (CAN IDs 0x01 and 0x02), `*` broadcasts |
