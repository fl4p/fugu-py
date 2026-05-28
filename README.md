
## Transports (`transport.py`)

`SerialTransport`, `SocketTransport` (telnet), `BleTransport` (local BLE / Nordic UART Service),
`EspHomeBleTransport` (BLE NUS through an ESPHome `bluetooth_proxy` with `active: true`, over the
plaintext native API — no noise PSK; needs `aioesphomeapi`), and `MqttTransport`.

## has_crashed(reset=False)
if the device has crashed (panic) since the console was created or since the last call with reset=True
* detect a panic with re.compile(r"Guru Meditation|panic'?ed|Backtrace:|abort\(\) was called|StoreProhibited|LoadProhibited|assert failed")
* you can add more to the pattern
* `rst:0x..` is deliberately NOT in the pattern: the bootloader prints it on every reset (incl. a
  clean `restart` and power-on), so it signals a reboot, not a crash.

When transport is Telnet/BLE/MQTT, we are unlikely to catch a crash. would need an unclean shutdown flag. has_crashed only works with serial transport.
Throw if this function is used with another transport.
Even on serial, native USB-CDC (usb_serial_jtag) re-enumerates on the reboot that follows a panic,
so the dump can be cut off; a hardware UART bridge is more reliable. More universal is has_rebooted:

## has_rebooted(reset=False)
if the device has rebooted since the console was created or since the last call with reset=True
* detected by polling the `uptime` command and flagging a reboot when uptime regresses (uptime is
  monotonic seconds since boot, resets only on a real reboot). Works on any transport.
* the status-line `N` is NOT used: it is `Vout->numSamples` and zeroes on every MPPT sweep / ADC
  recalibration (~every 30 min and on power changes), so it is not monotonic.
* call periodically so a reboot is caught before uptime climbs back past the previous reading.

## crashed(since=None, baseline_uptime=None, margin=5.0)
stateless variant against a caller-held baseline (see crash_mark()). True if a panic marker was
logged after `since` (serial only; silently skipped on other transports), or the device has
rebooted since `baseline_uptime` — checked as `uptime + margin < baseline_uptime + elapsed`, so a
late check still catches the reset.
