- [ ] Investigate why enabling `wmbus_meter.on_telegram` triggers FreeRTOS stack overflow (vApplicationStackOverflowHook) while disabling it avoids crash.
- [ ] Implement safer deferred callback execution for `Meter::handle_frame()` so automations/logger run without overflowing an undersized ESPHome/defer execution stack.
- [ ] Rebuild/flash and verify crash is gone.
- [ ] If still crashing, reduce DEBUG logging for wmbus_meter/wmbus and/or add stack watermark logging to identify the overflowing task.

