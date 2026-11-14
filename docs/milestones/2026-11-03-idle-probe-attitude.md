# 2026-11-03 USB standby orientation

## Contract and lifecycle

After a valid HELLO on a connected USB link, standby starts an ICM-only 200 SPS
service through the existing FIFO, time estimator, FIR and AGM path. ADXL remains
powered down. There are no new commands, frame fields, sessions, or unsolicited
frames: orientation still leaves only inside a host FETCH window.

The acquisition machine has an explicit pose-only mode. It sends ICM reads to
the attitude tap without enqueueing raw batches. The last stopped measurement
status (samples, drops, segments and STOP boundary) is saved and restored around
that mode. A pose start requires both hardware channels OFF and both raw queues
empty, so undelivered STOP tail batches remain available to FETCH.

START and SAVE_CAL safely stop the private pose service asynchronously before
starting raw acquisition or programming calibration flash. Their normal cached
responses are emitted after that transition, and exact request retries do not
repeat side effects. Calibration capture still requires explicit measurement;
standby does not feed the calibration accumulator or change saved records.

HELLO replacement and USB disconnect cancel any old operation, stop hardware,
clear old pose outputs and discard the old raw tail without relabelling it with
a new session. Disconnect invalidates the binding; another HELLO is needed after
re-enumeration. Raw streaming is suppressed while the abandoned tail is being
cleared. In ordinary measurement STOP, the tail is retained and standby resumes
only after it has drained.

Standby attitude output is a latest-value mailbox. It replaces an older unsent
pose, so a paused FETCH pump cannot return a stale eight-output queue on resume.
Measurement start restores the original eight-output FIFO/drop policy. Existing
200 SPS FIR timing is unchanged: 245 ms group delay, represented in sample times;
sensor startup and filter warmup precede the first valid pose.

## Software verification

- `run_probe_time.py`: PASS.
- `run_probe_agm.py`: official reference vectors PASS.
- `run_probe_acquisition.py`: PASS, including ICM-only standby, frozen raw
  statistics/boundaries, restart and STOP-tail protection.
- `run_probe_spi.py`: SPI, validation, FIFO, frame, LED, attitude, calibration,
  USB, protocol, board-check and IRQ modes 0 through 3 PASS. Attitude coverage
  includes the standby latest-value mailbox; protocol coverage includes no
  pre-HELLO service, idle FETCH without raw frames, async START/save, disconnect
  and rebind. Independent-review additions verify duplicate requests during
  pending START/save, cached completion without repeated side effects, HELLO
  replacement during measurement/pending START, and disconnect cancellation of
  pending flash save. The existing IRQ fixture has an implicit USB handler prototype
  warning; all native firmware builds are warning-free.
- `python -m unittest discover -s tests -p test_pc_host.py`: 6 tests PASS.
- `verify_build_modes.py`: all four modes PASS, 0 errors / 0 warnings.
  Production/default: Code=62740, RO=2636, RW=288, ZI=53160. SRAM total 53448
  bytes, leaving 12088 bytes within the 64 KiB SRAM layout.
- `verify_board_image.py`: actual C startup and stack watermark PASS; allocated
  flash stays below the calibration sectors beginning at `0x0803F000`.
- RTT project integration and four logging macro contracts: PASS.

No project integration change, commit, flash download or calibration write was
performed by the implementation task. Existing IDE project/workspace changes
were preserved. Hardware acceptance and final image download are coordinated by
the parent task; software emulation does not establish physical sensor timing or
USB delivery rates.

## Parent-task hardware acceptance

Both sides were flashed and verified on 2026-11-03. Probe J-Link SN20721552,
device -AT32F413CCU7, SWD1MHz; HEX SHA256
9725f3e5f0b1ba1a7212e295a1f2609045546e4766ab8e9c3d0debe61882b3cb.
The final4096 calibration bytes read before/after programming are identical
(SHA256ffca1184fc52e41178c8b343b581eb58a6cd1dc5a7c6e0e7404a0ee3218c1751),
including valid v3 flags07 generation3. No calibration write command was sent.

TAB5 logs show STOP with raw batch counters frozen364/1067 while valid pose
angles continue changing; user confirmed live orientation. A subsequent probe
reset triggered CDC disconnect/reconnect and recovered valid pose with run=0
and samples=0/0/0, requiring no RUN action. Short-run CRC/gap/drop/PPAerror
counters stayed zero. Full cross-side evidence resides in the TAB5 milestone
with the same date/name. No long-duration or exact delivery-rate claim follows.
