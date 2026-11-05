# Waveshare ESP32-S3 2.8" display

Parts for a Waveshare ESP32-S3-Touch-LCD-2.8 in the controller case, in
place of the LCDWiki lid in [`../controller-case`](../controller-case).

- `CFBF LCD Lid Waveshare Flush v3.stl`: the controller-case lid that holds
  the display flush. The glass sits in a pocket on the outside face. It is
  the same size as `CFBF LCD Lid.stl` (118 x 90 x 21 mm).
- `Waveshare Rear Bar x2.stl`: print 2. The file is the pair of rear bars
  that clamp the board. Each bar screws into the board standoffs and into
  the tall bosses on the inside of the lid.
- `Waveshare Wiring - Case Feeder Pro.pdf`: wiring diagram from the SKR Pico
  Raspberry Pi UART header to the Waveshare board.
- `make_lid_flush.py`: helper that `make_lid_bracket.py` reads from this
  directory. It holds the Waveshare glass, pocket, and boss dimensions.
- `make_lid_bracket.py`: generator for the flush lid and the rear bars. It
  reads `make_lid_flush.py` from this directory. The holes are sized for
  M2.5 screws, and the script can be switched to M2.
- `waveshare-lid-v3-preview.png`: preview of the lid. One view is the
  outside front with the glass pocket; the other is the inside, with the
  opening and the bosses.
