# Packet framing for the Dwarka SkyGate fused telemetry link (Pico -> GCS over
# E32-900T20D, transparent mode). NMEA-style ASCII frame so it survives a
# plain UART->LoRa->UART hop with no binary alignment issues:
#
#   $DSKY,<pkt_id>,<t_s>,<alt_m>,<press_pa>,<temp_c>,<ax_g>,<ay_g>,<az_g>,<gx_dps>,<gy_dps>,<gz_dps>*<XOR_CHECKSUM_HEX2>\r\n
#
# Checksum is an 8-bit XOR over every byte between '$' and '*' (same scheme
# NMEA/GPS receivers use), so both ends can validate a frame without a CRC
# table. pkt_id wraps at 16 bits and is used ground-side to detect drops.

def build_packet(pkt_id, t_s, alt_m, press_pa, temp_c, accel_g, gyro_dps):
    ax, ay, az = accel_g
    gx, gy, gz = gyro_dps
    body = "DSKY,{:d},{:.1f},{:.2f},{:.1f},{:.1f},{:.3f},{:.3f},{:.3f},{:.1f},{:.1f},{:.1f}".format(
        pkt_id, t_s, alt_m, press_pa, temp_c, ax, ay, az, gx, gy, gz
    )
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return "${}*{:02X}\r\n".format(body, checksum)
