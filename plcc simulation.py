"""
Substation Monitoring using Power Line Carrier Communication (PLCC) - simulation
Author: Rachana Santhebachahalli Umesha

Pipeline:
  substation sensors -> packet -> FSK modulator -> power line channel
  (50 Hz mains + noise) -> band-pass filter -> FSK demodulator -> control room alarms
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt

rng = np.random.default_rng(42)

# ---------------- System parameters ----------------
FS = 400_000          # sample rate (Hz)
F0, F1 = 90_000, 110_000   # FSK tones for bit 0 / bit 1 (CENELEC-style carrier band)
BIT_RATE = 2_000      # bits per second
SPB = FS // BIT_RATE  # samples per bit
MAINS_F, MAINS_V = 50, 230 * np.sqrt(2)   # mains interference (peak volts)
CARRIER_AMP = 1.0     # carrier amplitude at the receiver (volts)

# ---------------- Substation sensors ----------------
LIMITS = {
    "transformer_temp_C": (None, 95.0),
    "line_voltage_kV":    (10.0, 12.0),
    "line_current_A":     (None, 400.0),
    "battery_voltage_V":  (105.0, None),   # 110 V DC station battery
}
SCALE = {"transformer_temp_C": 10, "line_voltage_kV": 100,
         "line_current_A": 1, "battery_voltage_V": 10}


def read_sensors(fault=False):
    r = {
        "transformer_temp_C": rng.normal(65, 3),
        "line_voltage_kV":    rng.normal(11.0, 0.1),
        "line_current_A":     rng.normal(250, 15),
        "battery_voltage_V":  rng.normal(110, 0.5),
    }
    if fault:
        r["transformer_temp_C"] = 102.4
        r["battery_voltage_V"] = 101.8
    return r


def encode(readings):
    """Each reading -> 16-bit integer; 8-bit preamble + 64 data bits + 8-bit checksum."""
    words = [int(round(readings[k] * SCALE[k])) & 0xFFFF for k in LIMITS]
    data = np.array([(w >> (15 - i)) & 1 for w in words for i in range(16)], dtype=np.uint8)
    checksum = sum(words) & 0xFF
    chk = np.array([(checksum >> (7 - i)) & 1 for i in range(8)], dtype=np.uint8)
    preamble = np.array([1, 0, 1, 0, 1, 0, 1, 1], dtype=np.uint8)
    return np.concatenate([preamble, data, chk])


def decode(bits):
    data, chk = bits[8:72], bits[72:80]
    words = [int("".join(map(str, data[i * 16:(i + 1) * 16])), 2) for i in range(4)]
    ok = (sum(words) & 0xFF) == int("".join(map(str, chk)), 2)
    return {k: w / SCALE[k] for k, w in zip(LIMITS, words)}, ok


# ---------------- Modulation / channel / demodulation ----------------
def fsk_modulate(bits):
    t = np.arange(len(bits) * SPB) / FS
    f = np.repeat(np.where(bits == 1, F1, F0), SPB)
    phase = 2 * np.pi * np.cumsum(f) / FS            # continuous-phase FSK
    return t, CARRIER_AMP * np.sin(phase)


def power_line_channel(t, signal, snr_db):
    mains = MAINS_V * np.sin(2 * np.pi * MAINS_F * t)
    noise_power = (CARRIER_AMP ** 2 / 2) / (10 ** (snr_db / 10))
    noise = rng.normal(0, np.sqrt(noise_power), len(t))
    return signal + mains + noise


def bandpass(x, lo, hi, order=4):
    sos = butter(order, [lo, hi], btype="band", fs=FS, output="sos")
    return sosfiltfilt(sos, x)


def fsk_demodulate(rx):
    x = bandpass(rx, 80_000, 120_000)                # removes 50 Hz mains
    e0 = np.abs(bandpass(x, F0 - 5_000, F0 + 5_000)) # energy detector per tone
    e1 = np.abs(bandpass(x, F1 - 5_000, F1 + 5_000))
    n = len(rx) // SPB
    e0 = e0[:n * SPB].reshape(n, SPB).sum(axis=1)
    e1 = e1[:n * SPB].reshape(n, SPB).sum(axis=1)
    return (e1 > e0).astype(np.uint8), x


def check_alarms(r):
    alarms = []
    for k, (lo, hi) in LIMITS.items():
        if lo is not None and r[k] < lo:
            alarms.append(f"LOW {k} = {r[k]:.1f}")
        if hi is not None and r[k] > hi:
            alarms.append(f"HIGH {k} = {r[k]:.1f}")
    return alarms


# ---------------- Experiments ----------------
def run_link(readings, snr_db):
    bits = encode(readings)
    t, tx = fsk_modulate(bits)
    rx = power_line_channel(t, tx, snr_db)
    rx_bits, filtered = fsk_demodulate(rx)
    return bits, rx_bits, t, tx, rx, filtered


def main():
    print("=== Normal operation (SNR 10 dB) ===")
    r = read_sensors()
    bits, rx_bits, t, tx, rx, filt = run_link(r, 10)
    decoded, ok = decode(rx_bits)
    print("Sent:    ", {k: round(v, 1) for k, v in r.items()})
    print("Received:", decoded, "| checksum OK:", ok)
    print("Alarms:  ", check_alarms(decoded) or "none")

    print("\n=== Fault condition (SNR 10 dB) ===")
    rf = read_sensors(fault=True)
    _, rxf, *_ = run_link(rf, 10)
    df, okf = decode(rxf)
    print("Received:", df, "| checksum OK:", okf)
    print("Alarms:  ", check_alarms(df))

    # Bit error rate vs SNR
    # SNR is measured over the full 0-200 kHz line bandwidth, so the receiver's
    # band-pass filters give a large processing gain: the link still works below 0 dB.
    snrs = np.arange(-24, -7, 2)
    bers = []
    for s in snrs:
        errs = total = 0
        for _ in range(60):
            b, rb, *_ = run_link(read_sensors(), s)
            errs += np.sum(b != rb)
            total += len(b)
        bers.append(errs / total)
    print("\nSNR (dB, full line bandwidth) vs BER:")
    for s, b in zip(snrs, bers):
        print(f"  {s:>4}  {b:.5f}" + ("  (no errors in 4,800 bits)" if b == 0 else ""))

    # Figures
    fig, ax = plt.subplots(3, 1, figsize=(9, 8))
    w = slice(SPB - 40, SPB + 40)   # 0.2 ms window around the 1 -> 0 change between bits 1 and 2
    ax[0].plot(t[w] * 1e3, tx[w]); ax[0].axvline(t[SPB] * 1e3, ls="--", c="grey")
    ax[0].set_title("Transmitted FSK carrier: 110 kHz (bit 1) then 90 kHz (bit 0)")
    ax[1].plot(t * 1e3, rx)
    ax[1].set_title("Received on power line: ~1 V carrier riding on 325 V peak, 50 Hz mains")
    ax[2].plot(t[w] * 1e3, filt[w]); ax[2].axvline(t[SPB] * 1e3, ls="--", c="grey")
    ax[2].set_title("After 80-120 kHz band-pass filter: mains removed, carrier recovered")
    for a in ax:
        a.set_xlabel("Time (ms)"); a.set_ylabel("Volts")
    fig.tight_layout(); fig.savefig("waveforms.png", dpi=150)

    plt.figure(figsize=(7, 4.5))
    nz = np.array(bers) > 0
    plt.semilogy(snrs[nz], np.array(bers)[nz], "o-")
    plt.xlabel("SNR over full line bandwidth (dB)"); plt.ylabel("Bit error rate")
    plt.grid(True, which="both")
    plt.title("PLCC link: bit error rate vs SNR")
    plt.tight_layout(); plt.savefig("ber_vs_snr.png", dpi=150)
    print("\nSaved waveforms.png and ber_vs_snr.png")


if __name__ == "__main__":
    main()
