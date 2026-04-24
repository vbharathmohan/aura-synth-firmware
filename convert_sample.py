"""
convert_sample.py — Convert audio files to Aura-Synth ESP32 format.

Output format: 16-bit signed PCM, mono, 22050 Hz, standard 44-byte WAV header.

Usage:
    python convert_sample.py input.wav output.wav
    python convert_sample.py input.wav output.wav --trim 0.5
    python convert_sample.py input.wav output.wav --trim 0.3 --rate 22050

    # Batch convert all files in a folder:
    python convert_sample.py --batch input_folder/ output_folder/
    python convert_sample.py --batch input_folder/ output_folder/ --trim 0.5

Requirements: Python 3.x (no external packages needed)
"""

import wave
import struct
import sys
import os
import argparse


def read_wav(path):
    """Read a WAV file and return (samples_as_list_of_ints, sample_rate, num_channels, sample_width)."""
    with wave.open(path, "rb") as wf:
        n_channels = wf.getnchannels()
        samp_width = wf.getsampwidth()  # bytes per sample (1=8bit, 2=16bit, 3=24bit)
        samp_rate = wf.getframerate()
        n_frames = wf.getnframes()
        raw = wf.readframes(n_frames)

    # Decode raw bytes into integer samples
    samples = []
    if samp_width == 1:
        # 8-bit unsigned
        for b in raw:
            samples.append(b - 128)  # convert to signed
    elif samp_width == 2:
        # 16-bit signed little-endian
        for i in range(0, len(raw), 2):
            val = struct.unpack_from("<h", raw, i)[0]
            samples.append(val)
    elif samp_width == 3:
        # 24-bit signed little-endian
        for i in range(0, len(raw), 3):
            b0, b1, b2 = raw[i], raw[i + 1], raw[i + 2]
            val = b0 | (b1 << 8) | (b2 << 16)
            if val >= 0x800000:
                val -= 0x1000000
            samples.append(val)
    elif samp_width == 4:
        # 32-bit signed
        for i in range(0, len(raw), 4):
            val = struct.unpack_from("<i", raw, i)[0]
            samples.append(val)
    else:
        raise ValueError(f"Unsupported sample width: {samp_width} bytes")

    return samples, samp_rate, n_channels, samp_width


def to_mono(samples, n_channels):
    """Mix down to mono by averaging channels."""
    if n_channels == 1:
        return samples
    mono = []
    for i in range(0, len(samples), n_channels):
        avg = sum(samples[i:i + n_channels]) // n_channels
        mono.append(avg)
    return mono


def normalize_to_16bit(samples, original_width):
    """Scale samples to 16-bit range (-32768 to 32767)."""
    if original_width == 2:
        return samples  # already 16-bit
    elif original_width == 1:
        # 8-bit signed (-128..127) → 16-bit
        return [s * 256 for s in samples]
    elif original_width == 3:
        # 24-bit → 16-bit (shift right by 8)
        return [s >> 8 for s in samples]
    elif original_width == 4:
        # 32-bit → 16-bit (shift right by 16)
        return [s >> 16 for s in samples]
    else:
        # Fallback: normalize to max range
        if not samples:
            return samples
        peak = max(abs(s) for s in samples) or 1
        return [int(s / peak * 32767) for s in samples]


def resample(samples, src_rate, dst_rate):
    """Simple linear interpolation resampling."""
    if src_rate == dst_rate:
        return samples
    ratio = src_rate / dst_rate
    out_len = int(len(samples) / ratio)
    result = []
    for i in range(out_len):
        src_pos = i * ratio
        idx = int(src_pos)
        frac = src_pos - idx
        s0 = samples[idx] if idx < len(samples) else 0
        s1 = samples[idx + 1] if idx + 1 < len(samples) else s0
        val = int(s0 + frac * (s1 - s0))
        result.append(val)
    return result


def trim_samples(samples, sample_rate, max_seconds):
    """Trim to a maximum duration."""
    max_samples = int(sample_rate * max_seconds)
    if len(samples) > max_samples:
        return samples[:max_samples]
    return samples


def clamp_16bit(samples):
    """Clamp all values to valid 16-bit signed range."""
    return [max(-32768, min(32767, s)) for s in samples]


def write_wav(path, samples, sample_rate):
    """Write samples as a standard 16-bit mono WAV file."""
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        raw = struct.pack(f"<{len(samples)}h", *samples)
        wf.writeframes(raw)


def convert_file(input_path, output_path, target_rate=22050, max_seconds=None):
    """Full conversion pipeline: read → mono → normalize → resample → trim → write."""
    print(f"Reading: {input_path}")
    samples, src_rate, n_channels, samp_width = read_wav(input_path)

    src_duration = len(samples) / n_channels / src_rate
    print(f"  Source: {src_rate} Hz, {samp_width * 8}-bit, {n_channels}ch, "
          f"{len(samples) // n_channels} samples, {src_duration:.2f}s")

    # Step 1: Mix to mono
    samples = to_mono(samples, n_channels)

    # Step 2: Normalize to 16-bit
    samples = normalize_to_16bit(samples, samp_width)

    # Step 3: Resample
    if src_rate != target_rate:
        print(f"  Resampling: {src_rate} → {target_rate} Hz")
        samples = resample(samples, src_rate, target_rate)

    # Step 4: Trim if requested
    if max_seconds is not None:
        before = len(samples)
        samples = trim_samples(samples, target_rate, max_seconds)
        if len(samples) < before:
            print(f"  Trimmed: {before} → {len(samples)} samples ({max_seconds}s)")

    # Step 5: Clamp and write
    samples = clamp_16bit(samples)
    write_wav(output_path, samples, target_rate)

    out_duration = len(samples) / target_rate
    file_size = os.path.getsize(output_path)
    print(f"  Output: {target_rate} Hz, 16-bit, mono, "
          f"{len(samples)} samples, {out_duration:.2f}s, {file_size // 1024} KB")
    print(f"  Saved: {output_path}")
    return True


def batch_convert(input_dir, output_dir, target_rate=22050, max_seconds=None):
    """Convert all WAV files in a directory."""
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    wav_files = [f for f in os.listdir(input_dir)
                 if f.lower().endswith(".wav")]

    if not wav_files:
        print(f"No .wav files found in {input_dir}")
        return

    print(f"Found {len(wav_files)} WAV files in {input_dir}\n")

    success = 0
    for fname in sorted(wav_files):
        input_path = os.path.join(input_dir, fname)
        # Clean up filename: lowercase, replace spaces with underscores
        clean_name = fname.lower().replace(" ", "_")
        output_path = os.path.join(output_dir, clean_name)
        try:
            convert_file(input_path, output_path, target_rate, max_seconds)
            success += 1
        except Exception as e:
            print(f"  ERROR: {e}")
        print()

    print(f"Converted {success}/{len(wav_files)} files")


def main():
    parser = argparse.ArgumentParser(
        description="Convert audio files to Aura-Synth ESP32 format (16-bit mono WAV)")
    parser.add_argument("input", help="Input WAV file or folder (with --batch)")
    parser.add_argument("output", help="Output WAV file or folder (with --batch)")
    parser.add_argument("--rate", type=int, default=22050,
                        help="Target sample rate in Hz (default: 22050)")
    parser.add_argument("--trim", type=float, default=None,
                        help="Max duration in seconds (default: no trim)")
    parser.add_argument("--batch", action="store_true",
                        help="Convert all WAV files in input folder")

    args = parser.parse_args()

    if args.batch:
        batch_convert(args.input, args.output, args.rate, args.trim)
    else:
        try:
            convert_file(args.input, args.output, args.rate, args.trim)
        except Exception as e:
            print(f"Error: {e}")
            sys.exit(1)


if __name__ == "__main__":
    main()
