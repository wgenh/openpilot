import os
import time
import numpy as np
import wave
#import scipy


def read_wav(filename):
  # scipy can read wavs too
  with wave.open(filename, "rb") as wav_file:
    nchannels = wav_file.getnchannels()
    sample_rate = wav_file.getframerate()
    nframes = wav_file.getnframes()

    audio_data = wav_file.readframes(nframes)
    audio_data = np.frombuffer(audio_data, dtype=np.int16)

    # if stereo, return two arrays
    if nchannels == 2:
      audio_data = audio_data.reshape((-1, 2))
      return sample_rate, audio_data

  return sample_rate, audio_data


#def get_spectrogram(signal):
#  import scipy
#  import scipy.signal
#  return scipy.signal.spectrogram(signal, fs=44100, window=('hamming'), nperseg=5000, scaling='spectrum')

def get_spectrogram(signal, fs=44100, window_size=5000):
  overlap    = window_size//8
  window = np.hanning(window_size)
  windows = [signal[i:i+window_size] * window for i in range(0, len(signal)-window_size, window_size-overlap)]

  spectrogram = [np.abs(np.fft.rfft(win))**2 for win in windows]
  spectrogram = np.array(spectrogram).T

  freq = np.fft.rfftfreq(window_size, d=1.0/fs)
  time = np.arange(len(spectrogram[0])) * (window_size - overlap) / fs
  return freq, time, spectrogram

def get_freqs(signal):
  source_t, source_freq = [], []
  Y, X, Z = get_spectrogram(signal)
  for i in range(len(X)):
    idx = np.argmax(Z[:,i])
    source_freq.append(Y[idx])
    source_t.append(X[i])
  return source_t, source_freq


def correlate_fft(in2, in1):
  # Numpy correlate works too, but is too slow for large signals
  # Only works for same length signals for now
  assert len(in1) == len(in2)
  n = len(in1) + len(in2) - 1
  in1_padded = np.concatenate([in1, np.zeros(n-len(in1))])
  in2_padded = np.concatenate([in2, np.zeros(n-len(in2))])
  in1_fft = np.fft.fft(in1_padded, n)
  in2_fft = np.fft.fft(in2_padded, n)
  result_fft = np.conj(in1_fft) * in2_fft
  corr = np.fft.fftshift(np.fft.ifft(result_fft))
  return corr


def get_THD(reference, signal, harmonic_orders=[2,4,6]):
  ref_t, ref_freq = get_freqs(reference)
  Y, X, Z = get_spectrogram(signal)

  freqs = []
  THD = []
  amplitudes = []
  hz = Y
  for i, t in enumerate(X):
    freq = np.interp(t, ref_t, ref_freq)
    if freq < 10:
      continue
    correct_band = slice(np.argmin(abs(hz - (1 - 0.25)*freq)), np.argmin(abs(hz - (1 + .25) * freq)))
    harmonics = []
    for j in harmonic_orders:
      harmonics.append(slice(np.argmin(abs(hz - (j - 0.25)*freq)), np.argmin(abs(hz - (j + .25)*freq))))
    correct_amp = np.sum(Z[correct_band,i])
    harmonic_amp = np.sum([np.sum(Z[slicee, i]) for slicee in harmonics])
    THD.append(harmonic_amp / (correct_amp + harmonic_amp))
    amplitudes.append(correct_amp + harmonic_amp)
    freqs.append(freq)
  return np.array(freqs), np.array(THD), np.array(amplitudes)


def align_signal(reference, signal):
  signal = signal[:,0]
  idx = np.argmax(correlate_fft(signal.astype(np.float64)[:len(reference)],
                                    reference.astype(np.float64))) - len(reference) + 1
  #idx = np.argmax(scipy.signal.correlate(signal.astype(np.float64),
  #                                  reference.astype(np.float64))) - len(reference) + 1
  signal = signal[idx:]
  return signal


def test_sound(test_file, reference_file='/home/batman/xx/projects/sound_test/freq_sweep.wav', device='tizi'):
  samplerate, reference = read_wav(reference_file)
  samplerate, signal = read_wav(test_file)
  signal = align_signal(reference, signal)
  THD_hz, THD, amplitudes = get_THD(reference, signal, harmonic_orders=[2,3,4,5,6])
  if device == 'tizi':
    ranges = [[300,1000], [1000, 2800]]
    thresholds = [0.5, 0.2]
  elif device == 'tici':
    ranges = [[300,1000], [1000, 2800]]
    thresholds = [0.8, 0.2]
  else:
    raise NotImplementedError(f'Device {device} is not supported')
  for t, r in zip(thresholds, ranges):
    good_hz = (THD_hz > r[0]) & (THD_hz < r[1])
    print(f'Frequency range {r[0]}-{r[1]} Hz, max THD: {max(THD[good_hz]):.2f}')
    assert max(THD[good_hz]) < t
  return THD_hz, THD, amplitudes


def run_sound_test():
  print('RUNNING TOTAL HARMONIC DISTORTION TEST')
  filename = '/tmp/freq_sweep_record.wav'
  ref_file = '/data/openpilot/selfdrive/ui/tests/freq_sweep.wav'
  os.system(f'arecord --format=cd --duration=12 {filename} &')
  time.sleep(1)
  os.system(f'/data/openpilot/selfdrive/ui/tests/playsound {ref_file}')
  time.sleep(2)
  test_sound(filename, ref_file)

if __name__ == "__main__":
  run_sound_test()
