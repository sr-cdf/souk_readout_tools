#!/usr/bin/env python3

import subprocess
import time
import os
import sys
import signal

def start_stream_receiver():
    stream_script = './client_scripts/receive_stream.py'
    stream_options = ['-n', '16','-f', 'tmp_stream','-d', './tmp']
    process = subprocess.Popen(['python3', stream_script] + stream_options, stdout=sys.stdout, stderr=sys.stderr)
    print(f"Started stream process with PID {process.pid}")
    return process

#import the readout client from the directory above
print("Importing readout_client")
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))
import readout_client
client = readout_client.ReadoutClient()

# Start the stream receiver
print("Starting stream receiver process")
process = start_stream_receiver()
time.sleep(1.0)

#enable streaming in the server if it is not already enabled
if client.get_server_status()['stream_enabled']:
    remember_to_disable_stream = False
else:
    print("Enabling streaming")
    client.enable_stream()
    remember_to_disable_stream = True
time.sleep(3)

#setup the measurement - loopback connection required, not open circuit.
print("Setting up the measurement")
client.set_cal_freeze(0)
time.sleep(0.5)
client.set_tone_frequencies([1e9])
time.sleep(0.5)
client.set_tone_amplitudes([1.0])
time.sleep(0.5)
client.set_tone_phases([0.0])
time.sleep(0.5)
#wait a few seconds to settle
time.sleep(10)

#change the tone frequencies and wait a few seconds
print("Changing tone frequencies")
client.set_tone_frequencies([2.0e9])
time.sleep(10.0)

#change the tone frequencies back and wait a few seconds
print("Changing tone frequencies back")
client.set_tone_frequencies([1.0e9])
time.sleep(10.0)

#now repeat with cal freeze on
print('repeat with cal freeze on')
client.set_cal_freeze(1)
time.sleep(10.0)
client.set_tone_frequencies([2.0e9])
time.sleep(10.0)
client.set_tone_frequencies([1.0e9])
time.sleep(10.0)

#set the cal freeze back to 0
print('set cal freeze back to 0')
client.set_cal_freeze(0)
time.sleep(3)

#disable streaming if it was enabled
if remember_to_disable_stream:
    print("Disabling streaming")
    client.disable_stream()
time.sleep(1)

#kill the stream receiver process
print("Killing stream process")
os.kill(process.pid, signal.SIGTERM)
print(f"Killed stream process with PID {process.pid}")

#now load the data
data = client.load_stream('./tmp/tmp_stream')

from matplotlib import pyplot as plt
import numpy as np

plt.ion()

f,(s1,s2,s3) = plt.subplots(3,1,sharex=True)
cnt=data['packet_counter']
cnt-=cnt[0]
t=cnt/client.get_sample_rate()

s1.plot(t,data['i_data']['0000'],'.',label='Channel 0 i')
s2.plot(t[:-1],np.diff(cnt),label='packet counter difference')
s3.plot(t,data['stream_flags']['flag0'],label='server_activity_flag')
s3.plot(t,data['stream_flags']['flag1']+2,label='set_tone_freqs_flag')
s3.plot(t,data['stream_flags']['flag2']+4,label='set_tone_amps_flag')
s3.plot(t,data['stream_flags']['flag3']+6,label='set_tone_phases_flag')
s3.plot(t,data['stream_flags']['flag4']+8,label='cal_freeze_on_flag')

s3.set_xlabel('Time (s)')
s1.set_ylabel('Signal (a.u.)')
s2.set_ylabel('Packet counter difference\n(samples)')
s3.set_ylabel('Flags')
s1.legend(loc='right')
s2.legend(loc='right')
s3.legend(loc='right')
s1.grid(visible=True)
s2.grid(visible=True)
s3.grid(visible=True)
s2.semilogy()
plt.show()


title =  f'Stream data and change tone frequency with cal freeze on and off'
title += f'\nSample rate={client.get_sample_rate()} f0 = 1GHz f1 = 2GHz'
plt.suptitle(title)

