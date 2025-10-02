import heartpy as hp
import matplotlib.pyplot as plt

sample_rate = 100.0  # in Hz
upperlimit = 200.0  # upper limit for heart rate in BPM
hrv_limit = 20.0  # limit for HRV in BPM

def sign(x):
    x = float(x)
    return (x > 0) - (x < 0)

def get_hr(peaklist, sample_rate, upperlimit=upperlimit, printout=True):
    bpm = []

    rr_dist = [peaklist[idx] - peaklist[idx-1] for idx in range(1, len(peaklist))] # in samples
    rr_time = [rd / sample_rate for rd in rr_dist]  # in seconds
    for i, rd in enumerate(rr_time):
        bpm.append(1.0 / rd)
        if len(bpm) > 1: # no difference check for first element
            bpm_diff = bpm[-1] - bpm[-2] 
            if abs(bpm_diff) > hrv_limit: # limit exceeded (indicates unrealistic spike)
                print(f"bpm_diff: {bpm_diff}")
                bpm.pop()
                bpm.append(bpm[-1] + sign(bpm_diff) * hrv_limit) # limit the change to the limit value
        if bpm[-1] > upperlimit:
            bpm.pop()
            if len(bpm) == 0:
                print(f"bpm is empty, appending 0.0")
                bpm.append(0.0)
            else: bpm.append(bpm[-1]) # ignore the change and repeat the last value

    if printout:
        print(f"bpm: {bpm}")
    return bpm

def plot_bpm(peak_times, bpm):
    ax_bpm = plt.gca().twinx()
    ax_bpm.plot(
        peak_times[1:len(peak_times)],  # x-axis
        bpm,  # y-axis
        marker='o',
        label='Instantaneous HR')
    ax_bpm.set_xlabel('Time (s)')
    ax_bpm.set_ylabel('Instantaneous Heart Rate (BPM)')
    ax_bpm.set_ylim(0, upperlimit)
    return

# # example 1
# data, timer = hp.load_exampledata(0)
# wd, m = hp.process(data, sample_rate = sample_rate) # wd: working data, m: measures
# my_bpm = get_hr(wd['peaklist'], sample_rate)

# # convert peaklist to t[s]
# wd['peaklist_t'] = [idx / sample_rate for idx in wd['peaklist']]

# # hr_inst = [wd['hr'][i] for i in wd['peaklist']]
# # print(f"hr_inst: {hr_inst}")

# plt.figure(figsize=(12,4))
# hp.plotter(wd, m)
# plt.plot(
#     wd['peaklist_t'][0:len(wd['peaklist_t'])-1], # x-axis 
#     my_bpm, # y-axis 
#     marker='o',
#     label='Instantaneous HR')
# plt.xlabel('Sample Index')
# plt.ylabel('Instantaneous Heart Rate (BPM)')
# plt.title('PPG')
# plt.legend()
# plt.show()

# for measure in m.keys():
#     print('%s: %f' %(measure,m[measure]))

# # example 2
# data, timer =hp.load_exampledata(1)

# # plt.figure(figsize=(12,4))
# # plt.plot(data)
# # plt.show()

# # sample_rate = hp.get_samplerate_mstimer(timer)
# # wd, m = hp.process(data, sample_rate)
# # wd['peaklist_t'] = [idx / sample_rate for idx in wd['peaklist']]
# # bpm = get_hr(wd['peaklist_t'], sample_rate)

# # plt.figure(figsize=(12,4))
# # hp.plotter(wd, m)
# # plt.plot(
# #     wd['peaklist_t'][0:len(wd['peaklist_t'])-1],  # x-axis
# #     bpm,  # y-axis
# #     marker='o',
# #     label='Instantaneous HR')

# # for measure in m.keys():
# #     print(f"{measure}: {m[measure]})")

# plt.show()

# # example 3
# data, timer = hp.load_exampledata(2)
# print(timer[0])

# sample_rate = hp.get_samplerate_datetime(timer, timeformat = '%Y-%m-%d %H:%M:%S.%f')
# print('sample rate is: %f Hz' %sample_rate)

# wd, m = hp.process(data, sample_rate, report_time = True)
# wd['peaklist_t'] = [idx / sample_rate for idx in wd['peaklist']]
# bpm = get_hr(wd['peaklist_t'], sample_rate, False)

# # plt.figure(figsize=(12,4))
# # hp.plotter(wd,m)

# plt.figure(figsize=(12,4))
# plt.xlim(20000,30000)

# hp.plotter(wd,m)

# ax_bpm = plt.gca().twinx()
# ax_bpm.plot(
#     wd['peaklist_t'][0:len(wd['peaklist_t'])-1],  # x-axis
#     bpm,  # y-axis
#     marker='o',
#     label='Instantaneous HR')
# ax_bpm.set_ylabel('Instantaneous Heart Rate (BPM)')
# ax_bpm.set_ylim(0, 150)
# plt.title('PPG')
# plt.show()

# for measure in m.keys():
#     print(f"{measure}: {m[measure]})")

# my sample
file_name = 'ppg.csv'
ppg_data = hp.get_data(file_name, column_name = 'ppg')
print(ppg_data[0:10])  # print first 10 data points
mstimer_data = hp.get_data(file_name, column_name = 'timestamp')
print(mstimer_data[0:10])  # print first 10 data points
sample_rate = hp.get_samplerate_mstimer(mstimer_data)
print('sample rate is: %f Hz' %sample_rate)

wd, m = hp.process(ppg_data, sample_rate)
wd['peaklist_t'] = [idx / sample_rate for idx in wd['peaklist']]
bpm = get_hr(wd['peaklist_t'], sample_rate)
hp.plotter(wd, m)
plot_bpm(wd['peaklist_t'], bpm)

plt.show()