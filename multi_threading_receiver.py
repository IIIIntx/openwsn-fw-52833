import serial
import numpy as np
import math
import struct
import matplotlib.pyplot as plt
import binascii
import threading

import matplotlib.pyplot as plt
from math import pi, atan2, sqrt
from scipy.linalg import eig
import time
from sklearn.linear_model import LinearRegression
from collections import deque

ser1 = serial.Serial('COM7', 115200)
ser2 = serial.Serial('COM11', 115200)
# ser3 = serial.Serial('COM25', 115200)

SPEED_OF_LIGHT  = 299792458
num_iterations = 50     # 进行的循环次数
iteration = 0


music_list = []
grid_search_list = []
esprit_list = []
    
num_samples = 88

# 固定物理参数（可根据实际硬件修改）
SIGNAL_FREQUENCY_HZ = 2.402e9  # 例：BLE 2.402 GHz
SPACING_M_12 = 0.0625          # 例：1-2 接收基线间距（米）
SPACING_M_13 = 0.125          # 例：1-3 接收基线间距（米）


ser1_data = {
    'interval': [-1],
    'tx1_sqn': [-1],
    'tx2_sqn': [-1],
    'packet_1_I_data': [-1],
    'packet_1_Q_data': [-1],
    'packet_2_I_data': [-1],
    'packet_2_Q_data': [-1]
}

ser2_data = {
    'interval': [-1],
    'tx1_sqn': [-1],
    'tx2_sqn': [-1],
    'packet_1_I_data': [-1],
    'packet_1_Q_data': [-1],
    'packet_2_I_data': [-1],
    'packet_2_Q_data': [-1]
}

def compute_phase_difference(frequency_hz, spacing_m, angle_deg):
    """
    计算两相邻天线之间的理论相位差（单位：弧度，范围 [-pi, pi]）。

    参数:
        frequency_hz: 信号频率 (Hz)
        spacing_m: 相邻天线间距 (m)
        angle_deg: 到达角 (度)，相对于天线阵列法线，正向定义为入射方向的夹角

    公式:
        Δφ = 2π · d · sin(θ) / λ, 其中 λ = c / f
    """
    wavelength = SPEED_OF_LIGHT / float(frequency_hz)
    delta = 2 * np.pi * float(spacing_m) * np.sin(np.deg2rad(float(angle_deg))) / wavelength
    # 规约到 [-pi, pi]
    return np.arctan2(np.sin(delta), np.cos(delta))

def compute_phase_differences_for_array(frequency_hz, spacing_m, angle_deg, num_antennas):
    """
    计算线性均匀阵列中相邻天线对的理论相位差数组，长度为 num_antennas-1。
    所有相邻对的相位差在均匀线阵模型下相同。
    """
    if num_antennas < 2:
        return np.array([], dtype=np.float32)
    delta = compute_phase_difference(frequency_hz, spacing_m, angle_deg)
    return np.full(num_antennas - 1, delta, dtype=np.float32)
def thread(ser, id):
    rawFrame = []
    num_samples = 88
    while True:
        byte  = ser.read(1)        
        rawFrame += byte
        # print(rawFrame)
        if rawFrame[-6:]==[255, 255, 255, 255, 255, 255]:
            if len(rawFrame) == 4*num_samples*2+12:
                received_data_1 = rawFrame[:4*num_samples]
                received_data_2 = rawFrame[4*num_samples:4*num_samples*2]
                num_samples = 88
                
                interval = struct.unpack('>HH', bytes(rawFrame[4*num_samples*2:-8]))[1]
                tx1_sqn = rawFrame[-8]
                tx2_sqn = rawFrame[-7]

                packet_1_I_data = np.zeros(num_samples, dtype=np.int16)
                packet_1_Q_data = np.zeros(num_samples, dtype=np.int16)
                packet_2_I_data = np.zeros(num_samples, dtype=np.int16)
                packet_2_Q_data = np.zeros(num_samples, dtype=np.int16)

                for i in range(num_samples):
                    (packet_1_I) = struct.unpack('>h', bytes(received_data_1[4*i+2:4*i+4]))
                    (packet_1_Q) = struct.unpack('>h', bytes(received_data_1[4*i:4*i+2]))

                    (packet_2_I) = struct.unpack('>h', bytes(received_data_2[4*i+2:4*i+4]))
                    (packet_2_Q) = struct.unpack('>h', bytes(received_data_2[4*i:4*i+2]))
                    #print(phase)
                    #print((received_data[4*i+2] << 8) | received_data[4*i+3])
                    #phase_data[i] = (received_data[4*i+2] << 8) | received_data[4*i+3]
                    packet_1_I_data[i] = packet_1_I[0]
                    packet_1_Q_data[i] = packet_1_Q[0]

                    packet_2_I_data[i] = packet_2_I[0]
                    packet_2_Q_data[i] = packet_2_Q[0]

                packet_1_I_data = packet_1_I_data.astype(np.float32)
                packet_1_Q_data = packet_1_Q_data.astype(np.float32)

                packet_2_I_data = packet_2_I_data.astype(np.float32)
                packet_2_Q_data = packet_2_Q_data.astype(np.float32)

                # print('phase_{}_1:'.format(id), np.arctan2(packet_1_I_data[1], packet_1_Q_data[1]))

                #all_data['I_data'] = I_data
                #all_data['Q_data'] = Q_data
                if id == 1:
                    ser1_data['interval'][0] = interval
                    ser1_data['tx1_sqn'][0] = tx1_sqn
                    ser1_data['tx2_sqn'][0] = tx2_sqn
                    ser1_data['packet_1_I_data'][0] = packet_1_I_data
                    ser1_data['packet_1_Q_data'][0] = packet_1_Q_data
                    ser1_data['packet_2_I_data'][0] = packet_2_I_data
                    ser1_data['packet_2_Q_data'][0] = packet_2_Q_data
                    break
                elif id == 2:
                    ser2_data['interval'][0] = interval
                    ser2_data['tx1_sqn'][0] = tx1_sqn
                    ser2_data['tx2_sqn'][0] = tx2_sqn
                    ser2_data['packet_1_I_data'][0] = packet_1_I_data
                    ser2_data['packet_1_Q_data'][0] = packet_1_Q_data
                    ser2_data['packet_2_I_data'][0] = packet_2_I_data
                    ser2_data['packet_2_Q_data'][0] = packet_2_Q_data
                    break

                #np.savez('data_{}.npz'.format(id), **all_data)
            else:
                # 帧尾已到，但长度与预期不符，打印调试信息
                print(f"线程{id}: 帧长度不符 len={len(rawFrame)} 预期={4*num_samples*2+9}")
            rawFrame = []
    #np.savez('100_data_{}.npz'.format(id), **all_data)
    # np.savez('10_data_10_degree{}.npz'.format(id), **all_data)
        



def cal_slope(phase_diff):
    x = np.arange(len(phase_diff)).reshape(-1, 1)
    y = phase_diff
    model = LinearRegression()
    model.fit(x, y)
    return model.coef_[0], model.intercept_


def data_process(ser1_data, ser2_data, first_tx_angle_deg=None, use_music=True, music_grid_step=0.5):
    # 提取三个接收端的两包 IQ 数据
    rx1_packet1_I = ser1_data['packet_1_I_data'][0]
    rx1_packet1_Q = ser1_data['packet_1_Q_data'][0]
    rx1_packet2_I = ser1_data['packet_2_I_data'][0]
    rx1_packet2_Q = ser1_data['packet_2_Q_data'][0]
    interval1 = ser1_data['interval'][0]

    rx2_packet1_I = ser2_data['packet_1_I_data'][0]
    rx2_packet1_Q = ser2_data['packet_1_Q_data'][0]
    rx2_packet2_I = ser2_data['packet_2_I_data'][0]
    rx2_packet2_Q = ser2_data['packet_2_Q_data'][0]
    interval2 = ser2_data['interval'][0]

    # 取三路 interval 的平均作为两包间隔估计
    interval = (interval1 + interval2) / 2

    # 计算每路相位
    rx1_pkt1_phase = np.unwrap(np.arctan2(rx1_packet1_Q, rx1_packet1_I))
    rx1_pkt2_phase = np.unwrap(np.arctan2(rx1_packet2_Q, rx1_packet2_I))
    rx2_pkt1_phase = np.unwrap(np.arctan2(rx2_packet1_Q, rx2_packet1_I))
    rx2_pkt2_phase = np.unwrap(np.arctan2(rx2_packet2_Q, rx2_packet2_I))

    # 利用 pkt1 估计本地相位差与漂移（两两组合）
    # 1-2
    pkt1_diff_12 = np.unwrap(rx1_pkt1_phase - rx2_pkt1_phase)
    slope12, intercept12 = cal_slope(pkt1_diff_12)
    # 1-3

    # pkt2 的观测相位差
    pkt2_diff_12 = np.unwrap(rx1_pkt2_phase - rx2_pkt2_phase)


    # 若提供了第一发送方角度与阵列几何，用其理论几何相位差对本地相位进行标定
    geo12_expected = 0.0
    geo13_expected = 0.0
    if first_tx_angle_deg is not None:
        geo12_expected = compute_phase_difference(SIGNAL_FREQUENCY_HZ, SPACING_M_12, first_tx_angle_deg)

    # 按两包时间间隔对本地相位差进行外推补偿
    # 基于 pkt1 的拟合结果减去理论几何项，得到本地相位基准，再外推到 pkt2 时刻
    drift12_base = intercept12 - geo12_expected

    drift12 = drift12_base + interval/16 * slope12

    # 去除本地相位差与漂移，得到几何相位差估计
    geo12 = pkt2_diff_12 - drift12
    # 归一化到 [-pi, pi]
    geo12 = np.arctan2(np.sin(geo12), np.cos(geo12))

    # 由几何相位差估计角度（沿用原先常量与公式）
    def phase_to_angle(geo):
        val = geo/(2*np.pi)*12.5/6
        if val > 1:
            return 90
        if val < -1:
            return -90
        return np.arcsin(val)/np.pi*180

    angle12 = phase_to_angle(np.mean(geo12))

    fallback_angle = angle12

    return fallback_angle

def start_monitoring(ser1, ser2, first_tx_angle_deg=None):
    global ser1_data, ser2_data
    print("串口监控已启动...")
    
    angle_list = []
    # 存储所有触发次数的原始数据
    all_data_list = []
    # 为解决多串口异步到达，维护滑动窗口缓冲
    ser1_buffer = deque(maxlen=20)
    ser2_buffer = deque(maxlen=20)

    while len(angle_list)<=50:
        thread1 = threading.Thread(target=thread, args=(ser1, 1))
        thread2 = threading.Thread(target=thread, args=(ser2, 2))
        thread1.daemon = True
        thread2.daemon = True
        thread1.start()
        thread2.start()

        # # 等待三个线程终止
        thread1.join()
        thread2.join()
        
        print("两个线程均已终止，开始检查数据...")
        
        # 将本轮采集的三个端口数据拷贝入缓冲（避免被下轮覆盖）
        if ser1_data['tx1_sqn'][0] != -1 and ser1_data['tx2_sqn'][0] != -1:
            ser1_buffer.append({
                'interval': [ser1_data['interval'][0]],
                'tx1_sqn': [ser1_data['tx1_sqn'][0]],
                'tx2_sqn': [ser1_data['tx2_sqn'][0]],
                'packet_1_I_data': [ser1_data['packet_1_I_data'][0]],
                'packet_1_Q_data': [ser1_data['packet_1_Q_data'][0]],
                'packet_2_I_data': [ser1_data['packet_2_I_data'][0]],
                'packet_2_Q_data': [ser1_data['packet_2_Q_data'][0]],
            })
        if ser2_data['tx1_sqn'][0] != -1 and ser2_data['tx2_sqn'][0] != -1:
            ser2_buffer.append({
                'interval': [ser2_data['interval'][0]],
                'tx1_sqn': [ser2_data['tx1_sqn'][0]],
                'tx2_sqn': [ser2_data['tx2_sqn'][0]],
                'packet_1_I_data': [ser2_data['packet_1_I_data'][0]],
                'packet_1_Q_data': [ser2_data['packet_1_Q_data'][0]],
                'packet_2_I_data': [ser2_data['packet_2_I_data'][0]],
                'packet_2_Q_data': [ser2_data['packet_2_Q_data'][0]],
            })

        # 在缓冲中寻找三路匹配 (tx1_sqn, tx2_sqn) —— 使用键交集降低复杂度
        matched = False
        keys1 = set((x['tx1_sqn'][0], x['tx2_sqn'][0]) for x in ser1_buffer)
        keys2 = set((x['tx1_sqn'][0], x['tx2_sqn'][0]) for x in ser2_buffer)
        common_keys = keys1 & keys2
        # common_keys = keys2 & keys3
        if common_keys:
            k = next(iter(common_keys))
            a = next(x for x in ser1_buffer if (x['tx1_sqn'][0], x['tx2_sqn'][0]) == k)
            b = next(x for x in ser2_buffer if (x['tx1_sqn'][0], x['tx2_sqn'][0]) == k)

            print(a['tx1_sqn'][0], a['tx2_sqn'][0], b['tx1_sqn'][0], b['tx2_sqn'][0])
            angle = data_process(a, b, first_tx_angle_deg=first_tx_angle_deg)
            angle_list.append(angle)
            
            # 保存本次触发的三个串口数据
            trigger_data = {
                'trigger_index': len(angle_list),
                'angle': angle,
                'rx1_data': {
                    'interval': a['interval'][0],
                    'tx1_sqn': a['tx1_sqn'][0],
                    'tx2_sqn': a['tx2_sqn'][0],
                    'packet_1_I_data': a['packet_1_I_data'][0].copy(),
                    'packet_1_Q_data': a['packet_1_Q_data'][0].copy(),
                    'packet_2_I_data': a['packet_2_I_data'][0].copy(),
                    'packet_2_Q_data': a['packet_2_Q_data'][0].copy(),
                },
                'rx2_data': {
                    'interval': b['interval'][0],
                    'tx1_sqn': b['tx1_sqn'][0],
                    'tx2_sqn': b['tx2_sqn'][0],
                    'packet_1_I_data': b['packet_1_I_data'][0].copy(),
                    'packet_1_Q_data': b['packet_1_Q_data'][0].copy(),
                    'packet_2_I_data': b['packet_2_I_data'][0].copy(),
                    'packet_2_Q_data': b['packet_2_Q_data'][0].copy(),
                }
            }
            all_data_list.append(trigger_data)
            
            try:
                ser1_buffer.remove(a)
            except ValueError:
                pass
            try:
                ser2_buffer.remove(b)
            except ValueError:
                pass
            matched = True

        if not matched:
            print("暂未匹配成功，继续累积...")
        
        # 短暂延迟后继续下一轮
        time.sleep(0.1)
        print(f"当前已触发次数{len(angle_list)}")
    print(f"达到最大触发次数{len(angle_list)},监控结束")
    
    # 保存所有数据到文件
    # 将数据组织成字典格式以便保存
    save_dict = {
        'angle_list': np.array(angle_list),
        'num_triggers': len(all_data_list)
    }
    
    # 为每次触发创建独立的数据块
    for idx, trigger_data in enumerate(all_data_list):
        prefix = f'trigger_{idx:03d}'
        save_dict[f'{prefix}_angle'] = trigger_data['angle']
        save_dict[f'{prefix}_rx1_interval'] = trigger_data['rx1_data']['interval']
        save_dict[f'{prefix}_rx1_tx1_sqn'] = trigger_data['rx1_data']['tx1_sqn']
        save_dict[f'{prefix}_rx1_tx2_sqn'] = trigger_data['rx1_data']['tx2_sqn']
        save_dict[f'{prefix}_rx1_pkt1_I'] = trigger_data['rx1_data']['packet_1_I_data']
        save_dict[f'{prefix}_rx1_pkt1_Q'] = trigger_data['rx1_data']['packet_1_Q_data']
        save_dict[f'{prefix}_rx1_pkt2_I'] = trigger_data['rx1_data']['packet_2_I_data']
        save_dict[f'{prefix}_rx1_pkt2_Q'] = trigger_data['rx1_data']['packet_2_Q_data']
        
        save_dict[f'{prefix}_rx2_interval'] = trigger_data['rx2_data']['interval']
        save_dict[f'{prefix}_rx2_tx1_sqn'] = trigger_data['rx2_data']['tx1_sqn']
        save_dict[f'{prefix}_rx2_tx2_sqn'] = trigger_data['rx2_data']['tx2_sqn']
        save_dict[f'{prefix}_rx2_pkt1_I'] = trigger_data['rx2_data']['packet_1_I_data']
        save_dict[f'{prefix}_rx2_pkt1_Q'] = trigger_data['rx2_data']['packet_1_Q_data']
        save_dict[f'{prefix}_rx2_pkt2_I'] = trigger_data['rx2_data']['packet_2_I_data']
        save_dict[f'{prefix}_rx2_pkt2_Q'] = trigger_data['rx2_data']['packet_2_Q_data']
        
    save_filename = 'discrete_antenna_experiment/tx1d_{}_tx1a_{}_tx2d_{}_tx2a_{}.npz'.format(30, 30, 40, -80)
    # save_filename = 'discrete_antenna_experiment/ondesk_{}.npz'.format(-10)
    # save_filename = 'antenna_array_experiment/same_antenna.npz'
    np.savez(save_filename, **save_dict)
    print(f"数据已保存到 {save_filename}")
    print(f"共保存了 {len(all_data_list)} 次触发数据")



# 启动监控（如需使用已知第一个发送方角度进行补偿，在此传入角度，单位度）
start_monitoring(ser1, ser2, 0)