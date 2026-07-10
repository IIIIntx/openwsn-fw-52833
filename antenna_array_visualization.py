"""
离线数据处理脚本 - 数据读取
用于读取保存在 three_rx_experiment 文件夹下的实验数据
"""

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.mixture import GaussianMixture

import cmath

SPEED_OF_LIGHT  = 299792458

def load_data(filename):
    """
    加载保存的实验数据
    
    参数:
        filename: 数据文件路径 
    
    返回:
        data_dict: 包含所有数据的字典
            - angle_list: 角度列表
            - num_triggers: 触发次数
            - triggers: 每次触发的数据列表
                - index: 触发索引
                - angle: 角度
                - rx1_data: 接收端1的数据
                - rx2_data: 接收端2的数据
                - rx3_data: 接收端3的数据
    """
    data = np.load(filename, allow_pickle=True)
    
    # 提取基本信息
    angle_list = data['angle_list']
    num_triggers = int(data['num_triggers'])
    
    # 组织每次触发的数据
    triggers = []
    for idx in range(num_triggers):
        prefix = f'trigger_{idx:03d}'
        trigger_data = {
            'index': idx,
            'angle': float(data[f'{prefix}_angle']),
            'rx1_data': {
                'interval': int(data[f'{prefix}_rx1_interval']),
                'tx1_sqn': int(data[f'{prefix}_rx1_tx1_sqn']),
                'tx2_sqn': int(data[f'{prefix}_rx1_tx2_sqn']),
                'packet_1_I_data': data[f'{prefix}_rx1_pkt1_I'],
                'packet_1_Q_data': data[f'{prefix}_rx1_pkt1_Q'],
                'packet_2_I_data': data[f'{prefix}_rx1_pkt2_I'],
                'packet_2_Q_data': data[f'{prefix}_rx1_pkt2_Q'],
            },
            'rx2_data': {
                'interval': int(data[f'{prefix}_rx2_interval']),
                'tx1_sqn': int(data[f'{prefix}_rx2_tx1_sqn']),
                'tx2_sqn': int(data[f'{prefix}_rx2_tx2_sqn']),
                'packet_1_I_data': data[f'{prefix}_rx2_pkt1_I'],
                'packet_1_Q_data': data[f'{prefix}_rx2_pkt1_Q'],
                'packet_2_I_data': data[f'{prefix}_rx2_pkt2_I'],
                'packet_2_Q_data': data[f'{prefix}_rx2_pkt2_Q'],
            }
        }
        triggers.append(trigger_data)
    
    return {
        'angle_list': angle_list,
        'num_triggers': num_triggers,
        'triggers': triggers
    }


def show_slotframe(trigger_data, save_path=None):
    """
    可视化某一次触发中所有相位的变化
    
    参数:
        trigger_data: 单次触发的数据字典，包含 rx1_data, rx2_data, rx3_data
        save_path: 可选，保存图像的路径
    
    返回:
        无，直接显示或保存图像
    """
    # 提取三个接收端的数据
    rx1_data = trigger_data['rx1_data']
    rx2_data = trigger_data['rx2_data']
    
    # 计算每个接收端每个包的相位
    # RX1
    rx1_pkt1_phase = np.arctan2(rx1_data['packet_1_Q_data'], rx1_data['packet_1_I_data'])
    rx1_pkt2_phase = np.arctan2(rx1_data['packet_2_Q_data'], rx1_data['packet_2_I_data'])
    
    # RX2
    rx2_pkt1_phase = np.arctan2(rx2_data['packet_1_Q_data'], rx2_data['packet_1_I_data'])
    rx2_pkt2_phase = np.arctan2(rx2_data['packet_2_Q_data'], rx2_data['packet_2_I_data'])

    
    # 获取数据长度
    num_samples = len(rx1_pkt1_phase)
    sample_indices = np.arange(num_samples)
    
    # 创建图形
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle(f'Phase Changes - Trigger {trigger_data["index"]} (Angle: {trigger_data["angle"]:.2f}°)', 
                 fontsize=14, fontweight='bold')
    
    # RX1 Packet 1
    ax1 = axes[0, 0]
    ax1.plot(sample_indices, rx1_pkt1_phase, 'b-o', markersize=3, linewidth=1.5, label='RX1 Pkt1')
    ax1.set_xlabel('Sample Index')
    ax1.set_ylabel('Phase (rad)')
    ax1.set_title('RX1 Packet 1 Phase')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_ylim([-np.pi, np.pi])
    
    # RX1 Packet 2
    ax2 = axes[0, 1]
    ax2.plot(sample_indices, rx1_pkt2_phase, 'b-s', markersize=3, linewidth=1.5, label='RX1 Pkt2')
    ax2.set_xlabel('Sample Index')
    ax2.set_ylabel('Phase (rad)')
    ax2.set_title('RX1 Packet 2 Phase')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_ylim([-np.pi, np.pi])
    
    # RX2 Packet 1
    ax3 = axes[1, 0]
    ax3.plot(sample_indices, rx2_pkt1_phase, 'g-o', markersize=3, linewidth=1.5, label='RX2 Pkt1')
    ax3.set_xlabel('Sample Index')
    ax3.set_ylabel('Phase (rad)')
    ax3.set_title('RX2 Packet 1 Phase')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    ax3.set_ylim([-np.pi, np.pi])
    
    # RX2 Packet 2
    ax4 = axes[1, 1]
    ax4.plot(sample_indices, rx2_pkt2_phase, 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    ax4.set_xlabel('Sample Index')
    ax4.set_ylabel('Phase (rad)')
    ax4.set_title('RX2 Packet 2 Phase')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    ax4.set_ylim([-np.pi, np.pi])
    
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"图像已保存到 {save_path}")
    else:
        plt.show() 

def calculate_angle(I1, Q1, I2, Q2):
    """
    计算第一个IQ向量到第二个IQ向量之间的夹角
    
    参数:
        I1, Q1: 第一个IQ向量的I和Q分量（可以是标量或数组）
        I2, Q2: 第二个IQ向量的I和Q分量（可以是标量或数组）
    
    返回:
        theta: 夹角（弧度），范围 [-π, π]
               如果输入是数组，返回对应位置的夹角数组
    """
    # 转换为numpy数组以确保兼容性
    I1 = np.asarray(I1)
    Q1 = np.asarray(Q1)
    I2 = np.asarray(I2)
    Q2 = np.asarray(Q2)
    
    # 计算两个向量的相位
    phase1 = np.arctan2(Q1, I1) #- 0.5*np.pi  # 第一个向量的相位
    phase2 = np.arctan2(Q2, I2)  # 第二个向量的相位
    
    # 计算相位差（从phase1到phase2）
    theta = phase1 - phase2
    
    # 将角度约束到 [-π, π] 范围
    theta = np.arctan2(np.sin(theta), np.cos(theta))
    
    return theta

def cal_slope(phase_diff):
    x = np.arange(len(phase_diff)).reshape(-1, 1)
    y = phase_diff
    model = LinearRegression()
    model.fit(x, y)
    return model.coef_[0], model.intercept_

def  phase_to_angle(geo):
    # val = geo/(2*np.pi)*12.5/3.75
    val = geo/(2*np.pi)*12.5/4.3
    if val > 1:
        val = 1
    if val < -1:
        val = 1
    return np.arcsin(val)/np.pi*180
    
def unwrap_and_adjust(numpy_array):

    unwrapped = np.unwrap(numpy_array)
    adjusted_array = np.arctan2(np.sin(unwrapped), np.cos(unwrapped))
    return adjusted_array

def angle_to_phase(angle):
    return angle/180*np.pi*4.3/12.5*2*np.pi
def compensate_phase_offset(trigger_data, ref_position):
    # 提取三个接收端的数据
    rx1_data = trigger_data['rx1_data']
    rx2_data = trigger_data['rx2_data']
    
    # 计算每个接收端每个包的相位
    # RX1
    rx1_pkt1_phase = np.arctan2(rx1_data['packet_1_Q_data'], rx1_data['packet_1_I_data']) #+ 468/360*2*np.pi
    rx1_pkt2_phase = np.arctan2(rx1_data['packet_2_Q_data'], rx1_data['packet_2_I_data']) #+ 468/360*2*np.pi
    
    # RX2
    rx2_pkt1_phase = np.arctan2(rx2_data['packet_1_Q_data'], rx2_data['packet_1_I_data'])
    rx2_pkt2_phase = np.arctan2(rx2_data['packet_2_Q_data'], rx2_data['packet_2_I_data'])
    
    # plt.figure()
    # plt.plot(np.sqrt(rx1_data['packet_1_I_data']**2 + rx1_data['packet_1_Q_data']**2), 'b-o', markersize=3, linewidth=1.5, label='RX1 Pkt1')
    # plt.plot(np.sqrt(rx2_data['packet_1_I_data']**2 + rx2_data['packet_1_Q_data']**2), 'g-o', markersize=3, linewidth=1.5, label='RX2 Pkt1')
    # plt.plot(np.sqrt(rx1_data['packet_2_I_data']**2 + rx1_data['packet_2_Q_data']**2), 'b-s', markersize=3, linewidth=1.5, label='RX1 Pkt2')
    # plt.plot(np.sqrt(rx2_data['packet_2_I_data']**2 + rx2_data['packet_2_Q_data']**2), 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    # plt.legend()
    # plt.show()

    interval = (rx1_data['interval'] + rx2_data['interval'])/2
    # 计算每个接收端每个包的相位
    # RX1

    pkt1_phase_diff_12 = calculate_angle(rx1_data['packet_1_I_data'], rx1_data['packet_1_Q_data'], rx2_data['packet_1_I_data'], rx2_data['packet_1_Q_data'])

    pkt2_phase_diff_12 = calculate_angle(rx1_data['packet_2_I_data'], rx1_data['packet_2_Q_data'], rx2_data['packet_2_I_data'], rx2_data['packet_2_Q_data'])

    print('pkt1_phase_diff_12, pkt2_phase_diff_12:', pkt1_phase_diff_12[0], pkt2_phase_diff_12[0])
    pkt1_phase_diff_12 = np.unwrap(pkt1_phase_diff_12)
    pkt2_phase_diff_12 = np.unwrap(pkt2_phase_diff_12)

    slope_12, intercept_12 = cal_slope(pkt1_phase_diff_12)

    print('slope_12:', slope_12)
    print('interval:', interval)
    print('interval:', slope_12 * interval/16)
    anchor_phase = angle_to_phase(ref_position)

    target_phase_diff_12 = np.arctan2(np.sin(pkt2_phase_diff_12 - pkt1_phase_diff_12 - slope_12 * interval/16 - anchor_phase), np.cos(pkt2_phase_diff_12 - pkt1_phase_diff_12 - slope_12 * interval/16 - anchor_phase))
    # target_phase_diff_12 = np.arctan2(np.sin(pkt1_phase_diff_12 - pkt2_phase_diff_12), np.cos(pkt1_phase_diff_12 - pkt2_phase_diff_12))

    print(target_phase_diff_12)
    target_phase_diff_12_unwrap = unwrap_and_adjust(target_phase_diff_12)


    # fig, axes = plt.subplots(3, 3, figsize=(16, 12))

    # ax1 = axes[0, 0]
    # ax1.plot(rx1_pkt1_phase, 'b-o', markersize=3, linewidth=1.5, label='RX1 Pkt1')
    # ax1.set_xlabel('Sample Index')
    # ax1.set_ylabel('Phase (rad)')
    # ax1.set_title('RX1 Packet 1 Phase')
    # ax1.grid(True, alpha=0.3)
    # ax1.legend()
    # ax1.set_ylim([-np.pi, np.pi])
    
    # # RX1 Packet 2
    # ax2 = axes[0, 1]
    # ax2.plot(rx1_pkt2_phase, 'b-s', markersize=3, linewidth=1.5, label='RX1 Pkt2')
    # ax2.set_xlabel('Sample Index')
    # ax2.set_ylabel('Phase (rad)')
    # ax2.set_title('RX1 Packet 2 Phase')
    # ax2.grid(True, alpha=0.3)
    # ax2.legend()
    # ax2.set_ylim([-np.pi, np.pi])
    
    # # RX2 Packet 1
    # ax3 = axes[1, 0]
    # ax3.plot(rx2_pkt1_phase, 'g-o', markersize=3, linewidth=1.5, label='RX2 Pkt1')
    # ax3.set_xlabel('Sample Index')
    # ax3.set_ylabel('Phase (rad)')
    # ax3.set_title('RX2 Packet 1 Phase')
    # ax3.grid(True, alpha=0.3)
    # ax3.legend()
    # ax3.set_ylim([-np.pi, np.pi])
    
    # # RX2 Packet 2
    # ax4 = axes[1, 1]
    # ax4.plot(rx2_pkt2_phase, 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    # ax4.set_xlabel('Sample Index')
    # ax4.set_ylabel('Phase (rad)')
    # ax4.set_title('RX2 Packet 2 Phase')
    # ax4.grid(True, alpha=0.3)
    # ax4.legend()
    # ax4.set_ylim([-np.pi, np.pi])

    # ax5 = axes[2, 0]
    # ax5.plot(unwrap_and_adjust(rx1_pkt1_phase - rx2_pkt1_phase), 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    # ax5.set_xlabel('Sample Index')
    # ax5.set_ylabel('Phase (rad)')
    # ax5.set_title('RX1 Packet 1 - RX2 Packet 1')
    # ax5.grid(True, alpha=0.3)
    # ax5.legend()
    # ax5.set_ylim([-np.pi, np.pi])


    # ax5 = axes[2, 1]
    # ax5.plot(unwrap_and_adjust(rx1_pkt2_phase - rx2_pkt2_phase), 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    # ax5.set_xlabel('Sample Index')
    # ax5.set_ylabel('Phase (rad)')
    # ax5.set_title('RX1 Packet 2 - RX2 Packet 2')
    # ax5.grid(True, alpha=0.3)
    # ax5.legend()
    # ax5.set_ylim([-np.pi, np.pi])

    # ax6 = axes[0, 2]
    # ax6.plot(unwrap_and_adjust(rx1_pkt1_phase - rx1_pkt2_phase), 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    # ax6.set_xlabel('Sample Index')
    # ax6.set_ylabel('Phase (rad)')
    # ax6.set_title('RX1 Packet 1 - RX1 Packet 2')
    # ax6.grid(True, alpha=0.3)
    # ax6.legend()
    # ax6.set_ylim([-np.pi, np.pi])

    # ax7 = axes[1, 2]
    # ax7.plot(unwrap_and_adjust(rx2_pkt1_phase - rx2_pkt2_phase), 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    # ax7.set_xlabel('Sample Index')
    # ax7.set_ylabel('Phase (rad)')
    # ax7.set_title('RX2 Packet 1 - RX2 Packet 2')
    # ax7.grid(True, alpha=0.3)
    # ax7.legend()
    # ax7.set_ylim([-np.pi, np.pi])

    # ax8 = axes[2, 2]
    # ax8.plot(target_phase_diff_12_unwrap, 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    # ax8.set_xlabel('Sample Index')
    # ax8.set_ylabel('Phase (rad)')
    # ax8.set_title('RX2 Packet 2 Phase')
    # ax8.grid(True, alpha=0.3)
    # ax8.legend()
    # ax8.set_ylim([-np.pi, np.pi])
    # plt.tight_layout()
    # plt.show()


    # the amplitude can identifiy the traget is at the left side or right side

    # plt.figure()
    # plt.plot(rx1_data['packet_2_I_data']**2 + rx1_data['packet_2_Q_data']**2, 'b-o', markersize=3, linewidth=1.5, label='RX1 Pkt2')
    # plt.plot(rx1_data['packet_1_I_data']**2 + rx1_data['packet_1_Q_data']**2, 'b-s', markersize=3, linewidth=1.5, label='RX1 Pkt1')
    # plt.plot(rx2_data['packet_2_I_data']**2 + rx2_data['packet_2_Q_data']**2, 'g-o', markersize=3, linewidth=1.5, label='RX2 Pkt2')
    # plt.plot(rx2_data['packet_1_I_data']**2 + rx2_data['packet_1_Q_data']**2, 'g-s', markersize=3, linewidth=1.5, label='RX2 Pkt1')

    # plt.legend()
    # plt.show()

    phase_12 = np.mean(target_phase_diff_12_unwrap)
    # plt.figure()
    # plt.hist( target_phase_diff_12, bins=30)
    # plt.show()

    array1 = unwrap_and_adjust(rx1_pkt1_phase - rx1_pkt2_phase)
    array2 = unwrap_and_adjust(rx2_pkt1_phase - rx2_pkt2_phase)
    array12 = unwrap_and_adjust(array1 - array2)

    # if abs(np.mean(array12)) < np.pi: 
    #     if np.mean(array12) < 0:
    #         return phase_to_angle(phase_12 - np.pi)
    #     else:
    #         return phase_to_angle(phase_12)
    # else:
    #     return phase_to_angle(np.arctan2(np.sin(phase_12 - np.pi), np.cos(phase_12 - np.pi)))

    rx1_amplitude = np.mean(np.sqrt(rx1_data['packet_2_I_data']**2 + rx1_data['packet_2_Q_data']**2))
    rx2_amplitude = np.mean(np.sqrt(rx2_data['packet_2_I_data']**2 + rx2_data['packet_2_Q_data']**2))

    if rx1_amplitude < rx2_amplitude:
        if phase_12 > 0:
            return phase_to_angle(np.arctan2(np.sin(phase_12 - np.pi), np.cos(phase_12 - np.pi)))
        else:
            return phase_to_angle(np.arctan2(np.sin(phase_12), np.cos(phase_12)))
    else:
        if phase_12 < 0:
            return phase_to_angle(np.arctan2(np.sin(phase_12 - np.pi), np.cos(phase_12 - np.pi)))
        else:
            return phase_to_angle(np.arctan2(np.sin(phase_12), np.cos(phase_12)))

    return phase_to_angle(phase_12)


def filter_array(arr):
    arr = np.array(arr)
    value_to_remove = 90
    mask = (arr != value_to_remove) & (arr != -90)
    print(mask)
    new_arr = arr[mask]
    return new_arr



# 使用示例：
if __name__ == '__main__':
    # 指定要读取的数据文件名
    # filename = 'antenna_array_experiment/angle_-20.npz'
    # filename = 'antenna_array_experiment/same_antenna.npz'
    # filename = 'three_rx_experiment/angle_-20.npz'
    # filename = 'discrete_antenna_experiment/angle_10.npz'
    filename = 'discrete_antenna_experiment/tx1d_30_tx1a_0_tx2d_40_tx2a_-40.npz'
    # filename = 'discrete_antenna_experiment/ondesk_{}.npz'.format(-10)
    # 加载数据
    data_dict = load_data(filename)
    print(f"成功加载 {data_dict['num_triggers']} 次触发数据")
    print(f"角度列表: {data_dict['angle_list']}")

    est_angle_list = []
    phase12_list = []
    phase23_list = []

    for i in range(len(data_dict['triggers'])):
        trigger = data_dict['triggers'][i]
        # show_slotframe(trigger)
        phase12 = compensate_phase_offset(trigger, 0)
        phase12_list.append(phase12)
    plt.figure()
    plt.boxplot(phase12_list, positions=[1])
    plt.show()

    plt.figure()
    vp12 = plt.violinplot(phase12_list, positions=[1])
    for body in vp12['bodies']:
        body.set_label('rx12 angle')

    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(phase12_list, 'o')
    plt.show() 
    # plt.scatter([i for i in range(len(est_angle_list))], est_angle_list)
    # plt.show()

    # 创建包含两个子图的画布
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    position_list = [-80, -70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80]
    all_phase_data = []
    # position_list = [-20, -10, 10, 20, 30]
    # 收集所有数据
    for pos in position_list:
        anchor_angle = -20
        filename = 'discrete_antenna_experiment/tx1d_30_tx1a_{}_tx2d_40_tx2a_{}.npz'.format(anchor_angle, pos)
        # filename = 'discrete_antenna_experiment/angle_{}.npz'.format(pos)
        data_dict = load_data(filename)
        print(f"成功加载 {data_dict['num_triggers']} 次触发数据")
        print(f"角度列表: {data_dict['angle_list']}")
        
        phase12_list = []
        for i in range(len(data_dict['triggers'])):
            trigger = data_dict['triggers'][i]
            phase12 = compensate_phase_offset(trigger, anchor_angle)
            phase12_list.append(phase12)
        
        all_phase_data.append(phase12_list)

    # 子图1：小提琴图
    ax1 = axes[0]
    positions = range(len(position_list))
    vp = ax1.violinplot(all_phase_data, positions=positions, showmeans=True)

    # 设置小提琴图的x轴标签
    ax1.set_xticks(positions)
    ax1.set_xticklabels([f'{pos}°' for pos in position_list])
    ax1.set_xlabel('angle (°)')
    ax1.set_ylabel('estimate angle')
    ax1.grid(True, alpha=0.3)

    # 子图2：箱线图
    ax2 = axes[1]
    # 创建箱线图
    box = ax2.boxplot(all_phase_data, positions=positions, patch_artist=True, 
                    showmeans=True, meanline=True, showfliers=True)

    # 设置箱线图样式
    colors = ['lightblue'] * len(position_list)  # 所有箱线使用相同颜色
    for patch, color in zip(box['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    # 设置箱线图的x轴标签
    ax2.set_xticks(positions)
    ax2.set_xticklabels([f'{pos}°' for pos in position_list])
    ax2.set_xlabel('angle (°)')
    ax2.set_ylabel('estimate angle')

    ax2.grid(True, alpha=0.3)

    # 添加图例说明（可选）
    import matplotlib.patches as mpatches
    violin_patch = mpatches.Patch(color='blue', alpha=0.7, label='小提琴图')
    box_patch = mpatches.Patch(color='lightblue', alpha=0.7, label='箱线图')
    ax2.legend(handles=[box_patch], loc='upper right')
    ax2.set_ylim(-90, 90)

    # 调整布局并显示
    plt.tight_layout()
    plt.show()

    print(all_phase_data[0])
    plt.figure()
    for i in range(len(all_phase_data)):

        new_array = filter_array(all_phase_data[i])
        plt.violinplot(new_array, positions=[position_list[i]/10])
        plt.boxplot(new_array, positions=[position_list[i]/10])
    
    plt.plot([-8, 8], [80, -80], 'r--', label='Ideal Line')
    plt.grid()
    plt.show()