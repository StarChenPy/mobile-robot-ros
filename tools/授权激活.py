'''

    使用说明:
    执行此脚本, 会自动将当前目录下的signature.bin文件上传至机器人指定目录下.
    上传完成后, 会自动调用机器人服务, 进行激活验证.
    如果激活成功, 会自动重启机器人节点, 完成激活.

'''

import os.path
import time
import sys

sys.path.append('../')
import paramiko

# ip = '192.168.123.3'
ip = '10.12.34.2'


#SSH文件拷贝
def file_copy(ip, local_path, remote_path):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    success = False
    try:
        ssh.connect(ip, 22, 'vmxpi', 'password')
        sftp = ssh.open_sftp()
        sftp.put(local_path, remote_path)
        sftp.close()
        # print(f"文件 {local_path} 已成功上传到 {remote_path}")
        success = True
    except Exception as e:
        print(f"传输文件错误:{e}")
    finally:
        ssh.close()
    return success


def system_command(ip='', cmd=''):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    success = False
    try:
        ssh.connect(ip, 22, 'vmx', 'password')
        stdin, stdout, stderr = ssh.exec_command(cmd)
        time.sleep(2)
        exit_status = stdout.channel.recv_exit_status()
        if exit_status == 0:
            print('命令:{} 已发送'.format(cmd))
            success = True
        else:
            print('命令:{} 执行失败! {}'.format(cmd, stderr.read().decode()))
    except Exception as e:
        print('命令执行错误')
    finally:
        ssh.close()
    return success


def main():
    # 获取激活状态
    license_info = robot.license.call_license_srv()
    if license_info['is_authorize']:
        print('当前设备已激活. 无需授权')
        robot.stop_executor()
        return

    #提示用户确认
    print('')
    print('请将当前设备序列号:{} 签名文件signature.bin拷贝至工具目录下.'.format(license_info['serial_number']))
    text = input('输入yes进行授权, no退出: ')
    if text != 'yes':
        robot.stop_executor()
        return

    #远程写入签名文件 当前文件下的signature.bin
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    sig_file = os.path.join(curr_dir, 'signature.bin')
    success = file_copy(ip, sig_file, license_info['signature_path'])
    if not success:
        print('上传签名文件失败. 请检查网络连接是否正常.')
        robot.stop_executor()
        return
    print('上传签名文件完成. 正在验证激活是否有效...')
    time.sleep(2)
    #调用一次服务, ros端口进行一次公钥解密
    license_info = robot.license.call_license_srv()
    if license_info['is_authorize']:
        print('签名有效.')
        print('正在软重启节点, 重启后完成自动激活.')

        print('正在发送重启命令. 请等待...')
        cmd = "echo {} |sudo -S /home/vmx/WSR_HB_Robot/robot_manager.py restart".format('password')
        ret = system_command(ip, cmd)
        if ret:
            print('软重ros启节点完成.')
        else:
            print('软重ros启节点失败.')

    else:
        print('无法完成授权! 当前签名文件signature.bin无效.')

    robot.stop_executor()


if __name__ == "__main__":
    main()
