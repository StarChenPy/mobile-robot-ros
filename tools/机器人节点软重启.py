'''

    使用说明:
    执行此脚本, 会向指定的机器人节点发送重启命令, 重启节点.
    可修改为其他命令, 以实现停止节点, 启动节点等功能.
    (停止: stop, 启动: start, 重启: restart)

'''

import time
import paramiko

# ip ='192.168.123.3'
ip = '10.12.34.2'


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
        print('命令执行错误', e)
    finally:
        ssh.close()
    return success


def main():
    print('正在发送重启命令. 请等待...')
    cmd = "echo {} |sudo -S /home/vmx/WSR_HB_Robot/robot_manager.py restart".format('password')
    ret = system_command(ip, cmd)
    if ret:
        print('软重ros启节点完成.')
    else:
        print('软重ros启节点失败.')


if __name__ == "__main__":
    main()
