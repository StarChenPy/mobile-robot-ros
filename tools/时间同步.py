import paramiko
import time
import datetime

RPI_IP = '10.12.34.2'
RPI_USER = 'vmx'
RPI_PASS = 'password'

def get_local_time():
    now = datetime.datetime.now()
    return now.strftime('%Y-%m-%d %H:%M:%S')

def run_sudo_command_interactive(ssh, password, command):
    shell = ssh.invoke_shell()
    time.sleep(1)

    # 发送sudo命令
    shell.send(f"sudo {command}\n")
    buff = ''
    while not buff.endswith(': '):
        resp = shell.recv(1024).decode('utf-8')
        buff += resp
        if 'password' in buff.lower():
            # 收到密码请求，发送密码
            shell.send(password + '\n')
            break

    time.sleep(1)
    # 读取命令输出
    output = ''
    while shell.recv_ready():
        output += shell.recv(1024).decode('utf-8')

    return output

def sync_time_to_rpi():
    local_time = get_local_time()
    print(f"本地时间: {local_time}")

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(RPI_IP, username=RPI_USER, password=RPI_PASS, timeout=10)

    try:
        output = run_sudo_command_interactive(ssh, RPI_PASS, f'date -s "{local_time}"')
        print("设置时间输出:")
        print(output)

        output = run_sudo_command_interactive(ssh, RPI_PASS, 'timedatectl set-timezone Asia/Shanghai')
        print("设置时区输出:")
        print(output)

        print("时间同步完成")
    finally:
        ssh.close()

if __name__ == '__main__':
    sync_time_to_rpi()
