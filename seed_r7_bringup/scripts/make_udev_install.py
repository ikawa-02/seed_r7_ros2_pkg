#!/usr/bin/env python3
"""
make_udev_install.py - udev ルールインストールスクリプト
ROS2 Jazzy 対応: Python 2 -> Python 3 変換

変更点:
- raw_input() -> input()
- tempfile.NamedTemporaryFile を mode='w' で開きテキスト書き込み
- subprocess 出力を .decode() でデコード
- tf.write() はテキストモードで書き込み
"""

import os
import subprocess
import tempfile


class UdevInstall:

    def setup_serial(self):
        choice = input('install setserial yes(y) or none(n) : ')
        if choice in ['y', 'ye', 'yes']:
            os.system('sudo apt-get install setserial')

    def usb_id_write(self):
        header = '#aero_controller\n'
        upper_string = (
            'SUBSYSTEM=="tty",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",'
            'ATTRS{serial}=="111",MODE="666",SYMLINK+="aero_upper",'
            ' RUN+="/bin/setserial /dev/aero_upper low_latency"\n'
        )
        lower_string = (
            'SUBSYSTEM=="tty",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",'
            'ATTRS{serial}=="123",MODE="666",SYMLINK+="aero_lower",'
            ' RUN+="/bin/setserial /dev/aero_lower low_latency"\n'
        )
        space = '\n'
        header_hokuyo = '#hokuyo urg\n'
        hokuyo_string = (
            'SUBSYSTEM=="tty",ATTRS{idVendor}=="15d1",ATTRS{idProduct}=="0000",'
            'MODE="666",SYMLINK+="hokuyo"\n'
        )
        header_seed4 = '#aero_controller(SEED4)\n'
        seed4_upper = (
            'SUBSYSTEM=="tty",ATTRS{idVendor}=="0483",ATTRS{idProduct}=="a1e8",'
            'ATTRS{serial}=="000000000010",MODE="666",SYMLINK+="aero_upper",'
            ' RUN+="/bin/setserial /dev/aero_upper low_latency"\n'
        )
        seed4_lower = (
            'SUBSYSTEM=="tty",ATTRS{idVendor}=="0483",ATTRS{idProduct}=="a1e8",'
            'ATTRS{serial}=="000000000020",MODE="666",SYMLINK+="aero_lower",'
            ' RUN+="/bin/setserial /dev/aero_lower low_latency"\n'
        )

        print('Please insert upper USB to PC port')
        choice = input('yes(y) or none(n) : ')
        if choice in ['y', 'yes']:
            upper = upper_string.split(',')
            print(upper[3])
            p1 = subprocess.Popen(
                ['udevadm', 'info', '-n', '/dev/ttyUSB0'], stdout=subprocess.PIPE
            )
            p2 = subprocess.Popen(
                ['grep', 'SERIAL_SHORT'], stdin=p1.stdout, stdout=subprocess.PIPE
            )
            p1.stdout.close()
            out, _ = p2.communicate()
            upper[3] = 'ATTRS{serial}=="' + out.decode().split('=')[1].strip() + '"'
            print(upper[3])
            upper_string = ','.join(upper)

        print('Please insert lower USB to PC port')
        choice = input('yes(y) or none(n) : ')
        if choice in ['y', 'yes']:
            lower = lower_string.split(',')
            print(lower[3])
            p1 = subprocess.Popen(
                ['udevadm', 'info', '-n', '/dev/ttyUSB0'], stdout=subprocess.PIPE
            )
            p2 = subprocess.Popen(
                ['grep', 'SERIAL_SHORT'], stdin=p1.stdout, stdout=subprocess.PIPE
            )
            p1.stdout.close()
            out, _ = p2.communicate()
            lower[3] = 'ATTRS{serial}=="' + out.decode().split('=')[1].strip() + '"'
            print(lower[3])
            lower_string = ','.join(lower)

        # 一時ファイルに書き込み (テキストモード)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.rules', delete=False) as tf:
            tmp_path = tf.name
            tf.write(header)
            tf.write(upper_string)
            tf.write(lower_string)
            tf.write(space)
            tf.write(header_hokuyo)
            tf.write(hokuyo_string)
            tf.write(space)
            tf.write(header_seed4)
            tf.write(seed4_upper)
            tf.write(seed4_lower)

        with open(tmp_path) as f:
            print(f.read())

        subprocess.call(['sudo', 'cp', tmp_path, '/etc/udev/rules.d/90-aero.rules'])
        os.unlink(tmp_path)


if __name__ == '__main__':
    ui = UdevInstall()
    ui.setup_serial()
    ui.usb_id_write()
