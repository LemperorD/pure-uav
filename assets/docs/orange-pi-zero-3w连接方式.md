# Orange Pi Zero 3w连接方式

开发板系统：linux
用户名：orangepi
sudo密码：orangepi

## SSH无线连接

1. 连接机器人队的wifi
    名称：robomakima
    密码：12345678
2. 给板子插电，等待红灯闪烁
3. 在电脑上用ssh进行连接

    ```bash
    ssh orangepi@192.168.3.163
    ```

    密码：orangepi

## adb有线连接

> 已在板端配置开机自动运行adb启动脚本的systemctl服务

1. 用USB线连接板子和电脑
2. 红灯闪烁后等待大约1分钟
3. 执行`adb devices`查看是否连接成功
4. 执行`adb shell`进行板端开发
