# ms5837 Linux 驱动

本仓库包含适用于 Linux 内核的 ms5837 电池管理 IC 驱动代码。

## 目录结构
```
```
.
├── ms5837.c        # 主要的驱动源代码
├── ms5837-overlay.dts # 设备树覆盖文件
└── Makefile        # 编译驱动的 Makefile
```
```

## 编译和安装
### 1. 获取 Linux 内核源码
确保你已经安装了对应的 Linux 内核源码，并在 `Makefile` 中正确设置了 `KDIR` 变量。

### 2. 编译驱动
```sh
make
```

### 3. 加载驱动
```sh
sudo modprobe ms5837.ko
```

### 4. 卸载驱动
```sh
sudo rmmod ms5837
```

## 设备树配置
如果你的系统使用设备树，请将 `ms5837-overlay.dts` 编译为 `.dtbo` 并加载：

```sh
dtc -@ -I dts -O dtb -o ms5837-overlay.dtbo ms5837-overlay.dts
sudo cp ms5837-overlay.dtbo /boot/firmware/overlays/
echo "dtoverlay=ms5837-overlay" | sudo tee -a /boot/firmware/config.txt
```
