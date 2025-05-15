# raspberry-cane

raspberry-cane 是一个基于 Python 的开源项目，主要面向树莓派等嵌入式设备，集成了图像处理、数据分析等功能模块，便于开发者快速搭建和扩展相关应用。

## 主要特性

- 支持树莓派等 ARM 架构设备
- 集成 OpenCV、NumPy 等常用科学计算与图像处理库
- 提供丰富的代码示例，便于二次开发
- 适合嵌入式开发、物联网、智能家居等场景

## 文件结构

- `code/`：主要代码目录
- `test/`：测试代码
- `project/`：项目相关文件
- `.cache/`、`.config/`、`.local/` 等：运行环境相关配置
- `numpy-1.21.5-cp37-cp37m-linux_armv7l.whl`：NumPy 预编译包
- `opencv_python-4.5.5.62-cp37-cp37m-linux_armv7l.whl`：OpenCV 预编译包

## 安装依赖

推荐在树莓派（或兼容 ARM 设备）上使用 Python 3.7 环境。

```bash
pip install numpy-1.21.5-cp37-cp37m-linux_armv7l.whl
pip install opencv_python-4.5.5.62-cp37-cp37m-linux_armv7l.whl
```

## 快速开始

1. 克隆本项目：

   ```bash
   git clone https://github.com/helloaisvg/raspberry-cane.git
   cd raspberry-cane
   ```

2. 安装依赖（见上文）

3. 运行示例代码：

   ```bash
   python3 project.py
   ```

## 贡献

欢迎提交 issue 和 pull request 参与项目建设！

## 许可证

本项目采用 MIT License。

---

# English

raspberry-cane is an open-source project based on Python, mainly for Raspberry Pi and other embedded devices. It integrates image processing, data analysis, and other modules, making it easy for developers to quickly build and extend related applications.

## Features

- Supports Raspberry Pi and ARM devices
- Integrates OpenCV, NumPy, and other scientific computing/image processing libraries
- Rich code examples for secondary development
- Suitable for embedded, IoT, smart home, etc.

## File Structure

- `code/`: Main code
- `test/`: Test code
- `project/`: Project files
- `.cache/`, `.config/`, `.local/`: Environment configs
- `numpy-1.21.5-cp37-cp37m-linux_armv7l.whl`: NumPy wheel
- `opencv_python-4.5.5.62-cp37-cp37m-linux_armv7l.whl`: OpenCV wheel

## Install Dependencies

Recommended: Python 3.7 on Raspberry Pi (or ARM device).

```bash
pip install numpy-1.21.5-cp37-cp37m-linux_armv7l.whl
pip install opencv_python-4.5.5.62-cp37-cp37m-linux_armv7l.whl
```

## Quick Start

1. Clone the repo:

   ```bash
   git clone https://github.com/helloaisvg/raspberry-cane.git
   cd raspberry-cane
   ```

2. Install dependencies (see above)

3. Run example code:

   ```bash
   python code/your_example.py
   ```

## Contributing

Issues and pull requests are welcome!

## License

MIT License

---

参考：[raspberry-cane GitHub 仓库](https://github.com/helloaisvg/raspberry-cane)
