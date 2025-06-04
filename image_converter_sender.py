import argparse
import numpy as np
import cv2
from PIL import Image
import time
import struct
import asyncio
from bleak import BleakClient, BleakScanner

# 蓝牙服务和特征UUID
IMAGE_SERVICE_UUID = "00FF"
IMAGE_CHAR_UUID = "FF01"

# 目标图像尺寸
TARGET_WIDTH = 300
TARGET_HEIGHT = 396

def convert_to_4bit_grayscale(image_path, target_width=TARGET_WIDTH, target_height=TARGET_HEIGHT):
    """将图片转换为4位灰度图像格式"""
    # 读取图片
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"无法读取图片: {image_path}")
    
    # 调整图片大小
    img = cv2.resize(img, (target_width, target_height))
    
    # 转换为灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 应用Floyd-Steinberg抖动算法以获得更好的4位灰度效果
    pil_img = Image.fromarray(gray)
    pil_img = pil_img.convert('L')
    
    # 量化为16级灰度
    quantized = np.array(pil_img) // 16
    
    # 创建4位色深图像数据
    # 每个字节包含两个像素，每个像素4位
    result = np.zeros((target_height, target_width // 2), dtype=np.uint8)
    
    for y in range(target_height):
        for x in range(0, target_width, 2):
            if x + 1 < target_width:
                # 两个像素打包到一个字节中
                pixel1 = quantized[y, x]
                pixel2 = quantized[y, x + 1]
                # 高4位是第一个像素，低4位是第二个像素
                result[y, x // 2] = (pixel1 << 4) | pixel2
            else:
                # 如果宽度是奇数，最后一个像素单独处理
                result[y, x // 2] = quantized[y, x] << 4
    
    # 将二维数组展平为一维数组
    return result.flatten(), target_width, target_height

async def find_device(device_name):
    """查找指定名称的蓝牙设备"""
    print(f"正在搜索设备: {device_name}...")
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name and device_name in device.name:
            print(f"找到设备: {device.name} ({device.address})")
            return device.address
    return None

async def send_image(device_address, image_data, width, height):
    """通过蓝牙发送图像数据到ESP32"""
    try:
        async with BleakClient(device_address) as client:
            print(f"已连接到: {device_address}")
            
            # 检查服务和特征是否存在
            services = await client.get_services()
            target_service = None
            target_char = None
            
            for service in services:
                if IMAGE_SERVICE_UUID.lower() in service.uuid.lower():
                    target_service = service
                    for char in service.characteristics:
                        if IMAGE_CHAR_UUID.lower() in char.uuid.lower():
                            target_char = char
                            break
                    break
            
            if not target_char:
                print("未找到目标特征，请检查UUID是否正确")
                return False
            
            # 准备数据包：前8个字节是宽度和高度，然后是图像数据
            header = struct.pack('<II', width, height)  # 小端序，两个无符号整数
            
            # 将NumPy数组转换为bytes
            image_bytes = image_data.tobytes()
            
            # 发送第一个数据包（包含头信息）
            first_packet = header + image_bytes[:492]  # 500字节MTU - 8字节头信息
            # await client.write_gatt_char(target_char, first_packet, response=True)  # 添加response=True
            await asyncio.sleep(5)
            print(f"已发送头信息和第一部分数据: {len(first_packet)} 字节")
            
            # 发送剩余数据
            chunk_size = 500  # BLE MTU大小
            remaining_data = image_bytes[492:]
            total_chunks = (len(remaining_data) + chunk_size - 1) // chunk_size
            
            for i in range(0, len(remaining_data), chunk_size):
                chunk = remaining_data[i:i+chunk_size]
                # await client.write_gatt_char(target_char, chunk)
                chunk_num = i // chunk_size + 1
                print(f"已发送数据块 {chunk_num}/{total_chunks}: {len(chunk)} 字节")
                # 添加短暂延迟，避免发送过快导致数据丢失
                await asyncio.sleep(3)
            
            print(f"图像数据发送完成，总大小: {len(image_bytes) + len(header)} 字节")
            return True
    except Exception as e:
        print(f"发送图像时出错: {e}")
        return False

async def main():
    parser = argparse.ArgumentParser(description='将图片转换为4位灰度并通过蓝牙发送到ESP32')
    parser.add_argument('image_path', default='img.png', help='输入图片路径')
    parser.add_argument('--device', default='ESP32-EPaper', help='ESP32设备名称')
    parser.add_argument('--width', type=int, default=TARGET_WIDTH, help='目标图片宽度')
    parser.add_argument('--height', type=int, default=TARGET_HEIGHT, help='目标图片高度')
    parser.add_argument('--address', help='ESP32蓝牙地址（如果已知）')
    
    args = parser.parse_args()
    
    try:
        # 转换图片
        print(f"正在转换图片: {args.image_path}")
        image_data, width, height = convert_to_4bit_grayscale(
            args.image_path, args.width, args.height)
        print(f"图片已转换为4位灰度格式: {width}x{height}, {len(image_data)} 字节")
        
        # 查找设备
        device_address = args.address
        if not device_address:
            device_address = await find_device(args.device)
        
        if not device_address:
            print(f"未找到设备: {args.device}")
            return
        
        # 发送图像
        success = await send_image(device_address, image_data, width, height)
        if success:
            print("图像发送成功！")
        else:
            print("图像发送失败！")
    
    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    asyncio.run(main())