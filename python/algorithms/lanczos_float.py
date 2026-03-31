#!/usr/bin/env python3
# Lanczos插值算法 - 浮点实现
# Lanczos Interpolation - Floating Point Implementation
#
# 用途：
#     - 算法画质参考（浮点黄金参考）
#     - 与 OpenCV 对比验证
#     - 生成测试向量和可视化结果
#
# 坐标映射：
#     Backward Mapping + Center-Aligned
#     src_pos = (dst_idx + 0.5) / scale - 0.5
#
# 插值公式：
#     使用Lanczos核函数（sinc窗口化的sinc函数）
#     L(x) = sinc(x) * sinc(x/a), |x| < a
#          = 0, |x| >= a
#     其中 sinc(x) = sin(πx) / (πx)
#     a为核大小（通常a=2或3）
#
# 作者：
# 版本：1.0

import numpy as np
from PIL import Image
import os
from datetime import datetime


class LanczosFloat:
	# Lanczos插值缩放器（浮点实现）
	#
	# 参数:
	#     scale_factor: 缩放因子，>1为放大，<1为缩小
	#     target_size: 目标尺寸 (width, height)，与 scale_factor 二选一
	#     a: Lanczos核大小（2或3），a=2为Lanczos-2（4x4像素），a=3为Lanczos-3（6x6像素）

	def __init__(self, scale_factor=None, target_size=None, a=3):
		# 初始化缩放器
		#
		# Args:
		#     scale_factor: 缩放比例（dst/src），例如 2.0 表示放大2倍
		#     target_size: 目标尺寸元组 (width, height)
		#     a: Lanczos核大小，2或3（越大质量越高但越慢）
		#
		# Raises:
		#     ValueError: 同时指定或同时未指定 scale_factor 和 target_size

		if (scale_factor is None and target_size is None) or (scale_factor is not None and target_size is not None):
			raise ValueError("请指定 scale_factor 或 target_size 其中一个")

		self.scale_factor = scale_factor
		self.target_size  = target_size
		self.a            = a  # Lanczos核大小
		self.tap_size     = 2 * a  # 每方向采样点数（a=2时为4，a=3时为6）

	def _compute_scale(self, src_w, src_h):
		# 计算实际的缩放比例
		if self.target_size is not None:
			tgt_w, tgt_h = self.target_size
			scale_x      = tgt_w / src_w
			scale_y      = tgt_h / src_h
		else:
			scale_x = scale_y = self.scale_factor
			# 根据 scale 计算目标尺寸
			tgt_w = int(src_w * scale_x)
			tgt_h = int(src_h * scale_y)

		return scale_x, scale_y, int(tgt_w), int(tgt_h)

	def _compute_src_coord(self, dst_idx, scale, src_size):
		# 计算目标像素对应的原图浮点坐标（Backward Mapping + Center-Aligned）
		#
		# 公式：src_pos = (dst_idx + 0.5) / scale - 0.5
		#
		# Args:
		#     dst_idx: 目标像素索引（从0开始）
		#     scale: 缩放比例
		#     src_size: 原图尺寸（用于边界检查）
		#
		# Returns:
		#     src_pos: 原图像素浮点坐标

		# 几何坐标计算（浮点）
		src_pos = (dst_idx + 0.5) / scale - 0.5

		# 边界裁剪（防止越界）
		src_pos = max(0, min(src_pos, src_size - 1))

		return src_pos

	def _sinc(self, x):
		# sinc函数: sin(πx) / (πx)
		# 在x=0时，sinc(0) = 1
		if abs(x) < 1e-6:
			return 1.0
		return np.sin(np.pi * x) / (np.pi * x)

	def _lanczos_weight(self, t):
		# Lanczos核函数
		#
		# Args:
		#     t: 距离中心点的距离（浮点）
		#
		# Returns:
		#     float: 权重值

		t_abs = abs(t)

		if t_abs >= self.a:
			return 0.0

		# L(x) = sinc(x) * sinc(x/a)
		return self._sinc(t) * self._sinc(t / self.a)

	def _lanczos_interpolate(self, src_img, src_y, src_x):
		# 对单个像素进行Lanczos插值
		#
		# Args:
		#     src_img: 源图像数组
		#     src_y: 源图像Y坐标（浮点）
		#     src_x: 源图像X坐标（浮点）
		#
		# Returns:
		#     插值后的像素值 (R, G, B)

		src_h, src_w = src_img.shape[:2]

		# 获取整数坐标（中心参考点）
		y0 = int(np.floor(src_y))
		x0 = int(np.floor(src_x))

		# 获取小数部分
		dy = src_y - y0
		dx = src_x - x0

		# 计算权重矩阵 (tap_size x tap_size)
		tap = self.tap_size
		weights = np.zeros((tap, tap), dtype=np.float32)

		for m in range(tap):  # Y方向
			for n in range(tap):  # X方向
				# 计算当前像素到目标点的距离
				y_dist = (m - self.a + 1) - dy
				x_dist = (n - self.a + 1) - dx

				# Lanczos权重（可分离）
				wy = self._lanczos_weight(y_dist)
				wx = self._lanczos_weight(x_dist)
				weights[m, n] = wy * wx

		# 归一化权重（防止数值误差，确保权重和为1）
		weight_sum = np.sum(weights)
		if weight_sum > 0:
			weights = weights / weight_sum

		# 累加所有像素的贡献
		result = np.zeros(3, dtype=np.float32)

		for m in range(tap):
			for n in range(tap):
				# 边界保护后的像素坐标
				py = max(0, min(y0 - self.a + 1 + m, src_h - 1))
				px = max(0, min(x0 - self.a + 1 + n, src_w - 1))

				pixel = src_img[py, px].astype(np.float32)
				result += weights[m, n] * pixel

		# 饱和到 0-255
		result = np.clip(result, 0, 255).astype(np.uint8)

		return result

	def process(self, input_image):
		# 处理图像
		#
		# Args:
		#     input_image: PIL Image 对象 或 numpy 数组 (H, W, C)
		#
		# Returns:
		#     numpy.ndarray: 缩放后的图像数组 (H, W, C), uint8

		# 统一转换为 numpy 数组
		if isinstance(input_image, Image.Image):
			src_img = np.array(input_image)
		else:
			src_img = input_image.copy()

		# 确保是 RGB 格式
		if len(src_img.shape) == 2:
			# 灰度图转 RGB
			src_img = np.stack([src_img] * 3, axis=-1)
		elif len(src_img.shape) == 3 and src_img.shape[2] == 4:
			# RGBA 转 RGB（丢弃 Alpha）
			src_img = src_img[:, :, :3]

		src_h, src_w, src_c = src_img.shape
		assert src_c == 3, f"只支持 RGB 图像，当前通道数: {src_c}"

		# 计算缩放比例和目标尺寸
		scale_x, scale_y, dst_w, dst_h = self._compute_scale(src_w, src_h)

		print(f"原图尺寸: {src_w}x{src_h}")
		print(f"目标尺寸: {dst_w}x{dst_h}")
		print(f"缩放比例: X={scale_x:.4f}, Y={scale_y:.4f}")
		print(f"Lanczos核: a={self.a} ({self.tap_size}x{self.tap_size} 像素)")

		# 创建输出图像
		dst_img = np.zeros((dst_h, dst_w, 3), dtype=np.uint8)

		# 逐像素处理（模拟 RTL 的逐像素处理）
		for dst_y in range(dst_h):
			# 计算 Y 方向原图浮点坐标
			src_y = self._compute_src_coord(dst_y, scale_y, src_h)

			for dst_x in range(dst_w):
				# 计算 X 方向原图浮点坐标
				src_x = self._compute_src_coord(dst_x, scale_x, src_w)

				# Lanczos插值
				dst_img[dst_y, dst_x] = self._lanczos_interpolate(src_img, src_y, src_x)

		return dst_img

	def process_and_save(self, input_path, output_dir="results"):
		# 处理图像并保存结果（带可视化对比）
		#
		# Args:
		#     input_path: 输入图像路径
		#     output_dir: 输出目录
		#
		# Returns:
		#     output_path: 输出图像路径

		# 读取图像
		input_img   = Image.open(input_path).convert("RGB")
		input_array = np.array(input_img)

		# 处理
		output_array = self.process(input_img)

		# 创建对比图
		comparison = self._create_comparison(input_array, output_array)

		# 生成输出文件名
		base_name = os.path.splitext(os.path.basename(input_path))[0]

		if self.target_size:
			size_tag = f"{self.target_size[0]}x{self.target_size[1]}"
		else:
			size_tag = f"scale{self.scale_factor:.2f}"

		timestamp = datetime.now().strftime("%m%d_%H%M%S")

		# 保存结果
		os.makedirs(output_dir, exist_ok=True)

		# 1. 仅放大后的图像
		output_path = os.path.join(output_dir,
			f"{base_name}_lanczos_float_a{self.a}_{size_tag}_{timestamp}.png")
		Image.fromarray(output_array).save(output_path)
		print(f"输出图像: {output_path}")

		# 2. 对比图
		comparison_path = os.path.join(output_dir,
			f"{base_name}_lanczos_float_a{self.a}_{size_tag}_compare_{timestamp}.png")
		comparison.save(comparison_path)
		print(f"对比图像: {comparison_path}")

		return output_path

	def _create_comparison(self, src_img, dst_img, max_display_width=1920):
		# 创建原图和结果图的对比可视化
		#
		# 如果结果图太大，会缩放到适合屏幕显示的大小

		src_h, src_w = src_img.shape[:2]
		dst_h, dst_w = dst_img.shape[:2]

		# 计算显示缩放（如果结果图太宽）
		total_width = src_w + dst_w
		if total_width > max_display_width:
			display_scale = max_display_width / total_width
			display_src_w = int(src_w * display_scale)
			display_src_h = int(src_h * display_scale)
			display_dst_w = int(dst_w * display_scale)
			display_dst_h = int(dst_h * display_scale)

			# 使用 PIL 进行显示缩放（保持可视化清晰）
			src_display = np.array(Image.fromarray(src_img).resize(
				(display_src_w, display_src_h), Image.NEAREST))
			dst_display = np.array(Image.fromarray(dst_img).resize(
				(display_dst_w, display_dst_h), Image.NEAREST))
		else:
			src_display = src_img
			dst_display = dst_img
			display_src_h, display_src_w = src_h, src_w
			display_dst_h, display_dst_w = dst_h, dst_w

		# 创建对比画布
		gap          = 20  # 中间间隔
		label_height = 40  # 标签高度

		canvas_height = max(display_src_h, display_dst_h) + label_height
		canvas_width  = display_src_w + gap + display_dst_w

		from PIL import ImageDraw, ImageFont
		canvas = Image.new('RGB', (canvas_width, canvas_height), (40, 40, 40))
		draw   = ImageDraw.Draw(canvas)

		# 尝试使用默认字体
		try:
			font = ImageFont.truetype("arial.ttf", 20)
		except:
			font = ImageFont.load_default()

		# 粘贴原图
		canvas.paste(Image.fromarray(src_display), (0, label_height))
		draw.text((10, 10), f"Original: {src_w}x{src_h}", fill=(255, 255, 255), font=font)

		# 粘贴结果图
		canvas.paste(Image.fromarray(dst_display), (display_src_w + gap, label_height))
		draw.text((display_src_w + gap + 10, 10),
			f"Lanczos (Float, a={self.a}): {dst_w}x{dst_h}", fill=(255, 255, 255), font=font)

		return canvas


def compare_with_opencv(input_path, scale_factor=None, target_size=None):
	# 与 OpenCV 结果对比
	#
	# 验证本实现与 OpenCV INTER_LANCZOS4 的一致性
	# 注意：OpenCV使用Lanczos-4，与我们的a=2或a=3略有不同

	import cv2

	# 读取图像
	img     = cv2.imread(input_path)
	img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	src_h, src_w = img_rgb.shape[:2]

	# 确定目标尺寸
	if target_size:
		dst_w, dst_h = target_size
	else:
		dst_w = int(src_w * scale_factor)
		dst_h = int(src_h * scale_factor)

	print("=" * 50)
	print("与 OpenCV 对比验证")
	print("=" * 50)

	# 本实现结果（使用Lanczos-3）
	scaler     = LanczosFloat(target_size=(dst_w, dst_h), a=3)
	our_result = scaler.process(img_rgb)

	# OpenCV 结果（INTER_LANCZOS4）
	cv_result = cv2.resize(img_rgb, (dst_w, dst_h), interpolation=cv2.INTER_LANCZOS4)

	# 对比（按通道比较，取最大差异）
	diff           = np.abs(our_result.astype(int) - cv_result.astype(int))
	max_diff       = np.max(diff)
	mean_diff      = np.mean(diff)
	mismatch_count = np.sum(np.any(diff > 0, axis=2))

	print(f"像素差异统计:")
	print(f"  最大差异: {max_diff}")
	print(f"  平均差异: {mean_diff:.4f}")
	print(f"  不一致像素数: {mismatch_count} / {dst_w * dst_h}")

	if max_diff == 0:
		print("✓ 与 OpenCV 结果完全一致！")
	elif max_diff <= 1:
		print("○ 与 OpenCV 结果基本一致（误差<=1）")
	else:
		print("✗ 存在差异，检查 Lanczos 核或边界处理")
		# 显示差异位置（只取第一个通道的坐标）
		diff_mask = np.any(diff > 0, axis=2)
		y, x = np.where(diff_mask)
		if len(y) > 0:
			print(f"  差异位置示例 (前5个):")
			for i in range(min(5, len(y))):
				print(f"    ({y[i]}, {x[i]}): ours={our_result[y[i], x[i]]}, opencv={cv_result[y[i], x[i]]}")

	return max_diff == 0


# ==================== 主程序入口 ====================

if __name__ == "__main__":
	# 配置参数（直接修改这里，无需命令行）

	# 方式1：使用缩放比例（放大2倍）
	SCALE_FACTOR = 2.5
	TARGET_SIZE  = None  # (1920, 1080)

	# 方式2：指定目标尺寸
	# SCALE_FACTOR = None
	# TARGET_SIZE  = (1920, 1080)  # (width, height)

	# Lanczos核大小
	# a=2: Lanczos-2，4x4像素，较快
	# a=3: Lanczos-3，6x6像素，质量更高
	# a=4: Lanczos-4，8x8像素，质量更高，速度更慢
	# LANCZOS_A = 2
	LANCZOS_A = 3

	# 输入图像路径（放在 test_images 目录下）
	# INPUT_IMAGE = "test_images/test_pattern.png"
	# INPUT_IMAGE = "test_images/侧脸纹理.png"
	# INPUT_IMAGE = "test_images/文字球.png"
	# INPUT_IMAGE = "test_images/综合文字图形.png"
	# INPUT_IMAGE = "test_images/烤鸭美女眼耳发.png"
	INPUT_IMAGE = "test_images/烤鸭美女眼.png"


	# 如果测试图片不存在，创建一个标准测试图
	if not os.path.exists(INPUT_IMAGE):
		print(f"测试图片不存在，创建标准测试图: {INPUT_IMAGE}")
		os.makedirs("test_images", exist_ok=True)

		# 创建测试图：包含渐变、棋盘格、色块
		test_w, test_h = 320, 240
		test_img       = np.zeros((test_h, test_w, 3), dtype=np.uint8)

		# 水平渐变（测试插值连续性）
		for x in range(test_w):
			test_img[:, x, 0] = int(255 * x / test_w)          # R 渐变
			test_img[:, x, 1] = int(255 * (1 - x / test_w))    # G 反向渐变
			test_img[:, x, 2] = 128                            # B 固定

		# 叠加棋盘格（测试高频响应）
		checker_size = 20
		for y in range(0, test_h, checker_size):
			for x in range(0, test_w, checker_size):
				if ((x // checker_size) + (y // checker_size)) % 2 == 0:
					test_img[y:y+checker_size, x:x+checker_size] = [255, 255, 255]

		Image.fromarray(test_img).save(INPUT_IMAGE)
		print(f"已创建测试图: {test_w}x{test_h}")

	# 执行处理
	print("=" * 50)
	print("Lanczos插值 - 浮点实现")
	print("=" * 50)

	scaler = LanczosFloat(
		scale_factor = SCALE_FACTOR,
		target_size  = TARGET_SIZE,
		a            = LANCZOS_A
	)

	output_path = scaler.process_and_save(INPUT_IMAGE, output_dir="results")

	print("=" * 50)
	print("OpenCV 对比验证")
	print("=" * 50)
	compare_with_opencv(INPUT_IMAGE, SCALE_FACTOR, TARGET_SIZE)
