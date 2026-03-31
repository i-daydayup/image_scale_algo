#!/usr/bin/env python3
# B样条插值算法 - 浮点实现
# B-spline Interpolation - Floating Point Implementation
#
# 用途：
#     - 算法画质参考（浮点黄金参考）
#     - 超高质量图像缩放（C2连续，无振铃）
#     - 生成测试向量和可视化结果
#
# 坐标映射：
#     Backward Mapping + Center-Aligned
#     src_pos = (dst_idx + 0.5) / scale - 0.5
#
# 插值公式：
#     使用三次B样条基函数（Cubic B-spline），计算周围16个像素的加权平均
#     B3(x) = (2/3) - x² + (1/2)|x|³          , 0 ≤ |x| < 1
#           = (2-|x|)³ / 6                   , 1 ≤ |x| < 2
#           = 0                               , |x| ≥ 2
#
# 特性：
#     - C2连续（二阶导数连续），最平滑
#     - 无负旁瓣，无振铃效应
#     - 相比Catmull-Rom更柔和
#
# 作者：
# 版本：1.0

import numpy as np
from PIL import Image
import os
from datetime import datetime


class BSplineFloat:
	# B样条插值缩放器（浮点实现）
	#
	# 参数:
	#     scale_factor: 缩放因子，>1为放大，<1为缩小
	#     target_size: 目标尺寸 (width, height)，与 scale_factor 二选一
	#     prefilter: 是否启用预滤波（消除插值带来的模糊）

	def __init__(self, scale_factor=None, target_size=None, prefilter=False):
		# 初始化缩放器
		#
		# Args:
		#     scale_factor: 缩放比例（dst/src），例如 2.0 表示放大2倍
		#     target_size: 目标尺寸元组 (width, height)
		#     prefilter: 是否启用预滤波，True可改善锐利度但增加计算
		#
		# Raises:
		#     ValueError: 同时指定或同时未指定 scale_factor 和 target_size

		if (scale_factor is None and target_size is None) or (scale_factor is not None and target_size is not None):
			raise ValueError("请指定 scale_factor 或 target_size 其中一个")

		self.scale_factor = scale_factor
		self.target_size  = target_size
		self.prefilter    = prefilter

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

	def _bspline_weight(self, t):
		# 三次B样条基函数（Cubic B-spline kernel）
		#
		# Args:
		#     t: 距离中心点的距离（浮点）
		#
		# Returns:
		#     float: 权重值（非负）

		t_abs = abs(t)

		if t_abs < 1:
			# B3(x) = (2/3) - x² + (1/2)|x|³
			return (2.0/3.0) - t_abs**2 + 0.5 * t_abs**3
		elif t_abs < 2:
			# B3(x) = (2-|x|)³ / 6
			return ((2 - t_abs) ** 3) / 6.0
		else:
			return 0.0

	def _cubic_prefilter(self, img):
		# 三次B样条预滤波（补偿插值带来的模糊）
		#
		# 使用IIR滤波器实现逆B样条变换
		# 参考：M. Unser et al. "B-spline signal processing: Part II"

		if not self.prefilter:
			return img.astype(np.float32)

		# 使用scipy的spline滤波（更稳定）
		try:
			from scipy.ndimage import spline_filter
			# spline_filter默认就是B样条预滤波，order=3为三次
			result = spline_filter(img.astype(np.float64), order=3, mode='mirror')
			return result.astype(np.float32)
		except ImportError:
			# 如果没有scipy，使用简化的近似方法
			print("警告：scipy不可用，使用简化的预滤波近似")
			return self._approximate_prefilter(img)

	def _approximate_prefilter(self, img):
		# 简化的预滤波近似（使用高通滤波）
		# 效果不如完整IIR，但不会全黑
		from scipy.ndimage import gaussian_filter
		src = img.astype(np.float32)
		# Unsharp mask：原图 + (原图 - 高斯模糊) * 0.5
		blurred = gaussian_filter(src, sigma=0.8, mode='mirror')
		result = src + 0.5 * (src - blurred)
		return np.clip(result, 0, 255).astype(np.float32)

	def _bspline_interpolate(self, src_img, src_y, src_x):
		# 对单个像素进行B样条插值
		#
		# Args:
		#     src_img: 源图像数组（可能经过预滤波）
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

		# 计算16个像素的权重（4x4窗口）
		weights = np.zeros((4, 4), dtype=np.float32)

		for m in range(-1, 3):  # Y方向: -1, 0, 1, 2
			for n in range(-1, 3):  # X方向: -1, 0, 1, 2
				# 计算到目标点的距离
				y_dist = m - dy
				x_dist = n - dx

				# B样条权重（非负）
				wy = self._bspline_weight(y_dist)
				wx = self._bspline_weight(x_dist)
				weights[m + 1, n + 1] = wy * wx

		# 归一化权重（防止数值误差）
		weight_sum = np.sum(weights)
		if weight_sum > 0:
			weights = weights / weight_sum

		# 累加16个像素的贡献
		result = np.zeros(3, dtype=np.float32)

		for m in range(-1, 3):
			for n in range(-1, 3):
				# 边界保护后的像素坐标
				py = max(0, min(y0 + m, src_h - 1))
				px = max(0, min(x0 + n, src_w - 1))

				pixel = src_img[py, px].astype(np.float32)
				result += weights[m + 1, n + 1] * pixel

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

		# 预滤波（可选）
		if self.prefilter:
			print("应用B样条预滤波...")
			src_img = self._cubic_prefilter(src_img)
			print("预滤波完成")
		else:
			src_img = src_img.astype(np.float32)

		# 计算缩放比例和目标尺寸
		scale_x, scale_y, dst_w, dst_h = self._compute_scale(src_w, src_h)

		print(f"原图尺寸: {src_w}x{src_h}")
		print(f"目标尺寸: {dst_w}x{dst_h}")
		print(f"缩放比例: X={scale_x:.4f}, Y={scale_y:.4f}")
		print(f"预滤波: {'启用' if self.prefilter else '禁用'}")

		# 创建输出图像
		dst_img = np.zeros((dst_h, dst_w, 3), dtype=np.uint8)

		# 逐像素处理
		for dst_y in range(dst_h):
			# 计算 Y 方向原图浮点坐标
			src_y = self._compute_src_coord(dst_y, scale_y, src_h)

			for dst_x in range(dst_w):
				# 计算 X 方向原图浮点坐标
				src_x = self._compute_src_coord(dst_x, scale_x, src_w)

				# B样条插值
				dst_img[dst_y, dst_x] = self._bspline_interpolate(src_img, src_y, src_x)

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

		pf_tag = "pf" if self.prefilter else "nopf"
		timestamp = datetime.now().strftime("%m%d_%H%M%S")

		# 保存结果
		os.makedirs(output_dir, exist_ok=True)

		# 1. 仅放大后的图像
		output_path = os.path.join(output_dir,
			f"{base_name}_bspline_float_{pf_tag}_{size_tag}_{timestamp}.png")
		Image.fromarray(output_array).save(output_path)
		print(f"输出图像: {output_path}")

		# 2. 对比图
		comparison_path = os.path.join(output_dir,
			f"{base_name}_bspline_float_{pf_tag}_{size_tag}_compare_{timestamp}.png")
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
		pf_text = "(PF)" if self.prefilter else ""
		draw.text((display_src_w + gap + 10, 10),
			f"B-spline {pf_text}: {dst_w}x{dst_h}", fill=(255, 255, 255), font=font)

		return canvas


def compare_bspline_bicubic(input_path, scale_factor=None, target_size=None):
	# 对比B样条与双三次的结果差异

	from bicubic_float import BicubicFloat

	# 读取图像
	img = Image.open(input_path).convert("RGB")
	img_array = np.array(img)

	print("=" * 60)
	print("B样条 vs 双三次 对比")
	print("=" * 60)

	# B样条（无预滤波）
	print("\n--- B样条（无预滤波）---")
	bspline_np = BSplineFloat(scale_factor=scale_factor, target_size=target_size, prefilter=False)
	result_bspline = bspline_np.process(img_array)

	# B样条（有预滤波）
	print("\n--- B样条（预滤波）---")
	bspline_pf = BSplineFloat(scale_factor=scale_factor, target_size=target_size, prefilter=True)
	result_bspline_pf = bspline_pf.process(img_array)

	# 双三次
	print("\n--- 双三次（Catmull-Rom）---")
	bicubic = BicubicFloat(scale_factor=scale_factor, target_size=target_size, a=-0.5)
	result_bicubic = bicubic.process(img_array)

	# 对比统计
	print("\n" + "=" * 60)
	print("与双三次的差异统计")
	print("=" * 60)

	configs = [
		("B-spline (no PF)", result_bspline),
		("B-spline (PF)", result_bspline_pf),
	]

	for name, result in configs:
		diff = np.abs(result.astype(int) - result_bicubic.astype(int))
		max_diff = np.max(diff)
		mean_diff = np.mean(diff)
		print(f"{name:20s}: 最大差异={max_diff:3d}, 平均差异={mean_diff:.4f}")

	return result_bspline, result_bicubic


# ==================== 主程序入口 ====================

if __name__ == "__main__":
	# 配置参数（直接修改这里，无需命令行）

	# 方式1：使用缩放比例（放大2倍）
	SCALE_FACTOR = 2.5
	TARGET_SIZE  = None  # (1920, 1080)

	# 方式2：指定目标尺寸
	# SCALE_FACTOR = None
	# TARGET_SIZE  = (1920, 1080)  # (width, height)

	# 预滤波选项
	# True: 启用预滤波，改善锐利度（较慢，需要scipy）
	# False: 禁用预滤波，直接插值（较快，更平滑）
	# PREFILTER = False
	PREFILTER = True

	# 输入图像路径（放在 test_images 目录下）
	# INPUT_IMAGE = "test_images/test_pattern.png"
	# INPUT_IMAGE = "test_images/侧脸纹理.png"
	# INPUT_IMAGE = "test_images/文字球.png"
	INPUT_IMAGE = "test_images/综合文字图形.png"
	# INPUT_IMAGE = "test_images/烤鸭美女眼耳发.png"
	# INPUT_IMAGE = "test_images/烤鸭美女眼.png"

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
	print("B样条插值 - 浮点实现")
	print("=" * 50)

	scaler = BSplineFloat(
		scale_factor = SCALE_FACTOR,
		target_size  = TARGET_SIZE,
		prefilter    = PREFILTER
	)

	output_path = scaler.process_and_save(INPUT_IMAGE, output_dir="results")

	# 对比测试（可选）
	# print("\n")
	# compare_bspline_bicubic(INPUT_IMAGE, SCALE_FACTOR, TARGET_SIZE)
