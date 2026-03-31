#!/usr/bin/env python3
# Lanczos插值算法 - 定点实现
# Lanczos Interpolation - Fixed Point Implementation
#
# 用途：
#     - RTL 实现的定点化参考模型
#     - 评估定点数位宽对画质的影响
#     - 生成定点数格式的测试向量
#
# 坐标映射（定点化）：
#     与最近邻/双线性相同，使用Qm.n格式
#
# 权重定点化：
#     Lanczos权重范围约[-0.25, 1.0]，使用Q2.14格式
#     1.0 对应 16384 (1 << 14)
#     权重和可能因定点误差偏离1.0，需要归一化
#
# 作者：
# 版本：1.0

import numpy as np
from PIL import Image
import os
from datetime import datetime


class LanczosFixed:
	# Lanczos插值缩放器（定点实现）
	#
	# 定点格式：
	# - 坐标: Qm.n (如 Q8.8)
	# - 权重: Q2.14 (范围[-2, 1.999]，1.0对应16384)
	#
	# 参数:
	#     a: Lanczos核大小（2或3）
	#     int_bits/frac_bits: 坐标定点格式
	#     weight_int_bits/weight_frac_bits: 权重定点格式

	def __init__(self, int_bits=8, frac_bits=8, weight_int_bits=2, weight_frac_bits=14,
	             scale_factor=None, target_size=None, a=3):
		# 初始化定点缩放器
		#
		# Args:
		#     int_bits: 坐标整数位宽
		#     frac_bits: 坐标小数位宽
		#     weight_int_bits: 权重整数位宽（推荐2）
		#     weight_frac_bits: 权重小数位宽（推荐14）
		#     a: Lanczos核大小（2或3）
		#     scale_factor: 缩放比例（可选）
		#     target_size: 目标尺寸 (width, height)（可选）

		if int_bits < 1 or frac_bits < 0:
			raise ValueError(f"坐标位宽错误: int_bits={int_bits}, frac_bits={frac_bits}")

		if weight_int_bits < 1 or weight_frac_bits < 0:
			raise ValueError(f"权重位宽错误: weight_int_bits={weight_int_bits}, weight_frac_bits={weight_frac_bits}")

		self.m     = int_bits
		self.n     = frac_bits
		self.w_int = weight_int_bits
		self.w_frac = weight_frac_bits

		self.scale_coord  = 1 << frac_bits     # 坐标缩放因子 2^n
		self.scale_weight = 1 << weight_frac_bits  # 权重缩放因子 2^w_frac

		# 定点数范围
		self.coord_max = (1 << (int_bits + frac_bits - 1)) - 1
		self.coord_min = -(1 << (int_bits + frac_bits - 1))

		# 权重定点范围（有符号）
		self.weight_max = (1 << (weight_int_bits + weight_frac_bits - 1)) - 1
		self.weight_min = -(1 << (weight_int_bits + weight_frac_bits - 1))

		self.a        = a
		self.tap_size = 2 * a

		if (scale_factor is None and target_size is None) or (scale_factor is not None and target_size is not None):
			raise ValueError("请指定 scale_factor 或 target_size 其中一个")

		self.scale_factor = scale_factor
		self.target_size  = target_size

		print(f"定点格式: 坐标Q{int_bits}.{frac_bits}, 权重Q{weight_int_bits}.{weight_frac_bits}")
		print(f"  坐标缩放: 2^{frac_bits} = {self.scale_coord}")
		print(f"  权重缩放: 2^{weight_frac_bits} = {self.scale_weight}")
		print(f"  Lanczos核: a={a} ({2*a}x{2*a} 像素)")

	def _float_to_fixed_coord(self, x):
		# 浮点数转定点坐标
		fixed = np.round(x * self.scale_coord).astype(np.int64)
		return np.clip(fixed, self.coord_min, self.coord_max)

	def _float_to_fixed_weight(self, x):
		# 浮点数转定点权重（支持负值）
		fixed = np.round(x * self.scale_weight).astype(np.int64)
		return np.clip(fixed, self.weight_min, self.weight_max)

	def _sinc_float(self, x):
		# 浮点sinc函数（用于预计算权重表）
		if abs(x) < 1e-6:
			return 1.0
		return np.sin(np.pi * x) / (np.pi * x)

	def _lanczos_weight_float(self, t):
		# 浮点Lanczos权重（用于预计算）
		t_abs = abs(t)
		if t_abs >= self.a:
			return 0.0
		return self._sinc_float(t) * self._sinc_float(t / self.a)

	def _compute_inv_scale_fixed(self, src_size, dst_size):
		# 计算定点化的逆缩放比例
		inv_scale_float = src_size / dst_size
		inv_scale_fixed = self._float_to_fixed_coord(inv_scale_float)
		return inv_scale_fixed

	def _compute_src_coord_fixed(self, dst_idx, inv_scale_fixed, src_size):
		# 计算目标像素对应的原图坐标（定点实现）
		#
		# Returns:
		#     y_int: 整数坐标
		#     dy_fixed: Y方向小数部分（Q0.n格式，范围0~(2^n-1)）

		half_n = 1 << (self.n - 1) if self.n > 0 else 0

		# 计算定点坐标
		mult = (np.int64(2 * dst_idx + 1) * inv_scale_fixed)
		src_pos_fixed = (mult >> 1) - half_n

		# 饱和处理
		src_pos_fixed = np.clip(src_pos_fixed, self.coord_min, self.coord_max)

		# 边界保护
		src_pos_clamped = max(0, min(src_pos_fixed, (src_size - 1) << self.n))

		# 分离整数和小数部分
		y_int = src_pos_clamped >> self.n
		frac_mask = (1 << self.n) - 1
		dy_frac = src_pos_clamped & frac_mask

		return int(y_int), int(dy_frac)

	def _lanczos_interpolate_fixed(self, src_img, y_int, dy_fixed, x_int, dx_fixed):
		# 定点Lanczos插值
		#
		# Args:
		#     dy_fixed/dx_fixed: Q0.n格式的小数部分 (0 ~ 2^n-1)

		src_h, src_w = src_img.shape[:2]
		tap = self.tap_size

		# 将小数部分转换为浮点（用于查权重表）
		dy = dy_fixed / self.scale_coord
		dx = dx_fixed / self.scale_coord

		# 计算权重（浮点计算后转定点，RTL中会用查找表）
		weights = np.zeros((tap, tap), dtype=np.int64)

		for m in range(tap):
			for n in range(tap):
				y_dist = (m - self.a + 1) - dy
				x_dist = (n - self.a + 1) - dx

				wy = self._lanczos_weight_float(y_dist)
				wx = self._lanczos_weight_float(x_dist)
				w = wy * wx

				weights[m, n] = self._float_to_fixed_weight(w)

		# 定点归一化：权重和应该接近 scale_weight
		weight_sum = np.sum(weights)
		if weight_sum != 0:
			# 使用定点除法归一化：weight = weight * scale_weight / weight_sum
			# 这样保证权重和为 scale_weight
			weights = (weights * self.scale_weight) // weight_sum

		# 累加像素（定点计算）
		result = np.zeros(3, dtype=np.int64)

		for m in range(tap):
			for n in range(tap):
				py = max(0, min(y_int - self.a + 1 + m, src_h - 1))
				px = max(0, min(x_int - self.a + 1 + n, src_w - 1))

				pixel = src_img[py, px].astype(np.int64)
				w = weights[m, n]

				# pixel (uint8) * weight (Q2.14) -> 需要右移 w_frac 位
				result += (pixel * w) >> self.w_frac

		# 结果已经在像素值范围内，饱和处理
		result = np.clip(result, 0, 255).astype(np.uint8)

		return result

	def process(self, input_image, debug=False):
		# 处理图像（定点实现）

		# 统一转换为 numpy 数组
		if isinstance(input_image, Image.Image):
			src_img = np.array(input_image)
		else:
			src_img = input_image.copy()

		# 确保是 RGB 格式
		if len(src_img.shape) == 2:
			src_img = np.stack([src_img] * 3, axis=-1)
		elif len(src_img.shape) == 3 and src_img.shape[2] == 4:
			src_img = src_img[:, :, :3]

		src_h, src_w, src_c = src_img.shape
		assert src_c == 3, f"只支持 RGB 图像，当前通道数: {src_c}"

		# 计算目标尺寸
		if self.target_size:
			dst_w, dst_h = self.target_size
		else:
			dst_w = int(src_w * self.scale_factor)
			dst_h = int(src_h * self.scale_factor)

		# 计算定点化的逆缩放比例
		inv_scale_x_fixed = self._compute_inv_scale_fixed(src_w, dst_w)
		inv_scale_y_fixed = self._compute_inv_scale_fixed(src_h, dst_h)

		print(f"原图尺寸: {src_w}x{src_h}")
		print(f"目标尺寸: {dst_w}x{dst_h}")

		# 创建输出图像
		dst_img = np.zeros((dst_h, dst_w, 3), dtype=np.uint8)

		# 逐像素处理
		for dst_y in range(dst_h):
			y_int, dy_fixed = self._compute_src_coord_fixed(dst_y, inv_scale_y_fixed, src_h)

			for dst_x in range(dst_w):
				x_int, dx_fixed = self._compute_src_coord_fixed(dst_x, inv_scale_x_fixed, src_w)

				dst_img[dst_y, dst_x] = self._lanczos_interpolate_fixed(
					src_img, y_int, dy_fixed, x_int, dx_fixed)

		return dst_img

	def process_and_save(self, input_path, output_dir="results", debug=False):
		# 处理图像并保存结果

		# 读取图像
		input_img   = Image.open(input_path).convert("RGB")
		input_array = np.array(input_img)

		# 处理
		output_array = self.process(input_img, debug=debug)

		# 生成输出文件名
		base_name = os.path.splitext(os.path.basename(input_path))[0]

		if self.target_size:
			size_tag = f"{self.target_size[0]}x{self.target_size[1]}"
		else:
			size_tag = f"scale{self.scale_factor:.2f}"

		# 定点格式标记
		fixed_tag = f"Q{self.m}_{self.n}_W{self.w_int}_{self.w_frac}"

		timestamp = datetime.now().strftime("%m%d_%H%M%S")

		# 保存结果
		os.makedirs(output_dir, exist_ok=True)

		output_path = os.path.join(output_dir,
			f"{base_name}_lanczos_fixed_{fixed_tag}_a{self.a}_{size_tag}_{timestamp}.png")
		Image.fromarray(output_array).save(output_path)
		print(f"输出图像: {output_path}")

		return output_path


def compare_fixed_configs(input_path, target_size=None, scale_factor=None):
	# 对比不同定点配置的差异

	# 读取图像
	img       = Image.open(input_path).convert("RGB")
	img_array = np.array(img)

	# 不同定点配置 (int_bits, frac_bits, weight_bits, a)
	configs = [
		(8, 8, 2, 14, 2),   # Q8.8坐标 + Q2.14权重 + Lanczos-2
		(8, 8, 2, 14, 3),   # Q8.8坐标 + Q2.14权重 + Lanczos-3
		(4, 12, 2, 14, 2),  # Q4.12坐标 + Q2.14权重 + Lanczos-2
	]

	print("=" * 60)
	print("不同定点配置对比")
	print("=" * 60)

	results = {}

	for int_bits, frac_bits, w_int, w_frac, a in configs:
		print(f"\n测试 Q{int_bits}.{frac_bits} + Q{w_int}.{w_frac} + a={a}:")
		try:
			scaler = LanczosFixed(
				int_bits     = int_bits,
				frac_bits    = frac_bits,
				weight_int_bits  = w_int,
				weight_frac_bits = w_frac,
				a            = a,
				scale_factor = scale_factor,
				target_size  = target_size
			)
			result = scaler.process(img)
			results[f"Q{int_bits}.{frac_bits}_a{a}"] = result
		except Exception as e:
			print(f"  错误: {e}")

	# 如果有浮点参考，进行对比
	from lanczos_float import LanczosFloat
	float_scaler = LanczosFloat(
		scale_factor = scale_factor,
		target_size  = target_size,
		a            = 3
	)
	float_result = float_scaler.process(img)
	results["Float_a3"] = float_result

	# 计算各配置与浮点版本的差异
	print("\n" + "=" * 60)
	print("与浮点版本的差异统计")
	print("=" * 60)

	for name, result in results.items():
		if name != "Float_a3":
			diff           = np.abs(result.astype(int) - float_result.astype(int))
			max_diff       = np.max(diff)
			mean_diff      = np.mean(diff)
			mismatch_count = np.sum(np.any(diff > 0, axis=2))
			print(f"{name:20s}: 最大差异={max_diff:3d}, 平均差异={mean_diff:.4f}, "
				f"不一致像素={mismatch_count}")

	return results


# ==================== 主程序入口 ====================

if __name__ == "__main__":
	# 配置参数

	# 定点格式配置
	INT_BITS  = 8    # 坐标整数位
	FRAC_BITS = 8    # 坐标小数位
	W_INT     = 2    # 权重整数位（Q2.14）
	W_FRAC    = 14   # 权重小数位

	# Lanczos核大小（2或3）
	LANCZOS_A = 3

	# 缩放参数（二选一）
	SCALE_FACTOR = 2.0   # 放大2倍
	TARGET_SIZE  = None  # 或指定 (width, height)

	# 输入图像
	# INPUT_IMAGE = "test_images/test_pattern.png"
	# INPUT_IMAGE = "test_images/侧脸纹理.png"
	# INPUT_IMAGE = "test_images/文字球.png"
	INPUT_IMAGE = "test_images/综合文字图形.png"

	print("=" * 50)
	print("Lanczos插值 - 定点实现")
	print("=" * 50)

	# 创建测试图（如果不存在）
	if not os.path.exists(INPUT_IMAGE):
		print(f"测试图片不存在，请先运行 lanczos_float.py 创建测试图")
		exit(1)

	# 单配置测试
	scaler = LanczosFixed(
		int_bits         = INT_BITS,
		frac_bits        = FRAC_BITS,
		weight_int_bits  = W_INT,
		weight_frac_bits = W_FRAC,
		a                = LANCZOS_A,
		scale_factor     = SCALE_FACTOR,
		target_size      = TARGET_SIZE
	)

	output_path = scaler.process_and_save(
		INPUT_IMAGE,
		output_dir = "results",
		debug      = False
	)

	# 多配置对比（可选）
	# print("\n")
	# compare_fixed_configs(INPUT_IMAGE, TARGET_SIZE, SCALE_FACTOR)
