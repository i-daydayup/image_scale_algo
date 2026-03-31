#!/usr/bin/env python3
# B样条插值算法 - 定点实现
# B-spline Interpolation - Fixed Point Implementation
#
# 用途：
#     - RTL 实现的定点化参考模型
#     - 评估定点数位宽对画质的影响
#     - 生成定点数格式的测试向量
#
# 坐标映射（定点化）：
#     与双线性/双三次相同，使用Qm.n格式
#
# B样条权重特性：
#     - 全非负（0 ~ 2/3）
#     - 最大权重在中心处为2/3≈0.6667
#     - 使用Q0.8格式：2/3 ≈ 171/256
#
# 注意：
#     定点版本不包含预滤波（IIR难以定点化）
#
# 作者：
# 版本：1.0

import numpy as np
from PIL import Image
import os
from datetime import datetime


class BSplineFixed:
	# B样条插值缩放器（定点实现）
	#
	# 定点格式：
	# - 坐标: Qm.n (如 Q8.8)
	# - 权重: Q0.w (如 Q0.8)，范围 [0, 1.0]
	#
	# 参数:
	#     a: B样条阶数（固定为3，即三次B样条）
	#     int_bits/frac_bits: 坐标定点格式
	#     weight_frac_bits: 权重小数位宽

	def __init__(self, int_bits=8, frac_bits=8, weight_frac_bits=8,
	             scale_factor=None, target_size=None):
		# 初始化定点缩放器
		#
		# Args:
		#     int_bits: 坐标整数位宽
		#     frac_bits: 坐标小数位宽
		#     weight_frac_bits: 权重小数位宽（Q0.w）
		#     scale_factor: 缩放比例（可选）
		#     target_size: 目标尺寸 (width, height)（可选）

		if int_bits < 1 or frac_bits < 0:
			raise ValueError(f"坐标位宽错误: int_bits={int_bits}, frac_bits={frac_bits}")

		if weight_frac_bits < 1:
			raise ValueError(f"权重位宽错误: weight_frac_bits={weight_frac_bits}")

		self.m = int_bits
		self.n = frac_bits
		self.w = weight_frac_bits

		self.scale_coord  = 1 << frac_bits      # 坐标缩放因子 2^n
		self.scale_weight = 1 << weight_frac_bits  # 权重缩放因子 2^w

		# 定点数范围
		self.coord_max = (1 << (int_bits + frac_bits - 1)) - 1
		self.coord_min = -(1 << (int_bits + frac_bits - 1))

		# B样条为4-tap核（4x4像素）
		self.tap_size = 4

		if (scale_factor is None and target_size is None) or (scale_factor is not None and target_size is not None):
			raise ValueError("请指定 scale_factor 或 target_size 其中一个")

		self.scale_factor = scale_factor
		self.target_size  = target_size

		print(f"定点格式: 坐标Q{int_bits}.{frac_bits}, 权重Q0.{weight_frac_bits}")
		print(f"  坐标缩放: 2^{frac_bits} = {self.scale_coord}")
		print(f"  权重缩放: 2^{weight_frac_bits} = {self.scale_weight}")
		print(f"  B样条: 三次(4-tap)，无预滤波")

	def _float_to_fixed_coord(self, x):
		# 浮点数转定点坐标
		fixed = np.round(x * self.scale_coord).astype(np.int64)
		return np.clip(fixed, self.coord_min, self.coord_max)

	def _float_to_fixed_weight(self, x):
		# 浮点数转定点权重（非负）
		weight = np.round(x * self.scale_weight).astype(np.int64)
		return np.clip(weight, 0, self.scale_weight)

	def _bspline_weight_float(self, t):
		# 浮点B样条权重（用于预计算）
		t_abs = abs(t)

		if t_abs < 1:
			return (2.0/3.0) - t_abs**2 + 0.5 * t_abs**3
		elif t_abs < 2:
			return ((2 - t_abs) ** 3) / 6.0
		else:
			return 0.0

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
		#     dy_fixed: Y方向小数部分（Q0.n格式）

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

	def _bspline_weight_fixed(self, t_fixed):
		# 定点B样条权重计算
		#
		# Args:
		#     t_fixed: 距离（浮点值×scale_weight，即Q0.w格式）
		#
		# Returns:
		#     定点权重（Q0.w格式）

		# 将输入转换回浮点进行计算（RTL中会用查找表替代）
		t = t_fixed / self.scale_weight
		t_abs = abs(t)

		if t_abs >= 2:
			return 0

		# 计算浮点权重
		if t_abs < 1:
			w = (2.0/3.0) - t_abs**2 + 0.5 * t_abs**3
		else:
			w = ((2 - t_abs) ** 3) / 6.0

		# 转回定点
		return int(w * self.scale_weight)

	def _bspline_interpolate_fixed(self, src_img, y_int, dy_fixed, x_int, dx_fixed):
		# 定点B样条插值
		#
		# Args:
		#     dy_fixed/dx_fixed: Q0.n格式的小数部分

		src_h, src_w = src_img.shape[:2]
		tap = self.tap_size

		# 将小数部分转换为权重计算的尺度
		dy = dy_fixed / self.scale_coord
		dx = dx_fixed / self.scale_coord

		# 计算权重（浮点计算后转定点，RTL中用查找表）
		weights = np.zeros((tap, tap), dtype=np.int64)

		for m in range(tap):
			for n in range(tap):
				y_dist = (m - 1) - dy  # B样条中心在m=1处
				x_dist = (n - 1) - dx

				wy = self._bspline_weight_float(y_dist)
				wx = self._bspline_weight_float(x_dist)
				w = wy * wx

				weights[m, n] = self._float_to_fixed_weight(w)

		# 定点归一化：确保权重和为 scale_weight
		weight_sum = np.sum(weights)
		if weight_sum > 0:
			weights = (weights * self.scale_weight) // weight_sum

		# 累加像素（定点计算）
		result = np.zeros(3, dtype=np.int64)

		for m in range(tap):
			for n in range(tap):
				py = max(0, min(y_int - 1 + m, src_h - 1))
				px = max(0, min(x_int - 1 + n, src_w - 1))

				pixel = src_img[py, px].astype(np.int64)
				w = weights[m, n]

				# pixel (uint8) * weight (Q0.w) -> 需要右移 w 位
				result += (pixel * w) >> self.w

		# 结果饱和处理
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

				dst_img[dst_y, dst_x] = self._bspline_interpolate_fixed(
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
		fixed_tag = f"Q{self.m}_{self.n}_W0_{self.w}"

		timestamp = datetime.now().strftime("%m%d_%H%M%S")

		# 保存结果
		os.makedirs(output_dir, exist_ok=True)

		output_path = os.path.join(output_dir,
			f"{base_name}_bspline_fixed_{fixed_tag}_{size_tag}_{timestamp}.png")
		Image.fromarray(output_array).save(output_path)
		print(f"输出图像: {output_path}")

		return output_path


def compare_fixed_configs(input_path, target_size=None, scale_factor=None):
	# 对比不同定点配置的差异

	# 读取图像
	img       = Image.open(input_path).convert("RGB")
	img_array = np.array(img)

	# 不同定点配置 (int_bits, frac_bits, weight_bits)
	configs = [
		(8, 8, 8),    # Q8.8坐标 + Q0.8权重
		(8, 8, 12),   # Q8.8坐标 + Q0.12权重
		(4, 12, 8),   # Q4.12坐标 + Q0.8权重
		(4, 12, 12),  # Q4.12坐标 + Q0.12权重
	]

	print("=" * 60)
	print("不同定点配置对比")
	print("=" * 60)

	results = {}

	for int_bits, frac_bits, w_frac in configs:
		print(f"\n测试 Q{int_bits}.{frac_bits} + Q0.{w_frac}:")
		try:
			scaler = BSplineFixed(
				int_bits         = int_bits,
				frac_bits        = frac_bits,
				weight_frac_bits = w_frac,
				scale_factor     = scale_factor,
				target_size      = target_size
			)
			result = scaler.process(img)
			results[f"Q{int_bits}.{frac_bits}_w{w_frac}"] = result
		except Exception as e:
			print(f"  错误: {e}")

	# 如果有浮点参考，进行对比
	from bspline_float import BSplineFloat
	float_scaler = BSplineFloat(
		scale_factor = scale_factor,
		target_size  = target_size,
		prefilter    = False  # 与定点一致，不用预滤波
	)
	float_result = float_scaler.process(img)
	results["Float"] = float_result

	# 计算各配置与浮点版本的差异
	print("\n" + "=" * 60)
	print("与浮点版本的差异统计")
	print("=" * 60)

	for name, result in results.items():
		if name != "Float":
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
	W_FRAC    = 8    # 权重小数位（Q0.8）

	# 缩放参数（二选一）
	SCALE_FACTOR = 2.0   # 放大2倍
	TARGET_SIZE  = None  # 或指定 (width, height)

	# 输入图像
	INPUT_IMAGE = "test_images/test_pattern.png"

	print("=" * 50)
	print("B样条插值 - 定点实现")
	print("=" * 50)

	# 创建测试图（如果不存在）
	if not os.path.exists(INPUT_IMAGE):
		print(f"测试图片不存在，请先运行 bspline_float.py 创建测试图")
		exit(1)

	# 单配置测试
	scaler = BSplineFixed(
		int_bits         = INT_BITS,
		frac_bits        = FRAC_BITS,
		weight_frac_bits = W_FRAC,
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
