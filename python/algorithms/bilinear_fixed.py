#!/usr/bin/env python3
# 双线性插值算法 - 定点实现
# Bilinear Interpolation - Fixed Point Implementation
#
# 用途：
#     - RTL 实现的定点化参考模型
#     - 评估定点数位宽（Qm.n）对画质的影响
#     - 生成定点数格式的测试向量
#
# 坐标映射（定点化）：
#     src_pos_fixed = ((dst_idx + 0.5) / scale - 0.5) * 2^n
#
# 插值公式（定点化）：
#     权重使用 Q0.w 格式（纯小数，w 位小数位）
#     result = (P00*w00 + P10*w10 + P01*w01 + P11*w11) >> w
#
# 作者：
# 版本：1.0

import numpy as np
from PIL import Image
import os
from datetime import datetime


class BilinearFixed:
	# 双线性插值缩放器（定点实现）
	#
	# 定点格式 Qm.n：
	# - m: 整数位宽（包含符号位）
	# - n: 小数位宽
	#
	# 权重格式 Q0.w：
	# - 纯小数，范围 [0, 1 - 2^(-w)]
	# - 通常 w=8 或 w=12

	def __init__(self, int_bits=8, frac_bits=8, weight_bits=8, scale_factor=None, target_size=None):
		# 初始化定点缩放器
		#
		# Args:
		#     int_bits: 整数位宽 m（包含符号位）
		#     frac_bits: 坐标小数位宽 n
		#     weight_bits: 权重小数位宽 w（Q0.w）
		#     scale_factor: 缩放比例（可选）
		#     target_size: 目标尺寸 (width, height)（可选）
		#
		# Raises:
		#     ValueError: 位宽配置不合理或参数冲突

		if int_bits < 1 or frac_bits < 0 or weight_bits < 1:
			raise ValueError(f"位宽配置错误: int_bits={int_bits}, frac_bits={frac_bits}, weight_bits={weight_bits}")

		if int_bits + frac_bits > 32:
			print(f"警告: 总位宽 {int_bits + frac_bits} 位，建议使用 32 位以内")

		self.m = int_bits
		self.n = frac_bits
		self.w = weight_bits

		self.scale_coord = 1 << frac_bits   # 2^n，坐标缩放因子
		self.scale_weight = 1 << weight_bits  # 2^w，权重缩放因子

		# 计算定点数范围
		self.max_val = (1 << (int_bits + frac_bits - 1)) - 1
		self.min_val = -(1 << (int_bits + frac_bits - 1))

		if (scale_factor is None and target_size is None) or (scale_factor is not None and target_size is not None):
			raise ValueError("请指定 scale_factor 或 target_size 其中一个")

		self.scale_factor = scale_factor
		self.target_size  = target_size

		print(f"定点格式: Q{int_bits}.{frac_bits}, 权重 Q0.{weight_bits}")
		print(f"  坐标缩放因子: 2^{frac_bits} = {self.scale_coord}")
		print(f"  权重缩放因子: 2^{weight_bits} = {self.scale_weight}")
		print(f"  值域: [{self.min_val}, {self.max_val}]")

	def _float_to_fixed(self, x):
		# 浮点数转定点数（用于坐标）
		fixed = np.round(x * self.scale_coord).astype(np.int64)
		fixed = np.clip(fixed, self.min_val, self.max_val)
		return fixed

	def _float_to_weight(self, x):
		# 浮点数转权重定点数（Q0.w，范围 0~1）
		weight = np.round(x * self.scale_weight).astype(np.int64)
		weight = np.clip(weight, 0, self.scale_weight - 1)
		return weight

	def _fixed_to_float(self, fixed):
		# 定点数转浮点数（用于调试）
		return fixed / self.scale_coord

	def _compute_inv_scale_fixed(self, src_size, dst_size):
		# 计算定点化的逆缩放比例
		inv_scale_float = src_size / dst_size
		inv_scale_fixed = self._float_to_fixed(inv_scale_float)
		return inv_scale_fixed

	def _compute_src_coord_fixed(self, dst_idx, inv_scale_fixed, src_size):
		# 计算目标像素对应的原图坐标（定点实现）
		#
		# 公式推导（与最近邻相同）：
		# src_pos = (dst_idx + 0.5) * inv_scale - 0.5
		# src_pos_fixed = (((2*dst_idx + 1) * inv_scale_fixed) >> 1) - (1 << (n-1))
		#
		# Returns:
		#     src_pos_fixed: 原图像素定点坐标
		#     dy/dx: 小数部分（权重，Q0.w 格式）

		half_n = 1 << (self.n - 1) if self.n > 0 else 0

		# 计算定点坐标
		mult = (np.int64(2 * dst_idx + 1) * inv_scale_fixed)
		src_pos_fixed = (mult >> 1) - half_n

		# 饱和处理
		src_pos_fixed = np.clip(src_pos_fixed, self.min_val, self.max_val)

		# 边界保护后的坐标
		src_pos_clamped = max(0, min(src_pos_fixed, (src_size - 1) << self.n))

		# 分离整数部分和小数部分
		# 整数部分（用于取像素）
		y_int = src_pos_clamped >> self.n
		# 小数部分（用于权重，需要转换到 Q0.w 格式）
		frac_mask = (1 << self.n) - 1
		y_frac = src_pos_clamped & frac_mask  # 0 ~ (2^n - 1)

		# 将小数部分从 Q0.n 转换到 Q0.w
		if self.w == self.n:
			d = y_frac
		elif self.w > self.n:
			d = y_frac << (self.w - self.n)
		else:
			d = y_frac >> (self.n - self.w)

		return int(y_int), int(d)

	def _bilinear_interpolate_fixed(self, src_img, y_int, dy_fixed, x_int, dx_fixed):
		# 对单个像素进行双线性插值（定点实现）
		#
		# Args:
		#     src_img: 源图像数组
		#     y_int: 源图像Y整数坐标
		#     dy_fixed: Y方向小数部分（Q0.w）
		#     x_int: 源图像X整数坐标
		#     dx_fixed: X方向小数部分（Q0.w）
		#
		# Returns:
		#     插值后的像素值 (R, G, B)，uint8

		src_h, src_w = src_img.shape[:2]

		# 边界保护
		y0 = max(0, min(y_int, src_h - 1))
		x0 = max(0, min(x_int, src_w - 1))
		y1 = min(y0 + 1, src_h - 1)
		x1 = min(x0 + 1, src_w - 1)

		# 获取周围4个像素
		p00 = src_img[y0, x0].astype(np.int32)  # 左上
		p10 = src_img[y0, x1].astype(np.int32)  # 右上
		p01 = src_img[y1, x0].astype(np.int32)  # 左下
		p11 = src_img[y1, x1].astype(np.int32)  # 右下

		# 计算定点权重（Q0.w 格式）
		# w00 = (1-dx) * (1-dy) -> ((scale_weight - dx) * (scale_weight - dy)) >> w
		# w10 = dx * (1-dy)    -> (dx * (scale_weight - dy)) >> w
		# w01 = (1-dx) * dy    -> ((scale_weight - dx) * dy) >> w
		# w11 = dx * dy        -> (dx * dy) >> w

		sw = self.scale_weight
		dx = dx_fixed
		dy = dy_fixed

		# 预计算 (1-dx) 和 (1-dy) 的定点表示
		one_minus_dx = sw - dx
		one_minus_dy = sw - dy

		# 计算权重（使用 int64 防止溢出）
		w00 = (np.int64(one_minus_dx) * one_minus_dy) >> self.w
		w10 = (np.int64(dx) * one_minus_dy) >> self.w
		w01 = (np.int64(one_minus_dx) * dy) >> self.w
		w11 = (np.int64(dx) * dy) >> self.w

		# 定点插值计算
		# result = (P00*w00 + P10*w10 + P01*w01 + P11*w11) >> w
		# 像素是 uint8，权重是 Q0.w，乘积是 Q8.w，需要右移 w 位
		result = (np.int64(p00) * w00 + np.int64(p10) * w10 +
		          np.int64(p01) * w01 + np.int64(p11) * w11) >> self.w

		# 饱和到 0-255
		result = np.clip(result, 0, 255).astype(np.uint8)

		return result

	def process(self, input_image, debug=False):
		# 处理图像（定点实现）
		#
		# Args:
		#     input_image: PIL Image 对象 或 numpy 数组 (H, W, C)
		#     debug: 是否输出调试信息
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

		# 逐像素处理（定点计算）
		for dst_y in range(dst_h):
			# 计算 Y 方向原图坐标
			y_int, dy_fixed = self._compute_src_coord_fixed(dst_y, inv_scale_y_fixed, src_h)

			for dst_x in range(dst_w):
				# 计算 X 方向原图坐标
				x_int, dx_fixed = self._compute_src_coord_fixed(dst_x, inv_scale_x_fixed, src_w)

				# 双线性插值（定点）
				dst_img[dst_y, dst_x] = self._bilinear_interpolate_fixed(
					src_img, y_int, dy_fixed, x_int, dx_fixed)

		return dst_img

	def process_and_save(self, input_path, output_dir="results", debug=False):
		# 处理图像并保存结果
		#
		# Args:
		#     input_path: 输入图像路径
		#     output_dir: 输出目录
		#     debug: 是否输出调试信息
		#
		# Returns:
		#     output_path: 输出图像路径

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
		fixed_tag = f"Q{self.m}_{self.n}_w{self.w}"

		timestamp = datetime.now().strftime("%m%d_%H%M%S")

		# 保存结果
		os.makedirs(output_dir, exist_ok=True)

		output_path = os.path.join(output_dir,
			f"{base_name}_bilinear_fixed_{fixed_tag}_{size_tag}_{timestamp}.png")
		Image.fromarray(output_array).save(output_path)
		print(f"输出图像: {output_path}")

		return output_path


def compare_fixed_configs(input_path, target_size=None, scale_factor=None):
	# 对比不同定点配置的差异
	#
	# 评估 Qm.n 位宽和权重精度对最终画质的影响

	# 读取图像
	img       = Image.open(input_path).convert("RGB")
	img_array = np.array(img)

	# 不同定点配置 (int_bits, frac_bits, weight_bits)
	configs = [
		(8, 8, 8),    # Q8.8 + Q0.8 权重
		(8, 8, 12),   # Q8.8 + Q0.12 权重
		(4, 12, 8),   # Q4.12 + Q0.8 权重
		(4, 12, 12),  # Q4.12 + Q0.12 权重
	]

	print("=" * 60)
	print("不同定点配置对比")
	print("=" * 60)

	results = {}

	for int_bits, frac_bits, weight_bits in configs:
		print(f"\n测试 Q{int_bits}.{frac_bits} + Q0.{weight_bits}:")
		try:
			scaler = BilinearFixed(
				int_bits     = int_bits,
				frac_bits    = frac_bits,
				weight_bits  = weight_bits,
				scale_factor = scale_factor,
				target_size  = target_size
			)
			result = scaler.process(img)
			results[f"Q{int_bits}.{frac_bits}_w{weight_bits}"] = result
		except Exception as e:
			print(f"  错误: {e}")

	# 如果有浮点参考，进行对比
	from bilinear_float import BilinearFloat
	float_scaler = BilinearFloat(
		scale_factor = scale_factor,
		target_size  = target_size
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
			mismatch_count = np.sum(diff > 0)
			print(f"{name:20s}: 最大差异={max_diff:3d}, 平均差异={mean_diff:.4f}, "
				f"不一致像素={mismatch_count}")

	return results


# ==================== 主程序入口 ====================

if __name__ == "__main__":
	# 配置参数

	# 定点格式配置
	INT_BITS    = 12   # 整数位
	FRAC_BITS   = 8   # 坐标小数位
	WEIGHT_BITS = 8   # 权重小数位（Q0.8）

	# 缩放参数（二选一）
	SCALE_FACTOR = 2.0   # 放大2倍
	TARGET_SIZE  = None  # 或指定 (width, height)

	# 输入图像
	# INPUT_IMAGE = "test_images/test_pattern.png"
	INPUT_IMAGE = "test_images/绿球右上.png"

	print("=" * 50)
	print("双线性插值 - 定点实现")
	print("=" * 50)

	# 创建测试图（如果不存在）
	if not os.path.exists(INPUT_IMAGE):
		print(f"测试图片不存在，请先运行 bilinear_float.py 创建测试图")
		exit(1)

	# 单配置测试
	scaler = BilinearFixed(
		int_bits     = INT_BITS,
		frac_bits    = FRAC_BITS,
		weight_bits  = WEIGHT_BITS,
		scale_factor = SCALE_FACTOR,
		target_size  = TARGET_SIZE
	)

	output_path = scaler.process_and_save(
		INPUT_IMAGE,
		output_dir = "results",
		debug      = False
	)

	# 多配置对比（可选）
	# print("\n")
	# compare_fixed_configs(INPUT_IMAGE, TARGET_SIZE, SCALE_FACTOR)
