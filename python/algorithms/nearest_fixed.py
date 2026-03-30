#!/usr/bin/env python3
# 最近邻插值算法 - 定点实现
# Nearest Neighbor Interpolation - Fixed Point Implementation
#
# 用途：
#     - RTL 实现的定点化参考模型
#     - 评估定点数位宽（Qm.n）对画质的影响
#     - 生成定点数格式的测试向量
#
# 坐标映射（定点化）：
#     src_pos_fixed = ((dst_idx + 0.5) / scale - 0.5) * 2^n
#                   = ((dst_idx + 0.5) * inv_scale - 0.5) * 2^n
#
#     其中 inv_scale = 1/scale = src_size / dst_size
#
# 取整方式：
#     floor(src_pos + 0.5)
#     定点实现: (src_pos_fixed + (1 << (n-1))) >> n
#
# 作者：
# 版本：1.0

import numpy as np
from PIL import Image
import os
from datetime import datetime


class NearestNeighborFixed:
	# 最近邻插值缩放器（定点实现）
	#
	# 定点格式 Qm.n：
	# - m: 整数位宽（包含符号位）
	# - n: 小数位宽
	# - 总值范围: [-2^(m-1), 2^(m-1) - 2^(-n)]
	# - 精度: 1/2^n
	#
	# 常用配置：
	# - Q8.8:  范围 [-128, 127.996], 精度 0.0039
	# - Q4.12: 范围 [-8, 7.9998], 精度 0.00024
	# - Q12.4: 范围 [-2048, 2047.9375], 精度 0.0625

	def __init__(self, int_bits=8, frac_bits=8, scale_factor=None, target_size=None):
		# 初始化定点缩放器
		#
		# Args:
		#     int_bits: 整数位宽 m（包含符号位）
		#     frac_bits: 小数位宽 n
		#     scale_factor: 缩放比例（可选）
		#     target_size: 目标尺寸 (width, height)（可选）
		#
		# Raises:
		#     ValueError: 位宽配置不合理或参数冲突

		if int_bits < 1 or frac_bits < 0:
			raise ValueError(f"位宽配置错误: int_bits={int_bits}, frac_bits={frac_bits}")

		if int_bits + frac_bits > 32:
			print(f"警告: 总位宽 {int_bits + frac_bits} 位，建议使用 32 位以内")

		self.m     = int_bits
		self.n     = frac_bits
		self.scale = 1 << frac_bits  # 2^n

		# 计算定点数范围
		self.max_val = (1 << (int_bits + frac_bits - 1)) - 1  # 正最大值
		self.min_val = -(1 << (int_bits + frac_bits - 1))     # 负最小值

		if (scale_factor is None and target_size is None) or (scale_factor is not None and target_size is not None):
			raise ValueError("请指定 scale_factor 或 target_size 其中一个")

		self.scale_factor = scale_factor
		self.target_size  = target_size

		print(f"定点格式: Q{int_bits}.{frac_bits}")
		print(f"  整数位: {int_bits}, 小数位: {frac_bits}")
		print(f"  缩放因子: 2^{frac_bits} = {self.scale}")
		print(f"  值域: [{self.min_val}, {self.max_val}]")

	def _float_to_fixed(self, x):
		# 浮点数转定点数
		#
		# Args:
		#     x: 浮点数或 numpy 数组
		#
		# Returns:
		#     定点数表示（整型）

		fixed = np.round(x * self.scale).astype(np.int64)

		# 饱和处理（防止溢出）
		fixed = np.clip(fixed, self.min_val, self.max_val)

		return fixed

	def _fixed_to_float(self, fixed):
		# 定点数转浮点数（用于调试）
		return fixed / self.scale

	def _compute_scale_fixed(self, src_w, src_h):
		# 计算定点化的缩放比例
		if self.target_size is not None:
			tgt_w, tgt_h = self.target_size
			# 定点数计算：scale_x = tgt_w / src_w
			scale_x = self._float_to_fixed(tgt_w / src_w)
			scale_y = self._float_to_fixed(tgt_h / src_h)
			dst_w   = int(tgt_w)
			dst_h   = int(tgt_h)
		else:
			scale_x = scale_y = self._float_to_fixed(self.scale_factor)
			dst_w   = int(src_w * self.scale_factor)
			dst_h   = int(src_h * self.scale_factor)

		return scale_x, scale_y, dst_w, dst_h

	def _compute_inv_scale_fixed(self, src_size, dst_size):
		# 计算定点化的逆缩放比例
		#
		# inv_scale = src_size / dst_size (浮点)
		# inv_scale_fixed = round(inv_scale * 2^n)

		inv_scale_float = src_size / dst_size
		inv_scale_fixed = self._float_to_fixed(inv_scale_float)
		return inv_scale_fixed

	def _nearest_round_fixed(self, pos_fixed):
		# 定点数最近邻取整
		#
		# 浮点: floor(pos + 0.5)
		# 定点: (pos_fixed + 2^(n-1)) >> n
		#
		# 注意：这里直接右移相当于向下取整

		# 加上 0.5 的定点表示 (2^(n-1))
		half = 1 << (self.n - 1) if self.n > 0 else 0
		return (pos_fixed + half) >> self.n

	def _compute_src_coord_fixed(self, dst_idx, inv_scale_fixed, src_size):
		# 计算目标像素对应的原图坐标（定点实现）
		#
		# 公式推导：
		# src_pos = (dst_idx + 0.5) * inv_scale - 0.5
		#
		# 定点化（保持精度）：
		# src_pos_fixed = ((dst_idx + 0.5) * inv_scale - 0.5) * 2^n
		#               = ((2*dst_idx + 1) * inv_scale - 1) * 2^(n-1)
		#               = (((2*dst_idx + 1) * inv_scale_fixed) >> 1) - (1 << (n-1))
		#
		# Args:
		#     dst_idx: 目标像素索引（整数）
		#     inv_scale_fixed: 逆缩放比例的定点表示（inv_scale * 2^n）
		#     src_size: 原图尺寸（用于边界检查）
		#
		# Returns:
		#     src_idx: 原图像素索引（整数，已边界裁剪）

		# 定点数计算
		half_n = 1 << (self.n - 1) if self.n > 0 else 0

		# 计算 (2*dst_idx + 1) * inv_scale_fixed，使用 int64 防止溢出
		mult = (np.int64(2 * dst_idx + 1) * inv_scale_fixed)

		# 右移 1 位（除以 2），然后减去 0.5 的定点表示
		src_pos_fixed = (mult >> 1) - half_n

		# 饱和处理（防止定点数溢出）
		src_pos_fixed = np.clip(src_pos_fixed, self.min_val, self.max_val)

		# 最近邻取整（floor(src_pos + 0.5) 的定点实现）
		src_idx = self._nearest_round_fixed(src_pos_fixed)

		# 边界裁剪
		src_idx = max(0, min(src_idx, src_size - 1))

		return int(src_idx)

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
			scale_x      = dst_w / src_w
			scale_y      = dst_h / src_h
		else:
			dst_w   = int(src_w * self.scale_factor)
			dst_h   = int(src_h * self.scale_factor)
			scale_x = scale_y = self.scale_factor

		# 计算定点化的逆缩放比例（用于坐标计算）
		inv_scale_x_fixed = self._compute_inv_scale_fixed(src_w, dst_w)
		inv_scale_y_fixed = self._compute_inv_scale_fixed(src_h, dst_h)

		print(f"原图尺寸: {src_w}x{src_h}")
		print(f"目标尺寸: {dst_w}x{dst_h}")
		print(f"缩放比例: X={scale_x:.4f}, Y={scale_y:.4f}")
		print(f"逆缩放定点: inv_scale_x={inv_scale_x_fixed}, inv_scale_y={inv_scale_y_fixed}")

		# 调试：打印前几个像素的坐标映射
		if debug:
			print("\n调试信息（前5个像素的坐标映射）:")
			for i in range(min(5, dst_w)):
				src_idx = self._compute_src_coord_fixed(i, inv_scale_x_fixed, src_w)
				# 计算浮点参考值
				src_pos_float = (i + 0.5) / scale_x - 0.5
				print(f"  dst_x={i}: src_pos_float={src_pos_float:.4f}, "
					f"src_idx_fixed={src_idx}")

		# 创建输出图像
		dst_img = np.zeros((dst_h, dst_w, 3), dtype=np.uint8)

		# 逐像素处理（定点计算）
		for dst_y in range(dst_h):
			# 计算 Y 方向原图坐标
			src_y = self._compute_src_coord_fixed(dst_y, inv_scale_y_fixed, src_h)

			for dst_x in range(dst_w):
				# 计算 X 方向原图坐标
				src_x = self._compute_src_coord_fixed(dst_x, inv_scale_x_fixed, src_w)

				# 像素复制
				dst_img[dst_y, dst_x] = src_img[src_y, src_x]

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
		fixed_tag = f"Q{self.m}_{self.n}"

		timestamp = datetime.now().strftime("%m%d_%H%M%S")

		# 保存结果
		os.makedirs(output_dir, exist_ok=True)

		output_path = os.path.join(output_dir,
			f"{base_name}_nearest_fixed_{fixed_tag}_{size_tag}_{timestamp}.png")
		Image.fromarray(output_array).save(output_path)
		print(f"输出图像: {output_path}")

		return output_path


def compare_fixed_configs(input_path, target_size=None, scale_factor=None):
	# 对比不同定点配置的差异
	#
	# 评估 Qm.n 位宽对最终画质的影响

	# 读取图像
	img       = Image.open(input_path).convert("RGB")
	img_array = np.array(img)

	# 不同定点配置
	configs = [
		(8, 8),   # Q8.8: 范围大，精度中等
		(4, 12),  # Q4.12: 范围小，精度高
		(12, 4),  # Q12.4: 范围大，精度低
		(2, 14),  # Q2.14: 范围很小，精度很高
	]

	print("=" * 60)
	print("不同定点配置对比")
	print("=" * 60)

	results = {}

	for int_bits, frac_bits in configs:
		print(f"\n测试 Q{int_bits}.{frac_bits}:")
		try:
			scaler = NearestNeighborFixed(
				int_bits     = int_bits,
				frac_bits    = frac_bits,
				scale_factor = scale_factor,
				target_size  = target_size
			)
			result = scaler.process(img)
			results[f"Q{int_bits}.{frac_bits}"] = result
		except Exception as e:
			print(f"  错误: {e}")

	# 如果有浮点参考，进行对比
	from nearest_float import NearestNeighborFloat
	float_scaler = NearestNeighborFloat(
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
			print(f"{name:10s}: 最大差异={max_diff:3d}, 平均差异={mean_diff:.4f}, "
				f"不一致像素={mismatch_count}")

	return results


# ==================== 主程序入口 ====================

if __name__ == "__main__":
	# 配置参数

	# 定点格式配置
	INT_BITS  = 8   # 整数位
	FRAC_BITS = 8   # 小数位

	# 缩放参数（二选一）
	SCALE_FACTOR = 2.0   # 放大2倍
	TARGET_SIZE  = None  # 或指定 (width, height)

	# 输入图像
	INPUT_IMAGE = "test_images/test_pattern.png"

	print("=" * 50)
	print("最近邻插值 - 定点实现")
	print("=" * 50)

	# 创建测试图（如果不存在）
	if not os.path.exists(INPUT_IMAGE):
		print(f"测试图片不存在，请先运行 nearest_float.py 创建测试图")
		exit(1)

	# 单配置测试
	scaler = NearestNeighborFixed(
		int_bits     = INT_BITS,
		frac_bits    = FRAC_BITS,
		scale_factor = SCALE_FACTOR,
		target_size  = TARGET_SIZE
	)

	output_path = scaler.process_and_save(
		INPUT_IMAGE,
		output_dir = "results",
		debug      = True  # 输出调试信息
	)

	# 多配置对比（可选）
	# print("\n")
	# compare_fixed_configs(INPUT_IMAGE, TARGET_SIZE, SCALE_FACTOR)
