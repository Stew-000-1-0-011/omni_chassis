#pragma once
/// @file kinematics.hpp
/// N輪オムニの逆運動学と車輪速度の制限。ROS非依存。
///
/// 座標系は機体座標系 (x 前、y 左、z 上)。角度は rad、長さは m、角速度は rad/s。

#include <span>
#include <vector>

namespace omni_chassis {
	/// 機体速度 (機体座標系)
	struct BodyTwist {
		double vx{};
		double vy{};
		double omega{};
	};

	/// 車輪1つ。
	///
	/// `angle` は車輪が正転したときに接地点が機体を押す向き (機体座標系)。
	/// オムニホイールのフリーローラ方向はこれに直交するとみなす。
	struct Wheel {
		double x{};
		double y{};
		double angle{};
		double radius{};
	};

	struct WheelLimits {
		/// 車輪角速度の上限 [rad/s]。<= 0 で無効
		double max_speed{};
		/// 車輪角加速度の上限 [rad/s^2]。<= 0 で無効
		double max_accel{};
	};

	/// 機体速度 -> 各車輪の角速度 [rad/s] (正転が正)。
	///
	/// 接地点の速度 v = (vx - omega * y, vy + omega * x) の、車輪の駆動方向成分を半径で割る。
	auto wheel_speeds(std::span<const Wheel> wheels, const BodyTwist& twist) -> std::vector<double>;

	/// 各車輪の角速度 -> 機体速度 (最小二乗)。車輪が滑らない前提でのオドメトリ用。
	/// `is_holonomic(wheels)` が偽なら結果は意味を持たない。
	auto body_twist(std::span<const Wheel> wheels, std::span<const double> speeds) -> BodyTwist;

	/// 車輪配置で (vx, vy, omega) をすべて独立に出せるか。
	/// 逆運動学の行列 J (N x 3) について J^T J が正則かどうかを見る。
	auto is_holonomic(std::span<const Wheel> wheels) -> bool;

	/// 車輪速度に制限をかける。
	///
	/// どちらの制限も全車輪に同じ倍率をかけるので、機体の進む向きと回転の比は変わらない
	/// (逆運動学は線形なので、前回値との内分は機体速度の内分に対応する)。
	///
	/// 1. 速度: 最大の |目標| が `max_speed` を超えたら全体を縮める
	/// 2. 加速度: 前回値からの変化の最大が `max_accel * dt` を超えたら変化を縮める
	///
	/// `previous` と `target` の要素数は同じであること。
	auto limit_wheel_speeds(
		std::span<const double> target,
		std::span<const double> previous,
		double dt,
		const WheelLimits& limits
	) -> std::vector<double>;
}
