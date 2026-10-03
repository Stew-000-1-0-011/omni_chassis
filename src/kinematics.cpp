#include "omni_chassis/kinematics.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace omni_chassis {
	namespace {
		/// 逆運動学の行列 J の1行 (車輪 i の角速度 = row · (vx, vy, omega))
		auto jacobian_row(const Wheel& w) -> std::array<double, 3> {
			const double c = std::cos(w.angle);
			const double s = std::sin(w.angle);
			return {c / w.radius, s / w.radius, (w.x * s - w.y * c) / w.radius};
		}

		using Mat3 = std::array<std::array<double, 3>, 3>;

		/// J^T J
		auto normal_matrix(const std::span<const Wheel> wheels) -> Mat3 {
			Mat3 m{};
			for (const auto& w : wheels) {
				const auto r = jacobian_row(w);
				for (std::size_t i = 0; i < 3; ++i) {
					for (std::size_t j = 0; j < 3; ++j) {
						m[i][j] += r[i] * r[j];
					}
				}
			}
			return m;
		}

		auto determinant(const Mat3& m) -> double {
			return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
				- m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
				+ m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
		}

		auto max_abs(const std::span<const double> v) -> double {
			double m = 0.0;
			for (const double x : v) {
				m = std::max(m, std::abs(x));
			}
			return m;
		}
	}

	auto wheel_speeds(const std::span<const Wheel> wheels, const BodyTwist& twist) -> std::vector<double> {
		std::vector<double> speeds{};
		speeds.reserve(wheels.size());
		for (const auto& w : wheels) {
			const auto r = jacobian_row(w);
			speeds.push_back(r[0] * twist.vx + r[1] * twist.vy + r[2] * twist.omega);
		}
		return speeds;
	}

	auto body_twist(const std::span<const Wheel> wheels, const std::span<const double> speeds) -> BodyTwist {
		assert(wheels.size() == speeds.size());

		// 正規方程式 (J^T J) t = J^T w をクラメルの公式で解く (3x3 なので十分)
		const Mat3 a = normal_matrix(wheels);
		std::array<double, 3> b{};
		for (std::size_t k = 0; k < wheels.size(); ++k) {
			const auto r = jacobian_row(wheels[k]);
			for (std::size_t i = 0; i < 3; ++i) {
				b[i] += r[i] * speeds[k];
			}
		}

		const double det = determinant(a);
		if (det == 0.0) {
			return {};
		}
		std::array<double, 3> t{};
		for (std::size_t col = 0; col < 3; ++col) {
			Mat3 m = a;
			for (std::size_t row = 0; row < 3; ++row) {
				m[row][col] = b[row];
			}
			t[col] = determinant(m) / det;
		}
		return {t[0], t[1], t[2]};
	}

	auto is_holonomic(const std::span<const Wheel> wheels) -> bool {
		if (wheels.size() < 3) {
			return false;
		}
		// 行列の大きさに対する相対値で見る (半径や配置の単位に依らないように)
		const Mat3 m = normal_matrix(wheels);
		const double scale = m[0][0] + m[1][1] + m[2][2];
		if (!(scale > 0.0)) {
			return false;
		}
		return std::abs(determinant(m)) > 1e-9 * scale * scale * scale;
	}

	auto limit_wheel_speeds(
		const std::span<const double> target,
		const std::span<const double> previous,
		const double dt,
		const WheelLimits& limits
	) -> std::vector<double> {
		assert(target.size() == previous.size());

		std::vector<double> out(target.begin(), target.end());

		if (limits.max_speed > 0.0) {
			const double peak = max_abs(out);
			if (peak > limits.max_speed) {
				const double k = limits.max_speed / peak;
				for (auto& x : out) {
					x *= k;
				}
			}
		}

		if (limits.max_accel > 0.0 && dt > 0.0) {
			std::vector<double> delta(out.size());
			for (std::size_t i = 0; i < out.size(); ++i) {
				delta[i] = out[i] - previous[i];
			}
			const double peak = max_abs(delta);
			const double allowed = limits.max_accel * dt;
			if (peak > allowed) {
				const double k = allowed / peak;
				for (std::size_t i = 0; i < out.size(); ++i) {
					out[i] = previous[i] + k * delta[i];
				}
			}
		}

		return out;
	}
}
