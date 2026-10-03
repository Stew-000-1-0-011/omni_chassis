/// @file kinematics_test.cpp
/// 逆運動学と車輪速度の制限のテスト (ROS不要)。

#include <cmath>
#include <cstdio>
#include <numbers>
#include <source_location>
#include <vector>

#include "omni_chassis/kinematics.hpp"

namespace {
	using omni_chassis::BodyTwist;
	using omni_chassis::Wheel;
	using omni_chassis::WheelLimits;

	int failures = 0;

	void check(const bool ok, const char* what, const std::source_location loc = std::source_location::current()) {
		if (!ok) {
			std::printf("FAIL %s:%u: %s\n", loc.file_name(), static_cast<unsigned>(loc.line()), what);
			++failures;
		}
	}

	auto near(const double a, const double b, const double tol = 1e-9) -> bool {
		return std::abs(a - b) <= tol;
	}

	auto deg(const double d) -> double { return d * std::numbers::pi / 180.0; }

	/// 4輪オムニ X 配置 (config/chassis_node.yaml の例と同じ)
	auto x_layout() -> std::vector<Wheel> {
		return {
			{.x = 0.2, .y = 0.2, .angle = deg(135), .radius = 0.05},
			{.x = -0.2, .y = 0.2, .angle = deg(225), .radius = 0.05},
			{.x = -0.2, .y = -0.2, .angle = deg(315), .radius = 0.05},
			{.x = 0.2, .y = -0.2, .angle = deg(45), .radius = 0.05},
		};
	}

	/// 3輪オムニ (120度間隔、接線方向に駆動)
	auto three_wheel_layout() -> std::vector<Wheel> {
		std::vector<Wheel> wheels{};
		for (const double a : {0.0, 120.0, 240.0}) {
			wheels.push_back({
				.x = 0.15 * std::cos(deg(a)),
				.y = 0.15 * std::sin(deg(a)),
				.angle = deg(a + 90.0),
				.radius = 0.04,
			});
		}
		return wheels;
	}

	void test_pure_rotation() {
		// 接線方向に駆動する配置では、その場回転で全輪が同じ速度になる
		const auto w = omni_chassis::wheel_speeds(x_layout(), {.omega = 1.0});
		const double expected = 0.2 * std::numbers::sqrt2 / 0.05;
		for (const double s : w) {
			check(near(s, expected), "pure rotation: every wheel turns at omega * R / r");
		}
	}

	void test_translation() {
		// 前進: 前輪 (135, 45 deg) と後輪 (225, 315 deg) で符号が分かれ、大きさは v cos45 / r
		const auto w = omni_chassis::wheel_speeds(x_layout(), {.vx = 1.0});
		const double m = std::cos(deg(45)) / 0.05;
		check(near(w[0], -m), "forward: wheel 0");
		check(near(w[1], -m), "forward: wheel 1");
		check(near(w[2], m), "forward: wheel 2");
		check(near(w[3], m), "forward: wheel 3");
	}

	void test_contact_velocity() {
		// どの配置・どの機体速度でも、車輪の周速 = 接地点の速度の駆動方向成分
		const BodyTwist t{.vx = 0.3, .vy = -0.7, .omega = 2.1};
		for (const auto& layout : {x_layout(), three_wheel_layout()}) {
			const auto w = omni_chassis::wheel_speeds(layout, t);
			for (std::size_t i = 0; i < layout.size(); ++i) {
				const auto& wh = layout[i];
				const double cx = t.vx - t.omega * wh.y;
				const double cy = t.vy + t.omega * wh.x;
				const double rim = cx * std::cos(wh.angle) + cy * std::sin(wh.angle);
				check(near(w[i] * wh.radius, rim), "rim speed matches contact point velocity");
			}
		}
	}

	void test_round_trip() {
		// 逆運動学 -> 最小二乗の順運動学で元に戻る
		const BodyTwist t{.vx = -0.4, .vy = 1.2, .omega = -0.8};
		for (const auto& layout : {x_layout(), three_wheel_layout()}) {
			const auto w = omni_chassis::wheel_speeds(layout, t);
			const auto back = omni_chassis::body_twist(layout, w);
			check(near(back.vx, t.vx, 1e-9) && near(back.vy, t.vy, 1e-9) && near(back.omega, t.omega, 1e-9),
				"body_twist inverts wheel_speeds");
		}
	}

	void test_holonomic() {
		check(omni_chassis::is_holonomic(x_layout()), "X layout is holonomic");
		check(omni_chassis::is_holonomic(three_wheel_layout()), "3-wheel layout is holonomic");

		// 全輪が同じ向きに駆動すると横に動けない
		std::vector<Wheel> parallel = x_layout();
		for (auto& w : parallel) {
			w.angle = 0.0;
		}
		check(!omni_chassis::is_holonomic(parallel), "all wheels parallel is not holonomic");

		// 2輪では足りない
		auto two = x_layout();
		two.resize(2);
		check(!omni_chassis::is_holonomic(two), "two wheels are not enough");
	}

	void test_speed_limit_keeps_direction() {
		const auto layout = x_layout();
		const BodyTwist t{.vx = 3.0, .vy = 1.0, .omega = 2.0};
		const auto target = omni_chassis::wheel_speeds(layout, t);
		const std::vector<double> zero(layout.size(), 0.0);
		const auto out = omni_chassis::limit_wheel_speeds(target, zero, 0.0, {.max_speed = 10.0});

		double peak = 0.0;
		for (const double s : out) {
			peak = std::max(peak, std::abs(s));
		}
		check(near(peak, 10.0), "fastest wheel is clamped to max_speed");

		// 機体速度に戻すと、元の機体速度の定数倍になっている
		const auto back = omni_chassis::body_twist(layout, out);
		const double k = back.vx / t.vx;
		check(k > 0.0 && k < 1.0, "scaled down");
		check(near(back.vy, k * t.vy, 1e-9) && near(back.omega, k * t.omega, 1e-9), "direction is preserved");
	}

	void test_accel_limit() {
		const std::vector<double> prev{1.0, -1.0, 0.0, 2.0};
		const std::vector<double> target{11.0, -1.0, 5.0, 2.0};
		const auto out = omni_chassis::limit_wheel_speeds(target, prev, 0.01, {.max_accel = 100.0});
		// 最大の変化 (10) が 100 * 0.01 = 1 に縮む -> 全変化を 1/10 倍
		check(near(out[0], 2.0), "accel: largest change is limited");
		check(near(out[1], -1.0), "accel: unchanged wheel stays");
		check(near(out[2], 0.5), "accel: other wheels scale by the same factor");
		check(near(out[3], 2.0), "accel: unchanged wheel stays");

		// 制限に掛からなければそのまま
		const auto free = omni_chassis::limit_wheel_speeds(target, prev, 0.01, WheelLimits{});
		check(free == target, "no limits: passthrough");
	}
}

auto main() -> int {
	test_pure_rotation();
	test_translation();
	test_contact_velocity();
	test_round_trip();
	test_holonomic();
	test_speed_limit_keeps_direction();
	test_accel_limit();

	if (failures == 0) {
		std::printf("all kinematics tests passed\n");
		return 0;
	}
	std::printf("%d failure(s)\n", failures);
	return 1;
}
