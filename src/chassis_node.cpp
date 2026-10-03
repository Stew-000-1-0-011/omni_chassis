/// @file chassis_node.cpp
/// N輪オムニの足回りドライバ。
///
/// - `cmd_vel` (機体座標系の vx, vy, omega) を受け取る
/// - 逆運動学で各車輪の角速度にし、速度・加速度の制限をかける
/// - `gear_ratio` を掛けて、車輪ごとの目標角速度 [rad/s] (std_msgs/Float64) を出す。
///   モータドライバとの通信 (例: mini_shirasu_ros) はこのノードの仕事ではない
///
/// 運動学と制限は ROS 非依存 (kinematics.cpp)。ここは ROS の入出力とパラメータだけを見る。

#include <chrono>
#include <cstddef>
#include <format>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "omni_chassis/kinematics.hpp"

namespace {
	using omni_chassis::BodyTwist;
	using omni_chassis::Wheel;
	using omni_chassis::WheelLimits;

	/// 車輪1つぶんの、運動学以外の設定
	struct Motor {
		/// 目標角速度を出すトピック
		std::string topic{};
		/// 出力軸の回転 / 車輪の回転。負にすると回転方向を反転する
		double gear_ratio{1.0};
	};

	class ChassisNode final : public rclcpp::Node {
	public:
		ChassisNode() : rclcpp::Node("chassis_node") {
			// --- 起動時のみ ---
			const auto cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
			const bool cmd_vel_stamped = this->declare_parameter<bool>("cmd_vel_stamped", false);
			const double rate = this->declare_parameter<double>("control_rate", 50.0);

			this->load_wheels();

			// --- 実行中に変えられる ---
			this->declare_parameter<double>("cmd_timeout", 0.2);
			this->declare_parameter<double>("limits.max_wheel_speed", 0.0);
			this->declare_parameter<double>("limits.max_wheel_accel", 0.0);
			this->read_runtime_params();
			this->param_cb_ = this->add_post_set_parameters_callback(
				[this](const std::vector<rclcpp::Parameter>&) { this->read_runtime_params(); }
			);

			if (!(rate > 0.0)) {
				throw std::invalid_argument("control_rate must be positive");
			}
			this->dt_ = 1.0 / rate;

			// --- 出力 ---
			for (const auto& m : this->motors_) {
				this->target_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(m.topic, 10));
			}
			this->wheel_speed_pub_ =
				this->create_publisher<std_msgs::msg::Float64MultiArray>("~/wheel_speeds", 10);

			// --- 入力 ---
			if (cmd_vel_stamped) {
				this->cmd_stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
					cmd_vel_topic,
					10,
					[this](const geometry_msgs::msg::TwistStamped& m) { this->on_cmd_vel(m.twist); }
				);
			} else {
				this->cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
					cmd_vel_topic,
					10,
					[this](const geometry_msgs::msg::Twist& m) { this->on_cmd_vel(m); }
				);
			}

			this->previous_speeds_.assign(this->wheels_.size(), 0.0);
			this->timer_ = this->create_wall_timer(
				std::chrono::duration<double>(this->dt_), [this] { this->on_timer(); }
			);

			RCLCPP_INFO(this->get_logger(), "%zu wheels, %.1f Hz", this->wheels_.size(), rate);
		}

		~ChassisNode() override {
			// 終了時は止める (届くかは保証できないので、下流のタイムアウトも併用すること)
			try {
				this->publish_targets(std::vector<double>(this->wheels_.size(), 0.0));
			} catch (...) {
			}
		}

	private:
		void load_wheels() {
			const auto x = this->declare_parameter<std::vector<double>>("wheels.x", std::vector<double>{});
			const auto y = this->declare_parameter<std::vector<double>>("wheels.y", std::vector<double>{});
			const auto angle =
				this->declare_parameter<std::vector<double>>("wheels.angle_deg", std::vector<double>{});
			const auto radius = this->declare_parameter<std::vector<double>>("wheels.radius", std::vector<double>{});
			const auto topic =
				this->declare_parameter<std::vector<std::string>>("wheels.topic", std::vector<std::string>{});
			const auto gear =
				this->declare_parameter<std::vector<double>>("wheels.gear_ratio", std::vector<double>{});

			const std::size_t n = x.size();
			if (n == 0) {
				throw std::invalid_argument("wheels.x is empty; set the wheel layout in the parameter file");
			}
			const auto check = [n](const std::size_t size, const char* name) {
				if (size != n) {
					throw std::invalid_argument(
						std::format("{} has {} elements, but wheels.x has {}", name, size, n)
					);
				}
			};
			check(y.size(), "wheels.y");
			check(angle.size(), "wheels.angle_deg");
			check(radius.size(), "wheels.radius");
			check(topic.size(), "wheels.topic");
			check(gear.size(), "wheels.gear_ratio");

			for (std::size_t i = 0; i < n; ++i) {
				if (!(radius[i] > 0.0)) {
					throw std::invalid_argument(std::format("wheels.radius[{}] must be positive", i));
				}
				if (topic[i].empty()) {
					throw std::invalid_argument(std::format("wheels.topic[{}] is empty", i));
				}
				if (gear[i] == 0.0) {
					throw std::invalid_argument(std::format("wheels.gear_ratio[{}] must not be zero", i));
				}
				for (std::size_t j = 0; j < i; ++j) {
					if (topic[i] == topic[j]) {
						throw std::invalid_argument(
							std::format("wheels.topic[{}] and wheels.topic[{}] are both '{}'", j, i, topic[i])
						);
					}
				}
				this->wheels_.push_back(Wheel{
					.x = x[i],
					.y = y[i],
					.angle = angle[i] * std::numbers::pi / 180.0,
					.radius = radius[i],
				});
				this->motors_.push_back(Motor{.topic = topic[i], .gear_ratio = gear[i]});
			}

			if (!omni_chassis::is_holonomic(this->wheels_)) {
				throw std::invalid_argument(
					"the wheel layout cannot produce vx, vy and omega independently; check wheels.*"
				);
			}
		}

		void read_runtime_params() {
			this->cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
			this->limits_ = WheelLimits{
				.max_speed = this->get_parameter("limits.max_wheel_speed").as_double(),
				.max_accel = this->get_parameter("limits.max_wheel_accel").as_double(),
			};
		}

		void on_cmd_vel(const geometry_msgs::msg::Twist& m) {
			this->cmd_ = BodyTwist{.vx = m.linear.x, .vy = m.linear.y, .omega = m.angular.z};
			this->cmd_received_ = this->now();
		}

		void on_timer() {
			// cmd_vel が途切れたら目標をゼロにする (加速度制限で減速しながら止まる)
			BodyTwist cmd{};
			if (this->cmd_received_ && (this->now() - *this->cmd_received_).seconds() <= this->cmd_timeout_) {
				cmd = this->cmd_;
			}

			const auto target = omni_chassis::wheel_speeds(this->wheels_, cmd);
			const auto limited =
				omni_chassis::limit_wheel_speeds(target, this->previous_speeds_, this->dt_, this->limits_);
			this->previous_speeds_ = limited;
			this->publish_targets(limited);
		}

		/// 車輪角速度 [rad/s] -> 出力 [rad/s]
		void publish_targets(const std::vector<double>& wheel_speeds) {
			for (std::size_t i = 0; i < this->motors_.size(); ++i) {
				std_msgs::msg::Float64 msg{};
				msg.data = wheel_speeds[i] * this->motors_[i].gear_ratio;
				this->target_pubs_[i]->publish(msg);
			}

			std_msgs::msg::Float64MultiArray debug{};
			debug.data = wheel_speeds;
			this->wheel_speed_pub_->publish(debug);
		}


		std::vector<Wheel> wheels_{};
		std::vector<Motor> motors_{};

		double dt_{};
		double cmd_timeout_{};
		WheelLimits limits_{};

		BodyTwist cmd_{};
		std::optional<rclcpp::Time> cmd_received_{};
		std::vector<double> previous_speeds_{};

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_{};
		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_{};
		std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> target_pubs_{};
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_pub_{};
		rclcpp::TimerBase::SharedPtr timer_{};
		rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ChassisNode>());
	rclcpp::shutdown();
	return 0;
}
