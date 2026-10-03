/// @file chassis_node.cpp
/// N輪オムニの足回りドライバ。
///
/// - `cmd_vel` (機体座標系の vx, vy, omega) を受け取る
/// - 逆運動学で各車輪の角速度にし、速度・加速度の制限をかける
/// - 減速比と単位を掛けて robomas_plugins の `robomas_target<N>` に出す
/// - robomas_bridge とつながったとき (と `~/enable`) に、`robomas_frame` で
///   各モータを速度制御モードに設定する
///
/// 運動学と制限は ROS 非依存 (kinematics.cpp)。ここは ROS の入出力とパラメータだけを見る。

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
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
#include <robomas_plugins/msg/robomas_frame.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "omni_chassis/kinematics.hpp"

namespace {
	using omni_chassis::BodyTwist;
	using omni_chassis::Wheel;
	using omni_chassis::WheelLimits;
	using robomas_plugins::msg::RobomasFrame;
	using robomas_plugins::msg::RobomasTarget;

	/// robomas_plugins の RobomasFrame.mode
	namespace robomas_mode {
		constexpr std::uint8_t disable = 0;
		constexpr std::uint8_t velocity = 1;
	}

	/// 車輪1つぶんの、運動学以外の設定
	struct Motor {
		/// robomas のモータ番号 (1..8)。`robomas_target<index>` に出す
		int index{};
		/// モータ回転 / 車輪回転。負にすると回転方向を反転する
		double gear_ratio{1.0};
		/// C620 (M3508) なら true、C610 (M2006) なら false
		bool c620{true};
	};

	class ChassisNode final : public rclcpp::Node {
	public:
		ChassisNode() : rclcpp::Node("chassis_node") {
			// --- 起動時のみ ---
			const auto cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
			const bool cmd_vel_stamped = this->declare_parameter<bool>("cmd_vel_stamped", false);
			const double rate = this->declare_parameter<double>("control_rate", 50.0);
			const auto frame_topic = this->declare_parameter<std::string>("robomas_frame_topic", "robomas_frame");
			const auto target_prefix =
				this->declare_parameter<std::string>("robomas_target_prefix", "robomas_target");
			this->enabled_ = this->declare_parameter<bool>("enable_on_start", true);

			this->load_wheels();

			this->target_scale_ = this->declare_parameter<double>("target_per_motor_rad_s", 1.0);
			this->velkp_ = this->declare_parameter<double>("motor.velkp", 0.15);
			this->velki_ = this->declare_parameter<double>("motor.velki", 9.0);
			this->temp_limit_ = static_cast<std::uint8_t>(
				std::clamp<std::int64_t>(this->declare_parameter<std::int64_t>("motor.temp_limit", 50), 0, 255)
			);

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
			rclcpp::PublisherOptions frame_options{};
			// robomas_bridge が後から起動したり再起動したりしても設定が届くように、
			// つながるたびにモータのモードを送り直す
			frame_options.event_callbacks.matched_callback = [this](const rclcpp::MatchedInfo& info) {
				if (info.current_count_change > 0) {
					RCLCPP_INFO(this->get_logger(), "robomas_frame subscriber connected; configuring motors");
					this->send_mode(this->enabled_ ? robomas_mode::velocity : robomas_mode::disable);
				}
			};
			this->frame_pub_ = this->create_publisher<RobomasFrame>(frame_topic, 10, frame_options);

			for (const auto& m : this->motors_) {
				this->target_pubs_.push_back(
					this->create_publisher<RobomasTarget>(std::format("{}{}", target_prefix, m.index), 10)
				);
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

			this->enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
				"~/enable",
				[this](
					const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
					std::shared_ptr<std_srvs::srv::SetBool::Response> response
				) {
					this->set_enabled(request->data);
					response->success = true;
					response->message = request->data ? "enabled" : "disabled";
				}
			);

			this->previous_speeds_.assign(this->wheels_.size(), 0.0);
			this->timer_ = this->create_wall_timer(
				std::chrono::duration<double>(this->dt_), [this] { this->on_timer(); }
			);

			RCLCPP_INFO(
				this->get_logger(),
				"%zu wheels, %.1f Hz, %s",
				this->wheels_.size(),
				rate,
				this->enabled_ ? "enabled" : "disabled"
			);
		}

		~ChassisNode() override {
			// 終了時はモータを止めて脱力させる (届くかは保証できないので、ファーム側の
			// タイムアウトも併用すること)
			try {
				this->publish_targets(std::vector<double>(this->wheels_.size(), 0.0));
				this->send_mode(robomas_mode::disable);
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
			const auto motor =
				this->declare_parameter<std::vector<std::int64_t>>("wheels.motor", std::vector<std::int64_t>{});
			const auto gear =
				this->declare_parameter<std::vector<double>>("wheels.gear_ratio", std::vector<double>{});
			const auto c620 = this->declare_parameter<std::vector<bool>>("wheels.c620", std::vector<bool>{});

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
			check(motor.size(), "wheels.motor");
			check(gear.size(), "wheels.gear_ratio");
			check(c620.size(), "wheels.c620");

			for (std::size_t i = 0; i < n; ++i) {
				if (!(radius[i] > 0.0)) {
					throw std::invalid_argument(std::format("wheels.radius[{}] must be positive", i));
				}
				if (motor[i] < 1 || motor[i] > 8) {
					throw std::invalid_argument(std::format("wheels.motor[{}] must be in 1..8", i));
				}
				if (gear[i] == 0.0) {
					throw std::invalid_argument(std::format("wheels.gear_ratio[{}] must not be zero", i));
				}
				for (std::size_t j = 0; j < i; ++j) {
					if (motor[i] == motor[j]) {
						throw std::invalid_argument(
							std::format("wheels.motor[{}] and wheels.motor[{}] are both {}", j, i, motor[i])
						);
					}
				}
				this->wheels_.push_back(Wheel{
					.x = x[i],
					.y = y[i],
					.angle = angle[i] * std::numbers::pi / 180.0,
					.radius = radius[i],
				});
				this->motors_.push_back(Motor{
					.index = static_cast<int>(motor[i]),
					.gear_ratio = gear[i],
					.c620 = c620[i],
				});
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
			if (!this->enabled_) {
				return;
			}

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

		/// 車輪角速度 [rad/s] -> robomas の目標値
		void publish_targets(const std::vector<double>& wheel_speeds) {
			for (std::size_t i = 0; i < this->motors_.size(); ++i) {
				RobomasTarget msg{};
				msg.target = static_cast<float>(wheel_speeds[i] * this->motors_[i].gear_ratio * this->target_scale_);
				this->target_pubs_[i]->publish(msg);
			}

			std_msgs::msg::Float64MultiArray debug{};
			debug.data = wheel_speeds;
			this->wheel_speed_pub_->publish(debug);
		}

		void send_mode(const std::uint8_t mode) {
			for (const auto& m : this->motors_) {
				RobomasFrame frame{};
				frame.motor = static_cast<std::uint8_t>(m.index - 1);
				frame.mode = mode;
				frame.c620 = m.c620;
				frame.temp = this->temp_limit_;
				frame.velkp = static_cast<float>(this->velkp_);
				frame.velki = static_cast<float>(this->velki_);
				this->frame_pub_->publish(frame);
			}
		}

		void set_enabled(const bool enabled) {
			if (enabled == this->enabled_) {
				return;
			}
			if (enabled) {
				this->previous_speeds_.assign(this->wheels_.size(), 0.0);
				this->send_mode(robomas_mode::velocity);
			} else {
				this->publish_targets(std::vector<double>(this->wheels_.size(), 0.0));
				this->send_mode(robomas_mode::disable);
			}
			this->enabled_ = enabled;
			RCLCPP_INFO(this->get_logger(), "%s", enabled ? "enabled" : "disabled");
		}

		std::vector<Wheel> wheels_{};
		std::vector<Motor> motors_{};
		double target_scale_{1.0};
		double velkp_{};
		double velki_{};
		std::uint8_t temp_limit_{};

		double dt_{};
		double cmd_timeout_{};
		WheelLimits limits_{};

		BodyTwist cmd_{};
		std::optional<rclcpp::Time> cmd_received_{};
		std::vector<double> previous_speeds_{};
		bool enabled_{true};

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_{};
		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_{};
		rclcpp::Publisher<RobomasFrame>::SharedPtr frame_pub_{};
		std::vector<rclcpp::Publisher<RobomasTarget>::SharedPtr> target_pubs_{};
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_pub_{};
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_{};
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
