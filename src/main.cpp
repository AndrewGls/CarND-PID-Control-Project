#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>

// This define was added to fix the running of PID controller with simulator on Windows 10.
// Uncomment it to run PID on Windows.
//#define  use_ipv4

// Uncomment this define to start to train PID controller.
#define enable_tuning_mode


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

namespace
{
	enum class Mode { driving, tuning };

	class Timer
	{
	public:
		using high_resolution_clock = std::chrono::high_resolution_clock;
		using ms = std::chrono::milliseconds;

		Timer(bool run = true) {
			if (run)
				start();
		}

		void start() {
			start_ = high_resolution_clock::now();
		}

		ms duration() {
			high_resolution_clock::time_point now = high_resolution_clock::now();
			ms dur = std::chrono::duration_cast<ms>(now - start_);
			start_ = now;
			return dur;
		}

	private:
		high_resolution_clock::time_point start_;
	};

	class PIDTuning
	{
		// throttle: 20
		// P.I.D. {2.1, 0, 15.9}

		// throttle: 30
		// P.I.D. {0.1, 0, 10.}
		// P.I.D. {1.09, 0, 27.}

		static constexpr double track_distance_ = 2900.;
		static constexpr double err_thresh_ = 6.;
		static constexpr double inc_step = 1.1;
		static constexpr double dec_step = 0.9;
	public:
		PIDTuning(PID& pid, double maxThrottle = 0.3, Mode mode = Mode::driving);

		bool isTuning() const { return mode_ == Mode::tuning; }
		Mode getMode() const { return mode_; }
		double getBestError() const { return best_err_; }
		double getCurrThrottle() const { return curThrottle_; }

		double integrateVelocity(double speed, Timer::ms duration);
		bool trackPassedOrCrash(bool isCrash);
		void startTwiddle();
		void accumError(double error);
		bool testCarLeftTheTrack(double speed, double cte);

		void printState();

	private:
		void updateParams(bool isCrash, bool isTrackPasses);
		void reset();
		void nextParamToTune();
		bool trackWasPassed() const { return total_dist_ > track_distance_; }
		double calc_err() const;

	private:
		double total_time_;
		double maxThrottle_;
		double curThrottle_;
		Mode mode_;
		double total_dist_;
		double best_err_;
		double err_;
		double err_ticks_;
		PID& pid_;
		size_t iters_; // # of iterations
		int tuned_param_ind_;

		enum ParamsState { inc_param, calc_best_err_after_inc, calc_best_err_after_dec };

		int state_; // 0-run after inc pd, 1-run after decrease dp

		// P.I.D. sequence
		std::vector<double> params_;
		std::vector<double> d_params_;
		std::vector<double> best_params_;

		static constexpr double tol_ = 0.2;
	};


	void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
	{
		// reset
		std::string msg("42[\"reset\", {}]");
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	}
}


// Implementation of PIDTuning
namespace
{
	PIDTuning::PIDTuning(PID& pid, double maxThrottle, Mode mode)
		: total_time_(0)
		, maxThrottle_(maxThrottle)
		, curThrottle_(0.3)
		, mode_(mode)
		, total_dist_(0)
		, best_err_(0)
		, err_(0)
		, err_ticks_(1)
		, pid_(pid)
		, iters_(0)
		, tuned_param_ind_(0)
		, state_(inc_param)
		, params_{ 0., 0., 0. }
//		, d_params_{ 1., 1., 1. } // is better to use if curThrottle_=0.2
		, d_params_{ 0.5, 0.5, 0.5 }  // is better to use if curThrottle_=0.3
		, best_params_{ 0., 0., 0. }
	{
		curThrottle_ = std::min(curThrottle_, maxThrottle_);
	}

	double PIDTuning::integrateVelocity(double speed, Timer::ms duration)
	{
		const auto dur_sec = (double)duration.count() / 1000.;
		total_time_ += dur_sec;
		total_dist_ += speed * dur_sec;
		return total_dist_;
	}

	void PIDTuning::reset()
	{
		total_time_ = 0;
		total_dist_ = 0;
		iters_++;
	}

	void PIDTuning::startTwiddle()
	{
		if (isTuning()) {
			pid_.Init(params_[0], params_[1], params_[2]);
			printState();
		}
	}

	bool PIDTuning::trackPassedOrCrash(bool isCrash)
	{
		bool isTrackPasses = false;

		if (!isCrash) {
			isTrackPasses = trackWasPassed();
		}

		if (isCrash || isTrackPasses) {
			updateParams(isCrash, isTrackPasses);
			reset();
		}

		return isTrackPasses || isCrash;
	}

	bool PIDTuning::testCarLeftTheTrack(double speed, double cte)
	{
		bool isCrash = false;
		if (total_time_ > 7. && std::abs(speed) < 3) {
			isCrash = true;
		}
		return isCrash;
	}

	double PIDTuning::calc_err() const
	{
		double err = err_ / err_ticks_;
		return err;
	}

	void PIDTuning::accumError(double error)
	{
		err_ticks_++;
		err_ += error * error;
	}

	void PIDTuning::updateParams(bool isCrash, bool isTrackPasses)
	{
		double err = calc_err();
		assert(err > 0);

		// Check if tuning is finished
		double p_sum = 0.;
		for (auto par : d_params_) {
			p_sum += par;
		}
		if (p_sum < tol_) {
			// tuning is finished for currect speed!
			double throttle = std::min(curThrottle_ + 0.05, maxThrottle_);
			if (curThrottle_ == throttle) {
				mode_ = Mode::driving;
				return;
			}
			else { // continue tuning
				curThrottle_ = throttle;
				if (curThrottle_ == maxThrottle_) {

				}
			}
		}

		if (!iters_) {
			best_err_ = err;
		}

		{
			if (state_ == inc_param) {
				params_[tuned_param_ind_] += d_params_[tuned_param_ind_];
				pid_.Init(params_[0], params_[1], params_[2]);
				state_ = calc_best_err_after_inc; // run next error calculation on the same track
			}
			else if (state_ == calc_best_err_after_inc) {
				if (err < best_err_) {
					best_err_ = err;
					d_params_[tuned_param_ind_] *= inc_step;
					best_params_ = params_;

					nextParamToTune();

					params_[tuned_param_ind_] += d_params_[tuned_param_ind_];
					pid_.Init(params_[0], params_[1], params_[2]);
					state_ = calc_best_err_after_inc; // run next error calculation on the same track
				}
				else {
					params_[tuned_param_ind_] -= 2* d_params_[tuned_param_ind_];
					pid_.Init(params_[0], params_[1], params_[2]);
					state_ = calc_best_err_after_dec; // run next error calculation on the same track
				}
			}
			else if (state_ == calc_best_err_after_dec) {
				if (err < best_err_) {
					best_err_ = err;
					d_params_[tuned_param_ind_] *= inc_step;
					best_params_ = params_;
				}
				else {
					params_[tuned_param_ind_] += d_params_[tuned_param_ind_];
					d_params_[tuned_param_ind_] *= dec_step;
				}

				nextParamToTune();

				params_[tuned_param_ind_] += d_params_[tuned_param_ind_];
				pid_.Init(params_[0], params_[1], params_[2]);
				state_ = calc_best_err_after_inc; // run next error calculation on the same track
			}
		}

		err_ = 0;
		err_ticks_ = 1;
	}

	void PIDTuning::nextParamToTune()
	{
		++tuned_param_ind_;

//		if (tuned_param_ind_ == 1) {
//			++tuned_param_ind_; // skip I-term;
//		}

		if (tuned_param_ind_ >= 3) {
			iters_++;
			tuned_param_ind_ = 0;
		}
	}

	void PIDTuning::printState()
	{
		std::cout << "Iter: " << iters_ << " best_err: " << best_err_ << " err: " << calc_err() << std::endl;
		std::cout << "params: " << params_[0] << ", " << params_[1] << ", " << params_[2] << ", learning:" << isTuning() << std::endl;
	}

}


int main()
{
	  uWS::Hub h;

	PID pid;
	// TODO: Initialize the pid variable.
	//pid.Init(0.3, 0.0005, 20.); // Manual selected params, throttle <= 0.5
	pid.Init(1, 0, 27); // Automatically learned params, throttle <= 0.5

	// Automatically learned params.
	// Twiddle: 0.2
	// P.I.D. {2.1, 0, 15.9}
	// Twiddle: 0.3
	// P.I.D. {0.1, 0, 7.8}

	Mode mode = Mode::driving;
	double maxThrottle = 0.5;

#ifdef enable_tuning_mode
	mode = Mode::tuning;
	maxThrottle = 0.5;
#endif

	Timer timer;
	PIDTuning tuning(pid, maxThrottle, mode);

	if (tuning.isTuning()) {
		tuning.startTwiddle();
	}

  h.onMessage([&pid, &tuning, &timer, maxThrottle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

		  pid.UpdateError(cte);
		  const double steer_value = pid.TotalError();

		  double dist = 0;
		  Timer::ms duration = timer.duration();
		  if (tuning.isTuning()) {
			  dist = tuning.integrateVelocity(speed, duration);
			  tuning.accumError(cte);
			  if (tuning.trackPassedOrCrash(tuning.testCarLeftTheTrack(speed, cte))) {
				  reset_simulator(ws);
				  timer.start();
			  }
		  }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Distance: " << dist << std::endl;

		  tuning.printState();

		  double throttle = !tuning.isTuning() ? maxThrottle : tuning.getCurrThrottle();

		  if (!tuning.isTuning()) {
			  if (std::abs(cte) > 2) {
				  throttle = -0.001;
			  }
		  }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
		  msgJson["throttle"] = throttle;// 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
#ifdef use_ipv4
  if (h.listen("0.0.0.0", port))
#else
  if (h.listen(port))
#endif
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
