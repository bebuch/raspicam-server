#include <http/server_server.hpp>
#include <http/server_file_request_handler.hpp>
#include <http/server_lambda_request_handler.hpp>
#include <http/websocket_server_request_handler.hpp>
#include <http/websocket_server_json_service.hpp>

#include <logsys/log.hpp>
#include <logsys/stdlogb.hpp>

#include <bitmap/bitmap.hpp>
#include <bitmap/pixel.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <raspicam/raspicam.h>

#include <turbojpeg.h>


template < typename T >
std::string to_jpg_image(bmp::bitmap< T > const& img, int quality){
	// JPEG encoding
	auto compressor_deleter = [](void* data){ tjDestroy(data); };
	std::unique_ptr< void, decltype(compressor_deleter) > compressor(
		tjInitCompress(),
		compressor_deleter
	);
	if(!compressor) throw std::runtime_error("tjInitCompress failed");

	// Image buffer
	std::uint8_t* data = nullptr;
	unsigned long size = 0;

	if(tjCompress2(
		compressor.get(),
		const_cast< std::uint8_t* >(
			reinterpret_cast< std::uint8_t const* >(img.data())),
		static_cast< int >(img.width()),
		static_cast< int >(img.width() * sizeof(T)),
		static_cast< int >(img.height()),
		sizeof(T) == 3 ? TJPF_RGB: TJPF_GRAY,
		&data,
		&size,
		sizeof(T) == 3 ? TJSAMP_420 : TJSAMP_GRAY,
		quality,
		0
	) != 0) throw std::runtime_error("tjCompress2 failed");

	auto data_deleter = [](std::uint8_t* data){ tjFree(data); };
	std::unique_ptr< std::uint8_t, decltype(data_deleter) > data_ptr(
		data,
		data_deleter
	);

	// output
	return std::string(data_ptr.get(), data_ptr.get() + size);
}


namespace pixel = bmp::pixel;

class camera{
public:
	camera(){
		cam_.setFormat(raspicam::RASPICAM_FORMAT_BGR);
		if(!cam_.open(true)){
			throw std::runtime_error("Can not connect to raspicam");
		}
	}

	bmp::bitmap< pixel::rgb8u > get(){
		std::lock_guard< std::mutex > lock(mutex_);

		cam_.grab();
		auto const width = cam_.getWidth();
		auto const height = cam_.getHeight();
		auto const data = reinterpret_cast< pixel::rgb8u const* >(
			cam_.getImageBufferData());
		if(data == nullptr){
			throw std::runtime_error("raspicam getImageBufferData failed");
		}

		bmp::bitmap< pixel::rgb8u > result(width, height);
		std::copy(data, data + result.point_count(), result.begin());
		return result;
	}

private:
	std::mutex mutex_;
	raspicam::RaspiCam cam_;
};


class live_service: public http::websocket::server::json_service{
public:
	live_service(std::string const& ready_signal):
		http::websocket::server::json_service(
			[this](
				boost::property_tree::ptree const& data,
				http::server::connection_ptr const& con
			){
				try{
					if(!data.get< bool >(ready_signal_)) return;
					std::lock_guard< std::mutex > lock(mutex_);
					ready_.at(con) = true;
				}catch(...){}
			},
			http::websocket::server::data_callback_fn(),
			[this](http::server::connection_ptr const& con){
				std::lock_guard< std::mutex > lock(mutex_);
				ready_.emplace(con, false);
			},
			[this](http::server::connection_ptr const& con){
				std::lock_guard< std::mutex > lock(mutex_);
				ready_.erase(con);
			}
		),
		ready_signal_(ready_signal) {}

	void send(std::string const& data){
		std::lock_guard< std::mutex > lock(mutex_);
		for(auto& pair: ready_){
			if(!pair.second) continue;
			pair.second = false;
			send_binary(data, pair.first);
		}
	}


private:
	std::string const ready_signal_;

	std::mutex mutex_;
	std::map< http::server::connection_ptr, bool > ready_;
};


class websocket_identifier{
private:
	websocket_identifier(std::string const& name)
		: name(name) {}

	std::string const& name;

	friend class request_handler;
};

struct http_server_init_t{
	websocket_identifier key;
	bool success;
};

class request_handler: public http::server::lambda_request_handler{
public:
	request_handler(std::string const& http_root_path)
		: http::server::lambda_request_handler(
			[this](
				http::server::connection_ptr con,
				http::request const& req,
				http::reply& rep
			){
				return
					websocket_handler_.handle_request(con, req, rep) ||
					http_file_handler_.handle_request(con, req, rep);
			},
			[this]{
				websocket_handler_.shutdown();
			}
		)
		, http_file_handler_(http_root_path)
		{}

	http_server_init_t init(std::string const& service_name){
		std::lock_guard< std::mutex > lock(mutex_);
		auto [iter, success] = websocket_services_.try_emplace(
			service_name, "ready");

		if(success){
			websocket_handler_.register_service(service_name, iter->second);
		}

		return http_server_init_t{iter->first, success};
	}

	websocket_identifier unique_init(std::string const& service_name){
		auto [key, success] = init(service_name);

		if(success) return key;

		throw std::runtime_error("service name already exist: "
			+ service_name);
	}

	void uninit(websocket_identifier key){
		std::lock_guard< std::mutex > lock(mutex_);
		websocket_handler_.shutdown_service(key.name);
		websocket_services_.erase(key.name);
	}

	void send(websocket_identifier key, std::string const& data){
		std::lock_guard< std::mutex > lock(mutex_);
		websocket_services_.at(key.name).send(data);
	}


private:
	/// \brief Protects websocket_services_
	std::mutex mutex_;

	/// \brief Handler for normal HTTP-File-Requests
	http::server::file_request_handler http_file_handler_;

	/// \brief Handler for Websocket-Requests
	http::websocket::server::request_handler websocket_handler_;

	/// \brief WebSocket live services
	std::map< std::string, live_service > websocket_services_;
};


class http_server{
public:
	http_server(
		std::string const& root,
		std::uint16_t port,
		std::size_t thread_count
	)
		: handler_(root)
		, server_(std::to_string(port), handler_, thread_count) {}

	http_server(http_server&&) = default;

	~http_server(){
		shutdown();
	}

	void shutdown(){
		handler_.shutdown();
	}

	http_server_init_t init(std::string const& service_name){
		return handler_.init(service_name);
	}

	websocket_identifier unique_init(std::string const& service_name){
		return handler_.unique_init(service_name);
	}

	void uninit(websocket_identifier key){
		return handler_.uninit(key);
	}

	void send(websocket_identifier key, std::string const& data){
		return handler_.send(key, data);
	}


private:
	request_handler handler_;
	http::server::server server_;
};




class live_chain{
public:
	live_chain(http_server& server, websocket_identifier const& identifier)
		: server_(server)
		, identifier_(identifier)
		, active_(true)
		, thread_([this]{
			while(active_){
				logsys::exception_catching_log(
					[](logsys::stdlogb& os){
						os << "server live exec";
					}, [this]{
						auto const interval = std::chrono::milliseconds(200);

						while(active_){
							logsys::exception_catching_log(
								[](logsys::stdlogb& os){
									os << "server live exec loop";
								}, [this, interval]{
									while(active_){
										auto const start = std::chrono
											::high_resolution_clock::now();

										auto const img = cam_.get();
										auto const data = to_jpg_image(img, 75);
										server_.send(identifier_, data);

										auto const end = std::chrono
											::high_resolution_clock::now();
										std::chrono::duration< double,
											std::milli > const diff
												= end - start;
										if(diff < interval){
											if(!active_) break;
											std::this_thread::sleep_for(
												interval - diff);
										}
									}
								});
						}
					});
			}
		}) {}

	live_chain(live_chain const&) = delete;
	live_chain(live_chain&&) = delete;

	void shutdown_hint(){
		active_ = false;
	}

	~live_chain(){
		shutdown_hint();
		thread_.join();
	}

private:
	camera cam_;
	http_server& server_;
	websocket_identifier identifier_;
	std::atomic< bool > active_;
	std::thread thread_;
};


int main(int argc, char* argv[]){
	try{
		// Check command line arguments.
		if(argc != 3){
			std::cerr << "Usage: http_server <doc_root> <port>\n";
			std::cerr << "  For IPv4, try:\n";
			std::cerr << "    receiver 0.0.0.0 80 .\n";
			std::cerr << "  For IPv6, try:\n";
			std::cerr << "    receiver 0::0 80 .\n";
			return 1;
		}

		// Initialise the server.
		http_server server(argv[1], std::stoul(argv[2]), 1);
		auto identifier = server.unique_init("live");
		live_chain chain(server, identifier);
		std::cin.get();

	}catch (std::exception& e){
		std::cerr << "exception: " << e.what() << "\n";
	}

	return 0;

}
