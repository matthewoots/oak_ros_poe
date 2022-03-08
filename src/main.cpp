#include <oak_ros/OakRosFactory.hpp>

#include <spdlog/spdlog.h>

#include <boost/program_options.hpp>

// Preset good for indoor calibration

inline OakRosParams getVIOParams()
{
    OakRosParams params;

    // enable raw stereo output, without depth generation
    params.enable_stereo = true;
    params.enable_imu = true;

    return params;
}

OakRosParams getIndoorLightingParams()
{
    auto params = getVIOParams();

    params.manual_exposure = 1000; // in usec
    params.manual_iso = 300; // 100 to 1600

    return params;
}

// preset that good for indoor low light
OakRosParams getLowLightParams()
{
    auto params = getVIOParams();

    params.manual_exposure = 8000; // in usec
    params.manual_iso = 1600; // 100 to 1600

    return params;
}

namespace po = boost::program_options;

int main(int argc, char **argv)
{
    // spdlog::set_level(spdlog::level::debug);

    ros::init(argc, argv, "oak_ros");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    po::options_description desc ("Oak ROS Wrapper for multiple camera setup, with calibration modes");

    int option_frequency;
    std::string option_exposure_mode;
    bool option_depth;
    bool option_apriltag_detection;
    bool option_rectified;
    bool option_rates_workaround;

    desc.add_options ()
        ("help,h", "print usage message")
        ("frequency,f", po::value(&option_frequency)->default_value(-1, "full-rate"), "set frequency not to be at full rate")
        ("depth,d", po::value(&option_depth)->default_value(false, "false"), "publish depth")
        ("apriltag_detection,a", po::value(&option_apriltag_detection)->default_value(false, "false"), "publish apriltag detection")
        ("rectifed,r", po::value(&option_rectified)->default_value(true, "true"), "rectify / undistort stereo image")
        ("exposure_mode,m", po::value(&option_exposure_mode)->default_value("auto", "auto exposure"), "Exposure mode: auto, indoor, low-light, calibration")
        ("rates_workaround", po::value(&option_rates_workaround)->default_value(true, "true"), "Enable to half the rates of OV7251 sensor, and use alternative rate control")
        ;

    po::variables_map vm;
    po::store (po::command_line_parser (argc, argv).options (desc).run (), vm);
    po::notify (vm);

    if (vm.count("help")) {  
        std::cout << desc << "\n";
        return 0;
    }

    auto device_ids = OakRosFactory::getAllAvailableDeviceIds();

    std::vector<OakRosInterface::Ptr> oak_handlers;

    size_t topic_name_seq = 1;
    for (auto& id : device_ids)
    {
        spdlog::info("main: start device with id {}", id);
        
        OakRosInterface::Ptr handler = oak_handlers.emplace_back(OakRosFactory::getOakRosHandler());

        OakRosParams params;
        
        // decide what params to use based on command-line inputs
        constexpr unsigned int FULL_FPS = 30;
        {
            if(option_exposure_mode == "auto")
                params = getVIOParams();
            else if (option_exposure_mode == "low-light")
                params = getLowLightParams();
            else if (option_exposure_mode == "indoor")
                params = getIndoorLightingParams();
            else if (option_exposure_mode == "calibration")
            {
                params = getIndoorLightingParams();

                option_frequency = 4;
                
            }else{
                spdlog::warn("invalid mode {}", option_exposure_mode);
                return -1;
            }
                

            if(option_frequency > 0)
            {
                if (option_rates_workaround)
                    params.stereo_fps_throttle = FULL_FPS / option_frequency + 1;
                else
                    params.stereo_fps = option_frequency;
            }
                
        }

        params.enable_depth = option_depth;
        
        params.device_id = id;
        params.topic_name = "oak" + std::to_string(topic_name_seq);

        params.rates_workaround = option_rates_workaround;
        params.enable_stereo_rectified = option_rectified;

        params.enable_apriltag = option_apriltag_detection;

        handler->init(nh_local, params);

        topic_name_seq++;
    }

    ros::spin();

    spdlog::info("main exits cleanly");
}