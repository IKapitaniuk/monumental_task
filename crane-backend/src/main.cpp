#include <application.h>
#include <json_utils.h>

#include <boost/program_options.hpp>
#include <exception>
#include <filesystem>
#include <iostream>

namespace po = boost::program_options;
namespace fs = std::filesystem;

namespace {
constexpr unsigned default_port = 8080;
}

int main(int argc, char* argv[]) {
  unsigned port = default_port;
  assignment::Configuration config;
  try {
    po::options_description desc{"Options"};
    desc.add_options()("help,h", "Help screen")(
        "port,p", po::value<unsigned>()->default_value(default_port),
        "Port number")("config-file,c", po::value<fs::path>(),
                       "Path to config file: /path/to/config/file");

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      std::cout << desc << '\n';
      return EXIT_SUCCESS;
    }
    if (vm.count("port")) {
      port = vm["port"].as<unsigned>();
    }
    if (vm.count("config-file")) {
      fs::path config_file = vm["config-file"].as<fs::path>();
      fs::path config_file_path = fs::canonical(config_file);
      std::cout << "Config file: " << config_file_path << '\n';
      config = assignment::read_configuration(config_file_path);
    } else {
      std::cout << "Unknown configuration!\n";
      return EXIT_FAILURE;
    }

  } catch (const po::error& ex) {
    std::cerr << ex.what() << '\n';
    return EXIT_FAILURE;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << '\n';
    return EXIT_FAILURE;
  }

  assignment::Application app{port, std::move(config)};
  app.run();
  return EXIT_SUCCESS;
}
