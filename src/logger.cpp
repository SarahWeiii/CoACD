
#include "logger.h"
#include <spdlog/sinks/stdout_color_sinks.h>

namespace coacd
{
    namespace logger
    {

        std::shared_ptr<spdlog::logger> get()
        {
            static std::shared_ptr<spdlog::logger> logger;
            if (!logger)
            {
                logger = spdlog::stdout_color_mt("CoACD");
                logger->set_level(spdlog::level::info);
            }
            return logger;
        }
    } // namespace log
}