#include "logger.h"

namespace coacd
{
    static std::ofstream _logger{};
    std::ostream &logger(bool _cout_enabled, bool _logger_enabled, std::string const filename)
    {
        if (_logger_enabled)
        {
            if (!_logger.is_open())
            {
                _logger.open(filename);
            }
            return _logger;
        }

        if (_cout_enabled)
        {
            return std::cout;
        }

        static NullBuffer null_buffer;
        static std::ostream null_stream(&null_buffer);
        return null_stream;
    }
}